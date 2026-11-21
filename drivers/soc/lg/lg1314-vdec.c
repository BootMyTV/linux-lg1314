// SPDX-License-Identifier: GPL-2.0
/*
 * LG1314 video-decode coprocessor (Xtensa MCU) bring-up.
 *
 * The LG1314 performs video decode on an on-chip Xtensa coprocessor that runs a
 * firmware image loaded at runtime from a reserved DRAM carveout. The host
 * programs the MCU control block to point the processor at the firmware and
 * releases it from reset, then communicates over a shared-memory mailbox with a
 * doorbell interrupt.
 *
 * This driver loads the firmware, boots the coprocessor, and exposes the
 * firmware's debug console as a virtual serial port (ttyLXIPC0): bytes are
 * carried through ring buffers in the shared carveout, with head/tail indices
 * in the mailbox and the doorbell signalling data in either direction. The
 * decode interfaces layered on the same mailbox are added separately.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>

#define VDEC_FW_NAME		"lg1314/vdec.bin"

static int vdec_debug;
module_param_named(debug, vdec_debug, int, 0644);
MODULE_PARM_DESC(debug, "dump the IPC mailbox on probe");

/* MCU control block ("RC"): processor control, address windows, params, doorbell. */
#define VDEC_MCU_RC_PHYS	0xc9b00700
#define VDEC_MCU_RC_SIZE	0x200	/* param regs live at +0x100.. */
#define RC_PROC_CTRL		0x00	/* bit0 halt, bit2 debug-en, bit28 reset */
#define RC_SROM_OFFSET		0x10	/* firmware (code) window base, phys */
#define RC_SRAM_OFFSET_0	0x14	/* data window bases, phys */
#define RC_SRAM_OFFSET_1	0x18
#define RC_SRAM_OFFSET_2	0x1c
/* Host-side doorbell block (the firmware/Xtensa side uses 0x40 instead). */
#define RC_INTR_EN		0x30
#define RC_INTR_STATUS		0x34
#define RC_INTR_CLEAR		0x38
#define RC_TO_INTERNAL		0x3c	/* write to signal the coprocessor */
#define RC_PARAM_100		0x100	/* cleared */
#define RC_BOOTPARAM_PTR	0x104	/* copro address of the Xtensa bootparams */
#define RC_PARAM_108		0x108	/* param_base in copro RAM (0x6000_0000) space */
#define RC_PARAM_180		0x180	/* cleared */
#define PROC_CTRL_HALT		BIT(0)
#define PROC_CTRL_DEBUG		BIT(2)
#define PROC_CTRL_RESET		BIT(28)
#define INTR_BIT		0x400

/*
 * Coprocessor address map (fixed in the firmware; the host picks the physical
 * targets via the window registers): code at copro 0x5000_0000 maps through
 * SROM_OFFSET; RAM at copro 0x6000_0000 maps through SRAM_OFFSET_0; the bootparam
 * pointer and param_base are given in copro 0x5000_0000 / 0x6000_0000 space.
 */
#define VDEC_COPRO_ROM		0x50000000
#define VDEC_COPRO_RAM		0x60000000
/* Reserved ROM region size (image + bss + embedded rootfs); M19 = 18 MiB + 512 KiB. */
#define VDEC_ROM_SIZE		(18 * 1024 * 1024 + 512 * 1024)
#define VDEC_BOOTPARAM_MAX	512
#define VDEC_CMDLINE \
	"console=ttyAMA0,115200n81 console=ttyIPC root=mtd1 quiet " \
	"vdec.pool_size=0 vdec.dpb_pool_size=0 vdec.dpb_base=0x%llx " \
	"vdec.vdec_run_in_lite_model=0 vdec.vo_run_in_lite_model=0"
/* Xtensa boot-parameter tags. */
#define BP_TAG_FIRST		0x7b0b
#define BP_TAG_COMMAND_LINE	0x1001
#define BP_TAG_LAST		0x7e0b

/*
 * MCU mailbox ("IPC"): word 0 is the firmware-ready flag; the rest holds the
 * {head,tail} descriptors for the shared-memory rings. The host owns the "send"
 * side of each pair; the coprocessor owns the matching "receive" side.
 */
#define VDEC_MCU_IPC_PHYS	0xc9b00800
#define VDEC_MCU_IPC_SIZE	0x100
#define IPC_READY_FLAG		0x00
#define IPC_LOG_SEND_ADDR	0x60	/* firmware writes this during IPC init */
/* Console ring descriptors (host view): tx owned by us, rx owned by firmware. */
#define IPC_UART_TX_HEAD	0x70
#define IPC_UART_TX_TAIL	0x74
#define IPC_UART_RX_HEAD	0x78
#define IPC_UART_RX_TAIL	0x7c

/* Shared carveout layout: a small queue area, then a 1 MiB argument area. */
#define VDEC_QUEUE_OFF		0x0
#define VDEC_QUEUE_SIZE		0x1000
#define VDEC_ARG_OFF		0x1000
#define VDEC_ARG_SIZE		0x100000
/* Console byte rings within the argument area (host view). */
#define VDEC_UART_TX_OFF	0x40000
#define VDEC_UART_RX_OFF	0x60000
#define VDEC_UART_RING_SIZE	0x20000

#define VDEC_READY_TIMEOUT_MS	1500

struct lg1314_vdec {
	struct device		*dev;
	void __iomem		*rc;		/* MCU control block */
	void __iomem		*ipc;		/* MCU mailbox */
	void			*arg;		/* argument area (carveout, WC) */
	int			irq;

	phys_addr_t		mem_phys;	/* firmware/work carveout */
	size_t			mem_size;
	void __iomem		*rom;		/* ROM region (image + bootparam) */

	struct uart_driver	uart_driver;
	struct uart_port	uart_port;
	bool			rx_running;
};

/* ---- doorbell ---- */

static void vdec_ring_doorbell(struct lg1314_vdec *v)
{
	writel(1, v->rc + RC_TO_INTERNAL);
}

/* ---- reset / boot ---- */

static int vdec_load_firmware(struct lg1314_vdec *v)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, VDEC_FW_NAME, v->dev);
	if (ret) {
		dev_err(v->dev, "cannot load %s: %d\n", VDEC_FW_NAME, ret);
		return ret;
	}
	if (fw->size > VDEC_ROM_SIZE - VDEC_BOOTPARAM_MAX) {
		dev_err(v->dev, "firmware (%zu) too large for ROM region\n", fw->size);
		ret = -EFBIG;
		goto out;
	}
	memcpy_toio(v->rom, fw->data, fw->size);
	dev_info(v->dev, "loaded %s (%zu bytes) at %pa\n",
		 VDEC_FW_NAME, fw->size, &v->mem_phys);
out:
	release_firmware(fw);
	return ret;
}

/*
 * Build the Xtensa boot-parameter tag list (just a command line) at the end of
 * the ROM region, and point the MCU param register at it.
 */
static void vdec_setup_bootparams(struct lg1314_vdec *v)
{
	u8 bp[VDEC_BOOTPARAM_MAX];
	char cmdline[160];
	unsigned int clen, a = 0;

	/* dpb_base is only consulted when decode starts; a reserved phys is fine. */
	snprintf(cmdline, sizeof(cmdline), VDEC_CMDLINE,
		 (u64)v->mem_phys + 0x10000000);
	clen = (strlen(cmdline) + 1 + 7) & ~7u;

	memset(bp, 0, sizeof(bp));
	/* BP_TAG_FIRST, size 0 */
	bp[a++] = BP_TAG_FIRST & 0xff; bp[a++] = BP_TAG_FIRST >> 8;
	bp[a++] = 0; bp[a++] = 0;
	/* BP_TAG_COMMAND_LINE, size clen, then the string */
	bp[a++] = BP_TAG_COMMAND_LINE & 0xff; bp[a++] = BP_TAG_COMMAND_LINE >> 8;
	bp[a++] = clen & 0xff; bp[a++] = clen >> 8;
	memcpy(bp + a, cmdline, strlen(cmdline) + 1);
	a += clen;
	/* BP_TAG_LAST, size 0 */
	bp[a++] = BP_TAG_LAST & 0xff; bp[a++] = BP_TAG_LAST >> 8;
	bp[a++] = 0; bp[a++] = 0;

	memcpy_toio(v->rom + (VDEC_ROM_SIZE - VDEC_BOOTPARAM_MAX), bp, sizeof(bp));
	writel(VDEC_COPRO_ROM + VDEC_ROM_SIZE - VDEC_BOOTPARAM_MAX,
	       v->rc + RC_BOOTPARAM_PTR);
}

/* Dump the mailbox so a failed boot shows whether the Xtensa executed at all. */
static void vdec_dump_mailbox(struct lg1314_vdec *v)
{
	int i;

	if (!vdec_debug)
		return;
	for (i = 0; i < 0x80; i += 0x10)
		dev_info(v->dev, "mailbox +%02x: %08x %08x %08x %08x\n", i,
			 readl(v->ipc + i), readl(v->ipc + i + 4),
			 readl(v->ipc + i + 8), readl(v->ipc + i + 12));
}

static int vdec_boot(struct lg1314_vdec *v)
{
	phys_addr_t base = v->mem_phys;		/* ROM load = RAM base */
	unsigned long deadline;
	u32 ready;

	/* Hold the core in reset + halt while we program it. */
	writel(PROC_CTRL_RESET | PROC_CTRL_HALT, v->rc + RC_PROC_CTRL);

	/* Clear the mailbox (RC+0x100.. == IPC) so the firmware starts from a known
	 * state and its writes are distinguishable from stale values. */
	memset_io(v->ipc, 0, VDEC_MCU_IPC_SIZE);

	/* Address windows: code and RAM both at the carveout base; the two spare
	 * data windows point at reserved DRAM so a stray access can't corrupt Linux. */
	writel(lower_32_bits(base), v->rc + RC_SROM_OFFSET);
	writel(lower_32_bits(base), v->rc + RC_SRAM_OFFSET_0);
	writel(0xaf000000, v->rc + RC_SRAM_OFFSET_1);
	writel(0xaf000000, v->rc + RC_SRAM_OFFSET_2);

	/* Param registers (RC+0x100..). param_base == RAM base, so its copro-RAM
	 * address is simply VDEC_COPRO_RAM. */
	writel(0, v->rc + RC_PARAM_100);
	writel(VDEC_COPRO_RAM, v->rc + RC_PARAM_108);
	writel(0, v->rc + RC_PARAM_180);
	vdec_setup_bootparams(v);

	/* Release reset, enable debug, release halt -> the core runs. */
	writel(PROC_CTRL_HALT, v->rc + RC_PROC_CTRL);
	writel(PROC_CTRL_HALT | PROC_CTRL_DEBUG, v->rc + RC_PROC_CTRL);
	writel(PROC_CTRL_DEBUG, v->rc + RC_PROC_CTRL);

	/* Wait for the firmware to bring up its IPC: it writes the log-queue
	 * descriptor (addr at mailbox +0x60) during early init. The ready flag at
	 * +0x00 is not a reliable signal (the core keeps running across host
	 * reboots, so it can hold a stale value). */
	deadline = jiffies + msecs_to_jiffies(VDEC_READY_TIMEOUT_MS);
	do {
		ready = readl(v->ipc + IPC_LOG_SEND_ADDR);
		if (ready)
			break;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));

	vdec_dump_mailbox(v);
	if (!ready) {
		dev_err(v->dev, "coprocessor IPC did not come up\n");
		return -ETIMEDOUT;
	}

	/* Enable the Xtensa->ARM doorbell only now that its IPC is up. */
	writel(INTR_BIT, v->rc + RC_INTR_EN);
	dev_info(v->dev, "coprocessor up (log queue @ %#x)\n", ready);
	return 0;
}

/* ---- console: shared-memory byte rings ---- */

/* Drain the firmware->host ring into the tty. Called from the doorbell IRQ. */
static void vdec_uart_rx(struct lg1314_vdec *v)
{
	struct tty_port *tport = &v->uart_port.state->port;
	u32 head = readl(v->ipc + IPC_UART_RX_HEAD);
	u32 tail = readl(v->ipc + IPC_UART_RX_TAIL);
	bool pushed = false;

	if (head == tail)
		return;		/* nothing to drain; don't ack (avoids a storm) */

	while (head != tail) {
		u32 chunk = (tail <= head) ? head - tail
					   : VDEC_UART_RING_SIZE - tail;
		u8 buf[256];
		int moved;

		if (chunk > sizeof(buf))
			chunk = sizeof(buf);
		memcpy(buf, v->arg + VDEC_UART_RX_OFF + tail, chunk);

		if (v->rx_running) {
			moved = tty_insert_flip_string(tport, buf, chunk);
			pushed |= moved > 0;
		} else {
			moved = chunk;	/* discard if the port isn't open */
		}
		if (moved <= 0)
			break;

		tail = (tail + moved) % VDEC_UART_RING_SIZE;
	}

	writel(tail, v->ipc + IPC_UART_RX_TAIL);
	vdec_ring_doorbell(v);
	if (pushed)
		tty_flip_buffer_push(tport);
}

/* Ring is a power of two, so masking is the modulo. */
#define VDEC_UART_RING_MASK	(VDEC_UART_RING_SIZE - 1)

/* Push pending tty data into the host->firmware ring, one byte at a time. */
static void vdec_uart_tx(struct lg1314_vdec *v)
{
	struct uart_port *port = &v->uart_port;
	u32 head0 = readl(v->ipc + IPC_UART_TX_HEAD);
	u32 head = head0;
	u8 ch;

	uart_port_tx(port, ch,
		/* tx_ready: the firmware has consumed enough to leave room */
		((readl(v->ipc + IPC_UART_TX_TAIL) - head - 1)
			& VDEC_UART_RING_MASK) != 0,
		/* put the byte into the ring and advance our head */
		({
			memcpy(v->arg + VDEC_UART_TX_OFF + head, &ch, 1);
			head = (head + 1) & VDEC_UART_RING_MASK;
		}));

	if (head != head0) {
		writel(head, v->ipc + IPC_UART_TX_HEAD);
		vdec_ring_doorbell(v);
	}
}

static irqreturn_t vdec_irq(int irq, void *data)
{
	struct lg1314_vdec *v = data;

	writel(INTR_BIT, v->rc + RC_INTR_CLEAR);
	vdec_uart_rx(v);
	return IRQ_HANDLED;
}

/* ---- uart_ops ---- */

static unsigned int vdec_tx_empty(struct uart_port *port)
{
	struct lg1314_vdec *v = container_of(port, struct lg1314_vdec, uart_port);

	return readl(v->ipc + IPC_UART_TX_HEAD) == readl(v->ipc + IPC_UART_TX_TAIL)
		? TIOCSER_TEMT : 0;
}

static void vdec_start_tx(struct uart_port *port)
{
	vdec_uart_tx(container_of(port, struct lg1314_vdec, uart_port));
}

static int vdec_startup(struct uart_port *port)
{
	container_of(port, struct lg1314_vdec, uart_port)->rx_running = true;
	return 0;
}

static void vdec_shutdown(struct uart_port *port)
{
	container_of(port, struct lg1314_vdec, uart_port)->rx_running = false;
}

static void vdec_stop_rx(struct uart_port *port)
{
	container_of(port, struct lg1314_vdec, uart_port)->rx_running = false;
}

static unsigned int vdec_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR;
}

static void vdec_set_mctrl(struct uart_port *port, unsigned int mctrl) { }
static void vdec_stop_tx(struct uart_port *port) { }
static void vdec_break_ctl(struct uart_port *port, int ctl) { }
static void vdec_set_termios(struct uart_port *port, struct ktermios *new,
			     const struct ktermios *old) { }
static const char *vdec_type(struct uart_port *port) { return "LXIPC"; }
static void vdec_release_port(struct uart_port *port) { }
static int vdec_request_port(struct uart_port *port) { return 0; }
static void vdec_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_GENERIC;
}

static const struct uart_ops vdec_uart_ops = {
	.tx_empty	= vdec_tx_empty,
	.set_mctrl	= vdec_set_mctrl,
	.get_mctrl	= vdec_get_mctrl,
	.stop_tx	= vdec_stop_tx,
	.start_tx	= vdec_start_tx,
	.stop_rx	= vdec_stop_rx,
	.break_ctl	= vdec_break_ctl,
	.startup	= vdec_startup,
	.shutdown	= vdec_shutdown,
	.set_termios	= vdec_set_termios,
	.type		= vdec_type,
	.release_port	= vdec_release_port,
	.request_port	= vdec_request_port,
	.config_port	= vdec_config_port,
};

static int vdec_console_register(struct lg1314_vdec *v)
{
	int ret;

	v->uart_driver.owner = THIS_MODULE;
	v->uart_driver.driver_name = "lxipc";
	v->uart_driver.dev_name = "ttyLXIPC";
	v->uart_driver.nr = 1;

	ret = uart_register_driver(&v->uart_driver);
	if (ret)
		return ret;

	v->uart_port.dev = v->dev;
	v->uart_port.ops = &vdec_uart_ops;
	v->uart_port.type = PORT_GENERIC;	/* virtual; name comes from ->type() */
	v->uart_port.iotype = UPIO_MEM;
	v->uart_port.fifosize = 128;
	v->uart_port.line = 0;

	ret = uart_add_one_port(&v->uart_driver, &v->uart_port);
	if (ret)
		uart_unregister_driver(&v->uart_driver);
	return ret;
}

/* ---- probe / remove ---- */

static int lg1314_vdec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lg1314_vdec *v;
	struct device_node *mem;
	struct resource res;
	int ret;

	v = devm_kzalloc(dev, sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	v->dev = dev;

	v->rc = devm_ioremap(dev, VDEC_MCU_RC_PHYS, VDEC_MCU_RC_SIZE);
	v->ipc = devm_ioremap(dev, VDEC_MCU_IPC_PHYS, VDEC_MCU_IPC_SIZE);
	if (!v->rc || !v->ipc)
		return -ENOMEM;

	mem = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!mem)
		return dev_err_probe(dev, -EINVAL, "no memory-region\n");
	ret = of_address_to_resource(mem, 0, &res);
	of_node_put(mem);
	if (ret)
		return dev_err_probe(dev, ret, "bad memory-region\n");
	v->mem_phys = res.start;
	v->mem_size = resource_size(&res);

	v->rom = devm_ioremap_wc(dev, v->mem_phys, VDEC_ROM_SIZE);
	v->arg = devm_memremap(dev, v->mem_phys + VDEC_ARG_OFF, VDEC_ARG_SIZE, MEMREMAP_WC);
	if (!v->rom || IS_ERR_OR_NULL(v->arg))
		return -ENOMEM;
	if (v->mem_size < VDEC_ROM_SIZE)
		return dev_err_probe(dev, -ENOMEM, "carveout smaller than ROM region\n");

	v->irq = platform_get_irq(pdev, 0);
	if (v->irq < 0)
		return v->irq;
	ret = devm_request_irq(dev, v->irq, vdec_irq, 0, "lg1314-vdec", v);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, v);

	ret = vdec_load_firmware(v);
	if (ret)
		return ret;
	ret = vdec_boot(v);
	if (ret)
		return ret;

	return vdec_console_register(v);
}

static void lg1314_vdec_remove(struct platform_device *pdev)
{
	struct lg1314_vdec *v = platform_get_drvdata(pdev);

	uart_remove_one_port(&v->uart_driver, &v->uart_port);
	uart_unregister_driver(&v->uart_driver);
}

static const struct of_device_id lg1314_vdec_of_match[] = {
	{ .compatible = "lge,lg1314-vdec" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg1314_vdec_of_match);

static struct platform_driver lg1314_vdec_driver = {
	.probe = lg1314_vdec_probe,
	.remove = lg1314_vdec_remove,
	.driver = {
		.name = "lg1314-vdec",
		.of_match_table = lg1314_vdec_of_match,
	},
};
module_platform_driver(lg1314_vdec_driver);

MODULE_DESCRIPTION("LG1314 video-decode coprocessor");
MODULE_LICENSE("GPL");
