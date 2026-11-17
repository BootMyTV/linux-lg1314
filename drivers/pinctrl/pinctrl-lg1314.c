// SPDX-License-Identifier: GPL-2.0
/*
 * Pin controller for the LG LG1314 (M19A1) DTV SoC.
 *
 * Each pin's function is selected by a (register, mask, value) field in the
 * pinmux banks. The vendor firmware only ever uses this to switch pins to GPIO
 * mode -- the field values for the non-GPIO functions are programmed by the
 * boot firmware and not described here -- so this driver deliberately exposes
 * only GPIO muxing, reached via the gpio_request_enable path that the on-SoC
 * PL061 GPIO banks use through their DT "gpio-ranges", and advertises no pinmux
 * functions. Register addresses come from the DT "reg" regions; each mux entry
 * carries its field's absolute address, resolved against those regions at probe.
 */
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#define LG1314_MAX_REGIONS	8

struct lg1314_mux {
	u16 pin;
	u8 pins;
	phys_addr_t pa;
	u32 mask;
	u32 enable;
	bool savable;
};

struct lg1314_pinctrl {
	void __iomem *region_va[LG1314_MAX_REGIONS];
	phys_addr_t region_pa[LG1314_MAX_REGIONS];
	resource_size_t region_sz[LG1314_MAX_REGIONS];
	unsigned int nregions;
	void __iomem **mux_va;
	u32 *mux_save;
};

static const struct lg1314_mux lg1314a0_pinmux_info[] = {
	/* CTOP_AUD_R16 */
	{  89, 1, 0xc9c60640, 0x00000001, 0x00000001,  true, },
	{ 115, 1, 0xc9c60640, 0x00000800, 0x00000800,  true, },
	{ 116, 1, 0xc9c60640, 0x00001000, 0x00001000,  true, },
	{ 117, 1, 0xc9c60640, 0x00002000, 0x00002000,  true, },
	{ 118, 1, 0xc9c60640, 0x00004000, 0x00004000,  true, },
	{ 119, 1, 0xc9c60640, 0x00008000, 0x00008000,  true, },
	{ 132, 1, 0xc9c60640, 0x10000000, 0x10000000,  true, },
	{ 133, 1, 0xc9c60640, 0x20000000, 0x20000000,  true, },
	{ 134, 1, 0xc9c60640, 0x40000000, 0x40000000,  true, },
	{ 135, 1, 0xc9c60640, 0x80000000, 0x80000000,  true, },
	/* CTOP_DPE_R37 */
	{   8, 8, 0xc930a090, 0x00780280, 0x00000000,  true, },
	{   0, 8, 0xc930a090, 0x0007e100, 0x00000000,  true, },
	{  24, 8, 0xc930a090, 0x0000007c, 0x00000000,  true, },
	/* CTOP_DPE_R78 */
	{ 105, 1, 0xc930a138, 0x00000002, 0x00000002,  true, },
	{ 106, 1, 0xc930a138, 0x00000004, 0x00000004,  true, },
	{ 107, 1, 0xc930a138, 0x00000008, 0x00000008,  true, },
	{ 108, 1, 0xc930a138, 0x00000010, 0x00000010,  true, },
	{ 109, 1, 0xc930a138, 0x00000020, 0x00000020,  true, },
	{ 110, 1, 0xc930a138, 0x00000040, 0x00000040,  true, },
	{  25, 2, 0xc930a138, 0x00000100, 0x00000000,  true, },
	{  24, 8, 0xc930a138, 0x00000200, 0x00000000,  true, },
	{  26, 1, 0xc930a138, 0x00000400, 0x00000000,  true, },
	/* CTOP_ND0_R04 */
	{  96, 1, 0xc9304c10, 0x00000100, 0x00000100,  true, },
	{  97, 1, 0xc9304c10, 0x00000200, 0x00000200,  true, },
	{  98, 1, 0xc9304c10, 0x00000400, 0x00000400,  true, },
	{  99, 1, 0xc9304c10, 0x00000800, 0x00000800,  true, },
	{ 100, 1, 0xc9304c10, 0x00001000, 0x00001000,  true, },
	{ 101, 1, 0xc9304c10, 0x00002000, 0x00002000,  true, },
	{ 102, 1, 0xc9304c10, 0x00004000, 0x00004000,  true, },
	{ 103, 1, 0xc9304c10, 0x00008000, 0x00008000,  true, },
	{  45, 1, 0xc9304c10, 0x00200000, 0x00200000,  true, },
	{  46, 1, 0xc9304c10, 0x00400000, 0x00400000,  true, },
	{  47, 1, 0xc9304c10, 0x00800000, 0x00800000,  true, },
	/* CTOP_ND0_R05 */
	{ 104, 1, 0xc9304c14, 0x00002000, 0x00002000,  true, },
	{ 111, 1, 0xc9304c14, 0x00004000, 0x00004000,  true, },
	{ 128, 1, 0xc9304c14, 0x00008000, 0x00008000,  true, },
	{ 129, 1, 0xc9304c14, 0x00010000, 0x00010000,  true, },
	{  40, 1, 0xc9304c14, 0x00020000, 0x00020000,  true, },
	{  44, 1, 0xc9304c14, 0x00040000, 0x00040000,  true, },
	{  65, 1, 0xc9304c14, 0x00400000, 0x00400000,  true, },
	{  66, 1, 0xc9304c14, 0x00800000, 0x00800000,  true, },
	{  67, 1, 0xc9304c14, 0x01000000, 0x01000000,  true, },
	{  68, 1, 0xc9304c14, 0x02000000, 0x02000000,  true, },
	{  69, 1, 0xc9304c14, 0x04000000, 0x04000000,  true, },
	{  70, 1, 0xc9304c14, 0x08000000, 0x08000000,  true, },
	{  71, 1, 0xc9304c14, 0x10000000, 0x10000000,  true, },
	{  72, 1, 0xc9304c14, 0x20000000, 0x20000000,  true, },
	{  73, 1, 0xc9304c14, 0x40000000, 0x40000000,  true, },
	/* CTOP_FMS_R00 */
	{  64, 1, 0xc930c400, 0x00000800, 0x00000800,  true, },
	{  43, 1, 0xc930c400, 0x00001000, 0x00001000,  true, },
	{  42, 1, 0xc930c400, 0x00002000, 0x00002000,  true, },
	{  41, 1, 0xc930c400, 0x00004000, 0x00004000,  true, },
	{  63, 1, 0xc930c400, 0x00008000, 0x00008000,  true, },
	{  62, 1, 0xc930c400, 0x00010000, 0x00010000,  true, },
	{  61, 1, 0xc930c400, 0x00020000, 0x00020000,  true, },
	{  60, 1, 0xc930c400, 0x00040000, 0x00040000,  true, },
	{  59, 1, 0xc930c400, 0x00100000, 0x00100000,  true, },
	{  58, 1, 0xc930c400, 0x00200000, 0x00200000,  true, },
	{  57, 1, 0xc930c400, 0x00400000, 0x00400000,  true, },
	{  56, 1, 0xc930c400, 0x00800000, 0x00800000,  true, },
	{  87, 1, 0xc930c400, 0x01000000, 0x01000000,  true, },
	{  86, 1, 0xc930c400, 0x02000000, 0x02000000,  true, },
	{  85, 1, 0xc930c400, 0x04000000, 0x04000000,  true, },
	{  84, 1, 0xc930c400, 0x08000000, 0x08000000,  true, },
	{  83, 1, 0xc930c400, 0x10000000, 0x10000000,  true, },
	{  82, 1, 0xc930c400, 0x20000000, 0x20000000,  true, },
	{  81, 1, 0xc930c400, 0x40000000, 0x40000000,  true, },
	{  80, 1, 0xc930c400, 0x80000000, 0x80000000,  true, },
	/* CTOP_FMS_R06 */
	{  16, 8, 0xc930c418, 0x00010000, 0x00000000,  true, },
	{  22, 2, 0xc930c418, 0x00020000, 0x00000000,  true, },
	{  17, 1, 0xc930c418, 0x00040000, 0x00000000,  true, },
	{  20, 1, 0xc930c418, 0x00100000, 0x00000000,  true, },
	{  19, 2, 0xc930c418, 0x00200000, 0x00000000,  true, },
	{  18, 2, 0xc930c418, 0x00800000, 0x00000000,  true, },
	{  21, 1, 0xc930c418, 0x01000000, 0x00000000,  true, },
	{  16, 1, 0xc930c418, 0x02000000, 0x00000000,  true, },
	{  16, 8, 0xc930c418, 0xfc000000, 0x00000000,  true, },
	/* CTOP_BMC_R04 */
	{ 136, 1, 0xc930e410, 0x00000100, 0x00002000,  true, },
	{ 138, 1, 0xc930e410, 0x00000200, 0x00002000,  true, },
	{ 137, 1, 0xc930e410, 0x00000400, 0x00002000,  true, },
	{ 139, 1, 0xc930e410, 0x00000800, 0x00002000,  true, },
	{  74, 1, 0xc930e410, 0x00040000, 0x00040000,  true, },
	{  75, 1, 0xc930e410, 0x00080000, 0x00080000,  true, },
	{  76, 1, 0xc930e410, 0x00100000, 0x00100000,  true, },
	{  77, 1, 0xc930e410, 0x00200000, 0x00200000,  true, },
	{  78, 1, 0xc930e410, 0x00400000, 0x00400000,  true, },
	{  79, 1, 0xc930e410, 0x00800000, 0x00800000,  true, },
	{  32, 1, 0xc930e410, 0x01000000, 0x01000000,  true, },
	{  33, 1, 0xc930e410, 0x02000000, 0x02000000,  true, },
	{  34, 1, 0xc930e410, 0x04000000, 0x04000000,  true, },
	{  35, 1, 0xc930e410, 0x08000000, 0x08000000,  true, },
	{  36, 1, 0xc930e410, 0x10000000, 0x10000000,  true, },
	{  37, 1, 0xc930e410, 0x20000000, 0x20000000,  true, },
	{  38, 1, 0xc930e410, 0x40000000, 0x40000000,  true, },
	{  39, 1, 0xc930e410, 0x80000000, 0x80000000,  true, },
};

static const struct pinctrl_pin_desc lg1314a0_pin_desc[] = {
	PINCTRL_PIN(0, "AJ12:GPIO0"),
	PINCTRL_PIN(1, "AH11:GPIO1"),
	PINCTRL_PIN(2, "AH9:GPIO2"),
	PINCTRL_PIN(3, "AJ10:GPIO3"),
	PINCTRL_PIN(4, "AH10:GPIO4"),
	PINCTRL_PIN(5, "AJ11:GPIO5"),
	PINCTRL_PIN(6, "AG9:GPIO6"),
	PINCTRL_PIN(7, "AJ9:GPIO7"),
	PINCTRL_PIN(8, "AH18:GPIO8"),
	PINCTRL_PIN(9, "AH19:GPIO9"),
	PINCTRL_PIN(10, "AK20:GPIO10"),
	PINCTRL_PIN(11, "AJ19:GPIO11"),
	PINCTRL_PIN(12, "AK19:GPIO12"),
	PINCTRL_PIN(13, "AK16:GPIO13"),
	PINCTRL_PIN(14, "AJ18:GPIO14"),
	PINCTRL_PIN(15, "AH20:GPIO15"),
	PINCTRL_PIN(16, "AD7:GPIO16/AUD_PCMI3LRCH1"),
	PINCTRL_PIN(17, "AD5:GPIO17/AUD_PCMI3KRCH2/RTOP_PCM_TRIGGER_VSD"),
	PINCTRL_PIN(18, "AC8:GPIO18/LED_SCL/PHY_SCL_HDMI"),
	PINCTRL_PIN(19, "AB6:GPIO19/LED_SDA/PHY_SDA_HDMI"),
	PINCTRL_PIN(20, "AC6:GPIO20/VS_IN"),
	PINCTRL_PIN(21, "AB7:GPIO21/DACRLRCH"),
	PINCTRL_PIN(22, "AD6:GPIO22"),
	PINCTRL_PIN(23, "AC7:GPIO23"),
	PINCTRL_PIN(24, "AJ14:GPIO24/SCL8/EB_ADDR_OUT16"),
	PINCTRL_PIN(25, "AH13:GPIO25/SDA8/EB_ADDR_OUT17"),
	PINCTRL_PIN(26, "AK14:GPIO26/AUDCLK_OUT_SUB/EB_ADDR_OUT18"),
	PINCTRL_PIN(27, "AK13:GPIO27/EB_ADDR_OUT19"),
	PINCTRL_PIN(28, "AJ16:GPIO28/EB_ADDR_OUT20"),
	PINCTRL_PIN(29, "AH14:GPIO29/EB_ADDR_OUT21"),
	PINCTRL_PIN(30, "AJ13:GPIO30"),
	PINCTRL_PIN(31, "AH12:GPIO31"),
	PINCTRL_PIN(32, "AJ26:SPI_CS1/GPIO32/JTMS1"),
	PINCTRL_PIN(33, "AK28:SPI_SCLK1/GPIO33/JTCK1"),
	PINCTRL_PIN(34, "AJ25:SPI_DO1/GPIO34/JTDO1"),
	PINCTRL_PIN(35, "AK26:SPI_DI1/GPIO35/JTDI1"),
	PINCTRL_PIN(36, "AJ27:SPI_CS0/GPIO36"),
	PINCTRL_PIN(37, "AK25:SPI_SCLK0/GPIO37"),
	PINCTRL_PIN(38, "AH27:SPI_DO0/GPIO38"),
	PINCTRL_PIN(39, "AJ28:SPI_DI0/GPIO39/JTRST1"),
	PINCTRL_PIN(40, "AL29:DISEQC/GPIO40"),
	PINCTRL_PIN(41, "AA8:BOOT_MODE/GPIO41"),
	PINCTRL_PIN(42, "AD8:GPIO42/PLLSET0"),
	PINCTRL_PIN(43, "N/A"),
	PINCTRL_PIN(44, "AN26:IFAGC_S/GPIO44"),
	PINCTRL_PIN(45, "AF28:STPI0_VAL/GPIO45"),
	PINCTRL_PIN(46, "AE28:STPI0_SOP/GPIO46"),
	PINCTRL_PIN(47, "AG28:STPI0_CLK/GPIO47"),
	PINCTRL_PIN(48, "Y30:HDMI2_0_SCL/GPIO48/JTAG_HDMI_TMS"),
	PINCTRL_PIN(49, "w28:HDMI2_0_SCL/GPIO49/JTAG_HDMI_TCK"),
	PINCTRL_PIN(50, "V28:HDMI2_1_SCL/GPIO50/JTAG_HDMI_TDO"),
	PINCTRL_PIN(51, "V28:HDMI2_1_SCL/GPIO51/JTAG_HDMI_TDI"),
	PINCTRL_PIN(52, "u29:HDMI2_2_SDA/GPIO52"),
	PINCTRL_PIN(53, "u30:HDMI2_2_SCL/GPIO53"),
	PINCTRL_PIN(54, "R28:HDMI2_3_SCL/GPIO54"),
	PINCTRL_PIN(55, "R29:HDMI2_3_SDA/GPIO55"),
	PINCTRL_PIN(56, "AE6:DIM1_MOSI/GPIO56/JTDI0"),
	PINCTRL_PIN(57, "AF6:DIM1_SCLK/GPIO57/JTCK0"),
	PINCTRL_PIN(58, "AF7:DIM0_MOSI/GPIO58/JTDO0"),
	PINCTRL_PIN(59, "AE7:DIM0_SCLK/GPIO59/JTMS0"),
	PINCTRL_PIN(60, "AC3:L_VSOUT_LD/GPIO60/JTRST0"),
	PINCTRL_PIN(61, "AB6:PWM2/GPIO61"),
	PINCTRL_PIN(62, "AA6:PWM1/GPIO62"),
	PINCTRL_PIN(63, "AA5:GPIO63/PWM_IN"),
	PINCTRL_PIN(64, "AE8:EXT_INTR0/GPIO64"),
	PINCTRL_PIN(65, "AB28:TP_DVB_DATA0/GPIO65"),
	PINCTRL_PIN(66, "AC28:TP_DVB_DATA1/GPIO66/DISEQCI"),
	PINCTRL_PIN(67, "AD28:TP_DVB_DATA2/GPIO67/DISEQC_CMD"),
	PINCTRL_PIN(68, "AD29:TP_DVB_DATA3/GPIO68/FEFCTRL"),
	PINCTRL_PIN(69, "AE29:TP_DVB_DATA4/GPIO69/DEMOD2SERIALTPDATA"),
	PINCTRL_PIN(70, "AE30:TP_DVB_DATA5/GPIO70/DEMOD2SERIALTPVAL"),
	PINCTRL_PIN(71, "AF29:TP_DVB_DATA6/GPIO71/DEMOD2SERIALTPSOP"),
	PINCTRL_PIN(72, "AH26:SDA5/GPIO72"),
	PINCTRL_PIN(73, "AH25:SCL5/GPIO73"),
	PINCTRL_PIN(74, "AH21:SDA4/GPIO74"),
	PINCTRL_PIN(75, "AJ21:SCL4/GPIO75"),
	PINCTRL_PIN(76, "AJ24:SDA3/GPIO76"),
	PINCTRL_PIN(77, "AH24:SCL3/GPIO77"),
	PINCTRL_PIN(78, "AH22:SDA2/GPIO78"),
	PINCTRL_PIN(79, "AH23:SCL2/GPIO79"),
	PINCTRL_PIN(80, "AE5:GPIO80/FRC_LRSYNC"),
	PINCTRL_PIN(81, "AJ6:DACSCK/GPIO81"),
	PINCTRL_PIN(82, "AH5:AUDCLK_OUT/GPIO82"),
	PINCTRL_PIN(83, "AJ5:DACCLFCH/GPIO83"),
	PINCTRL_PIN(84, "AJ7:DACSLRCH/GPIO84"),
	PINCTRL_PIN(85, "AG5:PCMI3SCK/GPIO85"),
	PINCTRL_PIN(86, "AG6:PCMI3LRCK/GPIO86"),
	PINCTRL_PIN(87, "AH5:PCMI3LRCH/GPIO87"),
	PINCTRL_PIN(88, "E30:EB_WAIT/GPIO88"),
	PINCTRL_PIN(89, "A27:GPIO89/EB_ADDR15"),
	PINCTRL_PIN(90, "B33:GPIO90/EB_CS0"),
	PINCTRL_PIN(91, "B32:GPIO91/EB_CS1"),
	PINCTRL_PIN(92, "B31:GPIO92/EB_CS2"),
	PINCTRL_PIN(93, "A31:GPIO93/EB_CS3"),
	PINCTRL_PIN(94, "AA29:GPIO94"),
	PINCTRL_PIN(95, "AB29:GPIO95"),
	PINCTRL_PIN(96, "AK30:STPI0_DATA0/GPIO96"),
	PINCTRL_PIN(97, "AG29:STPI0_DATA1/GPIO97/DISEQCI"),
	PINCTRL_PIN(98, "AJ30:STPI0_DATA2/GPIO98/DISEQC_CMD"),
	PINCTRL_PIN(99, "AH30:STPI0_DATA3/GPIO99/FEFCTRL"),
	PINCTRL_PIN(100, "AH29:STPI0_DATA4/GPIO100/DEMOD1SERIALTPDATA"),
	PINCTRL_PIN(101, "AJ29:STPI0_DATA5/GPIO101/DEMOD1SERIALTPVAL"),
	PINCTRL_PIN(102, "AH28:STPI0_DATA6/GPIO102/DEMOD1SERIALTPSOP"),
	PINCTRL_PIN(103, "AK29:STPI0_DATA7/GPIO103/DEMOD1SERIALTPCLK"),
	PINCTRL_PIN(104, "N/A"),
	PINCTRL_PIN(105, "AJ17:UART0_TXD/GPIO105"),
	PINCTRL_PIN(106, "AK17:UART0_RXD/GPIO106"),
	PINCTRL_PIN(107, "AH17:GPIO107/UART1_RTS"),
	PINCTRL_PIN(108, "AH16:GPIO108/UART1_CTS"),
	PINCTRL_PIN(109, "AJ15:UART1_TXD/GPIO109"),
	PINCTRL_PIN(110, "AH15:UART1_RXD/GPIO110"),
	PINCTRL_PIN(111, "N/A"),
	PINCTRL_PIN(112, "E26:EB_ADDR0/GPIO112"),
	PINCTRL_PIN(113, "F26:EB_ADDR1/GPIO113"),
	PINCTRL_PIN(114, "D24:EB_ADDR2/GPIO114"),
	PINCTRL_PIN(115, "E24:EB_ADDR3/GPIO115"),
	PINCTRL_PIN(116, "F24:EB_ADDR4/GPIO116"),
	PINCTRL_PIN(117, "D27:EB_ADDR5/GPIO117"),
	PINCTRL_PIN(118, "C27:EB_ADDR6/GPIO118"),
	PINCTRL_PIN(119, "A28:EB_ADDR7/GPIO119"),
	PINCTRL_PIN(120, "C30:EB_DATA0/GPIO120"),
	PINCTRL_PIN(121, "A30:EB_DATA1/GPIO121"),
	PINCTRL_PIN(122, "B30:EB_DATA2/GPIO122"),
	PINCTRL_PIN(123, "D30:EB_DATA3/GPIO123"),
	PINCTRL_PIN(124, "C29:EB_DATA4/GPIO124"),
	PINCTRL_PIN(125, "C28:EB_DATA5/GPIO125"),
	PINCTRL_PIN(126, "B28:EB_DATA6/GPIO126"),
	PINCTRL_PIN(127, "D29:EB_DATA7/GPIO127"),
	PINCTRL_PIN(128, "TP_DVB_SOP/GPIO128"),
	PINCTRL_PIN(129, "TP_DVB_CLK/GPIO129"),
	PINCTRL_PIN(130, "E23:GPIO130"),
	PINCTRL_PIN(131, "F23:GPIO131"),
	PINCTRL_PIN(132, "E22:JTRST2/SC_DATA0/GPIO132"),
	PINCTRL_PIN(133, "D22:JTDI2/SC_RST/GPIO133"),
	PINCTRL_PIN(134, "D21:JTMS2/SC_VCC_SEL/GPIO134"),
	PINCTRL_PIN(135, "E21:JTDO2/SC_VCCEN/GPIO135"),
	PINCTRL_PIN(136, "AJ22:SDA1/GPIO136"),
	PINCTRL_PIN(137, "AK22:SCL1/GPIO137"),
	PINCTRL_PIN(138, "AJ23:SDA0/GPIO138"),
	PINCTRL_PIN(139, "AK23:SCL0/GPIO139"),
	PINCTRL_PIN(140, "N/A"),
	PINCTRL_PIN(141, "N/A"),
	PINCTRL_PIN(142, "N/A"),
	PINCTRL_PIN(143, "N/A"),
};

static int lg1314_get_groups_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *lg1314_get_group_name(struct pinctrl_dev *pctldev,
					 unsigned int selector)
{
	return NULL;
}

static int lg1314_get_group_pins(struct pinctrl_dev *pctldev,
				 unsigned int selector,
				 const unsigned int **pins,
				 unsigned int *num_pins)
{
	return -EINVAL;
}

static const struct pinctrl_ops lg1314_pinctrl_ops = {
	.get_groups_count = lg1314_get_groups_count,
	.get_group_name = lg1314_get_group_name,
	.get_group_pins = lg1314_get_group_pins,
};

static int lg1314_get_functions_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *lg1314_get_function_name(struct pinctrl_dev *pctldev,
					    unsigned int selector)
{
	return NULL;
}

static int lg1314_get_function_groups(struct pinctrl_dev *pctldev,
				      unsigned int selector,
				      const char * const **groups,
				      unsigned int *num_groups)
{
	return -EINVAL;
}

static int lg1314_set_mux(struct pinctrl_dev *pctldev, unsigned int func,
			  unsigned int group)
{
	return -EOPNOTSUPP;
}

/*
 * Switch a pin to GPIO mode. Reached from the GPIO core via the PL061 banks'
 * DT "gpio-ranges"; writes the GPIO-enable field for every mux entry covering
 * this pin.
 */
static int lg1314_gpio_request_enable(struct pinctrl_dev *pctldev,
				      struct pinctrl_gpio_range *range,
				      unsigned int offset)
{
	struct lg1314_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(lg1314a0_pinmux_info); i++) {
		const struct lg1314_mux *m = &lg1314a0_pinmux_info[i];
		u32 v;

		if (offset < m->pin || offset >= m->pin + m->pins)
			continue;

		v = readl_relaxed(pc->mux_va[i]);
		v &= ~m->mask;
		v |= m->enable;
		writel_relaxed(v, pc->mux_va[i]);
	}

	return 0;
}

static const struct pinmux_ops lg1314_pinmux_ops = {
	.get_functions_count = lg1314_get_functions_count,
	.get_function_name = lg1314_get_function_name,
	.get_function_groups = lg1314_get_function_groups,
	.set_mux = lg1314_set_mux,
	.gpio_request_enable = lg1314_gpio_request_enable,
};

static const struct pinctrl_desc lg1314_pinctrl_desc = {
	.name = "lg1314-pinctrl",
	.pins = lg1314a0_pin_desc,
	.npins = ARRAY_SIZE(lg1314a0_pin_desc),
	.pctlops = &lg1314_pinctrl_ops,
	.pmxops = &lg1314_pinmux_ops,
	.owner = THIS_MODULE,
};

static int lg1314_pinctrl_suspend(struct device *dev)
{
	struct lg1314_pinctrl *pc = dev_get_drvdata(dev);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(lg1314a0_pinmux_info); i++)
		if (lg1314a0_pinmux_info[i].savable)
			pc->mux_save[i] = readl_relaxed(pc->mux_va[i]);

	return 0;
}

static int lg1314_pinctrl_resume(struct device *dev)
{
	struct lg1314_pinctrl *pc = dev_get_drvdata(dev);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(lg1314a0_pinmux_info); i++)
		if (lg1314a0_pinmux_info[i].savable)
			writel_relaxed(pc->mux_save[i], pc->mux_va[i]);

	return 0;
}

static const struct dev_pm_ops lg1314_pinctrl_pm_ops = {
	LATE_SYSTEM_SLEEP_PM_OPS(lg1314_pinctrl_suspend, lg1314_pinctrl_resume)
};

static int lg1314_pinctrl_map(struct platform_device *pdev,
			      struct lg1314_pinctrl *pc)
{
	struct device *dev = &pdev->dev;
	unsigned int i, j;

	for (i = 0; i < LG1314_MAX_REGIONS; i++) {
		struct resource *res;
		void __iomem *va;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			break;

		va = devm_ioremap_resource(dev, res);
		if (IS_ERR(va))
			return PTR_ERR(va);

		pc->region_va[i] = va;
		pc->region_pa[i] = res->start;
		pc->region_sz[i] = resource_size(res);
	}
	pc->nregions = i;
	if (!pc->nregions)
		return dev_err_probe(dev, -EINVAL, "no register regions\n");

	pc->mux_va = devm_kcalloc(dev, ARRAY_SIZE(lg1314a0_pinmux_info),
				  sizeof(*pc->mux_va), GFP_KERNEL);
	pc->mux_save = devm_kcalloc(dev, ARRAY_SIZE(lg1314a0_pinmux_info),
				    sizeof(*pc->mux_save), GFP_KERNEL);
	if (!pc->mux_va || !pc->mux_save)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(lg1314a0_pinmux_info); i++) {
		phys_addr_t pa = lg1314a0_pinmux_info[i].pa;

		for (j = 0; j < pc->nregions; j++) {
			if (pa >= pc->region_pa[j] &&
			    pa < pc->region_pa[j] + pc->region_sz[j]) {
				pc->mux_va[i] = pc->region_va[j] +
						(pa - pc->region_pa[j]);
				break;
			}
		}
		if (j == pc->nregions)
			return dev_err_probe(dev, -EINVAL,
					     "mux reg %pa outside DT regions\n",
					     &pa);
	}

	return 0;
}

static int lg1314_pinctrl_probe(struct platform_device *pdev)
{
	struct lg1314_pinctrl *pc;
	struct pinctrl_dev *pctl;
	int ret;

	pc = devm_kzalloc(&pdev->dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	platform_set_drvdata(pdev, pc);

	ret = lg1314_pinctrl_map(pdev, pc);
	if (ret)
		return ret;

	pctl = devm_pinctrl_register(&pdev->dev, &lg1314_pinctrl_desc, pc);
	if (IS_ERR(pctl))
		return dev_err_probe(&pdev->dev, PTR_ERR(pctl),
				     "failed to register pinctrl\n");

	return 0;
}

static const struct of_device_id lg1314_pinctrl_of_match[] = {
	{ .compatible = "lge,lg1314a0-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, lg1314_pinctrl_of_match);

static struct platform_driver lg1314_pinctrl_driver = {
	.driver = {
		.name = "lg1314-pinctrl",
		.of_match_table = lg1314_pinctrl_of_match,
		.pm = pm_sleep_ptr(&lg1314_pinctrl_pm_ops),
		.suppress_bind_attrs = true,
	},
	.probe = lg1314_pinctrl_probe,
};
builtin_platform_driver(lg1314_pinctrl_driver);
