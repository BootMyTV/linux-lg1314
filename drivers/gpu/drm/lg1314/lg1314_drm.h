/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DRM/KMS driver for the LG1314 display pipeline (OSD -> DE -> OVI -> VX1 ->
 * WRGB OLED) in the LG OLED55B9PLA.
 */

#ifndef _LG1314_DRM_H_
#define _LG1314_DRM_H_

#include <linux/bits.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>
#include <drm/drm_plane.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

struct drm_framebuffer;
struct drm_pending_vblank_event;
struct lg1314_vdec;
struct lg1314_vdec_vo;

/* MMIO regions (the bring-up tables use absolute phys addresses). */
enum lg1314_region {
	LG1314_REG_OSD,		/* 0xc9010000 : OSD top/ctrl/afbc/shp         */
	LG1314_REG_OVI,		/* 0xc9070000 : TCON / OIF / VX1              */
	LG1314_REG_CTOP,	/* 0xc9300000 : DPE / C4TX link / disp clocks */
	LG1314_REG_DE_IPC,	/* 0xc9026000 : DE IPC / firmware download    */
	LG1314_REG_DPE,		/* 0xc9020000 : DPE path enable              */
	LG1314_REG_PLL,		/* 0xc9c10000 : DISP PLL                      */
	LG1314_REG_IMX,		/* 0xc9014000 : DE input mux / sync pulses    */
	LG1314_REG_COUNT
};

struct lg1314_mmio {
	u32		phys;
	u32		size;
	void __iomem   *virt;
};

/* OSD graphics plane. osdx_ctrl[0] (OSD0) at bank + 0x200; within a block
 * osd_ctrl_main @ +0x00, osd_ctrl_hdr0..6 @ +0x20..+0x38, osd_ctrl_sync0 @ +0x40.
 */
#define OSD_BANK_PHYS			0xc9010100
#define OSD_TOP_PHYS			0xc9010000
#define OSD0_CTRL_BASE			0x200		/* osdx_ctrl[0] */
#define OSD_CTRL_MAIN			0x00		/* osd_en = BIT(0), ready = BIT(29) */
#define OSD_CTRL_SCALER			0x10
#define OSD_SCALER_BUF_MODE_EN_N	BIT(6)		/* reg_osd_buf_mode_en_n */
#define OSD_CTRL_HDR0			0x20		/* 7 consecutive u32: hdr0..hdr6 */
#define OSD_CTRL_SYNC0			0x40		/* osd_sync_enable = BIT(0) */
#define OSD_EN				BIT(0)
#define OSD_BITMAP_ONLY			BIT(6)		/* osd_bitmap_only_mode_en */
#define OSD_SYNC_EN			BIT(0)
#define OSD_HDR_READY			BIT(29)		/* osd_hdr_ready_flag */
#define OSD_BASE_ADDR			0x04
#define OSD_CTRL_SWAP			0x08
#define OSD_CTRL_PDEC			0x0c
#define OSD_PDEC_CH_MASK		GENMASK(31, 24)	/* osd_ch_{alpha,red,green,blue} */

/*
 * AFBC scanout (OSD0 only). Values are the ones stock webOS runs with (live
 * register dump), which match the GPL FBDEV_M19_CMN_SetupFBC/WriteFBCHdrRegs.
 * Linear values are what lxboot leaves and the GPL raw-mode defaults.
 */
#define OSD_TOP_CTRL_DPATH		0x20
#define OSD_DPATH_AFBC_EN		BIT(20)		/* osd0_afbc_en */
#define OSD_TOP_AUTO_INIT_AFBC		0x38
#define OSD_AUTO_INIT_AFBC_ON		0x10000011	/* afbc0 init + auto init */
#define OSD_SWAP_LINEAR			0x00000333
#define OSD_SWAP_AFBC			0x00000000
#define OSD_PDEC_CH_LINEAR		0x1b000000	/* a0 r1 g2 b3 */
#define OSD_PDEC_CH_AFBC		0xe4000000	/* a3 r2 g1 b0 */

#define OSD_AFBC_PHYS			0xc9010700
#define AFBC_CTRL0			0x00
#define AFBC_CTRL0_CFG			0x420c0000	/* run_mode, pl_addr_mode,
							 * update_ready_flag_en, wrap_en */
#define AFBC_CTRL0_UPDATE		BIT(26)		/* update_ready_flag */
#define AFBC_HD_BASE			0x04
#define AFBC_START			0x08		/* start_x | start_y << 16 */
#define AFBC_END			0x0c		/* end_x | end_y << 16 */
#define AFBC_IMG_WIDTH			0x10
#define AFBC_BUF_DEPTH			0x14
#define AFBC_BUF_DEPTH_VAL		0x03bf00ff	/* dec 959, pl 255 (HW fixed) */
#define AFBC_PL_BASE			0x20
#define AFBC_PL_END			0x24
#define AFBC_CTRL_C			0x30
#define AFBC_CTRL_C_VAL			0x0011ff54	/* endian/gmau ids, "HW test value" */
#define AFBC_DEFAULT			0xd0
#define AFBC_DEFAULT_VAL		0xffffffff

/* OSD mixer vsync interrupt (GIC SPI 72): fires every panel refresh. */
#define OSD_IRQ_PHYS			0xc9010144	/* osd_common_ctrl_irq */
#define OSD_IRQ_BOSD_EN			BIT(24)		/* blend-OSD mixer */
#define OSD_IRQ_BOSD_PENDING		BIT(8)		/* write 1 to clear */
#define OSD_IRQ_PENDING_MASK		GENMASK(15, 8)

/*
 * DE sync pulse i5, set to follow the video source (the decoder output's
 * tick): its status bit marks the panel refreshes where a video frame can
 * change. Routed to the CPU side without raising an interrupt.
 */
#define DE_IPC_CPU_FLAG			0x68	/* DE -> CPU interrupt sources */
#define DE_IPC_CPU_FLAG_EN		0x6c
#define DE_IPC_CPU_FLAG_IMX		BIT(1)
#define IMX_PHYS			0xc9014000
#define IMX_CTRL_INTR_PULSE		0x0c	/* per pulse: idx[2:0], sel[3] */
#define IMX_INTR_REG			0x10	/* status, write 1 to clear */
#define IMX_INTR_MASK			0x14
#define IMX_INTR_MUX			0x18	/* 1: to the CPU side */
#define IMX_PULSE_I5			BIT(5)
#define IMX_PULSE_I5_CFG		GENMASK(23, 20)
#define IMX_PULSE_SRC0			0	/* source, index 0 */

/* osd0 register (bank + OSD0_CTRL_BASE + field) */
#define OSD0(field)			(OSD0_CTRL_BASE + (field))

/* OSD HW init (FBDEV_M19_CMN_InitHW): geometry the plane header relies on.
 * disp-size fields: w [16:27/28], h [0:11]. Offsets are family-documented in the
 * GPL osd_reg_l18.h and confirmed against the stock M19 image.
 */
#define OSD_COMMON_DISP_SIZE0		0x50	/* osdx_common_disp_size[0..7], +i*4 */
#define OSD_SOSD_DISP_SIZE		0x80
#define OSD_BOSD0_DISP_SIZE		0x100
#define OSD_BOSD1_DISP_SIZE		0x180
#define OSD_SOSD_MUX1			0xa4
#define OSD_BOSD0_MUX1			0x124
#define OSD_BOSD1_MUX1			0x1a4
#define OSD_MIXER_MUX_ORDER		0x76543210	/* fixed on M19 */
#define OSD_DISP_SIZE_VAL		((LG1314_PANEL_W << 16) | LG1314_PANEL_H)
#define OSD_DISP_SIZE_MASK		0x1fff0fff
#define OSD_TOP_SOSD_PULSE		0x34		/* OSD TOP: sync pulse delay */
#define OSD_TOP_BOSD_PULSE		0x30
#define OSD_PULSE_TIMER			0x3000
#define OSD_PULSE_MASK			0x0fffffff
/* OSD top init writes (FBDEV_M19_InitHW) */
#define OSD_TOP_INIT_A_PHYS		0xc9010c20	/* = 0x000100fa */
#define OSD_TOP_INIT_A_VAL		0x000100fa
#define OSD_TOP_INIT_B_PHYS		0xc9010c60	/* = 0x400000fb */
#define OSD_TOP_INIT_B_VAL		0x400000fb

#define OSD_PXLFMT_ARGB8888		0xe
#define OSD_PXLDEPTH_32BPP		0x6
#define OSD_GLOBAL_ALPHA_EN		BIT(31)	/* hdr3: use global alpha, ignore per-pixel */

/* OSD SR scaler: 2x upscale of the 1920x1080 graphics plane to the 3840x2160
 * panel (M19 "video path is fixed to UHD display"). Enable follows the GPL
 * SetupSRScaler semantics; the M19 osd_bitmap_ctrl0 bit positions (absent from
 * the M19 GPL headers) are taken from the stock image.
 */
#define OSD_SHP_PHYS			0xc9011c00
#define SHP_CORE_CTRL_00		0x00	/* reg_top_bypass = BIT(5) */
#define SHP_CORE_CTRL_01		0x04	/* reg_top_width[0:11], reg_top_height[16:27] */
#define SHP_TOP_BYPASS			BIT(5)
#define SHP_SIZE_VAL			((LG1314_PANEL_H << 16) | LG1314_PANEL_W)
#define OSD_BITMAP_CTRL0		0x258	/* OSD bank; byte at +0x25b */
#define OSD_SHP_DETOUR_EN		BIT(28)
#define OSD_SR_CSC_EN			(BIT(30) | BIT(31))

/* OVI bring-up registers (preserved values are read back from lxboot state). */
#define LG1314_OVI_DPE_PATH_REG		0xc902001c
#define LG1314_OVI_DISP_PLL_REG		0xc9c10438
#define LG1314_OVI_SS_REG1		0xc930a084
#define LG1314_OVI_SS_REG2		0xc930a088
#define LG1314_OVI_CLKGATE_REG		0xc930a800	/* crg_dpe00 */
#define LG1314_OVI_CHPOWER_REG		0xc930a0a8	/* ctop_dpe_r42 */
#define LG1314_OVI_CHPOWER_OFF_MASK	0xff000000
#define LG1314_OVI_RESET_REL_REG	0xc930a808	/* crg_dpe02 */

/* DE firmware IPC block. */
#define DE_IPC_PHYS			0xc9026000
#define DE_IPC_SIZE			0x418
#define DE_IPC_CTRL			0x00	/* bit0 hold, bit1 aux */
#define DE_IPC_ADR5_LO			0x28	/* DDR base of the firmware image */
#define DE_IPC_ADR5_HI			0x2c
#define DE_IPC_CTRL_HOLD		BIT(0)
#define DE_IPC_CTRL_B1			BIT(1)
#define DE_IPC_EXT_STATUS		0x10	/* firmware -> host interrupts */
#define DE_IPC_EXT_CLEAR		0x14
#define DE_IPC_INT_EVENT		0x18	/* host -> firmware interrupts */
#define DE_IPC_MBOX_IRQ			BIT(21)	/* mailbox doorbell, both directions */
#define DE_IPC_TAG			0x100	/* memory layout table, read by the firmware */
#define DE_IPC_MBOX_TX			0x100	/* host -> firmware mailbox */
#define DE_IPC_MBOX_RX			0x180	/* firmware -> host mailbox */
#define DE_IPC_MBOX_SIZE		0x80
#define DE_FW_DDR_MIN			0x50000000u	/* ADR5 must be >= this */
#define DE_FW_NAME			"lg1314/de_fw.bin"

/* Panel output — VX1, 3840x2160, LG Display WRGB OLED. */
#define LG1314_PANEL_W			3840
#define LG1314_PANEL_H			2160

/* Graphics plane (framebuffer) — SR-scaled to the 3840x2160 panel. Exposed
 * input sizes range from 1280x720 to 1920x1080.
 */
#define LG1314_FB_W			1920
#define LG1314_FB_H			1080
#define LG1314_FB_MIN_W			1280
#define LG1314_FB_MIN_H			720
/* Smallest video-plane picture (and so the device's framebuffer minimum). */
#define LG1314_VIDEO_MIN_W		64
#define LG1314_VIDEO_MIN_H		64

/* A display-engine window rectangle. */
struct lg1314_de_rect {
	u16 w, h, x, y;
};

/* The video plane (lg1314_video.c). */
struct lg1314_video {
	struct drm_plane		plane;
	struct device_node		*vdec_np;
	struct lg1314_vdec		*vdec;	/* once available */
	struct lg1314_vdec_vo		*vo;	/* while the plane is on */

	u32				next_id;
	struct {
		u32			id;
		struct drm_framebuffer	*fb;
	}				held[8];	/* not yet off screen */
	struct mutex			held_lock;

	/* output tick rate in mHz, calibrated so the tick is 60 Hz */
	unsigned int			rate_mhz;
	struct delayed_work		calib_work;
	unsigned int			calib_passes;

	/* display-engine window state */
	bool				src_set;
	bool				blanked;
	struct lg1314_de_rect		in, out;
};

struct lg1314_drm {
	struct drm_device		drm;
	struct device			*dev;
	struct lg1314_mmio		mmio[LG1314_REG_COUNT];

	struct drm_crtc			crtc;
	struct drm_plane		primary;	/* the OSD */
	struct drm_encoder		encoder;
	struct drm_connector		connector;

	/* vblank: every other OSD mixer vsync (panel refresh is twice the mode's) */
	int				irq;
	unsigned int			vsync_count;
	bool				video_phase;	/* vblank at video ticks */
	/* video tick timing, for calibrating the output's rate */
	unsigned int			tick_n;
	u64				tick_first_ns, tick_last_ns;

	/* panel power sequencing */
	struct i2c_client		*micom;

	/* DE firmware image, and the memory layout handed to it. */
	phys_addr_t			de_fw_phys;
	size_t				de_fw_size;
	struct lg1314_de_mem {
		struct resource		fw_m0;	/* firmware shared/working memory */
		struct resource		prew;	/* pre-write + FRC buffers */
		struct resource		hdr;
		struct resource		frm;	/* frame buffers */
		struct resource		vt;	/* VTG buffers */
		struct resource		be;	/* back end */
		struct resource		vdec_dpb;
		struct resource		scanout;
		u32			model;
	} de_mem;

	bool				de_fw_loaded;

	struct lg1314_video		video;
	u32				video_mem_start;	/* pictures the DE */
	u32				video_mem_end;		/* can read */

	/* DE firmware command mailbox */
	struct mutex			de_lock;

	/* channel-power register value preserved from lxboot at probe */
	u32				chpower_save;
	bool				enabled;
};

#define to_lg1314(ddev)	container_of(ddev, struct lg1314_drm, drm)

/* MMIO helpers: resolve an absolute phys addr to the mapped span and access it. */
void __iomem *lg1314_map(struct lg1314_drm *lg, u32 phys);
void lg1314_wr(struct lg1314_drm *lg, u32 phys, u32 val);
u32  lg1314_rd(struct lg1314_drm *lg, u32 phys);

/* subsystem entry points */
int  lg1314_ovi_enable(struct lg1314_drm *lg);
void lg1314_ovi_disable(struct lg1314_drm *lg);
int  lg1314_de_load_fw(struct lg1314_drm *lg);
int  lg1314_de_cmd(struct lg1314_drm *lg, u32 cmd, const void *data, size_t len);
int  lg1314_de_query(struct lg1314_drm *lg, u32 cmd, void *out, size_t len);
int  lg1314_de_wait_ready(struct lg1314_drm *lg);
int  lg1314_de_frame_lock(struct lg1314_drm *lg);
void lg1314_de_debugfs_init(struct lg1314_drm *lg);
int  lg1314_video_init(struct lg1314_drm *lg);
void lg1314_video_fini(struct lg1314_drm *lg);
int  lg1314_panel_vcc(struct lg1314_drm *lg, bool on);
int  lg1314_panel_evdd(struct lg1314_drm *lg, bool on);
void lg1314_osd_init(struct lg1314_drm *lg);
void lg1314_osd_enable(struct lg1314_drm *lg, bool on);
void lg1314_osd_layer(struct lg1314_drm *lg, bool on);
void lg1314_osd_flip(struct lg1314_drm *lg, u32 fb_phys, u32 pitch,
		     u32 w, u32 h, bool opaque, bool afbc);
void lg1314_osd_vsync_irq(struct lg1314_drm *lg, bool on);
bool lg1314_osd_vsync_ack(struct lg1314_drm *lg);
void lg1314_vsync_video_phase(struct lg1314_drm *lg, bool on);

#endif /* _LG1314_DRM_H_ */
