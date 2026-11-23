# SPDX-License-Identifier: GPL-2.0
lg1314-drm-y := lg1314_drm.o lg1314_ovi.o lg1314_de.o lg1314_panel.o lg1314_osd.o lg1314_video.o lg1314_hdr.o lg1314_dovi.o lg1314_dovi_dm.o \
	lg1314_status.o

# Dolby Vision display management is floating point
CFLAGS_lg1314_dovi_dm.o += $(CC_FLAGS_FPU) -fno-math-errno
CFLAGS_REMOVE_lg1314_dovi_dm.o += $(CC_FLAGS_NO_FPU)

obj-$(CONFIG_DRM_LG1314) += lg1314-drm.o
