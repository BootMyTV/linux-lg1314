# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_LG1314_VDEC)	+= lg1314-vdec.o
lg1314-vdec-y			:= lg1314-vdec-core.o lg1314-vdec-v4l2.o lg1314-vdec-vo.o \
				   lg1314-dovi.o
lg1314-vdec-$(CONFIG_LG1314_VDEC_PASSTHROUGH) += lg1314-vdec-dev.o
obj-$(CONFIG_LG1314_ADSP)	+= lg1314-adsp.o
lg1314-adsp-y			:= lg1314-adsp-core.o lg1314-adsp-ipc.o lg1314-adsp-buf.o lg1314-adsp-pcm.o \
				   lg1314-adsp-se.o lg1314-adsp-hdmi.o
