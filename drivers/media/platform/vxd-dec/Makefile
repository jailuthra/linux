# SPDX-License-Identifier: GPL-2.0

#VXD
vxd-dec-objs += vxd_core.o

#PVDEC
vxd-dec-objs += vxd_pvdec.o

#MEM_MGR
vxd-dec-objs += img_mem_man.o img_mem_unified.o
vxd-dec-objs += imgmmu.o

#Utilities
vxd-dec-objs += lst.o dq.o
vxd-dec-objs += resource.o
vxd-dec-objs += dec_resources.o
vxd-dec-objs += rman_api.o
vxd-dec-objs += pixel_api.o
vxd-dec-objs += vdecdd_utils_buf.o
vxd-dec-objs += vdecdd_utils.o

#MMU
vxd-dec-objs += talmmu_api.o
vxd-dec-objs += pool.o
vxd-dec-objs += hash.o
vxd-dec-objs += ra.o
vxd-dec-objs += addr_alloc.o
vxd-dec-objs += vdec_mmu_wrapper.o

#DECODER
vxd-dec-objs += pool_api.o idgen_api.o
vxd-dec-objs += hw_control.o
vxd-dec-objs += vxd_int.o
vxd-dec-objs += translation_api.o
vxd-dec-objs += decoder.o
vxd-dec-objs += core.o

#BSPP
vxd-dec-objs += swsr.o
vxd-dec-objs += h264_secure_parser.o
vxd-dec-objs += bspp.o
vxd-dec-objs += hevc_secure_parser.o

#UM INTERFACE & SYSDEV
vxd-dec-objs += vxd_dec.o

obj-$(CONFIG_VIDEO_IMG_VXD_DEC) += vxd-dec.o

ifeq ($(CONFIG_HAS_HEVC),y)
KBUILD_CFLAGS   += -DHAS_HEVC
endif

ifeq ($(REDUCED_DPB_NO_PIC_REORDERING),y)
KBUILD_CFLAGS   += -DREDUCED_DPB_NO_PIC_REORDERING
endif
