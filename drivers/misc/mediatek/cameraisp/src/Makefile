#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

#include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y := cameraisp_dummy.o

obj-$(CONFIG_ARCH_MT6580) += mt6580/
obj-$(CONFIG_ARCH_MT6735M) += mt6735m/
obj-$(CONFIG_ARCH_MT6735) += mt6735/
obj-$(CONFIG_ARCH_MT6755) += mt6755/
obj-$(CONFIG_ARCH_MT6753) += mt6753/
#obj-$(CONFIG_ARCH_MT6795) += mt6795/
obj-$(CONFIG_ARCH_MT8127) += mt6797/
obj-$(CONFIG_ARCH_MT8163) += mt8163/
obj-$(CONFIG_ARCH_MT8173) += mt8173/
obj-$(CONFIG_ARCH_MT6797) += mt6797/

#this moment do not have other folder
#obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
