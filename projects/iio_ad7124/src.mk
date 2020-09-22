#See No-OS/tool/scripts/src_model.mk for variable description

SRC_DIRS += $(PROJECT)/src

# Add to SRCS source files to be build in the project
SRCS += $(NO-OS)/drivers/adc/ad7124/ad7124.c

# Add to INCS inlcude files to be build in the porject
INCS += $(NO-OS)/drivers/adc/ad7124/ad7124.h

SRC_DIRS += $(NO-OS)/network
SRC_DIRS += $(PLATFORM_DRIVERS)
SRC_DIRS += $(NO-OS)/util
SRC_DIRS += $(INCLUDE)

LIBRARIES += iio

ENABLE_IIO_NETWORK = n

