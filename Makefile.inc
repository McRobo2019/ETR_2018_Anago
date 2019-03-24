mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_COBJS += balancer.o balancer_param.o parameter.o

APPL_CXXOBJS += \
	ang_robo.o \
	BalancerCpp.o \
	util.o \
    ang_eye.o \
    ang_brain.o \
    motion_ctl.o 

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIR += $(mkfile_path)app $(mkfile_path)Utility $(mkfile_path)Ang_Eye $(mkfile_path)Ang_Brain $(mkfile_path)Ang_Robo

INCLUDES += \
        -I$(mkfile_path)app \
        -I$(mkfile_path)Utility \
        -I$(mkfile_path)Ang_Eye \
        -I$(mkfile_path)Ang_Brain \
        -I$(mkfile_path)Ang_Robo \

# COPTS += -fno-use-cxa-atexit
