## This cmakelist.txt file was generated from
## the Embedded Coder Support Package for PX4 Autopilots

add_definitions(
-DMODEL=TestConnExample -DNUMST=1 -DNCSTATES=0 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0 -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTID01EQ=0 -DTERMFCN=1 -DONESTEPFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0 -DPX4 -DFMUvx -DNULL=0 -DEXTMODE_DISABLETESTING -DEXTMODE_DISABLEPRINTF -DEXTMODE_DISABLE_ARGS_PROCESSING=1 -DDAEMON_MODE=1 -DSTACK_SIZE=64 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DRT )


include_directories(
"/usr/local/MATLAB/R2018b/extern/include"
"/usr/local/MATLAB/R2018b/simulink/include"
"/usr/local/MATLAB/R2018b/rtw/c/src"
"/usr/local/MATLAB/R2018b/rtw/c/src/ext_mode/common"
"/usr/local/MATLAB/R2018b/rtw/c/ert"
"/usr/local/MATLAB/R2018b/rtw/c/src/ext_mode/serial"
"/home/nighthawk/Documents/MATLAB/SupportPackages/R2018b/toolbox/target/supportpackages/px4/include"
"/home/nighthawk/Documents/MATLAB/SupportPackages/R2018b/toolbox/target/supportpackages/px4/lib/Customize_Setup"
"/home/nighthawk/Documents/MATLAB/SupportPackages/R2018b/toolbox/target/shared/svd/include"
)

px4_add_module(
MODULE modules__px4_simulink_app
MAIN px4_simulink_app
STACK_MAIN 2000
SRCS
MW_SCI.c
MW_uORB_Read.cpp
ert_main.cpp
TestConnExample.cpp
TestConnExample_data.cpp
MW_PX4_TaskControl.cpp
nuttxinitialize.cpp
DEPENDS
platforms__common
)

string (REPLACE "-Werror" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
string (REPLACE "-Werror" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -Wno-narrowing")

