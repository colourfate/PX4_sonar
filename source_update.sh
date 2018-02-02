#!/bin/bash
# 从共享文件夹更新更改，自动创建备份文件，目录结构请保证一致
DIR_EDIT="/home/colourfate/Share/Linux/PX4-Firmware/Firmware"
DIR_UPDATE="/home/colourfate/Workspace/PX4/Firmware"
# 需要修改的文件列表
FILES=("cmake/configs/nuttx_px4fmu-v3_default.cmake" 
	"src/modules/sonar_uart/CMakeLists.txt" 
	"src/modules/sonar_uart/sonar_uart.c"  )

# 创建备份并更新
for f in ${FILES[@]};
do
	echo ${f}
	if [ ! -f "${DIR_UPDATE}/${f}.bak" ];
	then
		echo not here
		cp "${DIR_UPDATE}/${f}" "${DIR_UPDATE}/${f}.bak"		
	fi
	cp "${DIR_EDIT}/${f}" "${DIR_UPDATE}/${f}"
done
