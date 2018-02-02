// 串口超声波测距程序
/* 操作系统设备与串口映射关系
	NuttX UART	Pixhawk UART
	/dev/ttyS0	IO DEBUG(RX ONLY)
	/dev/ttyS1	TELEM1(USART2)
	/dev/ttyS2	TELEM2(USART3)
	/dev/ttyS3	GPS(UART4)
	/dev/ttyS4	N/A(UART5, IO link)
	/dev/ttyS5	SERIAL5(UART7,NSH Console Only)
	/dev/ttyS6	SERIAL4(UART8)
*/

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <math.h>
#include <float.h>

#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/manual_control_setpoint.h>


// 超声波传感器被插到TELEM2
#define SONAR_DEV "/dev/ttyS2"
#define SONAR_ERR 0xFFFF
#define AGAINST	150.0f
#define constrain(val, min, max) (val < min) ? min : ((val > max) ? max : val)

__EXPORT int sonar_uart_main(int argc, char *argv[]);

// 初始化串口，dev: 设备号，baud: 串口波特率
static int sonar_uart_init(char *dev, unsigned int baud)
{
	int uart = open(dev, O_RDWR | O_NONBLOCK | O_NOCTTY); //

	if (uart < 0) {
		printf("ERROR opening %s, aborting..\n", dev);
		return uart;
	}

	struct termios uart_config;

	int termios_state = 0;

	int ret;

	// 获取串口的配置
	if ((termios_state = tcgetattr(uart, &uart_config)) < 0) {
		printf("ERROR getting termios config for UART: %d\n", termios_state);
		ret = termios_state;
		goto cleanup;
	}

	// 修改uart_config结构体
	if (cfsetispeed(&uart_config, baud) < 0 || cfsetospeed(&uart_config, baud) < 0) {
		printf("ERROR setting termios config for UART: %d\n", termios_state);
		ret = ERROR;
		goto cleanup;
	}
	// 使用uart_config配置波特率
	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		printf("ERROR setting termios config for UART\n");
		ret = termios_state;
		goto cleanup;
	} 

	printf("%s opened, baud rate: %d\n", dev, baud);

	return uart;
cleanup:
	close(uart);
	return ret;

}
static uint16_t dist;

// 这是一个线程函数，规定格式为 void *fun(void *)
static void *sonar_read_loop(void *arg)
{
	int sonar_dev = *((int *)arg);
	uint8_t start = 0x55;
	uint8_t data[2];
	int i;

	// 每写一次0x55，传感器测一次距离
	for(i=0; i<100; i++){
		write(sonar_dev, &start, 1);
		usleep(50000);
		read(sonar_dev, data, 2);
		dist = data[0]<<8 | data[1];
		if(dist < 20 || dist > 4500){	// 超出此范围表示错误
			dist = SONAR_ERR;
		}
		//printf("distance: %d\n", dist);
		usleep(50000);
	}
	
	close(sonar_dev);

	return NULL;
}




// 使用串口发送数据，注意，编译成一个模块的话，入口函数名必须为: 文件名_main
int sonar_uart_main(int argc, char *argv[])
{
	// 无需检查错误，函数中已经检查
	int sonar_dev = sonar_uart_init(SONAR_DEV, B9600);
	struct manual_control_setpoint_s manual = {};

	// 开始读取传感器
	pthread_t receive_thread;
	pthread_create(&receive_thread, NULL, sonar_read_loop, &sonar_dev);

	printf("thread start\n");
	//等待线程终止
	//pthread_join(receive_thread, NULL);

	// 订阅手动控制设定点消息，更新频率5Hz
	int man_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	orb_set_interval(man_sp_sub, 100);
	px4_pollfd_struct_t fd = { 
		.fd = man_sp_sub,
		.events = POLLIN
	};
	
	for(int i=0; i<100; i++){
		// 等待消息到达
		int poll_ret = px4_poll(&fd, 1, 1000);
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");
		}else if(poll_ret > 0){
			orb_copy(ORB_ID(manual_control_setpoint), man_sp_sub, &manual);
			// 根据测量的距离对遥控器输入的数据进行处理
			// 这里没有考虑飞机的速度，只是在缓慢靠近的情况下有效
			if(dist == SONAR_ERR){
				continue;
			}
			manual.x -= AGAINST/dist;
			manual.x = constrain(manual.x, -1.f, 1.f);
		}
		
		PX4_INFO("\t%dmm\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
					 dist, (double)manual.x, (double)manual.y, 
					 (double)manual.z, (double)manual.r);
	}

	printf("thread terminate\n");

	return 0;
}




