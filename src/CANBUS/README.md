# CANBUS

## Linux CAN通信
本文旨在利用虚拟can接口实现简单的通信，为后续canbus通信学习打下基础。
![在这里插入图片描述](https://img-blog.csdnimg.cn/93d51224cd9b4e9f9fc64568616c0137.png)

1. 创建虚拟CAN接口

**加载vcan内核模块:** `sudo modprobe vcan`

**创建虚拟CAN接口:** `sudo ip link add dev vcan0 type vcan`

**将虚拟CAN接口处于在线状态:** `sudo ip link set up vcan0`

通过命令`ip addr | grep "can"` 来**验证是否可用并处于在线状态** 

2. 使用 can-utils 测试CAN通信 基于虚拟CAN接口，测试一下CAN通信情况 `sudo apt install can-utils`
3. 打开两个终端接口（一个用来查看、一个用来发送）

```markdown
#查看
candump -tz vcan0
#发送
cansend vcan0 123#00FFAA5501020304
```

#### CAN通信参考代码

**CAN数据读取**

```c++
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

int main(void)
{
	struct ifreq ifr = {0};
	struct sockaddr_can can_addr = {0};
	struct can_frame frame = {0};
	int sockfd = -1;
	int i;
	int ret;

	/* 打开套接字 */
	sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(0 > sockfd) {
		perror("socket error");
		exit(EXIT_FAILURE);
	}

	/* 指定can0设备 */
	strcpy(ifr.ifr_name, "vcan0");
	ioctl(sockfd, SIOCGIFINDEX, &ifr);
	can_addr.can_family = AF_CAN;
	can_addr.can_ifindex = ifr.ifr_ifindex;

	/* 将can0与套接字进行绑定 */
	ret = bind(sockfd, (struct sockaddr *)&can_addr, sizeof(can_addr));
	if (0 > ret) {
		perror("bind error");
		close(sockfd);
		exit(EXIT_FAILURE);
	}

	/* 设置过滤规则 */
	//setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	/* 接收数据 */
	for ( ; ; ) {
		if (0 > read(sockfd, &frame, sizeof(struct can_frame))) {
			perror("read error");
			break;
		}

		/* 校验是否接收到错误帧 */
		if (frame.can_id & CAN_ERR_FLAG) {
			printf("Error frame!\n");
			break;
		}

		/* 校验帧格式 */
		if (frame.can_id & CAN_EFF_FLAG)	//扩展帧
			printf("扩展帧 <0x%08x> ", frame.can_id & CAN_EFF_MASK);
		else		//标准帧
			printf("标准帧 <0x%03x> ", frame.can_id & CAN_SFF_MASK);

		/* 校验帧类型：数据帧还是远程帧 */
		if (frame.can_id & CAN_RTR_FLAG) {
			printf("remote request\n");
			continue;
		}

		/* 打印数据长度 */
		printf("[%d] ", frame.can_dlc);

		/* 打印数据 */
		for (i = 0; i < frame.can_dlc; i++)
			printf("%02x ", frame.data[i]);
		printf("\n");
	}

	/* 关闭套接字 */
	close(sockfd);
	exit(EXIT_SUCCESS);
}
```

**CAN数据发送**

```c++
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

int main(void)
{
	struct ifreq ifr = {0};
	struct sockaddr_can can_addr = {0};
	struct can_frame frame = {0};
	int sockfd = -1;
	int ret;

	/* 打开套接字 */
	sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(0 > sockfd) {
		perror("socket error");
		exit(EXIT_FAILURE);
	}

	/* 指定can0设备 */
	strcpy(ifr.ifr_name, "vcan0");
	ioctl(sockfd, SIOCGIFINDEX, &ifr);
	can_addr.can_family = AF_CAN;
	can_addr.can_ifindex = ifr.ifr_ifindex;

	/* 将can0与套接字进行绑定 */
	ret = bind(sockfd, (struct sockaddr *)&can_addr, sizeof(can_addr));
	if (0 > ret) {
		perror("bind error");
		close(sockfd);
		exit(EXIT_FAILURE);
	}

	/* 设置过滤规则：不接受任何报文、仅发送数据 */
	setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	/* 发送数据 */
	frame.data[0] = 0xA0;
	frame.data[1] = 0xB0;
	frame.data[2] = 0xC0;
	frame.data[3] = 0xD0;
	frame.data[4] = 0xE0;
	frame.data[5] = 0xF0;
	frame.can_dlc = 6;	//一次发送6个字节数据
	frame.can_id = 0x123;//帧ID为0x123,标准帧

	for ( ; ; ) {

		ret = write(sockfd, &frame, sizeof(frame)); //发送数据
		if(sizeof(frame) != ret) { //如果ret不等于帧长度，就说明发送失败
			perror("write error");
			goto out;
		}

		sleep(1);		//一秒钟发送一次
	}

out:
	/* 关闭套接字 */
	close(sockfd);
	exit(EXIT_SUCCESS);
}
```