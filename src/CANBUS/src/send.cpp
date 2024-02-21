//导入头文件
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <time.h> //生成随机数用
#include <iostream>

#define CAN_EFF_FLAG 0x80000000U  // 扩展帧的标识
#define CAN_RTR_FLAG 0x40000000U // 远程帧的标识
#define CAN_ERR_FLAG 0x20000000U // 错误帧的标识，用于错误检查 

int main()
{
    int s,nbytes1,nbytes2,nbytes3,nbytes4;
    int num;
    srand(time(nullptr));//设置随机数种子
    rand();//产生一个随机数
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame1 = {0};
    struct can_frame frame2 = {0};
    struct can_frame frame3 = {0};
    struct can_frame frame4 = {0};

    s = socket(PF_CAN,SOCK_RAW,CAN_RAW); // 创建套接字
    strcpy(ifr.ifr_name,"vcan0");
    ioctl(s,SIOCGIFINDEX,&ifr);   // 指定can0设备
    addr.can_family = AF_CAN;
    addr.can_ifindex  = ifr.ifr_ifindex;
    bind(s,(struct sockaddr *)&addr,sizeof(addr)); //将套接字与can0设备绑定

    //setsockopt(s,SOL_CAN_RAW,CAN_RAW_FILTER,NULL,0); // 禁用过滤规则
    //生成报文
    frame1.can_id = CAN_EFF_FLAG | 0x18FEF121;// ID为扩展帧
    //车速信号
    frame1.can_dlc = 8;
    frame1.data[0] = 0x00;
    frame1.data[3] = 0x00;
    frame1.data[4] = 0x00;
    frame1.data[5] = 0x00;
    frame1.data[6] = 0x00;
    frame1.data[7] = 0x00;

    frame2.can_id = CAN_EFF_FLAG | 0x18FF0824;// ID为扩展帧
    //发动机转速/电动泵信号
    frame2.can_dlc = 8;
    frame2.data[0] = 0x00;
    frame2.data[1] = 0x00;
    frame2.data[2] = 0x00;
    frame2.data[3] = 0x00;
    frame2.data[5] = 0x00;
    frame2.data[6] = 0x00;
    frame2.data[7] = 0x00;

    frame3.can_id = CAN_EFF_FLAG | 0x0C02A224;// ID为扩展帧
    //发动机转速/电动泵信号
    frame3.can_dlc = 8;
    frame3.data[5] = 0x80;
    frame3.data[6] = 0x80;

    // 转向控制器发给整车控制
    frame4.can_id = CAN_EFF_FLAG | 0x0c02a0a2;// ID为扩展帧
    frame4.can_dlc = 8;
    
    frame4.data[2] = rand() % 0xFF;
    frame4.data[3] = rand() % 0xFF;
    
    while(1)
    {
        frame1.data[1] = rand() % 0xFAFF;
        frame1.data[2] = rand() % 0xFAFF;
        if ((frame1.data[1]+frame1.data[2])>0xFAFF)break;//限制范围在0-250.996km/h
        nbytes1 = write(s,&frame1,sizeof(frame1)); // 发送frame1
        if (nbytes1 != sizeof(frame1))
        {
            printf("Send Error frame1\n");
            break; //发送错误，跳出循环
        }
        sleep(0.1);  //发送周期0.1s

        frame2.data[4] = rand() % 0x07; // 0-0x07
        nbytes2 = write(s,&frame2,sizeof(frame2)); // 发送frame2
        if (nbytes2 != sizeof(frame2))
        {
            printf("Send Error frame2\n");
            break; //发送错误，跳出循环
        }
        sleep(0.1);  //发送周期0.1s
        frame3.data[0] = rand() % 0xf4;
        frame3.data[1] = rand() % 0x7e;//限制范围在0-32500
        frame3.data[3] = rand() % 0x100; // 0-256
        frame3.data[4] = 0x14 + rand() % 0xa; //200-300
        frame3.data[7] = rand() % 0xFA;
        if (frame2.data[4] == 0x03)
        {
            while(1)
            {
                frame3.data[2] = 0x34; // 默认进入手动指令(EPS模式状态4)
                int a;
                if (std::cin >> a,a) //如需进入自动驾驶模式，输入1，否则输入0
                {
                    frame3.data[2] = 0x31;
                } 
                if (std::cin >> a,a) //如需再次进入自动驾驶模式输入5介入恢复指令，否则输入0
                {
                    frame3.data[2] = 0x35;
                    continue;
                }
                break;
            }

            nbytes3 = write(s,&frame3,sizeof(frame3)); // 发送frame3
            if (nbytes3 != sizeof(frame3))
            {
                printf("Send Error frame3\n");
                break; //发送错误，跳出循环
            }
            sleep(0.02);  //发送周期20ms
            frame4.data[0] = frame3.data[0];
            frame4.data[1] = frame3.data[1];
            frame4.data[4] = frame3.data[4];
            frame4.data[5] = frame3.data[3];
            frame4.data[7] = frame3.data[7];
            if (frame3.data[2] == 0x34)
            {
                frame4.data[6] = 0x04; //EPS助力模式
            }
            else if (frame3.data[2] == 0x31)
            {
                frame4.data[6] = 0x01; //自动驾驶模式
            }
            else
            {
                frame4.data[6] = 0x05; // 发送手动介入指令
            }
            nbytes4 = write(s,&frame4,sizeof(frame4)); // 发送frame4
            if (nbytes4 != sizeof(frame4))
            {
                printf("Send Error frame4\n");
                break; //发送错误，跳出循环
            }
            sleep(0.02);  //发送周期20ms
        }
    }

    close(s);
    return 0;
    
}
