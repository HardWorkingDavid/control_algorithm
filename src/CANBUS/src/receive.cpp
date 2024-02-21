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

#define CAN_SFF_MASK 0x000007FFU /* 标准帧格式 (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* 扩展帧格式 (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* 忽略EFF, RTR, ERR标志 */

int main()
{
    int s,nbytes1,nbytes2,nbytes3,nbytes4;
    srand(time(nullptr));//设置随机数种子
    rand();//产生一个随机数
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame1 = {0};
    struct can_frame frame2 = {0};
    struct can_frame frame3 = {0};
    struct can_frame frame4 = {0};
    struct can_filter rfilter[4];
    int count = 0; //故障保护
    s = socket(PF_CAN,SOCK_RAW,CAN_RAW); //创建套接字
    strcpy(ifr.ifr_name,"vcan0");
    ioctl(s,SIOCGIFINDEX,&ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s,(struct sockaddr *)&addr,sizeof(addr));
    //定义接收规则
    rfilter[0].can_id = CAN_EFF_FLAG | 0x18FEF121;
    rfilter[0].can_mask = CAN_EFF_MASK;

    rfilter[1].can_id = CAN_EFF_FLAG | 0x18FF0824;
    rfilter[1].can_mask = CAN_EFF_MASK;

    rfilter[2].can_id = CAN_EFF_FLAG | 0x0C02A224;
    rfilter[2].can_mask = CAN_EFF_MASK;

    rfilter[3].can_id = CAN_EFF_FLAG | 0x0c02a0a2;
    rfilter[3].can_mask = CAN_EFF_MASK;
    //设置过滤规则
    setsockopt(s,SOL_CAN_RAW,CAN_RAW_FILTER,&rfilter,sizeof(rfilter));

    while(1)
    {
        double factor = 1/256.0;//车速的分辨率
        int Offset = 0;//车速的偏移量
        int num,num1;
        double zfactor = 0.1; //方向盘转向指令角度的分辨率
        int zOffset = -1575; // 方向盘转向指令角度的偏移量
        double nfactor = 0.1; //转向叠加扭矩信号的分辨率
        int nOffset = 0; //转向叠加扭矩信号的偏移量
        double ffactor = 10;//方向盘目标角速度的分辨率
        int fOffset = 0; //方向盘目标角速度的偏移量
        double dfactor = 0.000152;
        int dOffset = -5;
        nbytes1 = read(s,&frame1,sizeof(frame1));  //接收报文
        nbytes2 = read(s,&frame2,sizeof(frame2));  //接收报文
        
        if (nbytes1 > 0)
        {
            printf("ID = 0x%X   dlc = %d  data = 0x ",frame1.can_id,frame1.can_dlc);
            for (int i = 0; i < frame1.can_dlc; i++)printf("%02x ",frame1.data[i]);
            num = (frame1.data[2]<<8) | (frame1.data[1]); //将两个十六进制数字合并
            printf("v = %.2f km/h",factor*num+Offset); // 十六进制转为十进制输出
            printf("\n");
        }
        if (nbytes2 > 0)
        {
            printf("ID = 0x%X   dlc = %d  data = 0x ",frame2.can_id,frame2.can_dlc);
            for (int i = 0; i < frame2.can_dlc; i++)printf("%02x ",frame2.data[i]);
            printf("\n");
            if (frame2.data[4] == 0x01)
            {
                printf("Stop!\n");
            }
            else if (frame2.data[4] == 0x02){
                printf("Ready\n");
            }
            else if (frame2.data[4] == 0x03){
                nbytes3 = read(s,&frame3,sizeof(frame3));  //接收报文
                if (nbytes3 >0)
                {
                    count++;
                    if (count >5) //五个周期故障保护
                    {
                        if (frame3.data[7]>0xFA) //数据大于250
                        {
                            printf("此报文不可靠\n");
                        }
                        else printf("此报文可靠\n");
                    }
                    printf("ID = 0x%X   dlc = %d  data = 0x ",frame3.can_id,frame3.can_dlc);
                    for (int i = 0; i < frame3.can_dlc; i++)printf("%02x ",frame3.data[i]);
                    printf("\nGo转向可以启动!\n");//转向可以启动
                    num1 = (frame3.data[1]<<8) | (frame3.data[0]); //将两个十六进制数字合并
                    printf("方向盘转向指令角度 = %.1f度\n",num1*zfactor+zOffset); 
                    if (frame3.data[2] == 0x34)printf("收到EPS助力模式指令,进入EPS模式\n");
                    if (frame3.data[2] == 0x31)printf("收到自动驾驶模式指令,进入自动驾驶模式\n");

                    printf("转向叠加扭矩信号 =  %.1f  N.m\n",nfactor*frame3.data[3]+nOffset);
                    printf("方向盘目标角速度 =  %.1f  度/s\n",ffactor*frame3.data[4]+fOffset);
                    printf("\n\n");

                    nbytes4 = read(s,&frame4,sizeof(frame4));  //接收报文
                    if (nbytes4>0)
                    {
                        printf("ID = 0x%X   dlc = %d  data = 0x ",frame4.can_id,frame4.can_dlc);
                        for (int i = 0; i < frame4.can_dlc; i++)printf("%02x ",frame4.data[i]);
                        if (frame4.data[6] == 0x04)
                        {
                            printf("处于EPS模式状态\n");
                        }
                        else if (frame4.data[6] == 0x01)
                        {
                            printf("处于自动驾驶模式状态\n");
                        }
                        num1 = (frame4.data[1]<<8) | (frame4.data[0]); //将两个十六进制数字合并
                        printf("方向盘转向角度 = %.1f度\n",num1*zfactor+zOffset); 
                        num1 = (frame4.data[3]<<8) | (frame4.data[2]); //将两个十六进制数字合并
                        printf("电机助力扭矩 = %.2f Nm\n",num1*dfactor+dOffset);
                        printf("方向盘反馈角速度 = %.1f 度/s \n",ffactor*frame4.data[4]+fOffset);
                        printf("转向盘转矩信号 = %.1f Nm\n",nfactor*frame4.data[5]+nOffset);
                        
                        printf("\n");
                    }
                }
                
            }
                else if (frame2.data[4] == 0x04){
                    printf("远程升级\n");
                }
                else if (frame2.data[4] == 0x06){
                    printf("跛行\n");
                }
                else{
                    printf("失效\n");
                    }
            }
        }   
        close(s);
        return 0;
    }
    
