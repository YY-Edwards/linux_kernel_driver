#include <stdio.h>  
#include <unistd.h>  
#include <signal.h>  
#include <fcntl.h>  
#include <errno.h>
#include <stdlib.h> 
#include <linux/ioctl.h>  


#define HIGH_LEVEL 1
#define LOW_LEVEL  0
/* 定义幻数 */
#define CD_DEV_IOC_MAGIC  'k'

/* 定义命令 *///根据ioctl-number.h确定幻数编号不能重复
#define GET_CD_GPIO_LEVEL   _IOR(CD_DEV_IOC_MAGIC, 0x1a, int)

#define CD_DEV_IOC_MAXNR 0x1A

  
int fd;
int CD_level;  
  
void sig_handler(int sig)  
{  
    int ret ;
	if(sig == SIGIO)  
    {  
        printf("Receive IO signal from kernel!\n"); 
    }else{
		printf("Receive a signal from kernel, signalnum:%d \n", sig); 
	}		
}  
  
int main(void)  
{  
    int fd;  
	int oflags = 0;

    fd = open("/dev/globalfifo", O_RDWR, S_IRUSR | S_IWUSR);  
	if(fd != -1){
		signal(SIGIO, sig_handler);
		fcntl(fd, F_SETOWN, getpid());//当前进程拥有IO信号  
		oflags = fcntl(fd, F_GETFL);
		fcntl(fd, F_SETFL, oflags | FASYNC);  //开启异步通知模式
		printf("open FASYNC mode:\n"); 
		while(1)  
		{  
			sleep(100);
			//ret = ioctl(fd, GET_CD_GPIO_LEVEL, &cd_return_value);		
		}  
			
	}else{
		printf("device open failure.\n"); 
	}

}