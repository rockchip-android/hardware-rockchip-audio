#ifndef ANDROID_AUDIO_USBAUDIO_HARDWARE_H
#define ANDROID_AUDIO_USBAUDIO_HARDWARE_H
 
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/resource.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <utils/Log.h>
#include <utils/String8.h>

#define UA_Path "/proc/asound/card2/stream0"
#define UA_Record_SampleRate 48000
#define UA_Playback_SampleRate 48000
#define RETRY_TIMES 10
#define RETRY_SLEEPTIME 300*1000

bool   has_usbaudio_speaker_mic(const char *type){
	int fd;
	char buf[2048]={0};
	char *str,tmp[6]={0};
	for(int i=0;i<RETRY_TIMES;i++)
	{
		fd = open(UA_Path,O_RDONLY);
		if(fd<0)
		{
			ALOGD("can not open /proc/asound/card2/stream0,try time =%d",i+1);
			usleep(RETRY_SLEEPTIME);
			continue;
		}
		break;
	}
	if(fd<0){
		ALOGV("cant open /proc/asound/card2/stream0,giveup");
		return false;
	}else{
		read(fd,buf,sizeof(buf));
		str = strstr(buf,type);
		close(fd);
		if(str!=NULL){
			return true;
		}else{
			return false;
		}
	}
}

uint32_t get_usbaudio_cap(const char *type,const char *param){//support like this: Rates: 8000, 16000, 24000, 32000, 44100, 48000  or  Rates: 48000
	int fd;
	uint32_t sampleRate=0;
	char buf[2048]={0};
	char value[6][15];
	char *str,*pbuf,tmp[6]={0};
	ssize_t nbytes;
	bool find =false;
	for(int i=0;i<RETRY_TIMES;i++)
		{
			fd = open(UA_Path,O_RDONLY);
			if(fd<0)
			{
				ALOGD("can not open /proc/asound/card2/stream0,try time =%d",i+1);
				usleep(RETRY_SLEEPTIME);
				continue;
			}
			break;
		}
	if(fd<0){
		ALOGV("cant open /proc/asound/card2/stream0,giveup");
		return 0;
	}else{
		read(fd,buf,sizeof(buf)-1);
		close(fd);
		str = strstr(buf,type);
		if(str)
		{
			str = strstr(buf,param);//point to the param line
			if(str)
			{
				nbytes = strlen(str);
				pbuf =str;
				//ALOGD("---get_usbaudio_cap,nbytes=%d,str=%s",nbytes,str);
				while (nbytes > 0) {
					 int matches=0;
					 matches = sscanf(pbuf, "%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]",value[0],value[1],value[2],value[3],value[4],value[5]);
					 if(matches==6)
					 {
						value[5][14]='\0';//to prevent error.
						ALOGD("--------------matches=%d,get value,%s:%s:%s%s:%s:%s\n",matches,value[0],value[1],value[2],value[3],value[4],value[5]);		
						find =true;
						break;
					 }
		
					 // Eat the line.
					 while (nbytes > 0 && *pbuf != '\n') {
							 pbuf++;
							 nbytes--;
					 }
					 if (nbytes > 0) {
							 pbuf++;
							 nbytes--;
					 }
				}
				if(find)
				{
					sampleRate = atoi(value[5]);
				}
				else
				{
					strncpy(tmp,str+7,5);
					sampleRate = atoi(tmp);
				}
			}
		}
		return sampleRate;
	}
}

#endif
