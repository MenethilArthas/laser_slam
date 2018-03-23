#include "action_driver.h"
#include <linux/can.h>
#include <unistd.h>

void ActionDriver::SetParam(int canId,int s)
{
    m_canId=canId;
    m_s=s;
}


bool ActionDriver::SetOptMode()
{
	struct can_frame frame;
    int nbytes=0;
	unsigned int data[1][2]={   		 
				   0x01000000,0x00000000,	
				 };
	frame.can_id=0;
	frame.can_dlc=8;
	data[0][0]=0x01000000+(((unsigned int)m_canId)<<16); 
	frame.data[0]=(data[0][0]>>24)&0xff;
	frame.data[1]=(data[0][0]>>16)&0xff;
	frame.data[2]=(data[0][0]>>8)&0xff;
	frame.data[3]=(data[0][0])&0xff;
	frame.data[4]=(data[0][1]>>24)&0xff;
	frame.data[5]=(data[0][1]>>16)&0xff;
	frame.data[6]=(data[0][1]>>8)&0xff;
	frame.data[7]=(data[0][1])&0xff;
	nbytes = write(m_s, &frame, sizeof(frame)); 
	if(nbytes==sizeof(frame))
		return true;
	else
		return false;
}

bool ActionDriver::CfgVel(int acc,int dec)
{
	struct can_frame frame;
	bool result=true;
    int nbytes=0;
    char i=0; 
	unsigned int data[3][2]={    			
					//Index-6060(Mode of operation) data-03(velocity mode)		 
					0x2f606000,0x03000000,  
					//Index-6083(Profile acceleration)		 
					0x23836000,0x00000000,
					//Index-6084(Profile deceleration)		 
					0x23846000,0x00000000};
	frame.can_id=SDOTX+m_canId;
	frame.can_dlc=8;
	/*configure the acc and dec value*/
	data[1][1]=(((acc>>24)&0xff))+(((acc>>16)&0xff)<<8)+ (((acc>>8)&0xff)<<16)+((acc&0xff)<<24);		
	data[2][1]=(((dec>>24)&0xff))+(((dec>>16)&0xff)<<8)+ (((dec>>8)&0xff)<<16)+((dec&0xff)<<24);
	
	for(i=0;i<3;i++)
	{
		frame.data[0]=(data[i][0]>>24)&0xff;
		frame.data[1]=(data[i][0]>>16)&0xff;
		frame.data[2]=(data[i][0]>>8)&0xff;
		frame.data[3]=(data[i][0])&0xff;
		frame.data[4]=(data[i][1]>>24)&0xff;
		frame.data[5]=(data[i][1]>>16)&0xff;
		frame.data[6]=(data[i][1]>>8)&0xff;
		frame.data[7]=(data[i][1])&0xff;
		nbytes = write(m_s, &frame, sizeof(frame)); 

		result=result&&(nbytes==sizeof(frame));

	}
	return result;
}

bool ActionDriver::SetVel(int vel)
{
	struct can_frame frame;
    int nbytes=0;
	unsigned int data[1][2]={   		 
				   0x23ff6000,0x00000000,
				 };
	frame.can_id=SDOTX+m_canId;
	frame.can_dlc=8;
	data[0][1]=(((vel>>24)&0xff))+(((vel>>16)&0xff)<<8)+ (((vel>>8)&0xff)<<16)+((vel&0xff)<<24);	
	frame.data[0]=(data[0][0]>>24)&0xff;
	frame.data[1]=(data[0][0]>>16)&0xff;
	frame.data[2]=(data[0][0]>>8)&0xff;
	frame.data[3]=(data[0][0])&0xff;
	frame.data[4]=(data[0][1]>>24)&0xff;
	frame.data[5]=(data[0][1]>>16)&0xff;
	frame.data[6]=(data[0][1]>>8)&0xff;
	frame.data[7]=(data[0][1])&0xff;
	nbytes = write(m_s, &frame, sizeof(frame)); 
	if(nbytes==sizeof(frame))
		return true;
	else
		return false;
}
