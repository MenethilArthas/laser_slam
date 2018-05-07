#ifndef ACTION_DRIVER_H_
#define ACTION_DRIVER_H_

#define SDOTX  0x600

class ActionDriver
{
	public:
		ActionDriver()
        {}
        void SetParam(int canId,int s);
		bool SetOptMode();
		bool CfgVel(int acc,int dec);
		bool SetVel(int vel);
		int m_canId;
		int m_s;
};


#endif