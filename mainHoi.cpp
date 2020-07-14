#include "ATSCom.h"
#include "ManualMode.h"
// #include "Docking.h"

#define ARUCO_TEST 0

int main()
{
	ATSCom atsCom("/dev/ttyUSB0",115200,0,1);
	ManualMode manualMode(&atsCom);
	// Docking docking(&atsCom);
	// Docking docking(&atsCom, 1120.0);

	uint8_t stateData1 = 0;
	uint8_t stateData2 = 0;
	int sig = 0;

	printf("수동모드(0), 도킹모드(1)\n");
	printf("55모드를 입력해주세요 ");
	scanf("%d", &sig);
	
	// sig = 1;

	if(sig == 0)
	{
		// 수동모드를 하기 위해서 보드에 넣어주는 값 //
		// stateData1 = 5 | atsCom.atsData.LEDState << 4; // or num | 0x60;
		// stateData2 = atsCom.atsData.emergencyStop | atsCom.atsData.callStaff << 1 | atsCom.atsData.charingCableConnect << 2 | atsCom.atsData.chareState << 3 | atsCom.atsData.haptic << 4 | atsCom.atsData.stationDocking << 6;
		stateData1 &= 0xF0;
    	stateData1 |= 5;
		atsCom.set_state(stateData1, stateData2);
		
		puts("수동모드가 시작됩니다");

		while(1)
		{
			// printf("check %d\n", serialCom.check);
			if(atsCom.check == MANUAL)
			{
				manualMode.run();
			}
			
			// delay(50);
		}
	}
	else if(sig == 1)
	{
		// 자동모드를 하기 위해 보드에 넣어주는 값 //
		stateData1 &= 0xF0;
    	stateData1 |= 6;
		atsCom.set_state(stateData1, stateData2);

#if ARUCO_TEST
	puts("자동모드가 시작됩니다.");
	// while(docking.run());

#else
		// while(atsCom.atsData.autoModeCommandCheck != 0xff);

		puts("자동모드가 시작됩니다.");
		// serialCom.auto_set_vw(0, 0, 0, 1);

		// while(docking.run() );
#endif
	}

	return 0;
}
