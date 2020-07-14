/******************************************************************************
작성자 : lyan
목적 : ATS PCB 보드와 라즈베리파이 간 시리얼 통신
사용방식 : 보드와 정의한 프로토콜로 데이터 수신 및
중요사항 : 햅틱, 스탭모터 부분은 지속적인 테스트 필요. 변수에 대한 내용은 ATS 프로토콜 엑셀 참고
******************************************************************************/


#ifndef ATSCOM_H
#define ATSCOM_H
//include system librarys
#include "Constant.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>

#include <termios.h>
#include <fcntl.h>
#include <cstring>
#include <time.h>

// 데이터 정의
#define RECEIVE_PROTOCOL_CNT 4
#define SEND_PROTOCOL_CNT 5
#define BUFFER_SIZE 255
#define CHECK_ARRAY_SIZE 4
#define DA 0x81
#define TAIL 0xDA

// 프로토콜 HEAD
#define RECV_ALWAYS_DATA_HEAD 0x50
#define RECV_MANUAL_MODE_HEAD 0x51
#define RECV_AUTO_MODE_CONFIRM_HEAD 0x52
#define RECV_AUTO_MODE_HEAD 0x53
#define SEND_STATUS_HEAD 0xA1
#define SEND_MANUAL_MODE_HEAD 0xA2
#define SEND_AUTO_MODE_HEAD 0xA3
#define SEND_TEMP_HEAD 0xA4
#define SEND_UPDATE_HEAD 0xA5

#define DEVICE_NAME "/dev/ttyUSB0"
using namespace std;
typedef unsigned char uchar;

// 사용목적: ATS 보드와 시리얼 통신 
// 클래스 호출 시 내부 쓰레드를 시작하며, 하위 보드와 정의된 프로토콜로 통신함.
// 카트의 상태, 수동, 자동, 센서 세팅 등의 함수를 가지고 있음.
class ATSCom
{
private:
	int fd;
	int put_i = 0;
	int get_i = 0;

	unsigned char int2Byte[2];
	unsigned char long2Byte[4];
	unsigned char float2Byte[4];

	int byte2Int;
	float btye2Float;

	void Int_to_Byte(uint16_t tmp);
	void Long_to_Byte(int32_t tmp);
	void Byte_to_Int(unsigned char tmp1, unsigned char tmp2);

	void Float_to_Byte(float tmp);
	long Byte_to_long(unsigned char *tmp);

	uchar tempBuf[BUFFER_SIZE] = {
        0,
    };

    struct termios newtio;
    pthread_t th;
    pthread_mutex_t mutexLock = PTHREAD_MUTEX_INITIALIZER;

    int idx = 0; // 초기화할 인덱스
    int checkSig[CHECK_ARRAY_SIZE] = {
        0,
    }; // 연속으로 확인할 데이터
    uchar recvHeadArr[RECEIVE_PROTOCOL_CNT] = {0X50, 0x51, 0x52, 0x53};
    uchar recvLengthArr[RECEIVE_PROTOCOL_CNT] = {42, 15, 2, 9}; // revision 34 -> 42
    uchar length = 0;
	uchar checkSum = 0;
	uchar head = 0;
	uchar crcResult[2] = {0, };
	int checkCRC[2] = {0, };

    int confirm_data(uchar *buffer, int readLen);
    void seperate_buf();
    void init_data();

public:
	ATSCom(char *deviceName, int baudrate, int vTime, int vMin);
	~ATSCom();
	comCheck check = WAIT; // 수동모드에서 유효한 데이터가 들어오면 해당 변수를 1로 바꿔 수동모드를 시작.
	struct ATSdata atsData;
	int get_data(void *argument); // PCB datas get Function

	int check_emergency(); // 아직 미 구현....
	int set_state(uint8_t data1, uint8_t data2); // 프로토콜에서 카트의 상태, LED 상태 등을 받거나 세팅하는 함수.
	int manual_set_v(int16_t leftRPM, int16_t rightRPM); // 수동모드에서 왼쪽, 오른쪽 속도를 세팅
	int auto_set_vw(int32_t __distance, int32_t __omega, int8_t brakeMode, int8_t initialPosition); // 자동모드에서 거리, 각도, 현재 nPosition 값을 보드로 보냄
	// ATSdata ID;

	int auto_runCount = 1;

    long long startTime = 0;
    long long stopTime = 0;
    long long current_timestamp();


};
#endif
