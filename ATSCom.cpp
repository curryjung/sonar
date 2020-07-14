#include "ATSCom.h"
// #include <QDebug>

// 클래스 내부에서 쓰레드를 돌리기 위해서 아래와 같은 방식으로 한 번 거쳐서 돌려야 한다.
static void *run_thread(void *argument)
{
	((ATSCom *)argument)->get_data(NULL);
	return NULL;
}

// uint16 형을 바이트로 바꿈.
void ATSCom::Int_to_Byte(uint16_t tmp)
{
	this->int2Byte[0] = (tmp >> 8) & 0xFF;
	this->int2Byte[1] = tmp & 0xFF;
}

// int32 형을 바이트로 바꿈.
void ATSCom::Long_to_Byte(int32_t tmp)
{
	printf("%d\n", tmp);
	this->long2Byte[0] = (tmp >> 24) & 0xFF;
	this->long2Byte[1] = (tmp >> 16) & 0xFF;
	this->long2Byte[2] = (tmp >> 8) & 0xFF;
	this->long2Byte[3] = tmp & 0xFF;
}

// float 형을 바이트로 바꿈.
void ATSCom::Float_to_Byte(float tmp)
{
	this->float2Byte[0] = (*((int *)&tmp) >> 24) & 0xFF;
	this->float2Byte[1] = (*((int *)&tmp) >> 16) & 0xFF;
	this->float2Byte[2] = (*((int *)&tmp) >> 8) & 0xFF;
	this->float2Byte[3] = *((int *)&tmp) & 0xFF;
}

// 바이트를 int 형으로 바꿈.
void ATSCom::Byte_to_Int(unsigned char tmp1, unsigned char tmp2)
{
	this->byte2Int = (tmp2 << 8) | tmp1;
}

// 바이트를 long 형으로 바꿈.
long ATSCom::Byte_to_long(unsigned char *data)
{
//    printf("posi: %x %x %x %x \n", data[0], data[1], data[2], data[3]);
	long tmp = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0] << 0;
    return tmp;

}

// 목적: 초기 세팅
// 시리얼을 열고 wiringPi와 쓰레드를 시작함.
ATSCom::ATSCom(char *deviceName, int baudrate, int vTime, int vMin)
{
//    system("sudo chmod 777 /dev/ttyUSB0");
//    system("sudo chmod 777 /dev/ttyS0");

    //https://movehoon.tistory.com/115
	this->fd = open(deviceName, O_RDWR | O_NOCTTY| O_NONBLOCK);
//	printf("%s\n",deviceName);
	if (this->fd < 0)
	{
        fprintf(stderr, "Unable to open ATS serial device: %s\n", strerror(errno));
		exit(1); //error
	}
	else
	{
		puts("serial device open success");
	}

	memset(&newtio, 0, sizeof(newtio)); // termios 변수 초기화
	newtio.c_iflag = IGNPAR;			// Parity 에러가 있는 문자 바이트를 무시
	newtio.c_oflag = 0;
	newtio.c_cflag = CS8 | CLOCAL | CREAD; // 8bit | 모뎀 제어 x | 문자 수신 가능

	switch (baudrate)
	{
	case 115200:
		newtio.c_cflag |= B115200;
		break;

	case 57600:
		newtio.c_cflag |= B57600;
		break;

	case 38400:
		newtio.c_cflag |= B38400;
		break;

	case 19200:
		newtio.c_cflag |= B19200;
		break;

	case 9600:
		newtio.c_cflag |= B9600;
		break;

	case 4800:
		newtio.c_cflag |= B4800;
		break;

	case 2400:
		newtio.c_cflag |= B2400;
		break;

	default:
		newtio.c_cflag |= B115200;
		break;
	}

	newtio.c_lflag = 0;			// Non-Canonical 입력 처리 모드
	newtio.c_cc[VMIN] = vMin;   // VTIME 값이 0일 경우 read 함수가 리턴되기 위한 최소의 수신 문자 개수
	newtio.c_cc[VTIME] = vTime; // 수신이 없을 경우의 타임아웃 설정. 이때 read 함수는 0을 반환

	tcflush(this->fd, TCIFLUSH);		   // 전송되지 않았거나 수신 처리가 되어 있지 않은 데이터를 비워버림
	tcsetattr(this->fd, TCSANOW, &newtio); // 모뎀 라인을 초기화하고 포트 setting을 마침

	pthread_create(&th, NULL, run_thread, this);

}

// 소멸자는 사용하지 않음.
ATSCom::~ATSCom()
{
}

int ATSCom::get_data(void *argument)
{
	uchar buffer[BUFFER_SIZE] = {
				0,
			};
			int readLen = 0;

	// read signal
	while (1)
	{	
		while ( (readLen = read(this->fd, buffer, sizeof(buffer)) ) > 0 )
		{
			if (readLen > 0)
			{
                pthread_mutex_lock(&mutexLock);
				confirm_data(buffer, readLen);
                pthread_mutex_unlock(&mutexLock);
			}
		}
	}
}

void ATSCom::init_data()
{
	memset(this->tempBuf, 0, sizeof(this->tempBuf));
	this->idx = 0;
	memset(this->checkSig, 0, sizeof(this->checkSig));
	this->length = 0;
	this->checkSum = 0;
	this->head = 0;
}

int ATSCom::confirm_data(uchar *buffer, int readLen)
{
    int lenlen = 0;

	for (int j = 0; j < readLen; j++)
    {
    //    printf("0x%x\n", buffer[j]);
		// 데이터가 HEAD 데이터인지 확인
		if (this->checkSig[0] == 0 && this->checkSig[1] == 0 && this->checkSig[2] == 0) // 확인버퍼에 헤더데이터가 들어오지 않았을 경우
		{
			for (int i = 0; i < RECEIVE_PROTOCOL_CNT; i++)
			{
				if (buffer[j] == recvHeadArr[i])
				{
					this->checkSig[0] = 1;
                    this->length = recvLengthArr[i];

					this->head = recvHeadArr[i];

                //   printf("0x%x\n", buffer[j]);
					break;
				}
			}
		}
		// 다음 데이터가 DA 인지 확인
		else if (this->checkSig[0] == 1 && this->checkSig[1] == 0 && this->checkSig[2] == 0 && buffer[j] == DA)
		{
        //    printf("0x%x\n", buffer[j]);
			this->checkSig[1] = 1;
		}

		// 다음 데이터가 length 맞는지 확인
        else if (this->checkSig[0] == 1 && this->checkSig[1] == 1 && this->checkSig[2] == 0 &&  buffer[j] == this->length )// this->length ( lenlen = buffer[j] == 0x22 ?  34 : this->length )
		{
//           printf("%d\n", buffer[j]);
			this->checkSig[2] = 1;
		}


        else if (this->checkSig[0] == 1 && this->checkSig[1] == 1 && this->checkSig[2] == 1 && (idx < this->length+2) )
        {
//            printf("%d: ", idx);
//            printf("%x\n", buffer[j]);

//                       printf("0x%x\n", tempBuf[j]);
            this->tempBuf[idx++] = buffer[j];

            this->checkSig[3] = 1;

        }

//         else if (this->checkSig[0] == 1 && this->checkSig[1] == 1 && this->checkSig[2] == 1 && (idx == this->length) )
//         {
//            printf("index: %d, this->length: %d\n", idx, this->length );
//            printf("0x%x, ", buffer[j]);
//            this->idx++;
//            uint16_t crc;
//             crc = calculate_crc16(this->tempBuf, this->length );
////            crc = calculate_crc16(this->tempBuf, idx - 4);
//            memcpy(this->crcResult, &crc, sizeof(this->crcResult));

//            if (this->crcResult[0] == buffer[j]) // j의 index 때문에 crc 계산하자마자 바로 0번째 crc 확인
//            {
//                 puts("first CRC16 ok");
//                this->checkCRC[0] = 1;
//                this->checkSig[3] = 1;
//            }

//         }
//         else if (this->checkSig[0] == 1 && this->checkSig[1] == 1 && this->checkSig[2] == 1 && this->checkSig[3] == 1 && this->checkCRC[0] == 1 && this->crcResult[1] == buffer[j])
//         {
//            this->idx++;
//             puts("second CRC16 ok");
//            this->checkCRC[1] = 1;
//         }

		// TAIL 데이터가 나오기까지 계속 데이터 버퍼에 저장
//         else if (this->checkSig[0] == 1 && this->checkSig[1] == 1 && this->checkSig[2] == 1 && this->checkSig[3] == 1 && this->checkCRC[0] == 1 && this->checkCRC[1] == 1 && buffer[j] == TAIL)
        else if (this->checkSig[0] == 1 && this->checkSig[1] == 1 && this->checkSig[2] == 1 && this->checkSig[3] == 1 && buffer[j] == TAIL )
		{
			// crc 체크하는 부분 추가 할 것!!!!!!!!!!!!!!!!!!!!!!!!!
//            printf("%d\n", length);
            uint16_t crc;
            crc = calculate_crc16(this->tempBuf, this->length );
			for(int j=0; j< this->length; j++)
			{
				// printf("temp %d: 0x%2x, ", j,  tempBuf[j]);
			}
        //   printf("\ncrc\n%x %x %x %x\n\n", crc >> 7 & 0xff, crc & 0xff, tempBuf[idx-2], tempBuf[idx-1] );
//          puts("TAIL ok");

//         printf("receive \n");
//         for (int i = 0; i < this->length+2; i++)
//         {
//             printf("%d: %2X, ", i, tempBuf[i]);
//         }
//         puts("\n\n");

			seperate_buf();
			init_data();
		}
        else
        {
            printf("%d %d %d %d %d %x   \n",this->checkSig[0] , this->checkSig[1], this->checkSig[2],  this->checkSig[3], j, buffer[j] );
		//  for (int i = 0; i < this->length+5; i++)
        //  {
        //      printf("%d: %2X, ", i, tempBuf[i]);
        //  }
        //  puts("\n\n");

			init_data();
			puts("buffer order Error");
		}
	}
	return 0;
}

void ATSCom::seperate_buf()
{
	if (this->head == RECV_ALWAYS_DATA_HEAD)
    {
		this->atsData.cartState = this->tempBuf[0] & 0x0f;
		this->atsData.LEDState = this->tempBuf[0] >> 4 & 0x0f;

		this->atsData.emergencyStop = this->tempBuf[1] & 0x01;
		this->atsData.dockingConnector = this->tempBuf[1] >> 1 & 0x01;
		this->atsData.charingCableConnect = this->tempBuf[1] >> 2 & 0x01;
		this->atsData.cartBattCharge = this->tempBuf[1] >> 3 & 0x01;
		this->atsData.haptic = this->tempBuf[1] >> 4 & 0x01;
		this->atsData.FANState = this->tempBuf[1] >> 5 & 0x01;
		
		this->atsData.buzzer = (this->tempBuf[1] >> 6 & 0x01) << 1 | this->tempBuf[1] >> 7 & 0x01 ;

		this->atsData.battCapcity = this->tempBuf[2] & 0xFF;

		
//          printf("==============================\n\n\n");
//          printf("%d %d\n", this->tempBuf[0], this->tempBuf[4]);
//          printf("cartState: %d\n", this->atsData.cartState);
//          printf("LEDState: %d\n", this->atsData.LEDState);
		
//          printf("emergencyStop: %d\n", this->atsData.emergencyStop);
//          printf("dockingConnector: %d\n", this->atsData.dockingConnector);
//          printf("charingCableConnect: %d\n", this->atsData.charingCableConnect);
//          printf("cartBattCharge: %d\n", this->atsData.cartBattCharge);
//          printf("haptic: %d\n", this->atsData.haptic);
//          printf("FANState: %d\n", this->atsData.FANState);
//          printf("buzzer: %d\n", this->atsData.buzzer);


//          printf("battCapcity: %x\n", this->atsData.battCapcity);

        for (int k = 3; k <= 12; k += 2)
		{
			Byte_to_Int(this->tempBuf[k], this->tempBuf[k + 1]);
			this->atsData.sonar[k / 2 - 1] = this->byte2Int;
            //   printf("sonar%d : %d\n", k/2, this->atsData.sonar[k / 2 - 1]);
		}

        this->atsData.GPS_LAT = Byte_to_long(tempBuf+13);
        this->atsData.GPS_LNG = Byte_to_long(tempBuf+17);



        Byte_to_Int(tempBuf[21],tempBuf[22] );
        this->atsData.temperature = this->byte2Int;

        Byte_to_Int(tempBuf[23],tempBuf[24] );
        this->atsData.humidity = this->byte2Int;

        Byte_to_Int(tempBuf[25],tempBuf[26] );
        this->atsData.temperature2 = this->byte2Int;

        Byte_to_Int(tempBuf[27],tempBuf[28] );
        this->atsData.humidity2 = this->byte2Int;

        Byte_to_Int(tempBuf[29],tempBuf[30] );
        this->atsData.temperature3 = this->byte2Int;

        Byte_to_Int(tempBuf[31],tempBuf[32] );
        this->atsData.humidity3 = this->byte2Int;


        Byte_to_Int(tempBuf[33],tempBuf[34] );
        this->atsData.gyroX = this->byte2Int;

        Byte_to_Int(tempBuf[35],tempBuf[36] );
        this->atsData.gyroY = this->byte2Int;

        Byte_to_Int(tempBuf[37],tempBuf[38] );
        this->atsData.gyroZ = this->byte2Int;

//         printf("GPS_LAT: %x\n", this->atsData.GPS_LAT);
//         printf("GPS_LNG: %x\n", this->atsData.GPS_LNG);
//         printf("temperature: %x\n", this->atsData.temperature);
//         printf("humidity: %x\n", this->atsData.humidity);
//         printf("gyroX: %x\n", this->atsData.gyroX);
//         printf("gyroY: %x\n", this->atsData.gyroY);
//         printf("gyroZ: %x\n", this->atsData.gyroZ);

	}
	else if (this->head == RECV_MANUAL_MODE_HEAD)
	{
//        printf("receive data : ");
//        for (int i = 0; i < this->length; i++)
//        {
//            printf("%2X ", this->tempBuf[i]);
//        }
//        puts("");

        long tmp[2] = {0, };
		
//		memcpy(tmp, this->tempBuf, 4);
//		memcpy(tmp + 1, this->tempBuf + 4, 4);

        tmp[0] = Byte_to_long(this->tempBuf);
        tmp[1] = Byte_to_long(this->tempBuf + 4);
//        printf("loadcell raw: %d %d\n", tmp[0], tmp[1]);
		this->atsData.loadCell[0] = (float)tmp[0] / 100.0;
		this->atsData.loadCell[1] = (float)tmp[1] / 100.0;


        Byte_to_Int(this->tempBuf[8],this->tempBuf[9]);
        this->atsData.motorSpeed[0]  = this->byte2Int;
        Byte_to_Int(this->tempBuf[10],this->tempBuf[11]);
        this->atsData.motorSpeed[1]  = this->byte2Int * -1;


		this->check = MANUAL; // 통신 확인용

//          printf("loadCell: %f, %f\n", this->atsData.loadCell[0], this->atsData.loadCell[1]);
//          printf("motorSpeed: %d\t %d\n", this->atsData.motorSpeed[0], this->atsData.motorSpeed[1]);

          // stop timer
          this->stopTime = current_timestamp();

        //   qDebug() << "diff: " + QString::number(this->stopTime - this->startTime);

    }
	else if (this->head == RECV_AUTO_MODE_CONFIRM_HEAD)
	{
		this->atsData.autoModeCommandCheck = this->tempBuf[0]; // 그 위에서 1로 (맞게 들어왔다) 들어오면 호이가 0으로 초기화 시켜줘야함.
		this->atsData.moveCheck =  this->atsData.autoModeCommandCheck == 0xff ? 1 : 0 ;
		
         printf("CommandCheck %d\n", this->atsData.moveCheck );
	}
	else if (this->head == RECV_AUTO_MODE_HEAD)
	{
		this->atsData.motorDistance = this->tempBuf[3] << 24 | this->tempBuf[2] << 16 | this->tempBuf[1] << 8 | this->tempBuf[0] << 0;
		this->atsData.motorOmega = this->tempBuf[7] << 24 | this->tempBuf[6] << 16 | this->tempBuf[5] << 8 | this->tempBuf[4] << 0;
		this->check = AUTO; // 통신 확인용

        printf("motorDistance: %d \n", this->atsData.motorDistance);
        printf("motorOmega: %d \n", this->atsData.motorOmega);
	}
}

// 목적: 하위보드로 카트, LED 상태값 명령
/*
동작 알고리즘 :
data1, data2 가 매개변수로 입력.
8 byte 배열에 프로토콜에 맞게 변수 갱신.
하위보드로 갱신한 배열 송신
parameter :
data1 - 카트의 동작상태, LED 상태
data2 - 비상정지, 직원호출, 충전케이블 카트배터리 충.방전 여부, 햅틱, FAN, 도킹부, 부저 값 echo  및 명령
return : 0
*/
int ATSCom::set_state(uint8_t data1, uint8_t data2)
{
	uint8_t tmp[8];
	uint16_t crc = 0; // 체크섬 만들어 넣어주는 변수

	tmp[0] = 0xA1;
	tmp[1] = 0x81;
	tmp[2] = 0x02;

	tmp[3] = data1;
	tmp[4] = data2;

	crc = calculate_crc16(tmp+3,2);
    tmp[5] = crc & 0xff;
    tmp[6] = crc >> 8 & 0xff;
	
	tmp[7] = 0xDA;

//	printf("send cart state datas: ");
//	for(int i =0;i<8;i++)
//	{
//		printf("%2x ", tmp[i]);
//	}
//	puts("");

	write(this->fd, tmp, sizeof(tmp));

	return 0;
}

// 목적: 하위보드로 수동모드를 위한 모터 RPM 명령
/*
동작 알고리즘 :
leftRPM , rightRPM 이 매개변수로 입력.
10 byte 배열에 프로토콜에 맞게 변수 갱신.
하위보드로 갱신한 배열 송신
parameter :
leftRPM - 왼쪽 모터
rightRPM - 오른쪽 모터
return : 0
*/
int ATSCom::manual_set_v(int16_t leftRPM, int16_t rightRPM)
{ 
    // start timer
    this->startTime = current_timestamp();

	int i, emg;

	if (this->fd < 0)
	{
#if DEBUG_LEVEL >= DEBUG_MID
		printf("[%s] OmoRobot serial port is abnornal!!!\n", __func__);
#endif
		return -1;
	}

    uint8_t tmp[10];
	tmp[0] = SEND_MANUAL_MODE_HEAD;
	tmp[1] = DA;
	tmp[2] = 4;

	// int to byte

	// leftRPM
	//Int_to_Byte(leftRPM);
	this->int2Byte[0] = (leftRPM)&0xFF;
	this->int2Byte[1] = leftRPM >> 8 & 0xFF;
	tmp[3] = this->int2Byte[0];
	tmp[4] = this->int2Byte[1];
	//printf("%X, %X", int2Byte[0], int2Byte[1]);

	// rightRPM
	//Int_to_Byte(rightRPM);
	this->int2Byte[0] = (rightRPM)&0xFF;
	this->int2Byte[1] = rightRPM >> 8 & 0xFF;
	tmp[5] = this->int2Byte[0];
	tmp[6] = this->int2Byte[1];

    uint16_t crc = calculate_crc16(tmp+3,4);
    tmp[7] = crc & 0xff;
    tmp[8] = crc >> 8 & 0xff;

    tmp[9] = TAIL;

	write(this->fd, tmp, sizeof(tmp));

    // puts("manual_set_v");
    // for(int i =0;i<10;i++)
    // {
    //     printf("%2x ", tmp[i]);
    // }
    // puts("");

#ifdef d
	printf("\nManualMode sendData: %X %X %X %X %X %X %X %X %X\n", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8]);
#endif
	return 0;
}

// 목적: 하위보드로 자동모드에 필요한 거리, 각도, 정지모드, nPosinewtion 초기화 명령
/*
동작 알고리즘 :
__distance, __omega, brakeMode, initialPosinewtion이 매개변수로 입력.
15 byte 배열에 프로토콜에 맞게 변수 갱신.
하위보드로 갱신한 배열 송신
parameter :
__distance - 직진거리값 (단위: mm)
__omega - 회전각도값 (단위: ० -degree)
brakeMode -  정지 모드 설정 (0: 자연정지, 1: TQ_off, 2: 전기적 brake)
initialPosinewtion - nPosinewtion 값을 0으로 변경
return : 0
*/
int ATSCom::auto_set_vw(int32_t __distance, int32_t __omega, int8_t brakeMode, int8_t initialPosinewtion)
{
	int i, emg;

	if (this->fd < 0)
	{
#if DEBUG_LEVEL >= DEBUG_MID
		printf("[%s] OmoRobot serial port is abnornal!!!\n", __func__);
#endif
		return -1;
	}

	uint8_t tmp[16] = {0, };
	tmp[0] = SEND_AUTO_MODE_HEAD;
	tmp[1] = DA;
	tmp[2] = 10;

	// int to byte

	// distance
	Long_to_Byte(__distance);
	tmp[3] = this->long2Byte[3];
	tmp[4] = this->long2Byte[2];
	tmp[5] = this->long2Byte[1];
	tmp[6] = this->long2Byte[0];

	// Omega
	Long_to_Byte(__omega);
	tmp[7] = this->long2Byte[3];
	tmp[8] = this->long2Byte[2];
	tmp[9] = this->long2Byte[1];
	tmp[10] = this->long2Byte[0];

	// runCount
	tmp[11] = this->auto_runCount & 0xff ;
	tmp[12] = this->auto_runCount++ >> 8 & 0xff ;
	
	if(this->auto_runCount > 0xffff)
		this->auto_runCount = 1;

    // CRC 
	uint16_t crc = calculate_crc16(tmp+3,10);
    tmp[13] = crc & 0xff;
    tmp[14] = crc >> 8 & 0xff;

	tmp[15] = TAIL;

	write(this->fd, tmp, sizeof(tmp));

        // puts("auto_set_v");
        // for(int i =0;i<16;i++)
        // {
        //     printf("%2x ", tmp[i]);
        // }
        // puts("");


#ifdef DEBUG_ATSCOM1
	printf("\nsendData: %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
		   tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8],
		   tmp[9], tmp[10], tmp[11], tmp[12], tmp[13]);
#endif
	return 0;
}

long long ATSCom::current_timestamp() {
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000000 + te.tv_usec; // calculate milliseconds
//     printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}
