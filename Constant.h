#ifndef CONSTANT_H
#define CONSTANT_H

#include <stdio.h>	//for printf
#include <stdint.h> //uint8_t definitions
#include <stdlib.h> //for exit(int);
#include <string.h> //for errno
#include <errno.h>	//error output
#include <iostream>
#include <cmath>
#include <pthread.h>
#include <sys/time.h>
#include <time.h>

#include "CircularQueue.h"

// 비트연산 매크로 //
// ClearBit(char data, 1)
// ClearBits(char data)
#define ClearBit(Data, Loc) ((Data) &= ~(0x01 << (Loc)))		  // 한 bit Clear
#define ClearBits(Data, Area, Loc) ((Data) &= ~((Area) << (Loc))) // 연속된 bit Clear
#define SetBit(Data, Loc) ((Data) |= (0x01 << (Loc)))			  // 한 bit Set
#define SetBits(Data, Area, Loc) ((Data) |= ((Area) << (Loc)))	  // 연속된 bit
#define InvertBit(Data, Loc) ((Data) ^= (0x01 << (Loc)))		  // 한 bit 반전
#define InvertBits(Data, Area, Loc) ((Data) ^= ((Area) << (Loc))) // 연속된 bit 반전
#define CheckBit(Data, Loc) (((Data) & (0x01 << (Loc))) >> (Loc)) // 비트 검사
#define ExtractBit(Data, Area, Loc) (((Data) >> (Loc)) & (Area))  // 비트 추출


// Cart State
#define CS_SELF_CHECK 0
#define CS_STATION 1
#define CS_CART_CHARGE 2
#define CS_EV_CHARGE 3
#define CS_EV_CHARGE_STOP 4
#define CS_MANUAL_MODE 5
#define CS_AUTO_MODE 6
#define CS_REMOTE_MODE 7
#define CS_FAULT 8
#define CS_READY 9

// 카트의 방향 //
enum CartDir
{
    FF = 0, // + -
    LR,		// - -
    RR,		// + +
    BB,		// - +
    FC,
    BC
};

enum CartRL // 카트 머리의 방향
{
    STRAIGHT = 0, 
    LEFT,
    RIGHT
};

enum CartStat
{
    STOP = 0,
    STAT,
    BRAK,
    MOVE
};

enum SonarStat
{
    NORMAL = 0,
    WARNN1,
    WARNN2,
    WARNN3
};

struct ArucoData
{
    double Ddistance;
    double theta;
    double Xdistance;
};

enum comCheck
{
    WAIT = 0,
    MANUAL,
    AUTO
};

struct StatusInfo
{
    bool cableType;
    bool charging;
    bool discharging;
    bool cardRead;
    bool testMode;
    bool faultCode;
};

#pragma pack(push, 1) // 1바이트 단위로 정렬

struct ATSdata
{
    uint8_t cartState;
    uint8_t LEDState;

    // 상시 데이터 //
    bool emergencyStop;
    bool dockingConnector;
    bool charingCableConnect;
    bool cartBattCharge;
    bool haptic;
    bool FANState;
    uint8_t buzzer = 0;

    uint8_t battCapcity=0;
    int16_t sonar[6]={0,};
    
    int16_t temperature=0;
    uint16_t humidity=0;
    int16_t temperature2=0;
    uint16_t humidity2=0;
    int16_t temperature3=0;
    uint16_t humidity3=0;

    int16_t GPS_LAT = 0;
    int16_t GPS_LNG = 0;

    int16_t gyroX=0;
    int16_t gyroY=0;
    int16_t gyroZ=0;

    uint16_t runCount=0;
    ////////////////

    // 수동 모드 //
    float loadCell[2]; // 0: left, 1:right
    int16_t motorSpeed[2];
    //////////////

    // 자동 모드 확인 //
    uint8_t autoModeCommandCheck = 0;
    uint8_t moveCheck = 0;
    ///////////////////

    // 자동 모드 //
    int32_t nPosi[2];
    int32_t motorDistance;
    int32_t motorOmega;
    //////////////
};

// SECC Message 0x30001 //
struct SECCStatus
{
    bool seccReady;
    uint8_t seccStatusCode;
    uint8_t seccErrorCode;
    uint16_t reserved;
    uint16_t pwmDutyCycle;
    uint8_t pwmVoltage;
};
// SECC Message 0x30003 //
struct VehicleChargingService
{
    bool selectedPaymentOption;
    uint8_t requestedEnergyTransferType;
    uint16_t maxEntriesSAScheduleTuple;
    uint32_t departureTime;
};

// SECC Message 0x30004 //
struct VehicleDCchargingStatus
{
    uint8_t optionalParameterStatus;
    uint8_t dcChargingStatus;
    uint8_t evErrorCode;
    uint8_t ressSoc;
    uint16_t remainingTimeFullSoc;
    uint16_t remainingTimeBulkSoc;
};

// SECC Message 0x30005 //
struct VehicleDCchargingVariable
{
    uint16_t evTargetCurrent;
    uint16_t evTargetVoltage;
    uint16_t evMaximumCurrentLimit;
    uint16_t evMaximumVoltageLimit;
};

// SECC Message 0x30006 //
struct VehicleDCchargeParameter
{
    uint16_t evEnergyCapacity;
    uint16_t evEnergyRequest;
    uint16_t evMaximumPowerLimit;
    uint8_t fullSoc;
    uint8_t bulkSoc;
};

struct AMdata
{
    uint16_t status;
    uint8_t cardNumber[8];
    uint16_t boardVersion;
    uint16_t protocalVersion;
    uint16_t runCount;
    uint16_t fault1;
    uint16_t fault2;
    uint16_t fault3;

    struct BMSdata
    {
        uint16_t rackVoltage;
        int16_t rackCurrent;
        uint8_t bmsState;
        uint8_t faultPack;
        uint8_t warningPack;
        uint8_t faultCell;
        uint8_t waringCell;
        uint16_t SOC;
        uint16_t SOH;
        uint16_t averageCellVolt;
        uint16_t lowestCellVolt;
        uint16_t highestCellVolt;
        uint16_t differenceCellVolt;
        int16_t averageCellTemp;
        int16_t lowestCellTemp;
        int16_t highestCellTemp;
        int16_t differenceCellTemp;
        uint32_t dischargeAh;
        uint32_t chargeAh;
        int16_t battTemp1;
        int16_t battTemp2;
    } bms;

    struct ACDCconv
    {
        uint32_t outputVoltage;
        uint32_t outputCurrent;
        int8_t moduleTemp;
        uint8_t moduleStatus1;
        uint8_t moduleStatus2;
        uint8_t moduleStatus3;
    } pm1;

    struct DCDCconv
    {
        uint32_t outputVoltage;
        uint32_t outputCurrent;
        uint32_t moduleStatus;
    } pm2, pm3;

    uint16_t dcgfData;

    struct DCpower
    {
        uint32_t eletricPower;
        uint16_t outputVoltage;
        uint16_t outputCurrent;
    } electricPower;

    struct SECC
    {
        SECCStatus seccStatus;
        VehicleChargingService vehicleChargingService;
        VehicleDCchargingStatus vehicleDCchargingStatus;
        VehicleDCchargingVariable vehicleDCchargingVariable;
        VehicleDCchargeParameter vehicleDCchargeParameter;
    } seccData;
};
#pragma pack(pop) // push 부터 pop 까지 1바이트 단위로 정렬

static int16_t reverse_int16_t(int16_t inData)
{
    int16_t result = 0;

    for (int i = 0; i < 16; ++i)
    {
        result |= ((inData & 1) << (15 - i));
        inData >>= 1;
    }
    return result;
}

static unsigned char reverse_uchar(unsigned char inData)
{
    unsigned char result = 0;

    for (int i = 0; i < 8; ++i)
    {
        result |= ((inData & 1) << (7 - i));
        inData >>= 1;
    }
    return result;
}

static uint16_t reverse_uint16_t(uint16_t inData)
{
    uint16_t result = 0;

    for (int i = 0; i < 16; ++i)
    {
        result |= ((inData & 1) << (15 - i));
        inData >>= 1;
    }
    return result;
}

static uint16_t calculate_crc16(uint8_t *nData, uint16_t wLength)
{
    static const uint16_t wCRCTable[] = {
        0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
        0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
        0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
        0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
        0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
        0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
        0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
        0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
        0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
        0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
        0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
        0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
        0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
        0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
        0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
        0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
        0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
        0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
        0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
        0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
        0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
        0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
        0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
        0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
        0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
        0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
        0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
        0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
        0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
        0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
        0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
        0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

    uint8_t nTemp;
    uint16_t wCRCWord = 0xFFFF;

    while (wLength--)
    {
        nTemp = *nData++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord ^= wCRCTable[nTemp];
    }

    return wCRCWord;
}
#endif
