/******************************************************************************
작성자 : hoi
목적 : 수동모드를 작동
사용방식 : 보드에서 넘어오는 로드셀 데이터와 모터 RPM을 활용하여 새로운 RPM을 설정
중요사항 : 각 매크로값들을 신중하게 변영해야함
******************************************************************************/
#ifndef MANUAL_MODE_H
#define MANUAL_MODE_H

#include "Constant.h"
#include "Loadcell.h"
#include "Sonar.h"
#include "ATSCom.h"

#define NOT_USE_SONAR 1// 소나를 사용하지 않음
#define TIMESTAMP

// LPF alpha 값 변수 //
#define MANUALMODE_ALPHA 0.6
#define MANUALMODE_ALPHA_ROTATE 0.3

// 2차방정식을 활용하여 커브 조건 값 만드는 변수 //
#define LOADCELL_DIF_INIT 3 // 3
#define LOADCELL_DIF_MIN 1.8 // 1.8
#define LOADCELL_DIF_MIN2 0.9 // 0.9
#define LOADCELL_DIF_MAX 5// 5
#define MAKE_RPM_MIN 500 // 1000
#define MAKE_RPM_MAX 3000 // 3000
#define MAKE_RPM_MIN2 500 // 1000
#define MAKE_RPM_MAX2 1500 // 3000

// 로드셀에 곱해줄 속도 컨스탄트 값 //
#define FF_COEFFICIENT 800 // 기존값 : 600
#define BB_COEFFICIENT 700 // 기존값 : 500
#define FF_CURVE_COEF 800 // 기존값 : 300
#define BB_CURVE_COEF 700 // 기존값 : 250
#define ROTATION_COEFFICIENT 500 // 기존값 : 150

// 회전 속도의 컨스탄트 값을 바꿔주는 변수 //
#define LR_RPM_LIMIT 1500

// 카트 속도 제한 값 //
#define RPM_FF_LIMIT 3000
#define RPM_BB_LIMIT 3000
#define RPM_ROTATION_LIMIT 1000



class ManualMode
{
private:
    ATSCom *com;
    Loadcell loadCell;
    Sonar sonar;
    float loadcellData[2];
    float normalGraph[2];
    float slowGraph[2];
    int rpmData[2];
    int16_t sonarData[SONAR_NUM];
    CartDir cartDirection;
    SonarStat *sonarStatus;
    CartStat cartStatus;
    CartRL LeftRightDecision;
    float loadcellDifference;
    float curveLimit;

    void get_sensor_data(int16_t *rpmRawData, float *loadcellRawData, int16_t *sonarRawData);
    void make_rpm(int16_t *rpmResultData);
    void make_graph(float *result, float fInit, float fMin_RPM, float fMin_diff, float fMax_RPM, float fMax_diff);
    int make_speed_constant(int cartRpm);
    void state_speed(float *rpmCalculate);
    void rpm_limit(int16_t *rpmResultData);
    float make_alpha();

public:
    ManualMode(ATSCom *in);
    ~ManualMode();
    int sonarRisk[SONAR_NUM];
    void run();
    void data_print();
};

#endif
