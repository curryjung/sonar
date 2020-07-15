#include "ManualMode.h"

// 목적 : 생성자 함수
// 개요 : 클래스 생성시 진행되는 함수
// parameter : in - 통신 클래스 변수
ManualMode::ManualMode(ATSCom *in)
{
    this->com = in; // 외부 SerialCommunication 클래스 인자로 받기
    this->loadCell.set_up(); // loadcell 셋팅
    this->sonar.set_up(); // sonar 셋팅
    make_graph(this->normalGraph, LOADCELL_DIF_INIT, MAKE_RPM_MIN, LOADCELL_DIF_MIN, MAKE_RPM_MAX, LOADCELL_DIF_MAX); // 평속 그래프 제작
    make_graph(this->slowGraph, LOADCELL_DIF_INIT, MAKE_RPM_MIN2, LOADCELL_DIF_MIN2, MAKE_RPM_MAX2, LOADCELL_DIF_MAX); // 느린 속도 그래프 
}

// 목적 : 소멸자 함수
// 개요 : 클래스 소멸시 진행되는 함수
// 사용하지 않음
ManualMode::~ManualMode()
{
}

// 목적 : 센서 데이터를 각 변수에 저장
// 개요 : 통신을 통해 받은 데이터들을 클래스 변수에 저장해주는 함수
// parameter : rpmRawData - 모터 RPM, loadcellRawData - 로드셀 데이터, sonarRawData - 초음파 거리(mm)
void ManualMode::get_sensor_data(int16_t *rpmRawData, float *loadcellRawData, int16_t *sonarRawData)
{


    // Loadcell 데이터 받기 //
    this->loadCell.low_pass_filter(loadcellRawData, this->loadcellData);
    this->cartDirection = this->loadCell.confirm_direction();
    this->LeftRightDecision = this->loadcell.confirm_leftright();
    // 초음파 센서 데이터 받기 //
    printf("sonarRawData[0] : %d, sonar[1] : %d,sonar[2] : %d, sonar[3] : %d, sonar[4] : %d\n",sonarRawData[0],sonarRawData[1],sonarRawData[2],sonarRawData[3],sonarRawData[4]);
    printf("rpmData[0] : %d, rpmData[1]:%d\n",this->rpmData[0],this->rpmData[1]);
    this->sonar.median_filter(sonarRawData, this->sonarData);


    //실험중//
    // this->loadcellData[0] *= 1.8;
    // this->loadcellData[1] *= 1.8;

    // RPM 데이터 받기 //
    this->rpmData[0] = rpmRawData[0];
    this->rpmData[1] = rpmRawData[1];

    // printf("RPM_Left : %d, RPM_Right : %d\n", this->rpmData[0], this->rpmData[1]);
    // printf("LoadCell_Left : %f, LoadCell_Right : %f\n", this->loadcellData[0], this->loadcellData[1]);
    // printf("cartDirection : %d\n", this->cartDirection);
}

// 목적 : 커브 기준인 그래프 생성
// 개요 : 좌우 로드셀 차이와 현재 카트 속도를 두개의 인자로 함수를 만듬
// parameter : result - 결과 배열이고 2개의 값(a, b) 값, fInit - 초기 로드셀 차이값, fMin_RPM - 카트 커브 기준 최소 RPM, 
//             fMin_diff - 커브 기준 최소 로드셀 차이값, fMax_RPM - 커브 기준 최대 RPM, fMax_diff - 커브 기준 최대 로드셀 차이값
// 함수는 2차항을 가지며 각 변수들을 설정해줘서 각 항들을 찾음
// result 값을 활용해서 커브 기준 그래프 위쪽에 있는지 아래쪽에있는지 판단함
void ManualMode::make_graph(float *result, float fInit, float fMin_RPM, float fMin_diff, float fMax_RPM, float fMax_diff)
{
    float a, b, c, d;
    float ad_bc;
    a = fMin_RPM * fMin_RPM;
    b = fMin_RPM;
    c = fMax_RPM * fMax_RPM;
    d = fMax_RPM;

    ad_bc = 1 / (a * d - b * c);
    result[0] = ad_bc * (d * (fMin_diff - fInit) + (-1 * b) * (fMax_diff - fInit));
    result[1] = ad_bc * ((-1 * c) * (fMin_diff - fInit) + a * (fMax_diff - fInit));
}

// 목적 : rpm 값을 생성
// 개요 : loadcell data와 모터 rpm 값을 활용하여 최종값생성  
// parameter : rpmResultData - 모터 RPM 결과값
void ManualMode::make_rpm(int16_t *rpmResultData)
{
    float rpmCalculate[2]; // 계산하는 RPM
    int cartRpm = (int)(fabs(this->rpmData[0]) + fabs(this->rpmData[1])) / 2; // 카트 속도

    this->loadcellDifference = fabs(this->loadcellData[0] - this->loadcellData[1]); // 로드셀 좌우 값의 차이

// #if NOT_USE_SONAR
//     this->sonarStatus = NORMAL;

// #endif

    this->curveLimit = cartRpm * cartRpm * this->normalGraph[0] + cartRpm * this->normalGraph[1] + LOADCELL_DIF_INIT;

    // 초음파센서값을 활용한 커브 제한 //
    // if (this->sonarStatus == NORMAL)
    //     this->curveLimit = cartRpm * cartRpm * this->normalGraph[0] + cartRpm * this->normalGraph[1] + LOADCELL_DIF_INIT;
    // else
    //     this->curveLimit = cartRpm * cartRpm * this->slowGraph[0] + cartRpm * this->slowGraph[1] + LOADCELL_DIF_INIT;

    // 카트 상태에 따른 constant 값, cartDirection 값 변환 //
    int speedConstant = make_speed_constant(cartRpm);
    // printf("speedConstant : %d\n", speedConstant);

    this->sonar.make_decision(sonarStatus, sonarRisk, this->cartDirection, this->LeftRightDecision);

    // 카트 방향에 따른 alpha 값 변환 //
    float alpha = make_alpha();
    // printf("alpha : %f\n", alpha);



    // 최종 RPM 데이터 생성 //
    rpmCalculate[0] = alpha * this->rpmData[0] + (1 - alpha) * speedConstant * this->loadcellData[0];
    rpmCalculate[1] = alpha * this->rpmData[1] + (1 - alpha) * speedConstant * this->loadcellData[1];
    // printf("CAL : %f, CAL : %f\n", rpmCalculate[0], rpmCalculate[1]);


    ///////////////////////////////////
    // 전진, 후진시 좌우 바퀴에 같은 RPM 설정 및 부호 변경 //
    state_speed(rpmCalculate);

    for (int i = 0; i < 2; i++)
    {
        rpmResultData[i] = round(rpmCalculate[i]);
    }    
    
    //rpm 제한
    rpm_limit(rpmResultData);
    
    // Sonar status 에 따른 모터 입력값 변환
    if (this->sonarStatus == WARNN3)
    {
        rpmResultData[0] = 0;
        rpmResultData[1] = 0;
        printf("WARNN3\n");
    }
    else if (this->sonarStatus == NORMAL){
        printf("Normal\n");
    }
        // 모터 입력을 위한 형변환 //
    for (int i = 0; i < 2; i++)
    {
        rpmResultData[i] = round(rpmCalculate[i]);
    }


    printf("make rpm : %d\t%d\n", rpmResultData[0], rpmResultData[1]);
}

// 목적 : 카트의 방향과 curveLimit 값에 따른 Constant 값을 변환
// 개요 : 카트의 경우 속도를 만들어주기 위한 Constant 값이 있는데 상태에 따라 Constant 값을 변환
// parameter : cartRpm - 현재 카트의 RPM
// return : 직진 속도 계수(FF_COEFFICIENT), 직진커브 속도 계수(FF_CURVE_COEF), 후진 속도 계수(BB_COEFFICIENT), 후진 커브 속도 계수(BB_CURVE_COEF), 회전 속도 계수(ROTATION_COEFFICIENT)
int ManualMode::make_speed_constant(int cartRpm)
{
    // 상태와 차이값을 이용
    if (this->cartDirection == FF && this->loadcellDifference <= this->curveLimit) // 전진 상태에서 일자로 가려할때
        return FF_COEFFICIENT;
    else if (this->cartDirection == FF && this->loadcellDifference > this->curveLimit) // 전진 상태에서 회전하려 할때
    {
        this->cartDirection = FC;
        if(this->loadcellData[0] < this->loadcellData[1])
        {
            this->LeftRightDecision = LEFT;
        }
        else if(this->loadcellData[0] > this->loadcellData[1])
        {
            this->LeftRightDecision = RIGHT;
        }
        return FF_CURVE_COEF;
    }

    else if (this->cartDirection == BB && this->loadcellDifference <= this->curveLimit) // 후진 상태에서 일자로 가려할때
        return BB_COEFFICIENT;

    else if (this->cartDirection == BB && this->loadcellDifference > this->curveLimit) // 후진 상태에서 회전하려고 할때
    {
        this->cartDirection = BC;
        if(this->loadcellData[0] < this->loadcellData[1])
        {
            this->LeftRightDecision = LEFT;
        }
        else if(this->loadcellData[0] > this->loadcellData[1])
        {
            this->LeftRightDecision = RIGHT;
        }
        return BB_CURVE_COEF;
    }

    else if (this->cartDirection == LR || this->cartDirection == RR) // 제자리 회전 하려고 할때
    {
        if (cartRpm > LR_RPM_LIMIT)
        {
            return ROTATION_COEFFICIENT;
        }
        else
        {
            return ROTATION_COEFFICIENT / 2;
        }

        if(this->cartDirection == LR)
        {
            this->LeftRightDecision = LEFT;
        }
        else if(this->cartDirection == RR)
        {
            this->LeftRightDecision = RIGHT;
        }
    }
    return 0;
}

// 목적 : 모터의 로우패스 필터에서 alpha 값을 설정
// 개요 : 회전과 그 이외의 상태에서의 alpha 값이 변화
float ManualMode::make_alpha()
{
    if (this->cartDirection == LR || this->cartDirection == RR)
        return MANUALMODE_ALPHA_ROTATE;//0.3
    else
        return MANUALMODE_ALPHA;//0.6
}

// 목적 : 전진, 후진시에 좌우에 똑같은 rpm을 설정
// 개요 : 좌우 rpm 중 더 작은 rpm 값을 설정
void ManualMode::state_speed(float *rpmCalculate)
{
    if (this->cartDirection == FF || this->cartDirection == BB)
    {
        float lowValue;

        // 좌우 데이터 중 작은값 //
        if (fabs(rpmCalculate[0]) <= fabs(rpmCalculate[1]))
        {
            lowValue = rpmCalculate[0];
        }
        else
        {
            lowValue = rpmCalculate[1];
        }
        rpmCalculate[0] = lowValue;
        rpmCalculate[1] = lowValue;
    }
}

// 목적 : 각 상태에서의 rpm 최대값을 제한
// 개요 : 카트 방향에 따라서 최대값들이 다름
void ManualMode::rpm_limit(int16_t *rpmResultData)
{
    if (this->cartDirection == FF || this->cartDirection == FC)
    {
        if (fabs(rpmResultData[0]) > RPM_FF_LIMIT)
        {
            rpmResultData[0] = RPM_FF_LIMIT;
        }

        if (fabs(rpmResultData[1]) > RPM_FF_LIMIT)
        {
            rpmResultData[1] = RPM_FF_LIMIT;
        }
    }

    else if (this->cartDirection == BB || this->cartDirection == BC)
    {
        if (fabs(rpmResultData[0]) > RPM_BB_LIMIT)
        {
            rpmResultData[0] = -1 * RPM_BB_LIMIT;
        }

        if (fabs(rpmResultData[1]) > RPM_BB_LIMIT)
        {
            rpmResultData[1] = -1 * RPM_BB_LIMIT;
        }
    }

    else if (this->cartDirection == LR)
    {
        if (fabs(rpmResultData[0]) > RPM_ROTATION_LIMIT)
        {
            rpmResultData[0] = -1 * RPM_ROTATION_LIMIT;
        }
        if (fabs(rpmResultData[1]) > RPM_ROTATION_LIMIT)
        {
            rpmResultData[1] = RPM_ROTATION_LIMIT;
        }
    }

    else if (this->cartDirection == RR)
    {
        if (fabs(rpmResultData[0]) > RPM_ROTATION_LIMIT)
        {
            rpmResultData[0] = RPM_ROTATION_LIMIT;
        }
        if (fabs(rpmResultData[1]) > RPM_ROTATION_LIMIT)
        {
            rpmResultData[1] = -1 * RPM_ROTATION_LIMIT;
        }
    }
}

// 목적 : ManualMode 클래스에서의 메인 함수
// 개요 : 실제로 외부에서 접근해서 사용해야하는 함수
void ManualMode::run()
{
    static int state;
    int preState = state;
    int16_t rpmResultData[2];
    this->com->check = WAIT; // check 값을 바로 바꿔서 다음 데이터를 기다리게 함
#ifndef TIMESTAMP
    struct timeval time;
    gettimeofday(&time, NULL);
    printf("%d \n", time.tv_usec);
#endif
    get_sensor_data(this->com->atsData.motorSpeed, this->com->atsData.loadCell, this->com->atsData.sonar); // 각 변수에 센서 데이터 입력
    
    if (loadCell.is_touched()) // 손잡이를 잡고있는지 확인
    {
        state = 1;
        make_rpm(rpmResultData); // 잡았을 경우에만 rpm 값 생성
    }
    else
    {
        state = 0;

        // init 해야하는 변수들 //
        this->loadcellDifference = 0;
        this->curveLimit = LOADCELL_DIF_INIT;

        for (int i = 0; i < 2; i++)
        {
            rpmResultData[i] = 0;
        }
    }

    // 현재 상태를 나타내는 부분 //
    // 현재는 사용하고있지 않으나 추후 사용 가능
    if (state == 0 && preState == 1)
        this->cartStatus = STAT;
    else if (state == 1 && preState == 1)
        this->cartStatus = MOVE;
    else if (state == 1 && preState == 0)
        this->cartStatus = BRAK;
    else if (state == 0 && preState == 0)
        this->cartStatus = STOP;
    
    this->com->manual_set_v(rpmResultData[0], rpmResultData[1]); // 최종으로 만들어낸 rpm 데이터를 보드에 통신
    data_print(); // 필요 데이터 출력
}

// 목적 : 필요데이터를 출력
// 개요 : 터미널창에 필요 데이터를 출력 및 데이터 로깅시 엑셀화 가능하게 설정
void ManualMode::data_print()
{
    char *dir;
    if (this->cartDirection == FF)
    {
        dir = "FF";
    }
    else if (this->cartDirection == BB)
    {
        dir = "BB";
    }
    else if (this->cartDirection == FC)
    {
        dir = "FC";
    }
    else if (this->cartDirection == BC)
    {
        dir = "BC";
    }
    else if (this->cartDirection == LR)
    {
        dir = "LR";
    }
    else if (this->cartDirection == RR)
    {
        dir = "RR";
    }

    printf("%f\t%f\t%f\t%f\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
           this->loadcellData[0], this->loadcellData[1], this->loadcellDifference, this->curveLimit, dir, this->rpmData[0], this->rpmData[1],
           this->sonarData[0], this->sonarData[1], this->sonarData[2], this->sonarData[3], this->sonarData[4], this->sonarData[5]);
}
