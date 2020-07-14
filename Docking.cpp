#include "Docking.h"

// 목적 : 생성자 함수
// 개요 : 변수 초기화
// parameter : in - 외부 SerialCommunication 클래스 변수
Docking::Docking(ATSCom *in, float cam2Motor)
{
    this->com = in; // 외부 SerialCommunication 클래스 인자로 받기
    this->step = STEP_ZERO;
    this->cartStat = {cam2Motor};
    destDist = DESTINATION_DISTANCE;
}

// 목적 : 카트 회전
// 개요 : 카트를 원하는 각도만큼 제자리 회전
// parameter : theta - 회전할 각도(deg)
int Docking::turn_cart(double theta)
{
    if (fabs(theta) < LITTLE_ANGLE)
    {
        puts("LITTLE_ANGLE bleow data -> turn_cart exit");
        return 0;
    }

    unsigned long cnt = 0, cnt1 = 0;
    // printf("turn cart %f deg\n", theta);
    printf("turn cart %f deg\n", theta);
    //usleep(50);
    this->com->auto_set_vw(0, (int32_t)(theta * 10), 0, 0);

    while (com->atsData.moveCheck != 1)
    {
        // printf("cnt : %d\n", cnt);
        cnt++;
        if (cnt == 0xffffffff)
        {
            printf("turn again %f deg\n", theta);
            this->com->auto_set_vw(0, theta * 10, 0, 0);
            cnt1++;
            cnt = 0;
            puts("cnt initialize");
            if (cnt1 == 10)
            {
                puts("turn cart Error");
                exit(-1);
            }
        }
    }
    com->atsData.moveCheck = 0;
    // puts("moveCheck received");
    while (com->check != AUTO)
        ;
    com->check = WAIT;

    double moveAng = (double)com->atsData.motorOmega / 10;
    printf("result ang : %f deg\n", moveAng);

    this->com->manual_set_v(0, 0);
    usleep(100000);

    if (fabs(theta - moveAng) <= 1)
        return 0;
    else
    {
        printf("about data :%f\n", theta - moveAng);
        return turn_cart(theta - moveAng);
    }

    // this->com->auto_set_vw(0, 0, 0, 0);
    // usleep(300000);

    // uint8_t setCartTmp = 0x09; // or num | 0x60;
    // this->com->set_state(setCartTmp, 0);
    // usleep(3000000);

    // setCartTmp = 0x06 ; // or num | 0x60;
    // this->com->set_state(setCartTmp, 0);
    // usleep(3000000);
    // return 0;
}

// 목적 : 카트 이동
// 개요 : 카트를 원하는 거리 만큼 직선 이동
// parameter : distance - 이동할 거리(mm)
int Docking::go_straight(double distance)
{
    if (fabs(distance) < 10)
    {
        puts("10mm bleow data -> go_straight exit");
        return 0;
    }

    unsigned long cnt = 0, cnt1 = 0;
    printf("go straight cart %f mm\n", distance);
    //usleep(50);
    this->com->auto_set_vw(distance * 10, 0, 0, 0);

    while (com->atsData.moveCheck != 1)
    {
        //printf("cnt : %d\n", cnt);
        cnt++;
        if (cnt == 0xffffffff)
        {
            printf("straight again %fmm\n", distance);
            this->com->auto_set_vw(distance * 10, 0, 0, 0);
            cnt1++;
            cnt = 0;
            puts("cnt initialize");
            if (cnt1 == 10)
            {
                puts("turn cart Error");
                exit(-1);
            }
        }
    }
    com->atsData.moveCheck = 0;
    // puts("moveCheck received");
    while (com->check != AUTO)
        ;
    com->check = WAIT;

    double moveDist = (double)com->atsData.motorDistance / 10;
    printf("result dist : %f mm\n", moveDist);

    this->com->manual_set_v(0, 0);
    usleep(100000);

    if (fabs(distance - moveDist) <= 10)
        return 0;
    else
    {
        printf("about data :%f\n", distance - moveDist);
        return go_straight(distance - moveDist);
    }

    // uint8_t setCartTmp = 0x09; // or num | 0x60;
    // this->com->set_state(setCartTmp, 0);
    // usleep(3000000);

    // setCartTmp = 0x06 ; // or num | 0x60;
    // this->com->set_state(setCartTmp, 0);
    // usleep(3000000);

    // return 0;
}

// 목적 : 카트가 마커를 바라보게 하는 함수
// 개요 : 카메라와 마커의 Z 축이 서로 평행하게 되도록 회전
int Docking::set_theta_zero()
{
    if (abs(this->arucoResultData.theta) < THETA_LIMIT && abs(this->arucoResultData.theta) != 0)
        return 0;
    else
    {
        if (this->arucoResultData.theta < 0)
        {
            turn_cart(this->arucoResultData.theta);
        }
        else
        {
            turn_cart(this->arucoResultData.theta);
        }
        return 1;
    }
}

// 목적 : 카트가 station 앞으로 이동하는 함수
// 개요 : 카트가 station을 정면으로 바라보도록 이동하는 거리와 각도를 계산
void Docking::turn_objective(int destination)
{
    double theta;
    double dist;
    double alpha;
    double markerAng = this->arucoResultData.theta + DESTINATION_MARKER_ANGLE;

    dist = sqrt(pow((this->arucoResultData.Ddistance + this->cartStat.cam2Motor) * cos(markerAng * DEG2RAD) - destination - this->cartStat.cam2Motor, 2) +
                pow((this->arucoResultData.Ddistance + this->cartStat.cam2Motor) * sin(markerAng * DEG2RAD), 2));
    theta = atan2((this->arucoResultData.Ddistance + this->cartStat.cam2Motor) * sin(markerAng * DEG2RAD),
                  ((this->arucoResultData.Ddistance + this->cartStat.cam2Motor) * cos(markerAng * DEG2RAD) - destination - this->cartStat.cam2Motor)) *
            RAD2DEG;
    alpha = theta - markerAng;

#ifndef PRINT0
    printf("theta : %f\tdist : %f\t alpha : %f\n", theta, dist, alpha);
#endif
    // if (alpha > 0)
    // {
    //     turn_cart(-1 * alpha);
    //     go_straight(dist);
    //     turn_cart(theta);
    // }
    // else
    // {
    turn_cart(-1 * alpha);
    // usleep(100000);
    go_straight(dist);

    turn_cart(theta);
    // }
}

// 목적 : 카트가 마커로 천천히 접근하는 함수
// 개요 : turn_objective 함수 이후에 사용
// 매크로 LEAST_MOVE_DISTANCE 거리만큼 이동하면서 카메라의 중심에 마커를 위치하도록 함
int Docking::approach_marker(ArucoData *forStepTwo)
{
    if (abs(forStepTwo->Xdistance) < XDISTANCE_CENTER)
    {
        go_straight(LEAST_MOVE_DISTANCE);
    }
    else
    {
        if (forStepTwo->Xdistance < 0)
        {
            turn_cart(-1 * LITTLE_ANGLE);
        }
        else
        {
            turn_cart(LITTLE_ANGLE);
        }
    }
    return 0;
}

// 목적 : 이미지상 마커가 중앙에 오도록 카트를 회전하는 함수
// 개요 : Docking 시작 후 가장 먼저 실행
// 매크로 LITTLE_ANGLE 최소각도에 따라서 카트의 움직임 조절
int bigCurveFlag = 0;

int Docking::set_marker_center()
{

    if (confirm_marker_xDist(XDISTANCE_CENTER))
    {
        bigCurveFlag++;

        if (bigCurveFlag == 1)
        {
            XDISTANCE_CENTER = 5;
            bigCurveFlag = 0;
        }
        else
        {
            XDISTANCE_CENTER = 20;
        }
        // printf("XDISTANCE_CENTER: %d \n", XDISTANCE_CENTER);
        return 1;
    }
    else
    {
        if (this->aruco.rawData->Xdistance < 0)
        {
            turn_cart(-1 * LITTLE_ANGLE);
        }
        else
        {
            turn_cart(LITTLE_ANGLE);
        }
    }
    return 0;
}

// 목적 : 카메라 기준으로 마커의 위치를 확인하는 함수
// 개요 : Docking 중 다음 단계(step)로 넘어가기 위한 조건으로 사용
// parameter : xDist - Aruco 마커 검출 Xdistance, theta - Aruco 마커 검출 theta
int Docking::confirm_marker_theta(double theta)
{
    // printf("confirm_marker_theta: %.5f do\n",this->aruco.rawData->theta);
    if (fabs(this->aruco.rawData->theta + DESTINATION_MARKER_ANGLE) < theta)
    {
        return 1;
    }

    return 0;
}

// 목적 : 카메라 기준으로 마커의 위치를 확인하는 함수
// 개요 : Docking 중 다음 단계(step)로 넘어가기 위한 조건으로 사용
// parameter : xDist - Aruco 마커 검출 Xdistance
int Docking::confirm_marker_xDist(double xDist)
{
    if (fabs(this->aruco.rawData->Xdistance) < xDist)
    {
        return 1;
    }

    return 0;
}

// 목적 : 도킹을 실제로 실행하는 함수
// 개요 : 각 step에 따라서 카트의 움직임을 실행
// return : Docking이 끝나거나 Aruco 마커 검출이 되지 않을 경우(0), 나머지 경우(1)

int Docking::run()
{
    double curveRe = 0;

    if (this->aruco.detect_aruco(&this->arucoResultData) == 0)
        return 0; // GUI 프로그램에서 요구하는 바에 따라 수정해야함

#if DOCKING_START
    if (this->aruco.rawData->Ddistance != 0)
    {
        switch (this->step) // 첫번째 스텝
        {
        case STEP_ZERO:
            puts("STEP_ZERO");
            curveRe = asin(this->aruco.rawData->Xdistance / this->aruco.rawData->Ddistance) * RAD2DEG;
            turn_cart(curveRe);
            this->step = STEP_ONE;
            break;

        case STEP_ONE:
            puts("STEP_ONE");
            if (set_marker_center()) // 영상 중앙으로 마커가 위치할 때까지 계속 실행
                this->step = STEP_TWO;
            break;

        case STEP_TWO:
            puts("STEP_TWO");

            if (this->failCnt > FAIL_CNT)
            {
                puts("STEP_TWO END");
                this->step = STEP_THREE;
            }
            else
            {

                this->aruco.sig = 1; // sig 값을 1로 바꿈으로써 aruco data를 메디안 필터에 적용
                if (abs(this->arucoResultData.Ddistance) > 1)
                {
                    // 임시 부분 //
                    // turn_objective(500);
                    // this->step = STEP_THREE;
                    //////////////
#if PRINT_ARUCO_DATA
                    printf("Filtered Data : %6f %5f %6f\n", this->arucoResultData.Ddistance, this->arucoResultData.theta, this->arucoResultData.Xdistance);
                    printf("dist destination : %d\n", this->destDist);
#endif
                    turn_objective(this->destDist); // 메디안 필터에서 걸러진 결과값을 활용하여 station 앞으로 이동

                    // if (confirm_marker_theta(CONFIRM_ANGLE))
                    // {
                    //     this->step = STEP_THREE;
                    //     this->failCnt = 0;
                    // }
                    // else
                    // {
                    this->destDist = this->destDist - DESTINATION_MINUS;
                    this->failCnt++;
                    this->arucoResultData.Ddistance = 0;
                    this->aruco.sig = 0; // 메디안 필터 종료
                    this->step = STEP_ONE;

                    // }
                }
            }
            break;

        case STEP_THREE:
            puts("STEP_THREE");
            // approach_marker(this->aruco.rawData); // 카메라와 마커의 축이 맞았다는 가정하에 서서히 접근
            // this->step = STEP_FIVE;
            // break;
            if (set_marker_center()) // 영상 중앙으로 마커가 위치할 때까지 계속 실행
                this->step = STEP_FIVE;
            break;

            // case STEP_FOUR:
            //     puts("STEP_FOUR");
            //     if (this->aruco.rawData->Ddistance < CAMERA_TO_MARKER && this->aruco.rawData->Xdistance < XDISTANCE_CENTER)
            //         this->step = STEP_FIVE; // 카메라와 마커가 일정 거리 안에 있을 경우에만 다음스텝 진행
            //     else
            //         this->step = STEP_THREE; // 카메라와 마커가 일정 거리 안에 없을 경우 STEP_THREE
            //     break;

        case STEP_FIVE:
            puts("STEP_FIVE");
            go_straight(this->aruco.rawData->Ddistance - DOCKING_TO_STATION); // 카메라와 마커의 거리가 너무 가까울 경우 인식이 안되기 때문에 최종값을 활용하여 이동
            // this->com->auto_set_vw(0, 0, 2, 0);
            this->step = STEP_SIX;
            // break;

        case STEP_SIX:
            puts("END DOCKING");
            return 0;
        }
    }
#endif
    return 1;
}
