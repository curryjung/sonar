/******************************************************************************
작성자 : hoi
목적 : 카트가 station으로 가기 위한 제어 방법
사용방식 : 카메라의 정보를 받아서 마커를 검출 후 검출된 마커의 정보를 활용하여 station으로 진입하기 위한 제어
중요사항 : 카트의 외부파라미터들이 변경될 경우에 변경에 따른 매크로 값들을 변경해야함
******************************************************************************/
#ifndef DOCKING
#define DOCKING

#include "Constant.h"
#include "EvarAruco.h"
#include "ATSCom.h"

#define PRINT_ARUCO_DATA 0
#define PRINT_CURVE 0
#define DOCKING_START 1
#define THETA_LIMIT 5
#define CAMERA_TO_MARKER 700 // 이거리 안으로 오면 STEP_FIVE 스테이션까지 직진
#define DOCKING_TO_STATION 180
#define DESTINATION_DISTANCE 800 // 가장 먼저 카트를 위치하고 싶은 거리 / 기존값 1000
#define DESTINATION_MINUS 400 // 첫번쨰에서 두번째 까지의 거리값 차이 / 800 - 400 / 기본값 : 200
#define FAIL_CNT 1 // 종료 조건(최대 크게 움직일 횟수) 2->1 로 변경함
#define RAD2DEG 180 / 3.14159265359
#define DEG2RAD 3.14159265359 / 180
#define LITTLE_ANGLE 0.5 // station 근접 주행시 마커를 카메라 가운데로 옮기기 위한 스텝당 이동 각도
#define LEAST_MOVE_DISTANCE 50
#define CONFIRM_ANGLE 3.0  //original value 3.0
#define DESTINATION_MARKER_ANGLE 0

// #define XDISTANCE_CENTER 20 // 기존 10

struct cartBody
{
    float cam2Motor;
};

enum process
{
    STEP_ZERO,
    STEP_ONE,
    STEP_TWO,
    STEP_THREE,
    STEP_FOUR,
    STEP_FIVE,
    STEP_SIX,
    STEP_SEVEN
};

class Docking
{
private:
    EvarAruco aruco;
    enum process step = STEP_ZERO;
    ATSCom *com;
    ArucoData arucoResultData;
    cartBody cartStat;
    int failCnt = 0;
    int destDist = 0;
    int XDISTANCE_CENTER = 20;

    int turn_cart(double theta);
    int go_straight(double distance);
    int set_theta_zero();
    void turn_objective(int destination);
    int approach_marker(ArucoData *forStepTwo);
    int confirm_marker_theta(double theta);
    int confirm_marker_xDist(double xDist);
    void theta_to_nPosition(double theta, int32_t *arr);
    void distance_to_nPosition(double distance, int32_t *arr);
    int set_marker_center();

public:
    Docking(ATSCom *in, float cam2Motor);
    int run();
};

#endif
