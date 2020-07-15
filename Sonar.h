/******************************************************************************
작성자 : hoi
목적 : 보드에서 넘어오는 초음파 데이터를 관리
사용방식 : 보드에서 넘어오는 sonar raw 값을 받아서 후처리
중요사항 : Circular 큐를 사용하기 때문에 필요 헤더를 include 해야함
******************************************************************************/
#ifndef SONAR_H
#define SONAR_H

#include "Constant.h"

#define FILTERSIZE 3
#define Q_SIZE 5
#define SONAR_NUM 6

#define LIMIT_DIST_3 100 // distance limit value(cm)
#define LIMIT_DIST_2 50
#define LIMIT_DIST_1 30

#define CURVE_BOUNDARY_1 0.75
#define CURVE_BOUNDARY_2 1.15//예상값 : (카트 가로넓이*1)= 실험값 필요
#define CURVE_BOUNDARY_3 2.3//예상값 : (카트 가로넓이*3)실험값 필요

#define MAX_RPM_LIMIT 3000// 최대 RPM 값

class Sonar
{
private:
    int mainData[SONAR_NUM];
    float curve_rate_constant;
    CircularQueue *q[SONAR_NUM];

public:
    void set_up();
    void median_filter(int16_t *sonarData, int16_t *resultData);
    int make_decision(SonarStat sonarStatus, int *sonarRisk, int *rpmData, CartDir cartDirection, CartRL LeftRightDecision);

};

#endif
