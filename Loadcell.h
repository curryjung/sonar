
/******************************************************************************
작성자 : hoi
목적 : 보드에서 넘어오는 Loadcell 데이터를 관리
사용방식 : 보드에서 넘어오는 Loadcell raw 값을 받아서 후처리
중요사항 : Circular 큐를 사용하기 때문에 필요 헤더를 include 해야함
******************************************************************************/
#ifndef LOADCELL_H
#define LOADCELL_H

#include "Constant.h"
#include "ATSCom.h"

#define LPF_ALPHA 0.3
#define LOADCELL_STOP_QUE_SIZE 10
#define LOADCELL_MOVE_QUE_SIZE 5

#define LOGGING 1
#define LOADCELL_TOUCHED_COUNT 5
#define LOADCELL_LIMIT_MAX 10
#define LOADCELL_LIMIT_MIN 0.1
#define LOADCELL_STAT_STOP 0.5
#define LOADCELL_STAT_MOVE 1

class Loadcell
{
private:
  float tmpData[2];
  float lpfData[2];
  float mainData[2];
  CircularQueue *q, *q2;
  bool moveStat;

public:
  void set_up();
  void low_pass_filter(float *loadCellData, float *resultData);
  bool is_touched();
  CartDir confirm_direction();
};

#endif
