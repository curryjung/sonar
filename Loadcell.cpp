#include "Loadcell.h"

// 목적 : setup 함수
// 개요 : 로드셀 후처리에 필요한 것들을 셋팅
void Loadcell::set_up()
{
    this->q = new CircularQueue(LOADCELL_MOVE_QUE_SIZE);
    this->q2 = new CircularQueue(LOADCELL_STOP_QUE_SIZE);
    puts("Loadcell setup complete");
}

// 목적 : 로우 패스를 적용하는 함수
// 개요 : 로드셀 데이터가 민감하기 때문에 로우패스 필터를 적용
// parameter : loadcellData - 로드셀 raw 데이터, resultData - 로우 패스 필터를 걸쳐서 나올 데이터
void Loadcell::low_pass_filter(float *loadcellData, float *resultData)
{
    for (int i = 0; i < 2; i++)
    {
        // LPF 필터링 된 로드셀 데이터 //
        this->lpfData[i] = LPF_ALPHA * this->tmpData[i] + (1 - LPF_ALPHA) * loadcellData[i];

        // 로드셀 최대 및 최소값 제한 //
        if(fabs(this->lpfData[i]) < LOADCELL_LIMIT_MIN) // 최소값
        {
            this->mainData[i] = 0;;
        }
        else if (fabs(this->lpfData[i]) > LOADCELL_LIMIT_MAX) // 최대값
        {
            if (this->lpfData[i] < 0)
            {
                this->mainData[i] = -1 * LOADCELL_LIMIT_MAX;
            }
            else
            {
                this->mainData[i] = LOADCELL_LIMIT_MAX;
            }
        }
        else
        {
            this->mainData[i] = this->lpfData[i];
        }
    }

    // 이전 로드셀 데이터 //
    this->tmpData[0] = loadcellData[0];
    this->tmpData[1] = loadcellData[1];

    // return 해줄 결과 데이터 //
    resultData[0] = this->mainData[0];
    resultData[1] = this->mainData[1];
}

// 목적 : 사용자가 손잡이를 잡고있는지 확인
// 개요 : 로드셀 데이터의 연속성을 확인하여 사용자가 손잡이터치 여부를 확인
// 수동 운행일 경우 카트의 속도에 따라 로드셀 데이터가 0 이 될수도 있기때문에 사용
bool Loadcell::is_touched()
{
    if (this->moveStat == false) // STOP 조건
    {
        if(abs(this->mainData[0]) > LOADCELL_STAT_MOVE && abs(this->mainData[1]) > LOADCELL_STAT_MOVE)
        {
            this->q->en_queue(1);
        }
        else
        {
            this->q->init_queue();
        }

        if(this->q->is_full())
        {
            this->moveStat = true;
        }
    }
    else // MOVE 조건
    {
        if(abs(this->mainData[0]) < LOADCELL_STAT_STOP && abs(this->mainData[1]) < LOADCELL_STAT_STOP)
        {
            this->q2->en_queue(1);
        }
        else
        {
            this->q2->init_queue();
        }

        if(this->q2->is_full())
        {
            this->moveStat = false;
        }

    }
    // // 로드셀을 만졌을 때 //
    // if (this->mainData[0] != 0 || this->mainData[1] != 0)
    // {
    //     this->q->en_queue(1); // 로드셀 데이터가 0 이 아닐 경우
    // }
    // else
    // {
    //     this->q->en_queue(0); // 로드셀 데이터가 0 일 경우
    // }

    // if (q->is_full())
    // {
    //     int sum = 0;
    //     for (int i = 0; i < LOADCELL_TOUCHED_COUNT; i++)
    //     {
    //         sum += q->queue[i];
    //     }

    //     this->q->de_queue();

    //     if (sum != 0)
    //         return 1;
    //     else
    //         return 0;
    // }

    // return 0;
    return this->moveStat;
}

// 목적 : 카트 방향을 확인
// 개요 : 로드셀 결과 데이터의 부호에 따라서 카트 방향 설정
// return : 직진(FF) / 후진(BB) / 좌회전(LR) / 우회전(RR) / 후방커브(BC) / 전방커브(FC)
// CartDir을 설정함으로써 ManualMode 클래스에서 설정값 변경
CartDir Loadcell::confirm_direction()
{
    // 방향 확인 //
    if (this->mainData[0] > 0 && this->mainData[1] >  0)
    {
        return FF;
    }
    else if (this->mainData[0] < 0 && this->mainData[1] < 0)
    {
        return BB;
    }
    else if (this->mainData[0] < 0 && this->mainData[1] > 0)
    {
        return LR;
    }
    else if (this->mainData[0] > 0 && this->mainData[1] < 0)
    {
        return RR;
    }
    else if (this->mainData[0] == 0 && this->mainData[1] < 0)
    {
        return BC;
    }
    else if (this->mainData[0] < 0 && this->mainData[1] == 0)
    {
        return BC;
    }
    else if (this->mainData[0] == 0 && this->mainData[1] > 0)
    {
        return FC;
    }
    else if (this->mainData[0] > 0 && this->mainData[1] == 0)
    {
        return FC;
    }
    else
    {
        return FF;
    }
}
