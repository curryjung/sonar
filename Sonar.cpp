#include "Sonar.h"

// 목적 : 초음파 셋팅
// 개요 : circular 큐를 초음파 센서 개수 만큼 선언해서 사용
void Sonar::set_up()
{
    for (int i = 0; i < SONAR_NUM; i++)
    {
        q[i] = new CircularQueue(Q_SIZE);
    }

    puts("Sonar setup complete");
}

// 목적 : 메디안 필터 적용
// 개요 : 초음파 센서 각 데이터에 메디안 필터를 적용
// parameter : sonarData - 각 초음파 데이터, resultData - 필터 결과 값
// 2차 배열을 사용하며 static으로 선언하여 사용
void Sonar::median_filter(int16_t *sonarData, int16_t *resultData)
{
    static int arr[SONAR_NUM][FILTERSIZE];

    // 데이터 한칸씩 이동
    for (int j = 0; j < SONAR_NUM; j++)
    {
        for (int i = 0; i < FILTERSIZE - 1; i++)
        {
            arr[j][i] = arr[j][i + 1];
        }
    }

    // 마지막 위치에 현재 데이터 추가
    for (int j = 0; j < SONAR_NUM; j++)
    {
        arr[j][FILTERSIZE - 1] = sonarData[j];
    }

    // 데이터 복사한후 정렬
    for (int j = 0; j < SONAR_NUM; j++)
    {
        int median[FILTERSIZE] = {
            0,
        };

        for (int i = 0; i < FILTERSIZE; i++)
        {
            median[i] = arr[j][i];
        }

        for (int k = 0; k < FILTERSIZE - 1; k++)
        {
            if (median[k] < median[k + 1])
            {
                int temp = median[k];
                median[k] = median[k + 1];
                median[k + 1] = temp;
            }
        }

        this->mainData[j] = median[FILTERSIZE / 2];
        resultData[j] = median[FILTERSIZE / 2];
    }
}


//목적 : CURVE RATE를 계산
//개요 : RPM DATA 를 이용하여 카트가 얼마나 크게 커브를 도는지 계산한다. curve_rate_constant 가 클수록 크게 도는 것이다.
// parameter
// 
void Sonar::make_curve_rate_constant(int *rpmData)
{
    if (fabs(rpmData[0]-rpmData[1])<15) // 양쪽 모터 RPM 이 같을 경우 curve_rate_constant 가 무한대가 되기때문에 임의로 큰수를 설정하였다.
    {
        this->curve_rate_constant = MAX_RPM_LIMIT;
    }
    else if(rpmData[0]>rpmData[1])
    {
        this->curve_rate_constant = static_cast<float>(rpmData[1])/(rpmData[0]-rpmData[1]);//float 계산을 위해 int rpmData를 float으로 형변환시켜주었다.
    }
    else if (rpmData[0]<rpmData[1])
    {
        this->curve_rate_constant = static_cast<float>(rpmData[0])/(rpmData[1]-rpmData[0]);//float 계산을 위해 int rpmData를 float으로 형변환시켜주었다.
    }
    

    printf("curve rate constant : %f\n",this->curve_rate_constant);

}

// 목적 : 주변 물체를 감지하여 위험도 설정
// 개요 : 초음파 데이터의 거리값에 따라 위험도를 판단
// parameter : sonarStatus - 초음파 거리에 따른 위험도, sonarRisk - GUI에서 표현하기위해 int 형을 사용하는 위험도
// return : 고정(0)
int Sonar::make_decision(SonarStat sonarStatus, int *sonarRisk, int *rpmData)
{
    int sum[SONAR_NUM];
    int status_sum = 0;



    // 0으로 초기화 및 큐에 데이터 추가 //
    for (int i = 0; i < SONAR_NUM; i++)
    {
        sonarRisk[i] = 0;

        if (q[i]->is_full())
        {
            q[i]->de_queue();
        }

        q[i]->en_queue(this->mainData[i]);
    }

    for (int j = 0; j < SONAR_NUM; j++)
    {
        for (int i = 0; i < Q_SIZE; i++)
        {
            sum[j] += q[j]->queue[i];
        }
    }

    for (int i = 0; i < SONAR_NUM; i++)
    {
        sum[i] /= Q_SIZE;
    }

    // 초음파 거리에 따른 위험도 판단 //
    for (int i = 0; i < SONAR_NUM ; i++)
    {
        if (sum[i] > LIMIT_DIST_3 && sum[i] == 0)
        {
            sonarRisk[i] = 0;
        }
        else if(sum[i] <= LIMIT_DIST_3 && sum[i] > LIMIT_DIST_2)
        {
            sonarRisk[i] = 3;
        }

        else if (sum[i] <= LIMIT_DIST_2 && sum[i] >= LIMIT_DIST_2)
        {
            sonarRisk[i] = 2;
        }
        else if (sum[i] <= LIMIT_DIST_1 && sum[i] >= 30)
        {
            sonarRisk[i] = 1;
        }
    }





/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////v1.1onar////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
///rpm data와 sonar risk 값으로 sonarstatus를 변경/////
// 진행방향이 앞쪽일때 rpm data로 곡률(curve_rate) 를 계산하여 어느 방향으로 이동할지 예측하고 그 방향의 sonarRisk를 통해 sonarStatus 를 변경
//문제점 : 정지해있을 때 어느방향으로 갈지 알 수 없다.

if(rpmData[0]==0 &&rpmData[1]==0)// 정지해있을 때 sonarStatus 를 normal로 설정
{
    sonarStatus = NORMAL;
    return 0;
}
else if((rpmData[0]>0)&&(rpmData[1]>0)) //주행방향이 앞쪽일때
{
    if(rpmData[0]>rpmData[1])//오른쪽커브
    {
        if (this->curve_rate_constant >= CURVE_BOUNDARY_3)//크게 돌 때 (직선주행에 가까울때)
        {
            if(sonarRisk[2] == 1)//전방 센서만 확인
            {
                sonarStatus = WARNN3;
                return 0;                
            }
            else if(sonarRisk[2] ==2)
            {
                sonarStatus = WARNN2;
                return 0;                
            }
            else if(sonarRisk[2] ==3)
            {
                sonarStatus = WARNN1;
                return 0;
            }
            else{
                sonarStatus = NORMAL;
                return 0;
            }
        }
        else if (this->curve_rate_constant >= CURVE_BOUNDARY_2)//크게 커브돌때
        {   
            if((sonarRisk[2]==1)||(sonarRisk[3]==1))//전방 및 우측전방센서 확인
            {
                sonarStatus = WARNN3;
                return 0;   
            }
            else if((sonarRisk[2]==2)||(sonarRisk[3]==2))
            {
                sonarStatus = WARNN2;
                return 0;
            }
            else if((sonarRisk[2]==3)||(sonarRisk[3]==3))
            {
                sonarStatus = WARNN1;
                return 0;
            }
            else{
                sonarStatus = NORMAL;
                return 0;
            }
        }
        else if(this->curve_rate_constant >= CURVE_BOUNDARY_1)//작게 커브돌때
            {
             if((sonarRisk[3]==1)||(sonarRisk[4]==1))//우측전방 및 우측 후방 센서 확인
            {
                sonarStatus = WARNN3;
                return 0;                
            }
            else if((sonarRisk[3]==2)||(sonarRisk[4]==2))
            {
                sonarStatus = WARNN2;
                return 0;                
            }
            else if((sonarRisk[3]==3)||(sonarRisk[4]==3))
            {
                sonarStatus = WARNN1;
                return 0;
            }
            else{
                sonarStatus = NORMAL;
                return 0;
            }
        }
        else// 거의 회전에 가까울 때
        {
            if(sonarRisk[4]==1)//우측후방센서 확인
            {
                sonarStatus = WARNN3;
                return 0;   
            }
            else if(sonarRisk[4]==2)
            {
                sonarStatus = WARNN2;
                return 0;
            }
            else if(sonarRisk[4]==3)
            {
                sonarStatus = WARNN1;
                return 0;
            }
            else{
                sonarStatus = NORMAL;
                return 0;
            }
        }
        
    }
    else if(rpmData[0]<rpmData[1])//왼쪽커브
    {
        if (this->curve_rate_constant >= CURVE_BOUNDARY_3)//크게 돌 때 (직선주행에 가까울때)
        {
    
            if(sonarRisk[2] == 1)//전방 센서만 확인
            {
                sonarStatus = WARNN3;
                return 0;                
            }
            else if(sonarRisk[2] ==2)
            {
                sonarStatus = WARNN2;
                return 0;                
            }
            else if(sonarRisk[2] ==3)
            {
                sonarStatus = WARNN1;
                return 0;
            }
            else{
                sonarStatus = NORMAL;
                return 0;
            }
        }
        else if (this->curve_rate_constant >= CURVE_BOUNDARY_2)//중간정도로 돌 때(무난히)
        {   
            if((sonarRisk[2]==1)||(sonarRisk[1]==1))//전방 및 좌측전방센서 확인
            {
                sonarStatus = WARNN3;
                return 0;   
            }
            else if((sonarRisk[2]==2)||(sonarRisk[1]==2))
            {
                sonarStatus = WARNN2;
                return 0;
            }
            else if((sonarRisk[2]==3)||(sonarRisk[1]==3))
            {
                sonarStatus = WARNN1;
                return 0;
            }
            else{
                sonarStatus = NORMAL;
                return 0;
            }
        }
        else if(this->curve_rate_constant >= CURVE_BOUNDARY_1)
        {
            if((sonarRisk[0]==1)||(sonarRisk[1]==1))//좌측전방 및 후방센서 확인
            {
                sonarStatus = WARNN3;
                return 0;                
            }
            else if((sonarRisk[0]==2)||(sonarRisk[1]==2))
            {
                sonarStatus = WARNN2;
                return 0;                
            }
            else if((sonarRisk[0]==3)||(sonarRisk[1]==3))
            {
                sonarStatus = WARNN1;
                return 0;
            }
            else{
                sonarStatus = NORMAL;
                return 0;
            }
        }
        else//거의 회전할때
        {   
            if(sonarRisk[1] ==1)//좌측후방센서 확인
            {
                sonarStatus = WARNN3;
                return 0;   
            }
            else if(sonarRisk[1] ==2)
            {
                sonarStatus = WARNN2;
                return 0;
            }
            else if(sonarRisk[1] ==3)
            {
                sonarStatus = WARNN1;
                return 0;
            }
            else{
                sonarStatus = NORMAL;
                return 0;
            }
        }
    }
}
else if((rpmData[0]*rpmData[1]) <=0)// 회전할때
{
    if(rpmData[0]>0) //우회전
    {
        if((sonarRisk[3]==1)||(sonarRisk[4]==1))//우측전방 및 우측후방센서 확인
        {
            sonarStatus = WARNN3;
            return 0;   
        }
        else if((sonarRisk[3]==2)||(sonarRisk[4]==2))
        {
            sonarStatus = WARNN2;
            return 0;
        }
        else if((sonarRisk[3]==3)||(sonarRisk[4]==3))
        {
            sonarStatus = WARNN1;
            return 0;
        }
        else{
            sonarStatus = NORMAL;
            return 0;
        }
    }
    else if(rpmData[1]>0)//좌회전
    {
        if((sonarRisk[0]==1)||(sonarRisk[1]==1))//좌측전방 및 좌측후방센서 확인
        {
            sonarStatus = WARNN3;
            return 0;   
        }
        else if((sonarRisk[0]==2)||(sonarRisk[1]==2))
        {
            sonarStatus = WARNN2;
            return 0;
        }
        else if((sonarRisk[0]==3)||(sonarRisk[1]==3))
        {
            sonarStatus = WARNN1;
            return 0;
        }
        else{
            sonarStatus = NORMAL;
            return 0;
        }
    }
}
else{
    sonarStatus = NORMAL;//후진할때(양쪽rpm 이 모두 음수일때) 사람이 카트 진행방향의 앞에 있으므로 sonarStatus를 normal 로 하였다. 
    return 0;
}

}