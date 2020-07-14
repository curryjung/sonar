#ifndef CIRCULAR_QUEUE
#define CIRCULAR_QUEUE

class CircularQueue
{
private:
    int rear = 0;  //최초 0에서부터 시작
    int front = 0; //최초 0에서부터 시작

public:
    float *queue; // 큐
    int maxSize;       // 큐 사이즈

    CircularQueue(int queSize); // init 함수
    bool is_empty();
    bool is_full();
    float en_queue(float data);
    float de_queue();
    void init_queue();
};

#endif