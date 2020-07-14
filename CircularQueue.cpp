#include "CircularQueue.h"


CircularQueue::CircularQueue(int queSize)
{
	this->maxSize = queSize;
	queue = new float[this->maxSize];
}


bool CircularQueue::is_empty()
{
	if (rear == front)
		return true;
	
	return false;
}

bool CircularQueue::is_full()
{
	if ((rear + 1) % this->maxSize == front)
		return true;

	return false;
}

float CircularQueue::en_queue(float data)
{
	if (this->is_full())
		return -1;
	else 
	{
		rear = (rear + 1) % maxSize;
		float tmp = queue[rear];
		queue[rear] = data;
		return tmp;
	}
}

float CircularQueue::de_queue()
{
	if (is_empty())
		return -1;
	else 
	{
		front = (front + 1) % maxSize;
		return queue[front];
	}
}

void CircularQueue::init_queue()
{
	this->rear = 0;
	this->front = 0;
}