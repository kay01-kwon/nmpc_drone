import numpy as np

import sys


class CustomQueue():

    def __init__(self, maxsize=10):
        self.queue_ = []
        self.size_ = 0
        self.maxsize_ = maxsize

    def size(self) -> int:
        return self.size_

    def empty(self) -> bool:
        return self.size_ == 0

    def full(self) -> bool:
        if self.size_ == self.maxsize_:
            return True
        else:
            return False

    def push(self, data):
        self.size_ = self.size_ + 1
        if self.size_ <= self.maxsize_:
            self.queue_.append(data)
        else:
            print("Queue is full")
            print("Terminate the program")
            sys.exit(1)

    def pop(self) -> bool:
        self.size_ = self.size_ - 1
        if self.size_ <= 0:
            return False
        else:
            self.queue_.pop(0)
            return True

    def front(self):
        return self.queue_[0]

    def back(self):
        return self.queue_[-1]

if __name__ == '__main__':
    # Create object
    q = CustomQueue()


    for i in range(20):
        temp = np.array([i,0,0,0])
        if q.full():
            q.pop()
            q.push(temp)
        else:
            q.push(temp)

        print(f'At {i} iter: ',q.front()[0])