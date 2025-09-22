import sys
class CustomQueue():

    def __init__(self, maxsize=10):
        self.queue_ = []
        self.size_ = 0
        self.maxsize_ = maxsize

    def size(self)->int:
        return self.size_

    def empty(self)->bool:
        return self.size_ == 0

    def full(self)->bool:
        if self.size_ == self.maxsize_:
            return True
        else:
            return False

    def push(self, data):
        self.size_ = self.size_ + 1
        if self.size_ <= self.maxsize_:
            self.queue_.append(data)
        else:
            assert self.size_ > self.maxsize_, 'size is greater than maxsize'

    def pop(self)->bool:
        self.size_ = self.size_ - 1
        if self.size_ <= 0:
            return False
        else:
            self.queue_.pop(0)
            return True

    def clear(self):
        self.size_ = 0
        self.queue_ = []

    def front(self):
        return self.queue_[0]

    def back(self):
        return self.queue_[-1]