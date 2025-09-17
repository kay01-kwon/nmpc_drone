import numpy as np
import threading
import time
from enum import Enum
buffer = []
MAX_BUFFER_SIZE = 5
condition = threading.Condition()

def producer():
    global buffer
    for i in range(10):
        with condition:
            while len(buffer) == MAX_BUFFER_SIZE:
                print("Producer: Buffer full, waiting...")
                condition.wait()

            item = f"item_{i}"
            buffer.append(item)
            print(f"Producer: Added {item}, buffer: {buffer}")
            condition.notify()
            time.sleep(0.1)

def consumer():
    global buffer
    for i in range(10):
        with condition:
            while not buffer:
                print("Consumer: Buffer empty, waiting...")
                condition.wait()
            item = buffer.pop(0)
            print(f"Consumer: Consumed {item}, buffer: {buffer}")
            condition.notify()
            time.sleep(0.2)


if __name__ == "__main__":
    producer_thread = threading.Thread(target=producer)
    consumer_thread = threading.Thread(target=consumer)

    producer_thread.start()
    consumer_thread.start()

    producer_thread.join()
    consumer_thread.join()
    print("Done")