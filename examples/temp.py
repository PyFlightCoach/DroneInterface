from multiprocessing import Process, Queue
import time

def queuer(q):
    while True:
        q.put("JOB")
        print("Adding JOB")
        time.sleep(1)

def worker(q):  
    while True:
        if not q.empty():
            item = q.get()
            print("Running", item)
        else:
            print("No jobs")
            time.sleep(1)



if __name__ == '__main__':
    q = Queue()
    a = Process(target=queuer, args=(q,))
    b = Process(target=worker, args=(q,))
    a.start()
    b.start()