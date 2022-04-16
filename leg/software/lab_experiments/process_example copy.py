from multiprocessing import Process, Event, Pipe
from time import perf_counter, sleep
import sys
import traceback

class MyProcess(Process):
    def __init__(self):
        super().__init__()
        self.stop_event = Event()
        self.msg_pipe = Pipe()
        self.err_pipe = Pipe()

    def stop(self):
        self.stop_event.set()

    def send_msg(self,msg):
        self.msg_pipe[0].send(msg)

    def poll_msg(self,timeout=0.):
        return self.msg_pipe[1].poll(timeout)

    def recv_msg(self):
        return self.msg_pipe[1].recv()

    def _send_err(self,etype,value,tb):
        stacktrace = ''.join(traceback.format_exception(etype,value,tb))
        self.err_pipe[0].send(stacktrace)

    def poll_err(self,timeout=0.):
        return self.err_pipe[1].poll(timeout)

    def recv_err(self):
        return self.err_pipe[1].recv()

    def run(self):
        start_time = perf_counter()
        try:
            while not self.stop_event.is_set():
                curr_time = perf_counter()
                msg = "Current time: %.3f" % curr_time
                self.send_msg(msg)
                if curr_time - start_time > 10.:
                    raise Exception("TIMEBOMB!")
                else:
                    sleep(1.)
        except Exception:
            etype, value, tb = sys.exc_info()
            self._send_err(etype,value,tb)
            self.stop_event.set()

if __name__ == "__main__":
    myprocess = MyProcess()
    myprocess.start()
    while myprocess.is_alive():
        if myprocess.poll_err():
            print(myprocess.recv_err())
        elif myprocess.poll_msg():
            print(myprocess.recv_msg())
