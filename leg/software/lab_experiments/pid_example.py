from multiprocessing import Process, Event, Pipe
from time import perf_counter, sleep
import sys
import signal
import traceback
from pareto_leg.odrive_driver import OdriveDriver
import odrive
import numpy as np
from numpy import pi

class PIDController(object):

    def __init__(self,Kp,Ki,Kd,tau):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tau = tau
        self.integrator = 0.
        self.motor_axis_sign = np.array([-1,1])

    def update(self, p, q, qdot, dt):
        error = p - q
        self.integrator = self.integrator*(1-dt/self.tau)+dt*error
        return self.Kp*error + self.Ki*self.integrator - self.Kd*qdot

    def reset(self):
        self.integrator = 0.

CONTROL_LOOP_TIME_S = 0.01
MOTOR_AXIS_SIGN = np.array([-1,1])
CALIB_POSITION = np.array([pi/2,-pi/2])
CALIB_MEASUREMENT = np.array([-1.316,-2.99])

class MyProcess(Process):
    def __init__(self):
        super().__init__()
        self.stop_event = Event()
        self.msg_pipe = Pipe()
        self.err_pipe = Pipe()
        self.odrive = None

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
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        start_time = perf_counter()
        try:
            # configure odrive
            self.send_msg("Configuring Odrive...")
            self.odrive = OdriveDriver(odrive.find_any())
            self.odrive.set_torque_control_mode()
            self.odrive.arm()
            self.send_msg("Odrive configured!")

            # set up controller parameters
            Kp = 2.75/2
            Ki = 0.5
            Kd = 0.25*(Kp)**.5
            tau = 30.
            controller = PIDController(Kp,Ki,Kd,tau)
            setpoint = np.array([pi/2,-pi/2])

            # control loop
            prev_time = start_time
            self.send_msg("Starting control loop!")
            while not self.stop_event.is_set():
                curr_time = perf_counter()
                if curr_time - prev_time >= CONTROL_LOOP_TIME_S:
                    dt = curr_time - prev_time
                    prev_time = curr_time
                    q,qdot = self.get_motor_state()
                    u = controller.update(setpoint,q,qdot,dt)
                    self.set_torques(u)
                    error = setpoint-q
                    msg = "Norm of error: %.3f" % np.linalg.norm(error)
                    self.send_msg(msg)
        except Exception:
            self.send_msg("Error encountered...")
            etype, value, tb = sys.exc_info()
            self._send_err(etype,value,tb)
            self.stop_event.set()
        self.shutdown_odrive()

    def get_motor_state(self):
        # Get Measurements: flip sign on motor 0 to match world configuration. (see pic)
        thetas = np.array(self.odrive.get_motor_angles()) * MOTOR_AXIS_SIGN # flip sign
        q = thetas + CALIB_POSITION - CALIB_MEASUREMENT
        # Clamp thetas to within (-2pi, +2pi) range
        for i in range(len(thetas)):
            while thetas[i] < -2*pi:
                thetas[i] += 4*pi
            while thetas[i] > 2*pi:
                thetas[i] -= 4*pi
        qdot = np.array(self.odrive.get_motor_velocities()) * MOTOR_AXIS_SIGN # flip sign.
        return q,qdot

    def set_torques(self,torques):
        self.odrive.set_torques(*(torques * MOTOR_AXIS_SIGN))

    def shutdown_odrive(self):
        self.odrive.set_torques(0.,0.)
        self.odrive.disarm()

if __name__ == "__main__":
    try:
        myprocess = MyProcess()
        myprocess.start()
        start_time = perf_counter()
        while myprocess.is_alive():
            if myprocess.poll_err():
                print(myprocess.recv_err())
            elif myprocess.poll_msg():
                print(myprocess.recv_msg())
            if perf_counter()-start_time > 30. :
                myprocess.stop()
    except KeyboardInterrupt:
        print("Shutting down!")
        myprocess.stop()
        myprocess.join(timeout=10.)
        myprocess.terminate()
