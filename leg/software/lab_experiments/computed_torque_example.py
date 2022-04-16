from multiprocessing import Process, Event, Pipe
from time import perf_counter, sleep
import sys
import signal
import traceback
from matplotlib.pyplot import thetagrids
from pareto_leg.odrive_driver import OdriveDriver
import odrive
import numpy as np
import julia
from numpy import pi
import keyboard

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
        self.Kp = 25.
        self.Ki = 10.
        self.Kd = 0.5*(self.Kp)**.5
        self.tau = 20.
        self.pid_controller = PIDController(self.Kp,self.Ki,self.Kd,self.tau)
        self.leg_params = None
        self.torque_controller = None
        self.setpoint = np.array([-0.1,-.20])
        self.filename = "foo.pickle"
        self.data = []

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
        import psutil, os
        p = psutil.Process(os.getpid())
        p.nice(psutil.HIGH_PRIORITY_CLASS)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        start_time = perf_counter()
        try:
            # configure odrive
            self.send_msg("Configuring Odrive...")
            self.odrive = OdriveDriver(odrive.find_any())
            self.odrive.set_torque_control_mode()
            self.odrive.arm()
            self.send_msg("Odrive configured!")

            self.send_msg("Importing julia modules...")
            from julia import Pkg 
            package_dir = "../pareto-leg-control/LegControllers"
            Pkg.activate(package_dir)
            from julia import LegControllers
            self.leg_params = LegControllers.Designs.default_params
            self.torque_controller = LegControllers.ComputedTorque.control

            # we have been having some weird startup problems with latency
            # so the following loop is a dummy loop to try and wait out this problem
            count = 0
            prev_time = perf_counter()
            while count < 3:
                curr_time = perf_counter()
                if curr_time - prev_time >= CONTROL_LOOP_TIME_S:
                    count += 1
                    dt = curr_time - prev_time
                    prev_time = curr_time
                    q,qdot = self.get_leg_state()
                    accel = self.pid_controller.update(self.setpoint,q[[3,4]],qdot[[3,4]],dt)
                    torques = self.torque_controller(q,qdot,self.leg_params,accel)
                    self.set_current(np.zeros(2))

            # the control loop
            self.pid_controller.reset()
            prev_time = perf_counter() 
            self.send_msg("Starting control loop!")
            while not self.stop_event.is_set():
                curr_time = perf_counter()
                if curr_time - prev_time >= CONTROL_LOOP_TIME_S:
                    dt = curr_time - prev_time
                    prev_time = curr_time
                    q,qdot = self.get_leg_state()
                    accel = self.pid_controller.update(self.setpoint,q[[3,4]],qdot[[3,4]],dt)
                    torques = self.torque_controller(q,qdot,self.leg_params,accel)
                    # torques[0] = self.m1_lpf.update(torques[0],dt)
                    # torques[1] = self.m2_lpf.update(torques[1],dt)
                    # msg = "Torques norm: %.3f" % np.linalg.norm(accel)
                    # self.send_msg(msg)
                    self.set_current(torques/LegControllers.Model.Ke)
                    msg = "Error norm: %.3f" % np.linalg.norm(self.setpoint-q[[3,4]])
                    self.send_msg(msg)
                    row = [*q,*qdot,*torques,dt,curr_time]
                    self.data.append(row)
        except Exception:
            self.send_msg("Error encountered...")
            etype, value, tb = sys.exc_info()
            self._send_err(etype,value,tb)
            self.stop_event.set()
        self.shutdown_odrive()
        import pickle
        with open(self.filename,"wb") as my_pickle:
            my_pickle.write(pickle.dumps(self.data))

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

    def get_leg_state(self):
        from julia import LegControllers
        params = LegControllers.Designs.default_params
        constraints = LegControllers.ComputedTorque.constraints
        constraints_jac = LegControllers.ComputedTorque.constraints_jac
        theta, theta_dot = self.get_motor_state()
        q = np.array([0.,theta[0],theta[1],0.,0.])
        q[[3,4]] = -constraints(q,params)[[0,1]]
        qdot = np.array([0.,theta_dot[0],theta_dot[1],0.,0.])
        DA = constraints_jac(q,params)
        qdot[[3, 4]] = np.linalg.solve(DA[0:2, 3:5], np.dot(-DA[0:2, 0:3], qdot[0:3]))
        return q,qdot

    def set_current(self,current):
        self.odrive.set_torques(*(current * MOTOR_AXIS_SIGN))

    def shutdown_odrive(self):
        self.odrive.set_torques(0.,0.)
        self.odrive.disarm()

if __name__ == "__main__":
    try:
        myprocess = MyProcess()
        keyboard.add_hotkey('alt+q',lambda: myprocess.stop())
        myprocess.start()
        start_time = perf_counter()
        while myprocess.is_alive():
            if myprocess.poll_err():
                print(myprocess.recv_err())
            elif myprocess.poll_msg():
                print(myprocess.recv_msg())
    except KeyboardInterrupt:
        print("Shutting down!")
        myprocess.stop()
        myprocess.join(timeout=10.)
        myprocess.terminate()
