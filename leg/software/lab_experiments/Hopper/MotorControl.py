from multiprocessing import Process, Event, Pipe, Queue
from time import perf_counter
import sys, os, signal, traceback
import numpy as np
from pareto_leg.odrive_driver import OdriveDriver
import odrive
from julia import Pkg

LEG_CONTROLLER_PKG_DIR = "../../pareto-leg-control/LegControllers"
Pkg.activate(LEG_CONTROLLER_PKG_DIR)

CONTROL_LOOP_TIME_S = 0.01
CALIB_POSITION = np.array([np.pi/2,np.pi/2])
CALIB_MEASUREMENT = np.array([.195,4.5])

class PIDController():
    def __init__(self,Kp,Ki,Kd,tau):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tau = tau
        self.integrator = 0.
    
    def update(self, p, q, qdot, dt):
        error = p - q
        self.integrator = self.integrator*(1-dt/self.tau)+dt*error
        return self.Kp*error + self.Ki*self.integrator - self.Kd*qdot

    def reset(self):
        self.integrator = 0.

class Hopper():
    MODE_FLIGHT = 0
    MODE_STANCE = 1
    KNEE_MIN_DIST = 0.25+0.15

    def __init__(self, leg_params):
        self.leg_params = leg_params
        self.discrete_state = Hopper.MODE_FLIGHT
        self.mode = Hopper.MODE_FLIGHT
        from julia import LegControllers
        self.control_lib = LegControllers

    def update(self,q,qdot):
        if self.mode == Hopper.MODE_FLIGHT:
            if self.control_lib.Hopper.stance_guard(q,self.leg_params) < 0.:
                self.mode = Hopper.MODE_STANCE
        elif self.mode == Hopper.MODE_STANCE:
            if self.control_lib.Hopper.flight_guard(q,self.leg_params) < 0.:
                self.mode = Hopper.MODE_FLIGHT
        if self.mode == Hopper.MODE_FLIGHT:
            restoring_torque = np.array([-.1,-.1])
            return self.control_lib.Hopper.flight_control(q,qdot,self.leg_params)+restoring_torque
        elif self.mode == Hopper.MODE_STANCE:
            stance_torque = self.control_lib.Hopper.stance_control(q,qdot,self.leg_params)*.6
            return np.clip(stance_torque,-25.*self.control_lib.Model.Ke,-0.1)

class MotorController(Process):
    def __init__(self):
        super().__init__()
        self.stop_event = Event()
        self.msg_pipe = Pipe()
        self.err_pipe = Pipe()
        self.telemetry_queue = Queue()
        self.odrive = None
        self.hopper_controller = None 
        self.filename="foo.pickle"
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
            package_dir = "../../pareto-leg-control/LegControllers"
            Pkg.activate(package_dir)
            from julia import LegControllers
            self.leg_params = LegControllers.Designs.default_params
            self.hopper_controller = Hopper(self.leg_params)

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
                    torques = self.hopper_controller.update(q,qdot)
                    self.set_current(np.zeros(2))

            # the control loop
            prev_time = perf_counter() 
            self.send_msg("Starting control loop!")
            while not self.stop_event.is_set():
                curr_time = perf_counter()
                if curr_time - prev_time >= CONTROL_LOOP_TIME_S:
                    dt = curr_time - prev_time
                    prev_time = curr_time
                    # q,qdot = self.get_leg_state()
                    # torques = self.hopper_controller.update(q,qdot)
                    # current = np.clip(torques/LegControllers.Model.Ke,-25.,25.)
                    # self.send_msg(current)
                    # self.set_current(current)
                    self.send_msg(dt)
                    self.set_current(np.zeros(2))
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
        thetas = np.array(self.odrive.get_motor_angles())
        thetas = (thetas + CALIB_POSITION - CALIB_MEASUREMENT) % (2*np.pi)
        thetasdot = np.array(self.odrive.get_motor_velocities())
        return thetas, thetasdot

    def get_leg_state(self):
        from julia import LegControllers
        params = LegControllers.Designs.default_params
        constraints = LegControllers.ComputedTorque.constraints
        constraints_jac = LegControllers.ComputedTorque.constraints_jac
        theta, theta_dot = self.get_motor_state()
        q = np.array([0.,theta[0],theta[1],0.,0.])
        if self.hopper_controller.mode == Hopper.MODE_FLIGHT:
            q[[3,4]] = -constraints(q,params)[[0,1]]
        elif self.hopper_controller.mode == Hopper.MODE_STANCE:
            q[0] = constraints(q,params)[1]
        qdot = np.array([0.,theta_dot[0],theta_dot[1],0.,0.])
        DA = constraints_jac(q,params)
        qdot[[3, 4]] = np.linalg.solve(DA[0:2, 3:5], np.dot(-DA[0:2, 0:3], qdot[0:3]))
        return q,qdot

    def set_current(self,current):
        self.odrive.set_torques(*(current))

    def shutdown_odrive(self):
        self.odrive.set_torques(0.,0.)
        self.odrive.disarm()

import keyboard
if __name__ == "__main__":
    try:
        myprocess = MotorController()
        keyboard.add_hotkey('alt+q',lambda: myprocess.stop())
        myprocess.start()
        start_time = perf_counter()
        while myprocess.is_alive():
            if myprocess.poll_err():
                print(myprocess.recv_err())
            elif myprocess.poll_msg():
                print(myprocess.recv_msg())
    except:
        print("Shutting down!")
        myprocess.stop()
        myprocess.join(timeout=10.)
        myprocess.terminate()