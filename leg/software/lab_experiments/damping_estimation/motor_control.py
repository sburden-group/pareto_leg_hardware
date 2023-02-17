import sys, signal, traceback
from time import perf_counter
import numpy as np
from pareto_leg.odrive_driver import OdriveDriver
import odrive
import leg_controllers.model as model
from leg_controllers.designs import Params
import leg_controllers.hopper as hopper
from multiprocessing import Process, Pipe, Queue, Event
from queue import Empty as queue_empty
from math import remainder
import yaml

CONTROL_LOOP_TIME_S = 0.005

"""
PID control class is used by the MotorControl Process to servo the leg
to desired initial conditions in the experiment.
"""
class PIDController():
    def __init__(self,Kp,Ki,Kd,tau):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tau = tau
        self.integrator = 0.
    
    def update(self, r, x, xdot, dt):
        error = r - x
        self.integrator = self.integrator*(1-dt/self.tau)+dt*error
        return self.Kp*error + self.Ki*self.integrator - self.Kd*xdot

    def reset(self):
        self.integrator = 0.

"""
This process implements the motor control loop and sends motor telemetry to the main
program.
"""
class MotorControl(Process):
    def __init__(self, leg_params : Params, motor_config):
        super().__init__()
        self.stop_event = Event()
        self.msg_pipe = Pipe()                      # for sending debug messages to main program
        self.err_pipe = Pipe()                      # for transmitting errors to main program
        self.odrive = None
        self.leg_params = leg_params
        self.axis_sign = np.array(motor_config["motor axis sign"])
        self.calibration_pos = np.array(motor_config["calibration position"])
        self.calibration_meas = np.array(motor_config["calibreation measurement"])
        self.pid_controller = PIDController(75.,75.,0.,100.)
        self.pid_target = leg_params.l1+leg_params.l2-.1
        self.state = "init"

    def stop(self):
        """ triggers shutdown of this process through the stop event"""
        self.stop_event.set()

    def send_msg(self,msg):
        self.msg_pipe[0].send(msg)

    def poll_msg(self,timeout=0.):
        """ Polls the msg Pipe for a message, only called by the main process."""
        return self.msg_pipe[1].poll(timeout)

    def recv_msg(self):
        """ Retrieves a message from the msg Pipe, only called by the main process."""
        return self.msg_pipe[1].recv()

    def _send_err(self,etype,value,tb):
        """ Sends an error that occured in MotorControll as a formatted stacktrace to the main process."""
        stacktrace = ''.join(traceback.format_exception(etype,value,tb))
        self.err_pipe[0].send(stacktrace)

    def poll_err(self,timeout=0.):
        """ Polls the err Pipe for a stacktrace, only called by the main process."""
        return self.err_pipe[1].poll(timeout)

    def recv_err(self):
        """ Retrieves as stacktrace from the err Pipe, only called by the main process."""
        return self.err_pipe[1].recv()

    def empty_queue(self):
        """ Emptys the telemetry queue, returning any contents. This is called by the main process."""
        if self.telemetry_queue.empty():
            return None
        else:
            data = []
            while not self.telemetry_queue.empty():
                try:
                    data += [self.telemetry_queue.get_nowait()]
                except queue_empty:
                    break
            return data

    def controller(self,q,qdot,dt):
        if self.state == "init":
            return np.zeros(2)
        elif self.state == "pid control":
            a = np.array([self.pid_controller.update(self.pid_target,q[0],qdot[0],dt)])
            torques = hopper.stance_computed_torque(q,qdot,a,self.leg_params)
            return torques / model.Ke
        elif self.state == "pushoff":
            return np.array([-15,-15])
        else:
            return np.zeros(2)

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
                    self.set_current(np.zeros(2))

            # the control loop
            measurements = []
            prev_time = perf_counter() 
            self.send_msg("Starting control loop!")
            while not self.stop_event.is_set():
                # this block updates the controller, transmits commands, and telemetry
                curr_time = perf_counter()
                if curr_time - prev_time >= CONTROL_LOOP_TIME_S:
                    dt = curr_time - prev_time
                    prev_time = curr_time
                    q,qdot = self.get_leg_state()
                    if self.state == "init":
                        self.pid_controller.reset()
                        self.state = "pid control"
                    elif self.state == "pid control":
                        current = self.controller(q,qdot,dt)
                        self.set_current(current)
                        self.send_msg(abs(self.pid_target-q[0]))
                        if abs(self.pid_target-q[0]) < 5e-3:
                            self.state = "pushoff"
                    elif self.state == "pushoff":
                        ymax = self.leg_params.l1+self.leg_params.l2
                        if q[0] > ymax:
                            self.state = "passive"
                    elif self.state == "passive":
                        if q[0] < self.leg_params.l1+self.leg_params.l2+.02:
                            measurements.append((q[0],dt))
                    current = self.controller(q,qdot,dt)
                    self.set_current(current)
            # now estimate the damping and shit
            dt = CONTROL_LOOP_TIME_S
            y0 = self.leg_params.l1+self.leg_params.l2 + model.foot_offset
            y = np.array([m[0] for m in measurements])
            ydot = np.array([(-y[i-1]+y[i+1])/(2*dt) for i in range(1,len(y)-1)])
            yddot = np.array([(y[i-1]-2*y[i]+y[i+1])/(dt**2) for i in range(1,len(y)-1)])
            A = np.array([[ydot[i-1]] for i in range(1,len(y)-1)])
            b = np.array([-self.leg_params.s3_k*(y[i]-y0)-model.m_body*(yddot[i-1]+model.g) for i in range(1,len(y)-1)])
            with open("fit.yaml","w") as file:
                x = np.linalg.lstsq(A,b)[0]
                data = [float(x) for x in x]
                yaml.dump(data,file)

        # in the event of an unhandled exception, turn off the motors and send traceback to main
        except:
            self.send_msg("Error encountered...")
            etype, value, tb = sys.exc_info()
            self._send_err(etype,value,tb)
            self.stop_event.set()
        self.shutdown_odrive()

    def get_motor_state(self):
        # Get Measurements: flip sign on motor 0 to match world configuration. (see pic)
        thetas =  self.axis_sign*np.array(self.odrive.get_motor_angles())
        thetas = thetas+self.calibration_pos-self.calibration_meas
        thetas[0] = remainder(thetas[0],2*np.pi)
        thetas[1] = remainder(thetas[1],2*np.pi)
        thetasdot = self.axis_sign*np.array(self.odrive.get_motor_velocities())
        return thetas, thetasdot

    def get_leg_state(self):
        """ Computes the full state of the robot from the joint angles and velocities. """
        theta, theta_dot = self.get_motor_state()
        hip_foot_angle = model.hip_foot_angle(*theta)
        r = model.leg_length(model.interior_leg_angle(*theta),self.leg_params)
        q = np.array([r*np.cos(hip_foot_angle),theta[0],theta[1],r*np.sin(hip_foot_angle),0.])
        DA = hopper.flight_constraints_jac(q,self.leg_params)
        qdot = np.zeros(len(q))
        qdot[[1,2]] = theta_dot
        qdot[[0]] = np.linalg.lstsq(DA[...,[0]],-DA[...,[1,2,3,4]]@qdot[[1,2,3,4]])[0]
        return q,qdot
        
    def set_current(self,current):
        """ Sends current commands to the odrive. """
        current = np.clip(current,-25.,25.)
        self.odrive.set_torques(*(self.axis_sign*current))

    def shutdown_odrive(self):
        """ Shuts down the odrive. """
        self.odrive.set_torques(0.,0.)
        self.odrive.disarm()
