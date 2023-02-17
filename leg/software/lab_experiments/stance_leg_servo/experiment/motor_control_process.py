from re import template
import sys, signal, traceback
from time import perf_counter, sleep
import numpy as np
from scipy.linalg import expm
import leg_controllers.model as model
from leg_controllers.designs import Params
import leg_controllers.hopper as hopper
from .stance_control import StanceController 
from .pid_controller import PIDController
from multiprocessing import Process, Pipe, Queue, Event
from queue import Empty as queue_empty
from math import remainder
import asyncio
import motor_interface

CONTROL_LOOP_TIME_S = 0.01

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
        self.telemetry_queue = Queue(-1)           # for sending system telemetry to main program
        self.motors = None
        self.leg_params = leg_params
        self.axis_sign = np.array(motor_config["motor axis sign"])
        self.calibration_pos = np.array(motor_config["calibration position"])
        self.calibration_meas = np.array(motor_config["calibration measurement"])
        self.pid_target = leg_params.l1+leg_params.l2-.12
        self.stance_controller = StanceController(leg_params)
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

    def controller(self,q,qdot,t,dt):
        current = np.zeros(2)
        if self.state == "init":
            pass
        elif self.state == "stance control":
            current = self.stance_controller.output(q,qdot,dt)
        return current

    def run(self):
        import psutil, os
        p = psutil.Process(os.getpid())
        p.nice(psutil.HIGH_PRIORITY_CLASS)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            # configure odrive
            self.send_msg("Connecting to motors...")
            self.motors = motor_interface.MoteusInterface(1,2)
            asyncio.run(self.motors.connect())
            self.send_msg("Connected!")
            # the control loop
            prev_time = perf_counter() 
            self.send_msg("Starting control loop!")
            hop_count = 0
            stance_trj = []
            flight_trj = []
            while not self.stop_event.is_set():
                # this block updates the controller, transmits commands, and telemetry
                curr_time = perf_counter()
                if curr_time - prev_time >= CONTROL_LOOP_TIME_S:
                    dt = curr_time - prev_time
                    prev_time = curr_time
                    motor_state = asyncio.run(self.motors.query_state())
                    q,qdot = self.get_leg_state(motor_state)
                    current = self.controller(q,qdot,curr_time,dt)
                    self.set_current(current)
                    if self.state == "init":
                        self.state = "stance control"
                    elif self.state == "stance control":
                        pass
            self.set_current(np.zeros(2))
            self.send_msg("end motor control process")
            self.shutdown_controllers()
            while not self.telemetry_queue.empty():
                # waiting for the parent process to consume our data
                pass 
        # in the event of an unhandled exception, turn off the motors and send traceback to main
        except:
            self.send_msg("Error encountered...")
            etype, value, tb = sys.exc_info()
            self._send_err(etype,value,tb)
            self.stop_event.set()

    def get_joint_state(self, motor_state):
        thetas = self.axis_sign*motor_interface.get_motor_angles(motor_state)
        thetas = thetas+self.calibration_pos-self.calibration_meas
        thetas[0] = remainder(thetas[0],2*np.pi)
        thetas[1] = remainder(thetas[1],2*np.pi)
        thetasdot = self.axis_sign*motor_interface.get_motor_velocities(motor_state)
        return thetas, thetasdot

    def get_leg_state(self, motor_state):
        """ Computes the full state of the robot from the joint angles and velocities. """
        theta, theta_dot = self.get_joint_state(motor_state)
        hip_foot_angle = model.hip_foot_angle(*theta)
        r = model.leg_length(model.interior_leg_angle(*theta),self.leg_params)
        q = np.array([r*np.cos(hip_foot_angle),theta[0],theta[1],r*np.sin(hip_foot_angle),0.])
        DA = hopper.flight_constraints_jac(q,self.leg_params)
        qdot = np.zeros(len(q))
        qdot[[1,2]] = theta_dot
        qdot[[0]] = np.linalg.lstsq(DA[...,[0]],-DA[...,[1,2,3,4]]@qdot[[1,2,3,4]])[0]
        return q,qdot
        
    def set_current(self,current):
        """ Sends current commands to the motors """
        asyncio.run(self.motors.set_current(*(self.axis_sign*current)))

    def shutdown_controllers(self):
        asyncio.run(self.motors.stop())
