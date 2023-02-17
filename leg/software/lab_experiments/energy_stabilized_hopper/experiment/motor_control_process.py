from re import template
import sys, signal, traceback
from time import perf_counter, sleep
import numpy as np
from scipy.linalg import expm
import leg_controllers.model as model
from leg_controllers.designs import Params
import leg_controllers.hopper as hopper
from .stance_control import StanceController
from .flight_control import FlightController
from .pid_controller import PIDController
from .reference_trj import reference
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
    def __init__(self, leg_params : Params, motor_config, pull_force, push_force, switching_time):
        super().__init__()
        self.stop_event = Event()
        self.msg_pipe = Pipe()                      # for sending debug messages to main program
        self.err_pipe = Pipe()                      # for transmitting errors to main program
        self.telemetry_queue = Queue(-1)           # for sending system telemetry to main program
        self.motors= None
        self.leg_params = leg_params
        self.axis_sign = np.array(motor_config["motor axis sign"])
        self.calibration_pos = np.array(motor_config["calibration position"])
        self.calibration_meas = np.array(motor_config["calibration measurement"])
        self.pid_controller = PIDController(150.,75.,3.,10.)
        self.pid_target = leg_params.l1+leg_params.l2-.12
        self.stance_controller = StanceController(leg_params,pull_force,push_force,switching_time)
        self.flight_controller = FlightController(leg_params)
        self.state = "init"
        self.stance_guard_angle = np.pi/8+.2
        self.flight_guard_angle = np.pi/8-.1

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
        elif self.state == "pid control":
            a = np.array([self.pid_controller.update(self.pid_target,q[0],0.,dt)])
            torques = hopper.stance_computed_torque(q,qdot,a,self.leg_params)
            current = torques / model.Ke
        elif self.state == "pushoff":
            current = hopper.stance_computed_torque(q,qdot,np.array([model.g]),self.leg_params)/model.Ke
        elif self.state == "flight control":
            current = self.flight_controller.output(q,qdot,dt)
        elif self.state == "stance control":
            current = self.stance_controller.control(q,qdot,t,dt)
        return np.clip(current,-25.,25.)

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
                        if abs(qdot[0])<1e-3:
                            self.pid_controller.reset()
                            self.state = "pid control"
                    elif self.state == "pid control":
                        if abs(self.pid_target-q[0]) < 1e-2:
                            self.state = "pushoff"
                            self.send_msg(self.state)
                    elif self.state == "pushoff":
                        ymax = self.leg_params.l1+self.leg_params.l2
                        if q[0] > ymax:
                        # int_angle = model.interior_leg_angle(*q[[1,2]])
                        # if int_angle <= self.flight_guard_angle:
                            self.state = "flight control"
                            self.flight_controller.initialize()
                            self.send_msg(self.state)
                    elif self.state == "flight control":
                        flight_trj.append({'u':current, 'q': q, 'qdot': qdot, 't': curr_time, 'dt': dt})
                        ymax = self.leg_params.l1+self.leg_params.l2
                        if q[0] < ymax:
                            print("foo")
                        # int_angle = model.interior_leg_angle(*q[[1,2]])
                        # if int_angle >= self.flight_guard_angle:
                            if hop_count == 0:
                                hop_count += 1
                                flight_trj = []
                                stance_trj = []
                                self.state = "stance control"
                                self.send_msg(self.state)
                                self.stance_controller.t0 = -1.
                            elif hop_count < 10:
                                hop_count += 1
                                self.telemetry_queue.put((stance_trj,flight_trj))
                                stance_trj = []
                                flight_trj = []
                                self.state = "stance control"
                                self.send_msg(self.state)
                                self.stance_controller.t0 = -1.
                            else:
                                self.telemetry_queue.put((stance_trj,flight_trj))
                                self.stop_event.set()
                                break
                    elif self.state == "stance control":
                        stance_trj.append({'u':current, 'q': q, 'qdot': qdot, 't': curr_time, 'dt': dt})
                        ymax = self.leg_params.l1+self.leg_params.l2
                        if q[0] > ymax:
                            print("bar")
                            self.state = "flight control"
                            self.flight_controller.initialize()
                            self.send_msg(self.state)
            self.set_current(np.zeros(2))
            self.send_msg("end motor control process")
            self.shutdown_controllers()
            while not self.telemetry_queue.empty():
                # waiting for the parent process to consume our data
                pass 
        # in the event of an unhandled exception, turn off the motors and send traceback to main
        except:
            self.set_current(np.zeros(2))
            self.send_msg("Error encountered...")
            etype, value, tb = sys.exc_info()
            self._send_err(etype,value,tb)
            self.stop_event.set()

    def get_joint_state(self, motor_state):
        # Get Measurements: flip sign on motor 0 to match world configuration. (see pic)
        thetas =  self.axis_sign*motor_interface.get_motor_angles(motor_state)
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
        if self.state == "init" or self.state =="pushoff" or self.state=="stance control" or self.state=="pid control":
            DA = hopper.stance_constraints_jac(q,self.leg_params)
            qdot = np.zeros(len(q))
            qdot[[1,2]] = theta_dot
            qdot[[0]] = np.linalg.lstsq(DA[...,[0]],-DA[...,[1,2,3,4]]@qdot[[1,2,3,4]])[0]
            return q,qdot
        elif self.state == "flight control":
            DA = hopper.flight_constraints_jac(q,self.leg_params)
            qdot = np.zeros(len(q))
            qdot[[1,2]] = theta_dot
            qdot[[3,4]] = np.linalg.lstsq(DA[...,[3,4]],-DA[...,[0,1,2]]@qdot[[0,1,2]])[0]
            return q,qdot

    def set_current(self,current):
        """ Sends current commands to the odrive. """
        asyncio.run(self.motors.set_current(*(self.axis_sign*current)))

    def shutdown_controllers(self):
        asyncio.run(self.motors.stop())
