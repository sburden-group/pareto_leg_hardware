import sys, signal, traceback
from time import perf_counter
import numpy as np
from pareto_leg.odrive_driver import OdriveDriver
import odrive
import leg_controllers.handshake as handshake
import leg_controllers.model as model
from leg_controllers.designs import Params
import yaml
from multiprocessing import Process, Pipe, Queue, Event
from queue import Empty as queue_empty

CONTROL_LOOP_TIME_S = 0.01
CALIB_POSITION = np.array([np.pi/2,np.pi/2]) # need to eliminate this and make it a config file
CALIB_MEASUREMENT = np.array([.195,4.5])

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
    MODE_COMPUTED_TORQUE = 1
    MODE_HANDSHAKE = 2
    def __init__(self, leg_params: Params):
        super().__init__()
        self.stop_event = Event()
        self.msg_pipe = Pipe()                      # for sending debug messages to main program
        self.err_pipe = Pipe()                      # for transmitting errors to main program
        self.telemetry_queue = Queue(100)           # for sending system telemetry to main program
        self.odrive = None
        self.leg_params = leg_params 
        self._set_control_mode = Event()            # an event used to switch the control mode (computed torque vs handshake)
        self.controller = None
        self.pid = PIDController(25.0,2.5,10.,30.)  # used by the computed torque controller
        self.pid_target = None                      # target of PID above

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

    def computed_torque(self, q, qdot, dt):
        """ Controller used to servo the leg to target positions via
            feedback linearized PID control."""
        accel = self.pid.update(self.pid_target, q[[3,4]], qdot[[3,4]], dt)
        return handshake.computed_torque(q,qdot,accel,self.leg_params)

    def set_computed_torque_mode(self, target):
        """ Sets the control mode to computed torque."""
        self._set_control_mode.set()
        self.msg_pipe[0].send({"mode": MotorControl.MODE_COMPUTED_TORQUE, "target": target})

    def handshake_control(self, q, qdot, dt):
        return handshake.control(q,qdot,self.leg_params)

    def set_handshake_mode(self):
        """ Sets the control mode to handshake."""
        self._set_control_mode.set()
        self.msg_pipe[0].send({"mode": MotorControl.MODE_HANDSHAKE})

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
            prev_time = perf_counter() 
            self.send_msg("Starting control loop!")
            while not self.stop_event.is_set():

                # this block changes the control mode as required
                if self._set_control_mode.is_set():
                    msg = self.msg_pipe[1].get()
                    if msg["mode"] == MotorControl.MODE_COMPUTED_TORQUE:
                        self.pid.reset()
                        self.pid_target = msg["target"]
                        self.controller = self.computed_torque
                    elif msg["mode"] == MotorControl.MODE_HANDSHAKE:
                        self.controller = self.handshake_control
                
                # this block updates the controller, transmits commands, and telemetry
                curr_time = perf_counter()
                if curr_time - prev_time >= CONTROL_LOOP_TIME_S:
                    dt = curr_time - prev_time
                    prev_time = curr_time
                    q,qdot = self.get_leg_state()
                    if self.controller is not None:
                        torques = self.controller(q,qdot,dt,p)
                        current = np.clip(torques/model.Ke,-25.,25.)
                        self.set_current(current)
                        power = np.sum(model.R*current**2)+np.dot(torques,qdot[[1,2]])
                        self.telemetry_queue.put({
                            "q": q, "qdot": qdot, "torques": torques, "current": current, "power": power , "time": curr_time
                        })
                    else:
                        self.set_current(np.zeros(2))
        
        # in the event of an unhandled exception, turn off the motors and send traceback to main
        except Exception:
            self.send_msg("Error encountered...")
            etype, value, tb = sys.exc_info()
            self._send_err(etype,value,tb)
            self.stop_event.set()
        self.shutdown_odrive()

    def get_motor_state(self):
        # Get Measurements: flip sign on motor 0 to match world configuration. (see pic)
        thetas = np.array(self.odrive.get_motor_angles())
        thetas = (thetas + CALIB_POSITION - CALIB_MEASUREMENT) % (2*np.pi)
        thetasdot = np.array(self.odrive.get_motor_velocities())
        return thetas, thetasdot

    def get_leg_state(self):
        """ Computes the full state of the robot from the joint angles and velocities. """
        theta, theta_dot = self.get_motor_state()
        q = np.array([0.,theta[0],theta[1],0.,0.])
        q[[-2,-1]] = -model.kin_constraints(q,self.leg_params)
        DA = model.kin_constraints_jac(q,self.leg_params)
        qdot = np.zeros(len(q))
        qdot[[1,2]] = theta_dot
        qdot[[3,4]] = np.linalg.solve(DA[...,[3,4]],-DA[...,[1,2]@theta_dot])
        return q,qdot
        
    def set_current(self,current):
        """ Sends current commands to the odrive. """
        self.odrive.set_torques(*(current))

    def shutdown_odrive(self):
        """ Shuts down the odrive. """
        self.odrive.set_torques(0.,0.)
        self.odrive.disarm()
