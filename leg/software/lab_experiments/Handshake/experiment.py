import numpy as np
import pandas as pd
import threading
import leg_controllers.model as model
from leg_controllers.designs import Params
from time import perf_counter
from .motor_control import MotorControl

class Experiment(threading.Thread): # makes sense maybe to make this a thread in main

    INIT_ERROR_NORM = 0.01
    INIT_TIMEOUT = 30.

    def __init__(self, leg_params: Params, repeats, duration, output, motor_controller: MotorControl):
        self.leg_params = leg_params
        self.repeats = repeats
        self.duration = duration
        self.output = output
        self.outfile = None
        self.motor_controller = motor_controller
        self.state = None
        self.stop_event = threading.Event()

    def integration_mesh(self):
        p = self.leg_params
        r = np.linspace(model.leg_length(np.pi/2,p),model.leg_length(np.pi/8,p),num=4)
        tht = np.linspace(-np.pi/4,0.,num=4)
        return r,tht

    def joint_coord_transform(self,r,tht):
        return np.array([r*np.sin(tht),-r*np.cos(tht)])

    def write_rollout_data(self,sheet_name,telemetry):
        heading = ['x','y']+['q%d'%(i) for i in range(5)]+['qdot%d'%i for i in range(5)]+['u1','u2']+['i1','i2']+['power']
        df = pd.DataFrame(columns=heading)
        for data in telemetry:
            row = np.array(data['q'])
            np.append(row,data["qdot"])
            np.append(row,data["torques"])
            np.append(row,data["current"])
            np.append(row,[data["power"]])
            df.loc[data["time"]]=row
        df.to_excel(self.writer,sheet_name=sheet_name)
    
    def write_index(self, init_conds):
        n_rollouts = int(len(init_conds)/2)
        data = np.concatenate(init_conds).reshape((n_rollouts,2))
        df = pd.DataFrame(data=data,columns=["x","y"])
        df.to_excel(self.writer,sheet_name="index")

    def run(self):
        self.outfile = pd.ExcelWriter(self.output, engine="xlsxwriter")
        r,tht = self.integration_mesh()
        init_conds = []
        for i in range(len(r)):
            for j in range(len(tht)):
                for k in range(self.repeats):
                    target = self.joint_coord_transform(r[i],tht[j])
                    init_conds += [target]
                    rollout_data = []
                    self.state = "init"
                    start_time = perf_counter()
                    self.motor_controller.set_computed_torque_mode(target)
                    while not self.stop_event.is_set():
                        if self.state == "init":
                            self.motor_controller.empty_queue()
                            data = self.motor_controller.telemetry_queue.get()
                            if data is not None:
                                q = data["q"]
                                error = target - q[[3,4]]
                                if np.linalg.norm(error) < Experiment.INIT_ERROR_NORM or (perf_counter()-start_time) > Experiment.INIT_TIMEOUT:
                                    self.state = "rollout"
                                    self.motor_controller.empty_queue()
                                    start_time = perf_counter()
                                    self.motor_controller.set_handshake_mode()
                        if self.state == "rollout":
                            if perf_counter()-start_time > self.duration:
                                self.write_rollout_data("rollout %d"%(len(init_conds)),rollout_data)
                                break
                    if self.stop_event.is_set():
                        self.write_index(init_conds)
                        self.writer.save()
        if not self.stop_event.is_set():
            self.write_index(init_conds)
            self.writer.save()

    def stop(self):
        self.stop_event.set()

