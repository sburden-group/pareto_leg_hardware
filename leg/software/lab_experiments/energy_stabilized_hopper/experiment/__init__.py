from time import perf_counter
from .motor_control_process import MotorControl
from leg_controllers.designs import Params
import yaml
import keyboard
import argparse
import numpy as np
import pandas as pd

parser = argparse.ArgumentParser(
    prog = "Foo Bar Baz",
    description = "Foo Bar Baz",
)
parser.add_argument('output_file',type=str)
parser.add_argument('design', type=str, help="YAML file containing the design parameters")
parser.add_argument('config', type=str, help="YAML file containing motor calibration data (calibration position, measurement, axis sign")
parser.add_argument('push_force', type=float)
parser.add_argument('switching_time',type=float)

def main(output_file: str, 
        design: str, 
        config: str,
        push_force: float,
        switching_time: float):
    try:
        leg_params = None
        with open(design,"r") as file:
            leg_params = Params(*((yaml.load(file,yaml.Loader)).values()))
        motor_config = None
        with open(config,"r") as file:
            motor_config = yaml.load(file,yaml.Loader) 
        motorcontrol = MotorControl(leg_params, motor_config,push_force,switching_time)
        keyboard.add_hotkey('alt+q',lambda: motorcontrol.stop())
        motorcontrol.start()
        start_time = perf_counter()
        rollouts = []
        while motorcontrol.is_alive():
            if not motorcontrol.telemetry_queue.empty():
                rollouts += [motorcontrol.telemetry_queue.get()]
            if motorcontrol.poll_err():
                print(motorcontrol.recv_err())
            elif motorcontrol.poll_msg():
                print(motorcontrol.recv_msg())
            elif motorcontrol.stop_event.is_set():
                # break
                pass
        with pd.ExcelWriter(output_file, engine='xlsxwriter') as writer:
            columns = ['q%d'%(i,) for i in range(5)]+['qdot%d'%(i,) for i in range(5)]+['u0','u1']
            for i in range(len(rollouts)):
                stance_trj = rollouts[i][0]
                q = np.vstack([data['q'] for data in stance_trj])
                qdot = np.vstack([data['qdot'] for data in stance_trj])
                u = np.vstack([data['u'] for data in stance_trj])
                t = np.array([data['t'] for data in stance_trj])
                stance_df = pd.DataFrame(data=np.hstack((q,qdot,u)),index=t,columns=columns)
                stance_df.to_excel(writer,sheet_name="stance%d"%(i))
                flight_trj = rollouts[i][1]
                q = np.vstack([data['q'] for data in flight_trj])
                qdot = np.vstack([data['qdot'] for data in flight_trj])
                u = np.vstack([data['u'] for data in flight_trj])
                t = np.array([data['t'] for data in flight_trj])
                flight_df = pd.DataFrame(data=np.hstack((q,qdot,u)),index=t,columns=columns)
                flight_df.to_excel(writer,sheet_name="flight%d"%(i,))
        motorcontrol.stop()
        motorcontrol.join(timeout=10.)
        motorcontrol.terminate()
    except BaseException as e:
        print(e)
        print("Shutting down!")
        motorcontrol.stop()
        motorcontrol.join(timeout=10.)
        motorcontrol.terminate()

# if __name__ == "__main__":
#     args = parser.parse_args()
#     main(args.output_file, args.design, args.motor_config, args.push_force, args.switching_time)