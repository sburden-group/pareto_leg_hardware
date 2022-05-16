from time import perf_counter
from .motor_control_process import MotorControl
from leg_controllers.designs import Params
import yaml
import keyboard
import numpy as np
import pandas as pd

def main(design: str, 
        config: str):
    try:
        leg_params = None
        with open(design,"r") as file:
            leg_params = Params(*((yaml.load(file,yaml.Loader)).values()))
        motor_config = None
        with open(config,"r") as file:
            motor_config = yaml.load(file,yaml.Loader) 
        motorcontrol = MotorControl(leg_params, motor_config)
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