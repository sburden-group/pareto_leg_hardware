from time import perf_counter
from motor_control import MotorControl
from leg_controllers.designs import Params
import yaml
import keyboard
import argparse

parser = argparse.ArgumentParser(
    prog = "Hopping demo program",
    description = "This program makes the robot.",
)
parser.add_argument('design', type=str, help="YAML file containing the design parameters")
parser.add_argument('motor_config', type=str, help="YAML file containing motor calibration data (calibration position, measurement, axis sign")

if __name__ == "__main__":
    try:
        print("FOO")
        args = parser.parse_args()
        leg_params = None
        with open(args.design,"r") as file:
            leg_params = Params(*((yaml.load(file,yaml.Loader)).values()))
        motor_config = None
        with open(args.motor_config) as file:
            motor_config = yaml.load(file,yaml.Loader) 
        motorcontrol = MotorControl(leg_params, motor_config)
        keyboard.add_hotkey('alt+q',lambda: motorcontrol.stop())
        motorcontrol.start()
        start_time = perf_counter()
        while motorcontrol.is_alive():
            if motorcontrol.poll_err():
                print(motorcontrol.recv_err())
            elif motorcontrol.poll_msg():
                print(motorcontrol.recv_msg())
    except:
        print("Shutting down!")
        motorcontrol.stop()
        motorcontrol.join(timeout=10.)
        motorcontrol.terminate()