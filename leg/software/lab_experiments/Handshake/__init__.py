from time import perf_counter
from experiment import Experiment
from motor_control import MotorControl
from leg_controllers.designs import Params
import yaml
import keyboard
import argparse

parser = argparse.ArgumentParser(
    prog = "Handshake experiment program",
    description = "This program tests the handshake behavior for a robot design specified by a YAML file.",
)
parser.add_argument('design', type=str, help="YAML file containing the design parameters")
parser.add_argument('motor_config', type=str, help="YAML file containing motor calibration data (calibration position, measurement, axis sign")
parser.add_argument('repeats', type=int, help="Number of rollouts of each trajectory in the experiment.")
parser.add_argument('duration', type=float, help="The duration of each trajectory rollout.")
parser.add_argument('output', type=str, help="The path to a new file where the data will be saved.")

if __name__ == "__main__":
    try:
        args = parser.parse_args()
        leg_params = None
        with open(args.design,"r") as file:
            leg_params = Params(*((yaml.load(file,yaml.Loader)).values()))
        motor_config = None
        with open(args.motor_config) as file:
            motor_config = yaml.load(file,yaml.Loader) 
        with open(args.output,"w") as output:
            experiment = Experiment(leg_params, args.repeats, args.duration, output)
            motorcontrol = MotorControl(leg_params, motor_config)
            keyboard.add_hotkey('alt+q',lambda: motorcontrol.stop())
            motorcontrol.start()
            experiment.start()
            start_time = perf_counter()
            while True:
                if not motorcontrol.is_alive():
                    experiment.stop()
                elif not experiment.is_alive():
                    motorcontrol.stop()
                if motorcontrol.poll_err():
                    print(motorcontrol.recv_err())
                elif motorcontrol.poll_msg():
                    print(motorcontrol.recv_msg())
    except:
        print("Shutting down!")
        motorcontrol.stop()
        motorcontrol.join(timeout=10.)
        motorcontrol.terminate()
        experiment.stop()
        experiment.join(timeout=10.)