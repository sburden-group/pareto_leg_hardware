from time import perf_counter
from .experiment import Experiment
from .motor_control import MotorControl
from leg_controllers.designs import Params
import yaml
import keyboard
import argparse

parser = argparse.ArgumentParser(
    prog = "Handshake experiment program",
    description = "This program tests the handshake behavior for a robot design specified by a YAML file.",
)
parser.add_argument('design', type=str, help="YAML file containing the design parameters")
parser.add_argument('repeats', type=int, help="Number of rollouts of each trajectory in the experiment.")
parser.add_argument('duration', type=float, help="The duration of each trajectory rollout.")
parser.add_argument('output', type=str, help="The path to a new file where the data will be saved.")

if __name__ == "__main__":
    try:
        args = parser.parse_args()
        param_file = open(args.design,"r")
        leg_params = Params(*((yaml.load(param_file,yaml.Loader)).values()))
        param_file.close()
        with open(args.output,"w") as output:
            myexperiment = Experiment(leg_params, args.repeats, args.duration, output)
            myprocess = MotorControl(leg_params)
            keyboard.add_hotkey('alt+q',lambda: myprocess.stop())
            myprocess.start()
            start_time = perf_counter()
            while myprocess.is_alive():
                myexperiment.update()
                if myprocess.poll_err():
                    print(myprocess.recv_err())
                elif myprocess.poll_msg():
                    print(myprocess.recv_msg())
    except:
        print("Shutting down!")
        myprocess.stop()
        myprocess.join(timeout=10.)
        myprocess.terminate()