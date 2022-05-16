import numpy as np
from experiment import main
import os
import time

if __name__ == "__main__":
    output_dir = "data"
    design = "../designs/design3.yaml"
    config = "../motor_config/config.yaml"
    push_force = np.arange(11.5,12.0,step=.5)
    switching_time = 0.11

    for f in push_force:
        trial_dir = output_dir + "/design3"
        if not os.path.exists(trial_dir):
            os.makedirs(trial_dir)
        trial=trial_dir+"/f=%.1f.xlsx"%(f)
        print("Starting trial...")
        main(trial, design, config, f, switching_time)
        time.sleep(10.)
        input("Trial complete. Enter anything to continue: ")
    print("Finished.")
    