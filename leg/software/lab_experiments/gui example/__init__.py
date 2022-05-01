from cProfile import label
from re import I
import tkinter as tk
import tkinter.messagebox as tkmsgbox
from pareto_leg.odrive_driver import odrive, OdriveDriver
import numpy as np
import yaml

class PIController():
    def __init__(self,Kp,Ki,imax):
        self.Kp = Kp
        self.Ki = Ki
        self.integrator = 0.
        self.imax = imax
    
    def update(self, r, x, xdot, dt):
        error = r - x
        self.integrator = np.clip(-self.imax,self.imax,self.integrator*(1-dt/self.tau)+dt*error)
        return self.Kp*error + self.Ki*self.integrator - self.Kd*xdot

    def reset(self):
        self.integrator = 0.

class App(tk.Frame):
    LOOP_DELAY_MS = 10
    MAX_I = 10.
    def __init__(self, master):

        super().__init__(master)
        self.pack()

        # print("Configuring Odrive...")
        # self.odrive = OdriveDriver(odrive.find_any())
        # self.odrive.set_torque_control_mode()
        # self.odrive.arm()
        # print("Odrive configured!")

        self.button_frame = tk.Frame(master=self)
        self.update_button = tk.Button(master=self.button_frame,command=self.update,text="UPDATE")
        self.update_button.grid(row=0,column=0,sticky='s')
        self.start_button =  tk.Button(master=self.button_frame,command=self.start,text="START")
        self.start_button.grid(row=0,column=1,sticky='s')
        self.stop_button = tk.Button(master=self.button_frame,text="STOP")
        self.stop_button.grid(row=0,column=2,sticky='s')
        self.save_button = tk.Button(master=self.button_frame,text="SAVE")
        self.save_button.grid(row=0,column=3,sticky='s')
        self.button_frame.pack(side="bottom")

        self.paused = True

        self.setpoint_scale = tk.Scale(master = self,label="(M0+M1)/2 Setpoint",orient="horizontal")
        self.setpoint_scale["from"] = -90
        self.setpoint_scale["to"] = 90
        self.setpoint_scale["resolution"] = 0.5
        self.setpoint_scale.pack(side="bottom",fill="x")

        self.setpoint = np.NaN
        self.pi = PIController(0.,0.,App.MAX_I)

        self.conf_frame = tk.Frame(master=self)
        self.conf_frame.pack(side="left",fill="both")
        self.kp_label = tk.Label(master=self.conf_frame,text="Kp: ")
        self.kp_label.grid(row=0,column=0)
        self.kp_ent = tk.Entry(master=self.conf_frame)
        self.kp_ent.grid(row=0,column=1)
        self.ki_label = tk.Label(master=self.conf_frame,text="Ki: ")
        self.ki_label.grid(row=1,column=0)
        self.ki_ent = tk.Entry(master=self.conf_frame)
        self.ki_ent.grid(row=1,column=1)
        self.output_label = tk.Label(master=self.conf_frame,text="Output: ")
        self.output_label.grid(row=2,column=0)
        self.output_ent = tk.Entry(master=self.conf_frame)
        self.output_ent.grid(row=2,column=1)

        self.telemetry_frame= tk.Frame(master=self)
        self.telemetry_frame.pack(side="left",fill="both")
        self.m0_text = tk.Label(master=self.telemetry_frame,text="M0 angle: %.4f"%(np.NaN))
        self.m0_text.grid(row=0,column=0)
        self.I0_text = tk.Label(master=self.telemetry_frame,text="I0: %.4f"%(np.NaN))
        self.I0_text.grid(row=0,column=1)
        self.m1_text = tk.Label(master=self.telemetry_frame,text="M1 angle: %.4f"%(np.NaN))
        self.m1_text.grid(row=1,column=0)
        self.I1_text = tk.Label(master=self.telemetry_frame,text="I1: %.4f"%(np.NaN))
        self.I1_text.grid(row=1,column=1)
        self.avg_angle_text = tk.Label(master=self.telemetry_frame,text="(M0+M1)/2: %.4f"%(np.NaN))
        self.avg_angle_text.grid(row=2,column=0,columnspan=2)

        self.after(App.LOOP_DELAY_MS,self.apploop)

    def update(self):
        try:
            self.pi.Kp = float(self.kp_ent.get())
            self.pi.Ki = float(self.ki_ent.get())
            self.setpoint = self.setpoint_scale.get()
        except BaseException as e:
            return tkmsgbox.showerror(title="Error in entry", message=str(e))

    def start(self):
        self.paused = False
    def stop(self):
        self.paused = True
        # self.
    def save(self):
        pass
    def apploop(self):
        self.after(App.LOOP_DELAY_MS,self.apploop)
        if not self.paused:
            thetas = self.odrive.get_motor_angles()


            

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    app.mainloop()



