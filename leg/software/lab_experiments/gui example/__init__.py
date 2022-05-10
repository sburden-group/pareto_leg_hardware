import tkinter as tk
import tkinter.messagebox as tkmsgbox
import tkinter.simpledialog as tksimpledialog
import tkinter.filedialog as tkfiledialog
from pareto_leg.odrive_driver import odrive, OdriveDriver
import numpy as np
import yaml
from math import remainder

class PIController():
    """ this is math stuff for controlling motors """
    def __init__(self,Kp,Ki,imax):
        self.Kp = Kp
        self.Ki = Ki
        self.integrator = 0.
        self.imax = imax
    
    def update(self, error, dt):
        self.integrator = np.clip(-self.imax,self.imax,self.integrator+dt*error)
        return self.Kp*error + self.Ki*self.integrator

    def reset(self):
        self.integrator = 0.

from tkinter.commondialog import Dialog
from tkinter.ttk import Combobox

class ComboboxDialog(Dialog):
    """
    This class is used by the App class to present a pop-up window that contains a drop-down menu,
    and which persists until the OK button is clicked.
    """
    def __init__(self,callback):
        """ The argument callback is a function handle of the form fun(string) which returns the 
        value selected by the user to the main App."""
        self.master = tk.Toplevel()
        super().__init__(self.master)
        self.label = tk.Label(master=self.master,text="Select an odrive")
        self.label.pack()
        self.choices = {}
        while True:
            device = odrive.find_any()
            if device.serial_number in self.choices.keys():
                break
            self.choices[device.serial_number] = device
        self.cbox = Combobox(master=self.master,values=list(self.choices.keys()))
        self.cbox.pack()
        self.ok_button = tk.Button(master=self.master,command=self.ok,text="OK")
        self.ok_button.pack()
        self.callback = callback

    def ok(self):
        """ When the ok button is clicked, this function is executed, which in turn executes
        the callback function given to __init__
        """
        # self.callback(self.cbox.get())
        serial = int(self.cbox.get())
        self.callback(self.choices[serial])
        self.master.destroy()

class App(tk.Frame):
    """
    This is the main class which implements the bulk of the GUI. It has buttons, text entries, and a slider bar.
    """
    LOOP_DELAY_MS = 10
    MAX_I = 10.
    def __init__(self, master):
        self.master = master
        super().__init__(master)
        self.pack()
        
        self.paused = True
        self.setpoint = np.NaN
        self.pi = PIController(0.,0.,App.MAX_I)
        self.init_theta = np.NaN

        """ A TK Frame for holding several buttons, on the bottom of the window. """
        self.button_frame = tk.Frame(master=self)
        self.connect_button = tk.Button(master=self.button_frame,command=self.connect,text="CONNECT")
        self.connected = False
        self.connect_button.grid(row=0,column=0,sticky='s')
        self.update_button = tk.Button(master=self.button_frame,command=self.update,text="UPDATE")
        self.update_button.grid(row=0,column=1,sticky='s')
        self.start_button =  tk.Button(master=self.button_frame,command=self.start,text="START")
        self.start_button["state"]="disabled"
        self.start_button.grid(row=0,column=2,sticky='s')
        self.stop_button = tk.Button(master=self.button_frame,command=self.stop,text="STOP")
        self.stop_button["state"]="disabled"
        self.stop_button.grid(row=0,column=3,sticky='s')
        self.save_button = tk.Button(master=self.button_frame,command=self.save,text="SAVE")
        self.save_button["state"]="disabled"
        self.save_button.grid(row=0,column=4,sticky='s')
        self.button_frame.pack(side="bottom")

        """ A horizontal scale used to adjust the motor controller setpoint """
        self.m0_setpoint_scale = tk.Scale(master = self,label="M0 Setpoint",orient="horizontal")
        self.m0_setpoint_scale["from"] = -90
        self.m0_setpoint_scale["to"]= 90
        self.m0_setpoint_scale["resolution"] = 0.5
        self.m0_setpoint_scale.pack(side="bottom",fill="x")
        self.m1_setpoint_scale = tk.Scale(master = self,label="M1 Setpoint",orient="horizontal")
        self.m1_setpoint_scale["from"] = -90
        self.m1_setpoint_scale["to"]= 90
        self.m1_setpoint_scale["resolution"] = 0.5
        self.m1_setpoint_scale.pack(side="bottom",fill="x")

        """ A TK frame for holding several text labels and text entry fields, allowing the user
        to modify several different data.
        """
        self.conf_frame = tk.Frame(master=self)
        self.conf_frame.pack(side="left",fill="both")
        self.kp_label = tk.Label(master=self.conf_frame,text="Kp: ")
        self.kp_label.grid(row=0,column=0)
        self.kp_ent = tk.Entry(master=self.conf_frame)
        self.kp_ent.grid(row=0,column=1,columnspan=2)
        self.ki_label = tk.Label(master=self.conf_frame,text="Ki: ")
        self.ki_label.grid(row=1,column=0)
        self.ki_ent = tk.Entry(master=self.conf_frame)
        self.ki_ent.grid(row=1,column=1,columnspan=2)
        self.m0_sign_label = tk.Label(master=self.conf_frame,text="M0 sign:")
        self.m0_sign_label.grid(row=2,column=0)
        self.m0_sign_var = tk.IntVar()
        m0_btn1 = tk.Radiobutton(master=self.conf_frame,text="1",variable=self.m0_sign_var,value=1)
        m0_btn1.grid(row=2,column=1)
        m0_btn2 = tk.Radiobutton(master=self.conf_frame,text="-1",variable=self.m0_sign_var,value=-1)
        m0_btn2.grid(row=2,column=2)
        self.m1_sign_label = tk.Label(master=self.conf_frame,text="M1 sign:")
        self.m1_sign_label.grid(row=3,column=0)
        self.m1_sign_var = tk.IntVar()
        m1_btn1 = tk.Radiobutton(master=self.conf_frame,text="1",variable=self.m1_sign_var,value=1)
        m1_btn1.grid(row=3,column=1)
        m1_btn2 = tk.Radiobutton(master=self.conf_frame,text="-1",variable=self.m1_sign_var,value=-1)
        m1_btn2.grid(row=3,column=2)
        self.m0_target_label = tk.Label(master=self.conf_frame,text="M0 target: ")
        self.m0_target_label.grid(row=4,column=0)
        self.m0_target_ent = tk.Entry(master=self.conf_frame)
        self.m0_target_ent.grid(row=4,column=1,columnspan=2)
        self.m1_target_label = tk.Label(master=self.conf_frame,text="M1 target: ")
        self.m1_target_label.grid(row=5,column=0)
        self.m1_target_ent = tk.Entry(master=self.conf_frame)
        self.m1_target_ent.grid(row=5,column=1,columnspan=2)

        """ A TK Frame for holding several text labels that display motor telemetry for the user. """
        self.telemetry_frame= tk.Frame(master=self)
        self.telemetry_frame.pack(side="left",fill="both")
        self.m0_text = tk.Label(master=self.telemetry_frame,text="M0 angle: %.1f"%(np.NaN),anchor="w",width=16)
        self.m0_text.grid(row=0,column=0)
        self.I0_text = tk.Label(master=self.telemetry_frame,text="I0: %.1f"%(np.NaN),anchor="w",width=16)
        self.I0_text.grid(row=0,column=1)
        self.m1_text = tk.Label(master=self.telemetry_frame,text="M1 angle: %.1f"%(np.NaN),anchor="w",width=16)
        self.m1_text.grid(row=1,column=0)
        self.I1_text = tk.Label(master=self.telemetry_frame,text="I1: %.1f"%(np.NaN),anchor="w",width=16)
        self.I1_text.grid(row=1,column=1)
        self.avg_angle_text = tk.Label(master=self.telemetry_frame,text="(M0+M1)/2: %.1f"%(np.NaN),anchor="w",width=32)
        self.avg_angle_text.grid(row=2,column=0,columnspan=2)

        """ The following line schedules the function App.apploop to run in App.LOOP_DELAY_MS milliseconds in the future """
        self.after(App.LOOP_DELAY_MS,self.apploop)

    def update(self):
        """ This function is called when the update button is clicked, and takes
        data from the entry fields in the config frame, and converts them into numerical data
        """
        try:
            self.pi.Kp = float(self.kp_ent.get())
            self.pi.Ki = float(self.ki_ent.get())
        except BaseException as e:
            # a neat way to handle exceptions is to make a popup window which prints the error
            return tkmsgbox.showerror(title="Error in entry", message=str(e))

    def connect(self):
        """ This function is called when the connect button is pressed, it triggers the opening of a ComboboxDialog,
        (see above for the class definition of ComboboxDialog. """
        self.connected = False 
        self.odrive = None
        self.start_button["state"]="disabled"
        self.stop_button["state"]="disabled"
        self.save_button["state"]="disabled"
        dialog = ComboboxDialog(self.connect_callback)
    
    def connect_callback(self,device):
        """ This function is called by the ComboboxDialog after the user makes their selection."""
        try:
            self.odrive = OdriveDriver(device)
            self.odrive.set_torque_control_mode()
            self.odrive.arm()
            self.init_theta= np.array([self.m0_sign_var.get(),self.m1_sign_var.get()])*self.odrive.get_motor_angles()
            self.connected = True
            self.start_button["state"]="normal"
            self.stop_button["state"]="normal"
            self.save_button["state"]="normal"
        except BaseException as e:
            return tkmsgbox.showerror(title="Error in connection", message=str(e))

    def start(self):
        """ This is called when the start button is clicked. """
        self.paused = False

    def stop(self):
        """ This is called then the stop button is clicked. """
        self.paused = True

    def save(self):
        """ This is called when the save button is clicked, and it causes a filepicker to open to save
        data to the disk"""
        try:
            with tkfiledialog.asksaveasfile(mode="w") as file:
                thetas = np.array([self.m0_sign_var.get(),self.m1_sign_var.get()])*self.odrive.get_motor_angles()
                m0_target = float(self.m0_target_ent.get())*float(np.pi/180)
                m1_target = float(self.m1_target_ent.get())*float(np.pi/180)
                data = {"motor axis sign": [self.m0_sign_var.get(),self.m1_sign_var.get()],
                        "calibreation measurement": [float(thetas[0]),float(thetas[1])],
                        "calibration position": [m0_target,m1_target]}
                file.write(yaml.dump(data))

        except BaseException as e:
            return tkmsgbox.showerror(title="Error when saving file", message=str(e))
        
        
    def apploop(self):
        """ The apploop is a function which is called every App.LOOP_DELAY_MS, and does work of updating
        the telemetry text as well as some other things."""
        self.after(App.LOOP_DELAY_MS,self.apploop)
        dt = App.LOOP_DELAY_MS/1000.
        if not self.paused:
            thetas = np.array([self.m0_sign_var.get(),self.m1_sign_var.get()])*self.odrive.get_motor_angles()
            I = self.odrive.get_torques()
            m0_setpoint = self.m0_setpoint_scale.get()*(6.283/180)+self.init_theta[0]
            m0_setpoint = remainder(m0_setpoint,2*np.pi) 
            m1_setpoint = self.m1_setpoint_scale.get()*(6.283/180)+self.init_theta[1]
            m1_setpoint = remainder(m1_setpoint,2*np.pi) 
            error = np.array([remainder(m0_setpoint-thetas[0],2*np.pi),remainder(m1_setpoint-thetas[1],2*np.pi)])
            current_command = self.pi.update(error*180/6.283,dt)*np.array([self.m0_sign_var.get(),self.m1_sign_var.get()])
            print(current_command)
            self.odrive.set_torques(*current_command)
            self.m0_text["text"]="M0 angle: %.1f" % (thetas[0]*180/np.pi)
            self.m1_text["text"]="M1 angle: %.1f" % (thetas[1]*180/np.pi)
            self.I0_text["text"]="I0: %.1f" % (I[0])
            self.I1_text["text"]="I1: %.1f" % (I[1])
            self.avg_angle_text["text"]="(M0+M1)/2: %.1f" % ((.5*thetas[0]+.5*thetas[1])*180/np.pi)

if __name__ == "__main__":
    # program entry point
    root = tk.Tk() # gets handle of top-level TK instance (idk what this actually does)
    app = App(root)# constructs the App class (see definition above)
    app.mainloop() # starts the TK event loop (necessary for anything to run)

