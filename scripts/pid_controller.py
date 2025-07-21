#!/usr/bin/env mypython

import rospy
from sensor_msgs.msg import Joy
from ardrone_autonomy.msg import Navdata
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from simple_pid import PID
import threading
import tkinter as tk
from tkinter import ttk
import time

# --- Shared Data Structure ---
class SharedData:
    def __init__(self):
        self.navdata_altitude = 0
        self.odometry_altitude = 0.0
        self.active_altitude_source = "navdata" # one of "navdata" and "odometry"
        self.target_altitude = 0
        self.manual_control = False
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.pid_output_limits = (-1.0, 1.0)
        self.battery_percent = -1
        self.lock = threading.Lock() # For protecting access to shared data

# --- GUI Application Class (runs in its own thread) ---
class GuiApp:
    def __init__(self, root, shared_data_instance):
        self.root = root
        self.shared_data = shared_data_instance

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.title("PID Tuner")

        # Create StringVars
        self.kp_str = tk.StringVar(value="")
        self.ki_str = tk.StringVar(value="")
        self.kd_str = tk.StringVar(value="")
        self.target_alt_str = tk.StringVar(value="")
        self.current_alt_str = tk.StringVar(value="")
        self.manual_control_str = tk.StringVar(value="")
        self.battery_percent_str = tk.StringVar(value="")

        self.setup_gui()

        # Initial GUI update and start the periodic scheduling
        # Call it directly once, then it will reschedule itself
        self._periodic_gui_update() # Start the loop here

    def on_closing(self):
        """Called when the Tkinter window is closed."""
        rospy.signal_shutdown("GUI window closed.")
        self.root.destroy()

    def setup_gui(self):
        """Sets up the Tkinter GUI elements."""
        frame = ttk.Frame(self.root, padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Kp
        ttk.Label(frame, text="Kp:").grid(row=0, column=0, sticky=tk.W)
        self.kp_scale = ttk.Scale(frame, from_=0.0, to=0.5, orient=tk.HORIZONTAL,
                                  command=self._update_pid_gains_from_gui,
                                  length=200)
        self.kp_scale.grid(row=0, column=1, sticky=(tk.W, tk.E))
        self.kp_value_label = ttk.Label(frame, textvariable=self.kp_str) # Initialize with empty text
        self.kp_value_label.grid(row=0, column=2, sticky=tk.W)

        # Ki
        ttk.Label(frame, text="Ki:").grid(row=1, column=0, sticky=tk.W)
        self.ki_scale = ttk.Scale(frame, from_=0.0, to=0.1, orient=tk.HORIZONTAL,
                                  command=self._update_pid_gains_from_gui,
                                  length=200)
        self.ki_scale.grid(row=1, column=1, sticky=(tk.W, tk.E))
        self.ki_value_label = ttk.Label(frame, textvariable=self.ki_str)
        self.ki_value_label.grid(row=1, column=2, sticky=tk.W)

        # Kd
        ttk.Label(frame, text="Kd:").grid(row=2, column=0, sticky=tk.W)
        self.kd_scale = ttk.Scale(frame, from_=0.0, to=0.2, orient=tk.HORIZONTAL,
                                  command=self._update_pid_gains_from_gui,
                                  length=200)
        self.kd_scale.grid(row=2, column=1, sticky=(tk.W, tk.E))
        self.kd_value_label = ttk.Label(frame, textvariable=self.kd_str)
        self.kd_value_label.grid(row=2, column=2, sticky=tk.W)

        # Target Altitude Display
        ttk.Label(frame, text="Target Alt (mm):").grid(row=3, column=0, sticky=tk.W)
        self.target_alt_label = ttk.Label(frame, textvariable=self.target_alt_str)
        self.target_alt_label.grid(row=3, column=1, columnspan=2, sticky=tk.W)

        # Current Altitude Display
        ttk.Label(frame, text="Current Alt (mm):").grid(row=4, column=0, sticky=tk.W)
        self.current_alt_label = ttk.Label(frame, textvariable=self.current_alt_str)
        self.current_alt_label.grid(row=4, column=1, columnspan=2, sticky=tk.W)

        # Manual Control Status
        ttk.Label(frame, text="Manual Control:").grid(row=5, column=0, sticky=tk.W)
        self.manual_control_label = ttk.Label(frame, textvariable=self.manual_control_str)
        self.manual_control_label.grid(row=5, column=1, columnspan=2, sticky=tk.W)

        # Battery Indicator
        ttk.Label(frame, text="Battery:").grid(row=6, column=0, sticky=tk.W)
        self.battery_label = ttk.Label(frame, textvariable=self.battery_percent_str)
        self.battery_label.grid(row=6, column=1, columnspan=2, sticky=tk.W)

    def _update_pid_gains_from_gui(self, event=None):
        """Updates shared PID gains from GUI sliders and updates labels."""
        with self.shared_data.lock:
            self.shared_data.Kp = self.kp_scale.get()
            self.shared_data.Ki = self.ki_scale.get()
            self.shared_data.Kd = self.kd_scale.get()
        # Update StringVars
        self.kp_str.set(f"{self.shared_data.Kp:.3f}")
        self.ki_str.set(f"{self.shared_data.Ki:.3f}")
        self.kd_str.set(f"{self.shared_data.Kd:.3f}")

    def _periodic_gui_update(self):
        """
        Reads data from shared_data and updates GUI labels via StringVars.
        This function also schedules its next call.
        """
        # print("GUI: Performing periodic update...") # Debugging log

        with self.shared_data.lock:
            target_alt = self.shared_data.target_altitude
            navdata_alt = self.shared_data.navdata_altitude
            odometry_alt = self.shared_data.odometry_altitude
            manual_ctrl = self.shared_data.manual_control
            kp = self.shared_data.Kp
            ki = self.shared_data.Ki
            kd = self.shared_data.Kd
            battery_percent = self.shared_data.battery_percent
            active_source = self.shared_data.active_altitude_source

        # Update StringVars, which in turn update the labels
        self.kp_str.set(f"{kp:.3f}")
        self.ki_str.set(f"{ki:.3f}")
        self.kd_str.set(f"{kd:.3f}")
        self.target_alt_str.set(f"{target_alt}")


        if active_source == "odometry":
            self.current_alt_str.set(f"{odometry_alt:.0f} [Odometry]")
        else: # Default is Navdata
            self.current_alt_str.set(f"{navdata_alt:.0f} [Navdata]")
        self.manual_control_str.set(f"{manual_ctrl}")
        self.battery_percent_str.set(f"{battery_percent:.1f}")

        # Ensure scales are also updated from shared data, in case ROS changed them
        if hasattr(self, 'kp_scale'): # This check is good practice
            self.kp_scale.set(kp)
            self.ki_scale.set(ki)
            self.kd_scale.set(kd)

        # --- Crucially, schedule the NEXT call to *this same method* ---
        self.root.after(100, self._periodic_gui_update)


# --- Function to run in the GUI thread ---
def run_gui(shared_data_instance, gui_ready_event):
    """Function to be run in the GUI thread."""
    root = tk.Tk()
    app = GuiApp(root, shared_data_instance)
    gui_ready_event.set() # Signal that the GUI is ready and mainloop is about to start
    root.mainloop()


# --- ROS Drone PID Controller Class (runs in main ROS thread) ---
class DronePidController:
    def __init__(self, shared_data_instance, gui_ready_event_):
        rospy.init_node('drone_pid_controller')
        self.shared_data = shared_data_instance
        self.button_toggle_alt_source_pressed = False
        self.gui_ready_event = gui_ready_event_

        # Wait for GUI to signal it's ready before proceeding with critical ROS ops
        rospy.loginfo("Waiting for GUI to initialize...")
        self.gui_ready_event.wait(timeout=5.0) # Wait up to 5 seconds for GUI
        if not self.gui_ready_event.is_set():
            rospy.logwarn("GUI did not initialize in time. Continuing without full GUI sync.")
        else:
            rospy.loginfo("GUI initialized. Proceeding with ROS node setup.")

        # Initialize PID controller with default gains and output limits
        # These are set in __main__ before threads start, so we just read them.
        with self.shared_data.lock:
            self.pid = PID(Kp=self.shared_data.Kp, Ki=self.shared_data.Ki, Kd=self.shared_data.Kd,
                           output_limits=self.shared_data.pid_output_limits,
                           time_fn=rospy.get_time)

        # ROS subscribers and publishers
        self.navdata_sub = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdata_callback)
        self.odometry_sub = rospy.Subscriber('/ardrone/odometry', Odometry, self.odometry_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Set up a ROS Timer for the control loop
        self.control_timer = rospy.Timer(rospy.Duration(1.0/50.0), self.control_loop_timer_callback)

        rospy.loginfo("Drone PID Controller ROS components initialized.")

    def navdata_callback(self, data):
        """Update current altitude measurement and share with GUI."""
        if rospy.is_shutdown():
            rospy.loginfo("ROS is shutting down, skipping control loop.")
            return # Exit early if ROS is shutting down
        with self.shared_data.lock:
            self.shared_data.navdata_altitude = data.altd
            self.shared_data.battery_percent = data.batteryPercent


    def odometry_callback(self, data):
        """Update current altitude measurement from odometry."""
        if rospy.is_shutdown():
            return

        with self.shared_data.lock:
            # Odometry Z is usually in meters, convert to mm for consistency with navdata.altd
            self.shared_data.odometry_altitude = data.pose.pose.position.z * 1000.0

    def joy_callback(self, data):
        """Handle joystick input and share with GUI."""
        if rospy.is_shutdown():
            rospy.loginfo("ROS is shutting down, skipping control loop.")
            return # Exit early if ROS is shutting down
        with self.shared_data.lock:
            if len(data.buttons) > 0:
                self.shared_data.manual_control = (data.buttons[0] == 1)

            if len(data.buttons) > 8:
                if data.buttons[8] == 1 and not self.button_toggle_alt_source_pressed:
                    self.button_toggle_alt_source_pressed = True
                    self.shared_data.active_altitude_source = "navdata" if self.shared_data.active_altitude_source == "odometry" else "odometry"
                elif data.buttons[8] == 0 and self.button_toggle_alt_source_pressed:
                    self.button_toggle_alt_source_pressed = False

            if len(data.axes) > 2:
                self.shared_data.target_altitude = int((data.axes[2] + 1) * 1000)
                if self.shared_data.target_altitude < 0:
                    self.shared_data.target_altitude = 0

    def control_loop_timer_callback(self, event):
        """Main control loop, called periodically by rospy.Timer."""

        if rospy.is_shutdown():
            rospy.loginfo("ROS is shutting down, skipping control loop.")
            return # Exit early if ROS is shutting down

        # Get latest PID gains and control variables from shared data
        with self.shared_data.lock:
            self.pid.Kp = self.shared_data.Kp
            self.pid.Ki = self.shared_data.Ki
            self.pid.Kd = self.shared_data.Kd
            manual_control = self.shared_data.manual_control
            target_altitude = self.shared_data.target_altitude
            current_altitude = self.shared_data.odometry_altitude if self.shared_data.active_altitude_source == "odometry" else self.shared_data.navdata_altitude

        twist = Twist()

        if not manual_control:
            self.pid.setpoint = target_altitude
            cmd = self.pid(current_altitude)

            twist.linear.z = float(cmd)

            rospy.loginfo_throttle(1, f"PID Mode: Target={target_altitude:.0f}mm, Current={current_altitude:.0f}mm, Command={twist.linear.z:.3f}")
            self.cmd_vel_pub.publish(twist)
        # else:
        #     rospy.loginfo_throttle(1, "Manual Control Mode: PID Reset.")
        #     self.pid.reset()
        #     twist.linear.z = 0.0
        #     self.cmd_vel_pub.publish(twist)


# --- Main execution block ---
if __name__ == '__main__':
    # Create a shared data object
    shared_data = SharedData()

    # Create an event to signal when the GUI thread is ready
    gui_ready_event = threading.Event()

    # --- Initialize shared data with default values BEFORE starting threads ---
    with shared_data.lock:
        shared_data.Kp = 0.012
        shared_data.Ki = 0.022
        shared_data.Kd = 0.025
        shared_data.pid_output_limits = (-1.0, 1.0)

    # Start the GUI in its own thread, passing the event
    gui_thread = threading.Thread(target=run_gui, args=(shared_data, gui_ready_event))
    gui_thread.daemon = True
    gui_thread.start()

    try:
        # Initialize the ROS controller in the main thread, passing the event
        controller = DronePidController(shared_data, gui_ready_event)
        print("Spinning...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Drone PID Controller node interrupted.")
    finally:
        if gui_thread.is_alive():
            rospy.loginfo("Waiting for GUI thread to finish...")
            gui_thread.join(timeout=1.0)
        rospy.loginfo("Program exited.")
