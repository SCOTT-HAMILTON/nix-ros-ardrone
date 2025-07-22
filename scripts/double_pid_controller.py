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

class State:
    def __init__(self, x, y, z, vx, vy, vz):
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz

# --- Shared Data Structure ---
class SharedData:
    def __init__(self):
        self.target_state = State(0, 0, 0, 0, 0, 0)
        self.current_state = State(0, 0, 0, 0, 0, 0)
        self.manual_control = False
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.pid_output_limits = (-1.0, 1.0)
        self.battery_percent = -1
        self.lock = threading.Lock() # For protecting access to shared data
        self.target_pos_joy_axis = 1
        self.target_pos_joy_axis = 1

# --- GUI Application Class (runs in its own thread) ---
class GuiApp:
    def __init__(self, root, shared_data_instance):
        self.root = root
        self.shared_data = shared_data_instance

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.title("Double PID Tuner")

        # Create StringVars
        self.kp_str = tk.StringVar(value="")
        self.ki_str = tk.StringVar(value="")
        self.kd_str = tk.StringVar(value="")
        self.target_speed_str = tk.StringVar(value="")
        self.target_pos_str = tk.StringVar(value="")
        self.current_pos_str = tk.StringVar(value="")
        self.current_speed_str = tk.StringVar(value="")
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
        self.kp_scale = ttk.Scale(frame, from_=0.0, to=7e-2, orient=tk.HORIZONTAL,
                                  command=self._update_pid_gains_from_gui,
                                  length=200)
        self.kp_scale.grid(row=0, column=1, sticky=(tk.W, tk.E))
        self.kp_value_label = ttk.Label(frame, textvariable=self.kp_str) # Initialize with empty text
        self.kp_value_label.grid(row=0, column=2, sticky=tk.W)

        # Ki
        ttk.Label(frame, text="Ki:").grid(row=1, column=0, sticky=tk.W)
        self.ki_scale = ttk.Scale(frame, from_=0.0, to=2e-2, orient=tk.HORIZONTAL,
                                  command=self._update_pid_gains_from_gui,
                                  length=200)
        self.ki_scale.grid(row=1, column=1, sticky=(tk.W, tk.E))
        self.ki_value_label = ttk.Label(frame, textvariable=self.ki_str)
        self.ki_value_label.grid(row=1, column=2, sticky=tk.W)

        # Kd
        ttk.Label(frame, text="Kd:").grid(row=2, column=0, sticky=tk.W)
        self.kd_scale = ttk.Scale(frame, from_=0.0, to=0.0008, orient=tk.HORIZONTAL,
                                  command=self._update_pid_gains_from_gui,
                                  length=200)
        self.kd_scale.grid(row=2, column=1, sticky=(tk.W, tk.E))
        self.kd_value_label = ttk.Label(frame, textvariable=self.kd_str)
        self.kd_value_label.grid(row=2, column=2, sticky=tk.W)

        # Target speed Display
        ttk.Label(frame, text="Target Speed [m/s]:").grid(row=3, column=0, sticky=tk.W)
        self.target_speed_label = ttk.Label(frame, textvariable=self.target_speed_str)
        self.target_speed_label.grid(row=3, column=1, columnspan=2, sticky=tk.W)

        # Current Speed Display
        ttk.Label(frame, text="Current Speed [m/s]:").grid(row=4, column=0, sticky=tk.W)
        self.current_speed_label = ttk.Label(frame, textvariable=self.current_speed_str)
        self.current_speed_label.grid(row=4, column=1, columnspan=2, sticky=tk.W)


        # Target Pos Display
        ttk.Label(frame, text="Target Pos [mm]:").grid(row=5, column=0, sticky=tk.W)
        self.target_pos_label = ttk.Label(frame, textvariable=self.target_pos_str)
        self.target_pos_label.grid(row=5, column=1, columnspan=2, sticky=tk.W)

        # Current Pos Display
        ttk.Label(frame, text="Current Pos [mm]:").grid(row=6, column=0, sticky=tk.W)
        self.current_pos_label = ttk.Label(frame, textvariable=self.current_pos_str)
        self.current_pos_label.grid(row=6, column=1, columnspan=2, sticky=tk.W)

        # Manual Control Status
        ttk.Label(frame, text="Manual Control:").grid(row=7, column=0, sticky=tk.W)
        self.manual_control_label = ttk.Label(frame, textvariable=self.manual_control_str)
        self.manual_control_label.grid(row=7, column=1, columnspan=2, sticky=tk.W)

        # Battery Indicator
        ttk.Label(frame, text="Battery:").grid(row=8, column=0, sticky=tk.W)
        self.battery_label = ttk.Label(frame, textvariable=self.battery_percent_str)
        self.battery_label.grid(row=8, column=1, columnspan=2, sticky=tk.W)

    def _update_pid_gains_from_gui(self, event=None):
        """Updates shared PID gains from GUI sliders and updates labels."""
        with self.shared_data.lock:
            self.shared_data.Kp = self.kp_scale.get()
            self.shared_data.Ki = self.ki_scale.get()
            self.shared_data.Kd = self.kd_scale.get()
        # Update StringVars
        self.kp_str.set(f"{self.shared_data.Kp:.3e}")
        self.ki_str.set(f"{self.shared_data.Ki:.3e}")
        self.kd_str.set(f"{self.shared_data.Kd:.3e}")

    def _periodic_gui_update(self):
        """
        Reads data from shared_data and updates GUI labels via StringVars.
        This function also schedules its next call.
        """
        # print("GUI: Performing periodic update...") # Debugging log

        with self.shared_data.lock:
            target_speed = self.shared_data.target_state.vx
            current_speed = self.shared_data.current_state.vx
            target_pos = self.shared_data.target_state.x
            current_pos = self.shared_data.current_state.x
            manual_ctrl = self.shared_data.manual_control
            kp = self.shared_data.Kp
            ki = self.shared_data.Ki
            kd = self.shared_data.Kd
            battery_percent = self.shared_data.battery_percent

        # Update StringVars, which in turn update the labels
        self.target_speed_str.set(f"{target_speed:.2f}")
        self.current_speed_str.set(f"{current_speed:.2f}")
        self.target_pos_str.set(f"{target_pos:.2f}")
        self.current_pos_str.set(f"{current_pos:.2f}")
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
            self.pid_vx = PID(Kp=3.855e-4, Ki=2.627e-4, Kd=0.0,
                           output_limits=self.shared_data.pid_output_limits,
                           time_fn=rospy.get_time)
            self.pid_vy = PID(Kp=3.855e-4, Ki=2.627e-4, Kd=0.0,
                           output_limits=self.shared_data.pid_output_limits,
                           time_fn=rospy.get_time)
            self.pid_x = PID(Kp=self.shared_data.Kp, Ki=self.shared_data.Ki, Kd=self.shared_data.Kd,
                           output_limits=(-600, 600),
                           time_fn=rospy.get_time)
            self.pid_y = PID(Kp=self.shared_data.Kp, Ki=self.shared_data.Ki, Kd=self.shared_data.Kd,
                           output_limits=(-600, 600),
                           time_fn=rospy.get_time)
            self.pid_z = PID(Kp=0.012, Ki=0.010, Kd=0.007,
                           output_limits=self.shared_data.pid_output_limits,
                           time_fn=rospy.get_time)


        # ROS subscribers and publishers
        self.navdata_sub = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdata_callback)
        self.odometry_sub = rospy.Subscriber('/ardrone/odometry', Odometry, self.odometry_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_scaled_pub = rospy.Publisher('/cmd_vel_scaled', Twist, queue_size=1)

        self.cmd_pos_sub = rospy.Subscriber('/cmd_pos', Twist, self.cmd_pos_callback)

        # Set up a ROS Timer for the control loop
        self.control_timer = rospy.Timer(rospy.Duration(1.0/50.0), self.control_loop_timer_callback)

        rospy.loginfo("Drone PID Controller ROS components initialized.")

    def cmd_pos_callback(self, data):
        if rospy.is_shutdown():
            rospy.loginfo("ROS is shutting down, skipping control loop.")
            return # Exit early if ROS is shutting down
        with self.shared_data.lock:
            self.shared_data.target_state.x = data.linear.x
            self.shared_data.target_state.y = data.linear.y

    def navdata_callback(self, data):
        """Update current altitude measurement and share with GUI."""
        if rospy.is_shutdown():
            rospy.loginfo("ROS is shutting down, skipping control loop.")
            return # Exit early if ROS is shutting down
        with self.shared_data.lock:
            self.shared_data.current_state.vx = data.vx
            self.shared_data.current_state.vy = data.vy
            self.shared_data.current_state.vz = data.vz
            self.shared_data.current_state.z = data.altd
            self.shared_data.battery_percent = data.batteryPercent


    def odometry_callback(self, data):
        """Update current altitude measurement from odometry."""
        if rospy.is_shutdown():
            return

        with self.shared_data.lock:
            # Odometry pos is usually in meters, convert to mm for consistency
            self.shared_data.current_state.x = data.pose.pose.position.x*1000
            self.shared_data.current_state.y = data.pose.pose.position.y*1000
            # self.shared_data.current_state.z = data.pose.pose.position.z

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
                    rospy.loginfo("ROS joystick button 8 TOGGLE !")
                elif data.buttons[8] == 0 and self.button_toggle_alt_source_pressed:
                    self.button_toggle_alt_source_pressed = False

            if len(data.axes) > 2:
                self.shared_data.target_state.z = int((data.axes[2]+1) * 1000)

    def control_loop_timer_callback(self, event):
        """Main control loop, called periodically by rospy.Timer."""

        if rospy.is_shutdown():
            rospy.loginfo("ROS is shutting down, skipping control loop.")
            return # Exit early if ROS is shutting down

        # Get latest PID gains and control variables from shared data
        with self.shared_data.lock:
            self.pid_x.Kp = self.shared_data.Kp
            self.pid_x.Ki = self.shared_data.Ki
            self.pid_x.Kd = self.shared_data.Kd

            self.pid_y.Kp = self.shared_data.Kp
            self.pid_y.Ki = self.shared_data.Ki
            self.pid_y.Kd = self.shared_data.Kd

            manual_control = self.shared_data.manual_control

            target_z = self.shared_data.target_state.z
            current_z = self.shared_data.current_state.z

            target_x = self.shared_data.target_state.x
            current_x = self.shared_data.current_state.x
            target_vx = self.shared_data.target_state.vx
            current_vx = self.shared_data.current_state.vx

            target_y = self.shared_data.target_state.y
            current_y = self.shared_data.current_state.y
            target_vy = self.shared_data.target_state.vy
            current_vy = self.shared_data.current_state.vy

        twist = Twist()
        twist_scaled = Twist()

        if not manual_control:
            self.pid_x.setpoint = target_x
            cmd_vx = self.pid_x(current_x)

            self.pid_vx.setpoint = cmd_vx
            cmd_x = self.pid_vx(current_vx)

            self.pid_y.setpoint = target_y
            cmd_vy = self.pid_y(current_y)

            self.pid_vy.setpoint = cmd_vy
            cmd_y = self.pid_vy(current_vy)

            self.pid_z.setpoint = target_z
            cmd_z = self.pid_z(current_z)

            twist.linear.x = float(cmd_x)
            twist.linear.y = float(cmd_y)
            twist.linear.z = float(cmd_z)


            twist_scaled.linear.x = float(cmd_x)*1000
            twist_scaled.linear.y = float(cmd_y)*1000
            twist_scaled.linear.z = float(cmd_z)*1000

            twist_scaled.angular.x = float(cmd_vx)
            twist_scaled.angular.y = float(cmd_vy)

            rospy.loginfo_throttle(1, f"Pos x PID: Target={target_x:.2f} mm, Current={current_x:.2f} mm, Command={float(cmd_vx):.3f}")
            rospy.loginfo_throttle(1, f"Speed vx PID: Target={target_vx:.2f} m/s, Current={current_vx:.2f} mm/s, Command={float(cmd_x):.3f}")
            self.cmd_vel_pub.publish(twist)
            self.cmd_scaled_pub.publish(twist_scaled)
            with self.shared_data.lock:
                self.shared_data.target_state.vx = cmd_vx
        else:
            rospy.loginfo_throttle(1, "Manual Control Mode: PID Reset.")
            self.pid_vx.reset()
            self.pid_vy.reset()
            self.pid_z.reset()


# --- Main execution block ---
if __name__ == '__main__':
    # Create a shared data object
    shared_data = SharedData()

    # Create an event to signal when the GUI thread is ready
    gui_ready_event = threading.Event()

    # --- Initialize shared data with default values BEFORE starting threads ---
    with shared_data.lock:
        shared_data.Kp = 3.855e-2
        shared_data.Ki = 2e-2
        shared_data.Kd = 0.0
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
