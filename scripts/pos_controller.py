#!/usr/bin/env mypython

import tkinter as tk
from tkinter import ttk
import rospy
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry # Import Odometry message type
from geometry_msgs.msg import Twist
import queue
import threading

# Define the ROS coordinate limits
ROS_MIN_COORD = -6000.0
ROS_MAX_COORD = 6000.0
ROS_RANGE = ROS_MAX_COORD - ROS_MIN_COORD

class ROSPositionGUI:
    def __init__(self, master):
        self.master = master
        master.title("ROS Position Publisher")
        master.geometry("800x850") # Slightly larger window to accommodate controls

        # Initialize ROS node and publisher
        try:
            rospy.init_node('position_publisher_gui_node', anonymous=True)
            self.publisher = rospy.Publisher('/cmd_pos', Twist, queue_size=10)
            # Subscribe to odometry topic to get current position for reset and visualization
            self.odometry_sub = rospy.Subscriber('/ardrone/odometry', Odometry, self.odometry_callback)
            self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
            self.odometry_x = 0.0 # Initialize odometry x
            self.odometry_y = 0.0 # Initialize odometry y
            self.button_joy_reset_pressed = False
            rospy.loginfo("ROS node 'position_publisher_gui_node' initialized.")
            rospy.loginfo("Publisher created for topic '/cmd_pos'.")
            rospy.loginfo("Subscriber created for topic '/ardrone/odometry'.")


            # Start rospy.spin() in a separate thread to handle ROS callbacks
            # This allows the Tkinter main loop to run without being blocked by ROS
            self.ros_thread = threading.Thread(target=rospy.spin, daemon=True)
            self.ros_thread.start()
            rospy.loginfo("rospy.spin() started in a separate thread.")

            # Add a rospy.Timer to publish the current position periodically
            # The timer will call _periodic_publish every 0.5 seconds
            self.publish_timer = rospy.Timer(rospy.Duration(0.5), self._periodic_publish)
            rospy.loginfo("Periodic publishing timer started (every 0.5 seconds).")

        except rospy.ROSInitException as e:
            rospy.logerr(f"Failed to initialize ROS node: {e}")
            self.publisher = None # Handle case where ROS isn't running
            self.ros_thread = None
            self.publish_timer = None # Ensure timer is not created if ROS fails
            self._show_message("ROS Error", f"Failed to initialize ROS: {e}. GUI will run, but no messages will be published.", "red")

        self.current_x = 0.0
        self.current_y = 0.0
        self.target_point_id = None # To store the canvas item ID for the target point (red)
        self.odometry_point_id = None # To store the canvas item ID for the odometry point (green)

        # Queue for thread-safe GUI updates
        self.gui_update_queue = queue.Queue()

        self._create_widgets()
        self._draw_grid()
        self._draw_axes()
        self._draw_target_point() # Draw initial target point
        self._draw_odometry_point() # Draw initial odometry point

        # Start a periodic check for GUI updates from the queue
        self.master.after(100, self._process_gui_queue)

        # Handle window closing
        master.protocol("WM_DELETE_WINDOW", self._on_close)

    def odometry_callback(self, data):
        """Update current position measurement from odometry."""
        if rospy.is_shutdown():
            return

        # Odometry pos is usually in meters, convert to mm for consistency
        self.odometry_x = data.pose.pose.position.x * 1000
        self.odometry_y = data.pose.pose.position.y * 1000

        # Schedule GUI update for odometry point on the main Tkinter thread
        self.gui_update_queue.put(('odometry_update', self.odometry_x, self.odometry_y))

    def joy_callback(self, data):
        """Handle joystick input and share with GUI."""
        if rospy.is_shutdown():
            rospy.loginfo("ROS is shutting down, skipping control loop.")
            return # Exit early if ROS is shutting down

        if len(data.buttons) > 2:
            if data.buttons[2] == 1 and not self.button_joy_reset_pressed:
                self.button_joy_reset_pressed = True
                self._reset_position()
                rospy.loginfo("ROS joystick button 2 RESET POS to odometry !")
            elif data.buttons[2] == 0 and self.button_joy_reset_pressed:
                self.button_joy_reset_pressed = False

    def _create_widgets(self):
        # Input Frame
        input_frame = ttk.LabelFrame(self.master, text="Current Target Position", padding="10 10")
        input_frame.pack(pady=10, padx=10, fill="x")

        ttk.Label(input_frame, text="X Coordinate:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        # Make entries read-only as the target is set by clicking the canvas
        self.x_entry = ttk.Entry(input_frame, width=20, state='readonly')
        self.x_entry.insert(0, str(self.current_x))
        self.x_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        ttk.Label(input_frame, text="Y Coordinate:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.y_entry = ttk.Entry(input_frame, width=20, state='readonly')
        self.y_entry.insert(0, str(self.current_y))
        self.y_entry.grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        # Add the Reset button
        reset_button = ttk.Button(input_frame, text="Reset to Odometry", command=self._reset_position)
        reset_button.grid(row=0, column=2, rowspan=2, padx=10, pady=5, sticky="nsew")


        input_frame.grid_columnconfigure(1, weight=1) # Allow entry fields to expand

        # Canvas Frame
        canvas_frame = ttk.Frame(self.master, relief="sunken", borderwidth=2)
        canvas_frame.pack(pady=10, padx=10, fill="both", expand=True)

        self.canvas_width = 600
        self.canvas_height = 600
        self.canvas = tk.Canvas(canvas_frame, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.pack(fill="both", expand=True)

        # Bind click event to canvas for direct target setting
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        # Status Message Box
        self.message_label = ttk.Label(self.master, text="", foreground="blue", wraplength=780)
        self.message_label.pack(pady=5, padx=10, fill="x")

    def _show_message(self, title, message, color="blue"):
        """Displays a message in the status label."""
        self.message_label.config(text=f"{title}: {message}", foreground=color)

    def _ros_to_canvas(self, ros_x, ros_y):
        """
        Converts ROS coordinates [-1000, 1000] to canvas pixel coordinates.
        X-axis is vertical (positive up), Y-axis is horizontal (positive left).
        """
        # Scale factors for mapping ROS range to canvas dimensions
        scale_x_to_canvas_y = self.canvas_height / ROS_RANGE
        scale_y_to_canvas_x = self.canvas_width / ROS_RANGE

        # ROS X (vertical) maps to Canvas Y (inverted: positive ROS X is up, so smaller canvas Y)
        canvas_y = self.canvas_height - ((ros_x - ROS_MIN_COORD) * scale_x_to_canvas_y)
        # ROS Y (horizontal) maps to Canvas X (inverted: positive ROS Y is left, so smaller canvas X)
        canvas_x = self.canvas_width - ((ros_y - ROS_MIN_COORD) * scale_y_to_canvas_x)
        return canvas_x, canvas_y

    def _canvas_to_ros(self, canvas_x, canvas_y):
        """
        Converts canvas pixel coordinates to ROS coordinates [-1000, 1000].
        X-axis is vertical (positive up), Y-axis is horizontal (positive left).
        """
        # Scale factors for mapping canvas dimensions to ROS range
        scale_canvas_y_to_ros_x = ROS_RANGE / self.canvas_height
        scale_canvas_x_to_ros_y = ROS_RANGE / self.canvas_width

        # Canvas Y (inverted) maps to ROS X
        ros_x = ((self.canvas_height - canvas_y) * scale_canvas_y_to_ros_x) + ROS_MIN_COORD
        # Canvas X (inverted) maps to ROS Y
        ros_y = ((self.canvas_width - canvas_x) * scale_canvas_x_to_ros_y) + ROS_MIN_COORD
        return ros_x, ros_y

    def _draw_grid(self):
        # Clear existing grid lines (if any)
        self.canvas.delete("grid_line")

        # Draw horizontal grid lines (for ROS Y values - these are vertical lines on canvas)
        for i in range(int(ROS_MIN_COORD), int(ROS_MAX_COORD) + 1, 100):
            if i == 0: continue # Skip 0, will be drawn as axis
            canvas_x_pos = self._ros_to_canvas(0, i)[0] # Get canvas X for this ROS Y
            self.canvas.create_line(canvas_x_pos, 0, canvas_x_pos, self.canvas_height,
                                    fill="#e0e0e0", tags="grid_line")
            # Label for Y-axis grid lines (at the bottom of the canvas)
            self.canvas.create_text(canvas_x_pos, self.canvas_height - 10,
                                    text=str(i), anchor="s", fill="#888888", tags="grid_line")

        # Draw vertical grid lines (for ROS X values - these are horizontal lines on canvas)
        for i in range(int(ROS_MIN_COORD), int(ROS_MAX_COORD) + 1, 100):
            if i == 0: continue # Skip 0, will be drawn as axis
            canvas_y_pos = self._ros_to_canvas(i, 0)[1] # Get canvas Y for this ROS X
            self.canvas.create_line(0, canvas_y_pos, self.canvas_width, canvas_y_pos,
                                    fill="#e0e0e0", tags="grid_line")
            # Label for X-axis grid lines (at the left of the canvas)
            self.canvas.create_text(10, canvas_y_pos,
                                    text=str(i), anchor="w", fill="#888888", tags="grid_line")


    def _draw_axes(self):
        # Clear existing axes (if any)
        self.canvas.delete("axes")

        # Get canvas coordinates for the origin (0,0)
        center_x, center_y = self._ros_to_canvas(0, 0)

        # Draw Y-axis (horizontal line through center_y)
        self.canvas.create_line(0, center_y, self.canvas_width, center_y,
                                fill="black", width=2, tags="axes")
        # Label for Y-axis (at the right end, centered vertically on the line)
        self.canvas.create_text(self.canvas_width - 10, center_y, text="Y", anchor="e", fill="black", tags="axes")

        # Draw X-axis (vertical line through center_x)
        self.canvas.create_line(center_x, 0, center_x, self.canvas_height,
                                fill="black", width=2, tags="axes")
        # Label for X-axis (at the top end, centered horizontally on the line)
        self.canvas.create_text(center_x, 10, text="X", anchor="n", fill="black", tags="axes")

        # Draw origin label
        self.canvas.create_text(center_x + 10, center_y + 10, text="(0,0)", anchor="nw", fill="black", tags="axes")

    def _draw_target_point(self):
        # Delete previous target point if it exists
        if self.target_point_id:
            self.canvas.delete(self.target_point_id)
            self.canvas.delete("target_label") # Delete old label

        # Convert current ROS coordinates to canvas coordinates
        canvas_x, canvas_y = self._ros_to_canvas(self.current_x, self.current_y)

        # Draw the new target point (a small circle)
        point_radius = 5
        self.target_point_id = self.canvas.create_oval(
            canvas_x - point_radius, canvas_y - point_radius,
            canvas_x + point_radius, canvas_y + point_radius,
            fill="red", outline="darkred"
        )
        # Add text label for the point
        self.canvas.create_text(canvas_x, canvas_y - 15,
                                text=f"({self.current_x:.1f}, {self.current_y:.1f})",
                                fill="red", tags="target_label")
        self.canvas.tag_raise(self.target_point_id) # Ensure point is on top
        self.canvas.tag_raise("target_label") # Ensure label is on top

    def _draw_odometry_point(self):
        # Delete previous odometry point if it exists
        if self.odometry_point_id:
            self.canvas.delete(self.odometry_point_id)
            self.canvas.delete("odometry_label") # Delete old label

        # Convert odometry ROS coordinates to canvas coordinates
        canvas_x, canvas_y = self._ros_to_canvas(self.odometry_x, self.odometry_y)

        # Draw the new odometry point (a small circle)
        point_radius = 5
        self.odometry_point_id = self.canvas.create_oval(
            canvas_x - point_radius, canvas_y - point_radius,
            canvas_x + point_radius, canvas_y + point_radius,
            fill="lime green", outline="darkgreen"
        )
        # Add text label for the point
        self.canvas.create_text(canvas_x, canvas_y + 15, # Position label below the point
                                text=f"({self.odometry_x:.1f}, {self.odometry_y:.1f})",
                                fill="darkgreen", tags="odometry_label")
        self.canvas.tag_raise(self.odometry_point_id) # Ensure point is on top
        self.canvas.tag_raise("odometry_label") # Ensure label is on top


    def _on_canvas_click(self, event):
        """Callback when canvas is clicked to set a new target."""
        ros_x, ros_y = self._canvas_to_ros(event.x, event.y)
        # Round to nearest integer or one decimal for cleaner display/publishing
        ros_x = round(ros_x, 1)
        ros_y = round(ros_y, 1)

        # Update input entries (make them writable temporarily)
        self.x_entry.config(state='normal')
        self.y_entry.config(state='normal')
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, str(ros_x))
        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, str(ros_y))
        self.x_entry.config(state='readonly')
        self.y_entry.config(state='readonly')

        # Directly call update_target which also publishes
        self.update_target(ros_x, ros_y)
        self._show_message("Success", f"Target set to ({ros_x:.1f}, {ros_y:.1f}) via canvas click and Twist message published.", "green")

    def _reset_position(self):
        """Resets the target position to odometry and publishes."""
        new_x = self.odometry_x
        new_y = self.odometry_y
        self.x_entry.config(state='normal')
        self.y_entry.config(state='normal')
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, f"{new_x:.2f}")
        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, f"{new_y:.2f}")
        self.x_entry.config(state='readonly')
        self.y_entry.config(state='readonly')

        self.update_target(new_x, new_y)
        self._show_message("Success", f"Position reset to ({new_x:.2f}, {new_y:.2f}) and Twist message published.", "green")


    def _periodic_publish(self, event):
        """
        Callback for rospy.Timer to periodically publish the current position.
        The 'event' argument is passed by rospy.Timer but not used here.
        """
        # Ensure ROS publisher is initialized before attempting to publish
        if self.publisher:
            twist_msg = Twist()
            twist_msg.linear.x = self.current_x
            twist_msg.linear.y = self.current_y
            # Z, angular x, y, z remain 0 by default
            try:
                self.publisher.publish(twist_msg)
                # rospy.loginfo(f"Periodically published Twist: linear.x={self.current_x:.1f}, linear.y={self.current_y:.1f}")
            except rospy.ROSException as e:
                rospy.logerr(f"Failed to publish periodic Twist message: {e}")
                # Optionally, show message on GUI for critical errors
                # self._show_message("ROS Publish Error", f"Periodic publish failed: {e}", "red")
        else:
            rospy.logwarn("ROS Publisher not initialized for periodic publish.")


    def update_target(self, x, y):
        """
        Updates the target position and publishes the Twist message.
        This function is designed to be callable from any thread.
        """
        # Update internal state
        self.current_x = x
        self.current_y = y

        # Publish the Twist message immediately when target changes (in addition to periodic publish)
        if self.publisher:
            twist_msg = Twist()
            twist_msg.linear.x = x
            twist_msg.linear.y = y
            # Z, angular x, y, z remain 0 by default
            try:
                self.publisher.publish(twist_msg)
                rospy.loginfo(f"Published Twist (on target change): linear.x={x:.1f}, linear.y={y:.1f}")
            except rospy.ROSException as e:
                rospy.logerr(f"Failed to publish Twist message on target change: {e}")
                self._show_message("ROS Publish Error", f"Failed to publish: {e}", "red")
        else:
            rospy.logwarn("ROS Publisher not initialized. Cannot publish message on target change.")
            self._show_message("ROS Warning", "ROS not connected. Message not published.", "orange")


        # Schedule GUI update for target point on the main Tkinter thread
        self.gui_update_queue.put(('target_update', x, y))

    def _process_gui_queue(self):
        """
        Processes updates from the gui_update_queue.
        This method runs in the main Tkinter thread.
        """
        try:
            while True:
                # Get the latest update from the queue without blocking
                update_type, x, y = self.gui_update_queue.get_nowait()
                if update_type == 'target_update':
                    self.current_x = x
                    self.current_y = y
                    self._draw_target_point()
                elif update_type == 'odometry_update':
                    self.odometry_x = x
                    self.odometry_y = y
                    self._draw_odometry_point()
        except queue.Empty:
            pass # No updates in the queue
        finally:
            # Schedule itself to run again after 100ms
            self.master.after(100, self._process_gui_queue)

    def _on_close(self):
        """Handles closing the window."""
        rospy.loginfo("Shutting down ROS node.")
        # Invalidate the timer to stop periodic callbacks
        if self.publish_timer:
            self.publish_timer.shutdown()
            rospy.loginfo("Periodic publishing timer shut down.")

        # Signal ROS to shut down, which will stop rospy.spin() thread
        rospy.signal_shutdown("GUI closed by user.")
        # If the ROS thread is still alive, give it a moment to finish
        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0) # Wait for the thread to finish, with a timeout
            if self.ros_thread.is_alive():
                rospy.logwarn("ROS thread did not terminate gracefully.")
        self.master.destroy()

# Main execution block
if __name__ == "__main__":
    root = tk.Tk()
    app = ROSPositionGUI(root)

    # Start the Tkinter event loop
    root.mainloop()
