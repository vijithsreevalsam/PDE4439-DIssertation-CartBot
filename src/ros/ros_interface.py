import threading
import time
from typing import Optional, Tuple
from PySide6.QtCore import QObject, Signal
from rclpy.executors import SingleThreadedExecutor

try:
    from tf2_ros import Buffer, TransformListener
    TF_AVAILABLE = True
except ImportError:
    print("Warning: TF2 packages not available")
    TF_AVAILABLE = False
    # Dummy TF classes
    class Buffer:
        def __init__(self): pass
    class TransformListener:
        def __init__(self, buffer, node): pass
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int16MultiArray, String

    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"Warning: ROS 2 packages not available: {e}")
    ROS_AVAILABLE = False
    
    # Create dummy classes for when ROS is not available
    class Node:
        def __init__(self, name): pass
        def create_publisher(self, *args): return DummyPublisher()
        def create_subscription(self, *args): return None
        def get_logger(self): return DummyLogger()
        def get_clock(self): return DummyTime()
        
    class Twist:
        def __init__(self):
            self.linear = DummyVector3()
            self.angular = DummyVector3()
            
    class PoseStamped:
        def __init__(self):
            self.header = DummyHeader()
            self.pose = DummyPose()
            
    class DummyVector3:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            
    class DummyPose:
        def __init__(self):
            self.position = DummyVector3()
            self.orientation = DummyQuaternion()
            
    class DummyQuaternion:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0
            
    class DummyHeader:
        def __init__(self):
            self.frame_id = ""
            self.stamp = None
            
    class DummyTime:
        def now(self): return DummyStamp()
        
    class DummyStamp:
        def to_msg(self): return None
        
    class DummyPublisher:
        def publish(self, msg): pass
        
    class DummyLogger:
        def info(self, msg): print(f"INFO: {msg}")
        def warn(self, msg): print(f"WARN: {msg}")
        def error(self, msg): print(f"ERROR: {msg}")

# Import the new WaypointNavigator class
try:
    from .waypoint_navigator import WaypointNavigator
    WAYPOINT_NAVIGATOR_AVAILABLE = True
except ImportError as e:
    print(f"Warning: WaypointNavigator not available: {e}")
    WAYPOINT_NAVIGATOR_AVAILABLE = False

# Dynamic base class selection
if ROS_AVAILABLE:
    class ROSInterface(QObject):
        """ROS 2 interface for robot control and status monitoring."""
        
        # Define signals as class attributes
        goal_reached = Signal()
        # voice_text_received = Signal(str)
        
        def __init__(self):
            self.ros_available = ROS_AVAILABLE
            
            # Initialize QObject first
            QObject.__init__(self)
            
            # Initialize ROS if not already done
            try:
                if not rclpy.ok():
                    print("Initializing ROS...")
                    rclpy.init()
                    print("ROS initialized successfully")
            except Exception as e:
                print(f"Failed to initialize ROS: {e}")
                self.setup_dummy_interface()
                return
                
            try:
                print("Setting up ROS interface with WaypointNavigator...")
                
                # Create the waypoint navigator
                if not WAYPOINT_NAVIGATOR_AVAILABLE:
                    print("WARNING: WaypointNavigator not available, using dummy interface")
                    self.setup_dummy_interface()
                    return
                
                # Create the waypoint navigator
                self.waypoint_navigator = WaypointNavigator()
                print(f"WaypointNavigator created: {self.waypoint_navigator}")
                
                # Forward the goal_reached signal from waypoint_navigator
                self.waypoint_navigator.goal_reached.connect(self.goal_reached)
                
                # Forward navigation status updates
                self.waypoint_navigator.navigation_status.connect(self._on_navigation_status_changed)
                
                # Setup the main ROS interface for teleoperation and sensors
                self.setup_ros_interface()
                
                # Initialize TF listener
                try:
                    self.tf_buffer = Buffer()
                    self.tf_listener = TransformListener(self.tf_buffer, self.node)
                    print("TF listener initialized successfully")
                except Exception as e:
                    print(f"Warning: Failed to initialize TF listener: {e}")
                
                print("ROS interface setup complete")
            except Exception as e:
                print(f"Failed to create ROS interface: {e}")
                import traceback
                traceback.print_exc()
                self.setup_dummy_interface()
        def setup_ros_interface(self):
            """Setup the real ROS interface."""
            # QoS profiles
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # Create a ROS node for publishers and subscribers
            self.node = rclpy.create_node('ros_interface_node')
            
            # Publishers
            self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            
            # Subscribers
            self.odom_sub = self.node.create_subscription(
                Odometry, '/odom', self.odom_callback, 10)
            self.amcl_sub = self.node.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
            
            # self.voicetoText_sub = self.node.create_subscription(
            #     Int16MultiArray, '/recognized_text', self.voice_callback, 10)


            # Start a separate executor for this node (not including the waypoint navigator)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            
            # Spin in background
            self.spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
            self.spin_thread.start()

                    





            # State variables
            self.current_pose = None
            self.current_velocity = None
            self.current_odom = None
            self.current_voice = ""
            self.is_listening = False
            
            # Voice subscription
            # self.voice_sub = self.node.create_subscription(
            #     String,
            #     'recognized_text',
            #     self.voice_callback,
            #     10
            # )
            self.current_voice = None
            self.is_connected_flag = True
            
            print("ROS interface initialized successfully!")
            
        def _spin_executor(self):
            """Spin the executor in a separate thread."""
            # Add flag to safely stop the thread
            self._executor_should_exit = False
            
            try:
                while rclpy.ok() and not self._executor_should_exit:
                    try:
                        self.executor.spin_once(timeout_sec=0.1)
                    except Exception as e:
                        print(f"Error in executor spin: {e}")
                    time.sleep(0.01)  # Small sleep to prevent CPU thrashing
                print("ROSInterface executor thread exiting...")
            except Exception as e:
                print(f"Error in _spin_executor: {e}")
                
        def get_clock(self):
            """Get the node's clock."""
            if hasattr(self, 'node'):
                return self.node.get_clock()
            return None
            
        def setup_dummy_interface(self):
            """Setup dummy interface for when ROS is not available."""
            self.ros_available = False
            self.current_pose = None
            self.current_velocity = None
            self.current_odom = None
            # We'll check connection status directly in is_connected() instead of using this flag
            
            # Create dummy TF components
            if TF_AVAILABLE:
                try:
                    self.tf_buffer = Buffer()
                except Exception as e:
                    print(f"Warning: Could not create TF Buffer in dummy interface: {e}")
            
            # Create dummy waypoint navigator if needed
            if not hasattr(self, 'waypoint_navigator'):
                self.waypoint_navigator = WaypointNavigator()
                self.waypoint_navigator.goal_reached.connect(self.goal_reached)
                self.waypoint_navigator.navigation_status.connect(self._on_navigation_status_changed)
            
            # Create dummy pose for testing
            self.current_pose = type('DummyPose', (), {
                'position': type('Position', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})(),
                'orientation': type('Orientation', (), {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})()
            })()
            
            # Create dummy velocity for testing
            self.current_velocity = type('DummyVel', (), {
                'linear': type('Linear', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})(),
                'angular': type('Angular', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
            })()
            
        def _on_navigation_status_changed(self, status_string: str, is_active: bool):
            """Handle navigation status changes from waypoint navigator."""
            print(f"DEBUG: Navigation status changed to: {status_string}, active: {is_active}")
                    
        def odom_callback(self, msg):
            """Odometry callback."""
            self.current_odom = msg
            if self.current_pose is None:  # Use odom if AMCL not available
                self.current_pose = msg.pose.pose
                
        def amcl_callback(self, msg):
            """AMCL pose callback."""
            self.current_pose = msg.pose.pose
            
        # def voice_callback(self, msg):
        #     """Voice to text callback."""
        #     print(f"Voice to text callback received: {msg.data}")
        #     print(f"DEBUG: Available waypoints: {[wp for wp in self.waypoint_navigator.get_waypoint_names()]}") if hasattr(self, 'waypoint_navigator') and hasattr(self.waypoint_navigator, 'get_waypoint_names') else None
            
        #     self.current_voice = msg.data
            
        #     # Emit a signal when new voice text is received
        #     try:
        #         # Make sure the signal exists before trying to emit it
        #         if hasattr(self.__class__, 'voice_text_received'):
        #             print(f"Emitting voice_text_received signal with: {msg.data}")
        #             self.voice_text_received.emit(msg.data)
        #         else:
        #             print("Warning: voice_text_received signal not defined in class")
        #     except Exception as e:
        #         print(f"Error emitting voice text signal: {e}")
        #         import traceback
        #         traceback.print_exc()
                
        def current_voice_text(self) -> str:
            """Get current voice text."""
            if hasattr(self, 'current_voice'):
                return self.current_voice
            return "No voice text available"
            
        # def start_voice_processing(self):
        #     """Start the audio processing node to listen for voice commands."""
        #     try:
        #         # Create a subscription to the recognized text topic if not already created
        #         if not hasattr(self, 'voice_sub'):
        #             self.voice_sub = self.node.create_subscription(
        #                 String, 
        #                 'recognized_text',
        #                 self.voice_callback,
        #                 10
        #             )
        #             print("Subscribed to recognized_text topic")
                
        #         # Launch the audio_node.py script if not already running
        #         import subprocess
        #         import os
                
        #         # Check if we already have an audio process running
        #         if not hasattr(self, 'audio_process') or self.audio_process is None or self.audio_process.poll() is not None:
        #             audio_node_path = os.path.expanduser("~/UI/src/utils/audio_node.py")
                    
        #             # Make sure the script is executable
        #             if os.path.exists(audio_node_path):
        #                 os.chmod(audio_node_path, 0o755)
                        
        #                 # Start the audio node process
        #                 print(f"Starting audio node from {audio_node_path}")
        #                 self.audio_process = subprocess.Popen([audio_node_path], 
        #                                                      stdout=subprocess.PIPE,
        #                                                      stderr=subprocess.PIPE)
        #                 print(f"Audio node started with PID: {self.audio_process.pid}")
        #             else:
        #                 print(f"Error: Audio node not found at {audio_node_path}")
        #                 return False
        #         else:
        #             print("Audio process already running")
                
        #         # Set a flag that we're actively listening
        #         self.is_listening = True
        #         return True
        #     except Exception as e:
        #         print(f"Error starting voice processing: {e}")
        #         import traceback
        #         traceback.print_exc()
        #         return False
                
        # def stop_voice_processing(self):
        #     """Stop listening for voice commands."""
        #     try:
        #         # Set flag that we're no longer listening
        #         self.is_listening = False
                
        #         # Terminate the audio process if needed (comment this out if you want the audio node to keep running)
        #         if hasattr(self, 'audio_process') and self.audio_process is not None:
        #             # Just keep the process running - don't terminate
        #             # Uncomment the next line if you want to stop the process when releasing the button
        #             # self.audio_process.terminate()
        #             pass
                    
        #         return True
        #     except Exception as e:
        #         print(f"Error stopping voice processing: {e}")
        #         return False

        def current_robot_pose(self):
            # try:
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(
                    'map',       # target frame
                    'base_link', # source frame
                    now
                )

                pose = PoseStamped()
                pose.header.stamp = trans.header.stamp
                pose.header.frame_id = 'map'
                pose.pose.position.x = trans.transform.translation.x
                pose.pose.position.y = trans.transform.translation.y
                pose.pose.position.z = trans.transform.translation.z
                pose.pose.orientation = trans.transform.rotation

                self.current_pose = pose.pose
                # self.get_logger().info(f"Robot pose: {pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}")
            # except Exception as e:
            #     self.current_pose = None
                
        def cmd_vel_callback(self, msg):
            """Command velocity callback."""
            self.current_velocity = msg
            
        def publish_cmd_vel(self, twist_msg):
            """Publish velocity command."""
            if self.ros_available and hasattr(self, 'cmd_vel_pub'):
                self.cmd_vel_pub.publish(twist_msg)
            else:
                print(f"CMD_VEL: linear={twist_msg.linear.x:.2f}, angular={twist_msg.angular.z:.2f}")
            
        def get_current_pose(self):
            """Get current robot pose."""
            self.current_robot_pose()
            return self.current_pose
            
        def get_current_velocity(self):
            """Get current robot velocity."""
            return self.current_velocity
            
        def is_connected(self):
            """Check if ROS is connected."""
            # If ROS is available, ensure rclpy is initialized and running
            if self.ros_available:
                try:
                    return rclpy.ok() and hasattr(self, 'waypoint_navigator')
                except Exception as e:
                    print(f"Error checking ROS connection status: {e}")
                    return False
            # If in simulation mode, always return true if we have waypoint_navigator
            return hasattr(self, 'waypoint_navigator')
            
        def get_goal_status(self):
            """Get current goal status as tuple (status_string, is_active)."""
            return self.waypoint_navigator.get_status()
            
        def get_goal_status_string(self):
            """Get human-readable goal status string."""
            return self.waypoint_navigator.get_status_string()
            
        def navigate_to_point(self, x: float, y: float, theta: float = 0.0) -> bool:
            """Navigate to specified point using WaypointNavigator."""
            print(f"DEBUG: ROSInterface.navigate_to_point({x:.2f}, {y:.2f}, {theta:.2f})")
            return self.waypoint_navigator.navigate_to_point(x, y, theta)
                
        def cancel_navigation(self) -> bool:
            """Cancel current navigation goal."""
            return self.waypoint_navigator.cancel_navigation()
                
        def clear_costmaps(self) -> bool:
            """Clear costmaps."""
            return self.waypoint_navigator.clear_costmaps()
                
        def emergency_stop(self):
            """Emergency stop - publish zero velocity."""
            stop_twist = Twist()
            # All values default to 0.0
            self.publish_cmd_vel(stop_twist)
            print("EMERGENCY STOP: Robot halted!")
            
        def shutdown(self):
            """Shutdown ROS interface."""
            if not self.ros_available:
                return
                
            try:
                print("Shutting down ROSInterface...")
                
                # Terminate audio process if running
                if hasattr(self, 'audio_process') and self.audio_process is not None:
                    print("Terminating audio process...")
                    try:
                        self.audio_process.terminate()
                        self.audio_process.wait(timeout=2.0)
                    except Exception as e:
                        print(f"WARNING: Error terminating audio process: {e}")
                        # Force kill if needed
                        try:
                            self.audio_process.kill()
                        except:
                            pass
                
                # Clean up TF listener first
                if hasattr(self, 'tf_listener'):
                    print("Cleaning up TF listener...")
                    try:
                        # Set to None to trigger garbage collection
                        self.tf_listener = None
                    except Exception as e:
                        print(f"WARNING: Error cleaning up TF listener: {e}")
                
                # First shut down the waypoint navigator
                if hasattr(self, 'waypoint_navigator'):
                    print("Shutting down waypoint navigator...")
                    self.waypoint_navigator.shutdown()
                
                # Stop executor thread if running
                if hasattr(self, '_spin_executor') and hasattr(self, 'spin_thread') and self.spin_thread.is_alive():
                    print("Stopping ROSInterface executor thread...")
                    self._executor_should_exit = True
                    self.spin_thread.join(timeout=1.0)
                
                # Remove node from executor
                if hasattr(self, 'executor') and hasattr(self, 'node'):
                    print("Removing node from executor...")
                    try:
                        self.executor.remove_node(self.node)
                    except Exception as e:
                        print(f"WARNING: Error removing node from executor: {e}")
                
                # Then shut down our own resources
                if hasattr(self, 'executor'):
                    print("Shutting down executor...")
                    try:
                        self.executor.shutdown()
                    except Exception as e:
                        print(f"WARNING: Error shutting down executor: {e}")
                
                print("ROSInterface shutdown complete")
                
                # Don't shut down rclpy here as other components might be using it
            except Exception as e:
                print(f"Error during ROSInterface shutdown: {e}")
                import traceback
                traceback.print_exc()
                
        def quaternion_to_euler(self, q) -> float:
            """Convert quaternion to euler angle (yaw)."""
            import math
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)
else:
    class ROSInterface(QObject):
        """ROS 2 interface for robot control and status monitoring."""
        
        # Define the signals as class attributes
        goal_reached = Signal()
        # voice_text_received = Signal(str)
        
        def __init__(self):
            self.ros_available = ROS_AVAILABLE
            
            QObject.__init__(self)
            print("Warning: Running in simulation mode - ROS 2 not available")
            
            try:
                # Create dummy waypoint navigator if available
                if WAYPOINT_NAVIGATOR_AVAILABLE:
                    self.waypoint_navigator = WaypointNavigator()
                    
                    # Forward the goal_reached signal
                    self.waypoint_navigator.goal_reached.connect(self.goal_reached)
                    
                    # Forward navigation status updates
                    self.waypoint_navigator.navigation_status.connect(self._on_navigation_status_changed)
                    
                    print("Created waypoint navigator in simulation mode")
                else:
                    print("WARNING: WaypointNavigator not available, using limited functionality")
                    # Create a minimal simulation interface
                    self._status = ("idle", False)  # (status_string, is_active)
                    
                # Create dummy TF components
                if TF_AVAILABLE:
                    try:
                        self.tf_buffer = Buffer()
                        print("Created dummy TF buffer")
                    except Exception as e:
                        print(f"Warning: Could not create TF Buffer in dummy interface: {e}")
            except Exception as e:
                print(f"Error initializing simulation mode: {e}")
                import traceback
                traceback.print_exc()
                self._status = ("error", False)
                
            self.setup_dummy_interface()
            
        def setup_dummy_interface(self):
            """Setup dummy interface for when ROS is not available."""
            self.ros_available = False
            self.current_pose = None
            self.current_velocity = None
            self.current_odom = None
            # Connection status is determined by having waypoint_navigator initialized
            
            # Create dummy pose for testing
            self.current_pose = type('DummyPose', (), {
                'position': type('Position', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})(),
                'orientation': type('Orientation', (), {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})()
            })()
            
            # Create dummy velocity for testing
            self.current_velocity = type('DummyVel', (), {
                'linear': type('Linear', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})(),
                'angular': type('Angular', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
            })()
        
        def _on_navigation_status_changed(self, status_string: str, is_active: bool):
            """Handle navigation status changes from waypoint navigator."""
            print(f"DEBUG: Navigation status changed to: {status_string}, active: {is_active}")
                    
        def publish_cmd_vel(self, twist_msg):
            """Publish velocity command."""
            print(f"CMD_VEL: linear={twist_msg.linear.x:.2f}, angular={twist_msg.angular.z:.2f}")
            
        def get_current_pose(self):
            """Get current robot pose."""
            return self.current_pose
            
        def get_current_velocity(self):
            """Get current robot velocity."""
            return self.current_velocity
            
        def is_connected(self):
            """Check if ROS is connected."""
            # In simulation mode, return true if we have a waypoint_navigator initialized
            return hasattr(self, 'waypoint_navigator')
            
        def get_goal_status(self):
            """Get current goal status as tuple (status_string, is_active)."""
            return self.waypoint_navigator.get_status()
            
        def get_goal_status_string(self):
            """Get human-readable goal status string."""
            return self.waypoint_navigator.get_status_string()
            
        def navigate_to_point(self, x: float, y: float, theta: float = 0.0) -> bool:
            """Navigate to specified point using WaypointNavigator."""
            print(f"DEBUG: ROSInterface.navigate_to_point({x:.2f}, {y:.2f}, {theta:.2f})")
            return self.waypoint_navigator.navigate_to_point(x, y, theta)
                
        def cancel_navigation(self) -> bool:
            """Cancel current navigation goal."""
            return self.waypoint_navigator.cancel_navigation()
                
        def clear_costmaps(self) -> bool:
            """Clear costmaps."""
            return self.waypoint_navigator.clear_costmaps()
                
        def emergency_stop(self):
            """Emergency stop - publish zero velocity."""
            stop_twist = Twist()
            self.publish_cmd_vel(stop_twist)
            print("EMERGENCY STOP: Robot halted!")
            
        def shutdown(self):
            """Shutdown ROS interface."""
            print("Shutting down dummy interface")
            
            # Clean up TF listener if any
            if hasattr(self, 'tf_listener'):
                print("Cleaning up dummy TF listener...")
                self.tf_listener = None
                
            # Shutdown waypoint navigator
            if hasattr(self, 'waypoint_navigator'):
                print("Shutting down waypoint navigator...")
                self.waypoint_navigator.shutdown()
                
        def quaternion_to_euler(self, q) -> float:
            """Convert quaternion to euler angle (yaw)."""
            import math
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)