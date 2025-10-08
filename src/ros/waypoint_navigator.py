import threading
import time
from typing import Optional, Tuple, Dict, Any, List
from PySide6.QtCore import QObject, Signal

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from rclpy.duration import Duration
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"Warning: ROS 2 packages not available: {e}")
    ROS_AVAILABLE = False
    
class WaypointNavigator(QObject):
    """
    A dedicated class for waypoint navigation that uses BasicNavigator.
    This class handles navigation goals and provides detailed status feedback.
    """
    
    # Define signals
    goal_reached = Signal()  # Emitted when goal is reached
    navigation_status = Signal(str, bool)  # Emitted when status changes (status_string, is_active)
    
    def __init__(self):
        """Initialize the WaypointNavigator."""
        super().__init__()
        self.ros_available = ROS_AVAILABLE
        self._status = ("idle", False)  # (status_string, is_active)
        self._navigation_thread = None
        self._monitor_thread = None
        
        if not self.ros_available:
            print("Warning: Running in simulation mode - ROS 2 not available")
            return
            
        try:
            # Initialize ROS if not already done
            if not rclpy.ok():
                rclpy.init()
                
            # Create separate node for navigation
            self.node = rclpy.create_node('waypoint_navigator_node')
                
            # Initialize navigator
            self.navigator = BasicNavigator()
            
            # Create dedicated executor for this node
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.executor.add_node(self.navigator)
            
            # Start executor in a separate thread
            self.spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
            self.spin_thread.start()
            
            print("WaypointNavigator initialized successfully!")
            
        except Exception as e:
            print(f"Failed to initialize WaypointNavigator: {e}")
            import traceback
            traceback.print_exc()
            self.ros_available = False
    
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
            print("WaypointNavigator executor thread exiting...")
        except Exception as e:
            print(f"Error in _spin_executor: {e}")
    
    def navigate_to_point(self, x: float, y: float, theta: float = 0.0) -> bool:
        """
        Navigate to a specific point.
        
        Args:
            x: X coordinate
            y: Y coordinate
            theta: Orientation (yaw) in radians
            
        Returns:
            bool: True if navigation was successfully initiated, False otherwise
        """
        print(f"DEBUG: WaypointNavigator.navigate_to_point({x:.2f}, {y:.2f}, {theta:.2f})")
        
        if not self.ros_available:
            print(f"SIMULATION: Would navigate to ({x:.2f}, {y:.2f}, {theta:.2f})")
            self._set_status("simulating", True)
            
            # Simulate navigation in a separate thread
            def simulate_navigation():
                time.sleep(2)  # Simulate navigation delay
                self._set_status("success", False)
                self.goal_reached.emit()
                
            self._navigation_thread = threading.Thread(target=simulate_navigation, daemon=True)
            self._navigation_thread.start()
            return True
            
        try:
            # Check if there's an active navigation
            if self._status[1]:  # is_active
                print("WARNING: Navigation already in progress")
                return False
                
            # Create goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.position.z = 0.0
            
            # Convert theta to quaternion
            from math import sin, cos
            goal_pose.pose.orientation.z = sin(theta / 2.0)
            goal_pose.pose.orientation.w = cos(theta / 2.0)
            
            # Clear costmaps before navigation
            self.navigator.clearAllCostmaps()
            time.sleep(1)  # Give time for costmaps to clear
            
            # Send goal - BasicNavigator handles the execution internally
            self.navigator.goToPose(goal_pose)
            self._set_status("navigating", True)
            
            # Start a separate thread to monitor completion
            self._monitor_thread = threading.Thread(target=self._monitor_navigation, daemon=True)
            self._monitor_thread.start()
            
            return True
            
        except Exception as e:
            print(f"ERROR: Exception in navigate_to_point: {e}")
            import traceback
            traceback.print_exc()
            self._set_status("error", False)
            return False
    
    def _monitor_navigation(self):
        """Monitor the navigation progress and update status."""
        try:
            print("DEBUG: Starting navigation monitoring...")
            progress_counter = 0
            
            while not self.navigator.isTaskComplete():
                progress_counter += 1
                
                # Get feedback periodically
                feedback = self.navigator.getFeedback()
                if feedback and progress_counter % 30 == 0:  # Report every ~3 seconds
                    remaining_time = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                    print(f"DEBUG: Navigation in progress... ETA: {remaining_time:.1f}s")
                    
                    # Update status with progress information
                    self._set_status(f"navigating (ETA: {remaining_time:.0f}s)", True)
                    
                time.sleep(0.1)
            
            # Get final result
            result = self.navigator.getResult()
            print(f"DEBUG: Navigation completed with result: {result}")
            
            if result == TaskResult.SUCCEEDED:
                self._set_status("success", False)
                self.goal_reached.emit()
            elif result == TaskResult.CANCELED:
                self._set_status("canceled", False)
            elif result == TaskResult.FAILED:
                self._set_status("failed", False)
            else:
                self._set_status(f"unknown ({result})", False)
                
        except Exception as e:
            print(f"ERROR: Error monitoring navigation: {e}")
            import traceback
            traceback.print_exc()
            self._set_status("error", False)
    
    def _set_status(self, status_string: str, is_active: bool):
        """
        Set the navigation status and emit signal.
        
        Args:
            status_string: Human-readable status string
            is_active: Whether navigation is currently active
        """
        self._status = (status_string, is_active)
        self.navigation_status.emit(status_string, is_active)
    
    def get_status(self) -> Tuple[str, bool]:
        """
        Get current navigation status.
        
        Returns:
            Tuple[str, bool]: (status_string, is_active)
        """
        return self._status
    
    def get_status_string(self) -> str:
        """
        Get human-readable status string.
        
        Returns:
            str: Status string
        """
        status, is_active = self._status
        if is_active:
            return f"Navigation {status}..."
        else:
            return f"Navigation {status}"
    
    def cancel_navigation(self) -> bool:
        """
        Cancel current navigation.
        
        Returns:
            bool: True if cancellation was successful
        """
        if not self.ros_available:
            print("SIMULATION: Would cancel current navigation")
            self._set_status("canceled", False)
            return True
            
        if not self._status[1]:  # not active
            print("WARNING: No active navigation to cancel")
            return False
            
        try:
            self.navigator.cancelTask()
            self._set_status("canceling", True)  # Will be updated by the monitoring thread
            return True
        except Exception as e:
            print(f"ERROR: Failed to cancel navigation: {e}")
            return False
    
    def clear_costmaps(self) -> bool:
        """
        Clear navigation costmaps.
        
        Returns:
            bool: True if clearing was successful
        """
        if not self.ros_available:
            print("SIMULATION: Would clear costmaps")
            return True
            
        try:
            self.navigator.clearAllCostmaps()
            print("DEBUG: Costmaps cleared successfully")
            return True
        except Exception as e:
            print(f"ERROR: Failed to clear costmaps: {e}")
            return False
    
    def shutdown(self):
        """Shutdown the navigator and clean up resources."""
        if not self.ros_available:
            return
            
        try:
            # Stop executor thread if running
            if hasattr(self, '_spin_executor') and hasattr(self, 'spin_thread') and self.spin_thread.is_alive():
                print("Stopping WaypointNavigator executor thread...")
                self._executor_should_exit = True
                self.spin_thread.join(timeout=1.0)
            
            # Shutdown navigator
            if hasattr(self, 'navigator'):
                print("Shutting down navigator...")
                try:
                    self.navigator.lifecycleShutdown()
                except Exception as e:
                    print(f"WARNING: Error shutting down navigator: {e}")
            
            # Remove node from executor
            if hasattr(self, 'executor') and hasattr(self, 'node'):
                print("Removing node from executor...")
                try:
                    self.executor.remove_node(self.node)
                except Exception as e:
                    print(f"WARNING: Error removing node from executor: {e}")
                    
            # Shutdown executor
            if hasattr(self, 'executor'):
                print("Shutting down executor...")
                try:
                    self.executor.shutdown()
                except Exception as e:
                    print(f"WARNING: Error shutting down executor: {e}")
            
            print("WaypointNavigator shutdown complete")
            
        except Exception as e:
            print(f"ERROR: Error during WaypointNavigator shutdown: {e}")
            import traceback
            traceback.print_exc()