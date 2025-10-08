#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
import subprocess
import threading
import os
import signal
import json
from std_msgs.msg import String
import datetime
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class LaunchServiceNode(Node):
    waypoint_data = None
    def __init__(self):
        super().__init__('launch_service_node')
        
        # Track running processes
        self.processes = {}
        
        # Create confirmation subscription
        self.waiting_for_confirmation = False
        
        # Flag to control navigation processing
        self.navigation_active = False
        
        # Option to stream output to terminal (default: True for real-time debugging)
        self.stream_output = True
        
        # Create services for different launch commands
        self.start_robot = self.create_service(SetBool, 'start_robot', self.start_robot_callback)
        self.slam_service = self.create_service(SetBool, 'launch_slam', self.slam_callback)
        self.nav_service = self.create_service(SetBool, 'launch_navigation', self.nav_callback)
        self.kill_service = self.create_service(SetBool, 'kill_launches', self.kill_callback)
        
        # Create service to handle waypoint navigation requests
        self.waypoint_service = self.create_service(Trigger, 'navigate_to_shelf_position', self.nav2_waypoint_callback)
        
        # Subscribe to shelf navigation data topic
        self.subscribe_navigation_data = self.create_subscription(
            String,
            'shelf_navigation_data',
            self.shelf_navigation_callback,
            10
        )   
        
        # Subscribe to odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Store the latest shelf data received from the topic
        self.latest_shelf_data = None
        
        # Store the latest odometry data
        self.latest_odom = None
        
        # Create a publisher for sending logs to the web interface
        self.web_log_publisher = self.create_publisher(
            String,
            'web_console_logs',
            10
        )
        
        # Create a publisher for sending navigation success messages
        self.nav_success_publisher = self.create_publisher(
            String,
            'navigation_success',
            10
        )
        
        # Create a publisher for navigation status updates/heartbeats
        self.nav_status_publisher = self.create_publisher(
            String,
            'navigation_status',
            10
        )
        
        # State tracking for confirmation flow
        self.waiting_for_confirmation = False
        self.current_confirmation_id = None
        
        # Create subscription for confirmation messages
        self.create_confirmation_subscriber()
        
        self.log_to_web('Server info', 'Launch Service Node started - output streaming enabled for real-time debugging')
        
    def odom_callback(self, msg):
        """
        Callback function for the odom topic subscription.
        Stores the latest odometry data.
        """
        self.latest_odom = msg
    
    def shelf_navigation_callback(self, msg):
        """
        Callback function for the shelf_navigation_data topic subscription.
        Receives JSON data from the web interface and stores it for use when the service is called.
        """
        self.log_to_web('Server info', f'Received shelf navigation data: {msg.data}')
        try:
            # Parse and store the waypoint data
            self.latest_shelf_data = json.loads(msg.data)
            self.log_to_web('Server info', f"Stored shelf navigation data: {self.latest_shelf_data}")
        except json.JSONDecodeError as e:
            self.log_to_web('Server error', f'JSON parsing failed: {e}')
        except Exception as e:
            self.log_to_web('Server error', f'Error processing shelf navigation data: {e}')


    def nav2_waypoint_callback(self, request, response):
        """
        Service callback for navigate_to_shelf_position
        Uses the data previously received on the shelf_navigation_data topic
        """
        self.log_to_web('Server info', '🔔 Navigation service called - trigger received')
        self.get_logger().info('Navigation service triggered')
        
        # Check if navigation is already active
        if self.navigation_active:
            # Cancel any existing navigation before starting new one
            self.log_to_web('Server info', 'Canceling previous navigation to start new request')
            self.cancel_navigation()
        
        # Check if we have shelf data available
        if self.latest_shelf_data is None:
            response.success = False
            response.message = "Server Message: No shelf position data available. Make sure to publish data to shelf_navigation_data topic first."
            self.log_to_web('Server warn', "Navigation service called but no shelf data is available")
            return response
            
        try:
            # Use the most recent shelf data received from the topic
            waypoint_data = self.latest_shelf_data
            self.get_logger().info(f"Using waypoint data: {waypoint_data}")

            # Validate waypoint data
            if not isinstance(waypoint_data, dict) or not waypoint_data:
                self.log_to_web('Server error', f'Invalid waypoint data format: {type(waypoint_data)}, needs to be a non-empty dictionary')
                response.success = False
                response.message = "Server message: Invalid waypoint data format"
                return response
                
            # Validate each waypoint has proper coordinates
            for key, coords in waypoint_data.items():
                if not isinstance(coords, list) or len(coords) < 2:
                    self.log_to_web('Server warn', f'Invalid coordinates for {key}: {coords}, needs to be a list with at least 2 values')
                    continue
                    
                try:
                    x_coord = float(coords[0])
                    y_coord = float(coords[1])
                    if abs(x_coord) > 100 or abs(y_coord) > 100:
                        self.log_to_web('Server warn', f'Coordinates for {key} seem extreme: [{x_coord}, {y_coord}]')
                except (ValueError, TypeError):
                    self.log_to_web('Server warn', f'Coordinates for {key} are not valid numbers: {coords}')

            # Log basic info immediately to help with debugging
            self.log_to_web('Server info', f'Processing navigation request with data: {waypoint_data}')

            # Get item name for the response message
            item_name = "unknown"
            if isinstance(waypoint_data, dict) and waypoint_data:
                try:
                    # Get the first key from the dictionary
                    item_name = list(waypoint_data.keys())[0]
                except (IndexError, AttributeError):
                    pass
            
            # Set success response immediately
            response.success = True
            response.message = f"Server message: Navigating to shelf position: {item_name}"
            
            # Spawn a separate thread to handle the actual navigation processing
            # This allows the service to return quickly and avoid timeout
            navigation_thread = threading.Thread(
                target=self._process_navigation_request,
                args=(waypoint_data,),
                daemon=True
            )
            navigation_thread.start()
            
            self.log_to_web('Server info', f'Navigation thread started for destination: {item_name}')
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to navigate: {str(e)}"
            self.log_to_web('error', f'Shelf navigation failed: {e}')
        
        return response
        
    def _process_navigation_request(self, waypoint_data):
        """
        Process navigation request directly in this thread
        """
        self.log_to_web('Server info', 'Starting navigation processing')
        
        # Reset state variables
        self.navigation_active = True
        self.waypoint_data = waypoint_data
        self.waiting_for_confirmation = False
        self.current_confirmation_id = None
        
        self.log_to_web('Server info', 'State variables reset, checking navigation stack...')
        
        # Check if navigation stack is running
        try:
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=10)
            self.log_to_web('Server info', f'ros2 node list completed. Available ROS2 nodes: {result.stdout}')
        except subprocess.TimeoutExpired as e:
            self.log_to_web('Server error', f'ros2 node list timed out: {e}')
            self.navigation_active = False
            return
        except Exception as e:
            self.log_to_web('Server error', f'Error running ros2 node list: {e}')
            self.navigation_active = False
            return
        
        # Look for any navigation-related nodes (amcl, bt_navigator, etc.)
        nav_related_terms = ['amcl', 'bt_navigator', 'nav2', 'costmap', 'planner']
        nav2_nodes_running = any(any(term in node for term in nav_related_terms) for node in result.stdout.split('\n'))
        
        self.log_to_web('Server info', f'Navigation stack check complete. Nav2 running: {nav2_nodes_running}')
        
        if not nav2_nodes_running:
            self.log_to_web('Server error', 'Navigation stack does not appear to be running! Please start navigation first.')
            self.navigation_active = False
            return
            
        self.log_to_web('Server info', 'Navigation stack verified, initializing navigator...')
        
        # Initialize navigator and process directly (safer than using threads)
        try:
            # Create the navigator
            self.navigator = BasicNavigator()
            self.log_to_web('Server info', 'BasicNavigator created successfully')
            
            # Wait for Nav2 with a reasonable timeout
            self.log_to_web('Server info', 'Waiting for Navigation2 to become active...')
            try:
                # Custom implementation of waiting for Nav2 instead of using waitUntilNav2Active()
                # This avoids the "wait set index too big" error
                self.log_to_web('Server info', 'Checking if Nav2 is active...')
                
                # Wait for navigation nodes to be ready (up to 30 seconds)
                max_wait_time = 30
                start_time = time.time()
                nav2_active = False
                
                self.log_to_web('Server info', 'Starting Nav2 readiness check loop...')
                
                while time.time() - start_time < max_wait_time:
                    # Check if bt_navigator is available by checking its lifecycle state
                    try:
                        result = subprocess.run(['ros2', 'lifecycle', 'get', '/bt_navigator'], 
                                                capture_output=True, text=True, timeout=5)
                        self.log_to_web('Server info', f'bt_navigator lifecycle state: {result.stdout.strip()}')
                        if 'active' in result.stdout.lower():
                            nav2_active = True
                            break
                    except subprocess.TimeoutExpired:
                        self.log_to_web('Server warn', 'Timeout checking bt_navigator lifecycle state')
                    except Exception as e:
                        self.log_to_web('Server warn', f'Error checking bt_navigator: {e}')
                        
                    self.log_to_web('Server info', f'Waiting for Nav2 to become active... ({time.time() - start_time:.1f}s elapsed)')
                    time.sleep(1)
                
                if nav2_active:
                    self.log_to_web('Server info', 'Navigation2 is now active!')
                else:
                    self.log_to_web('Server error', 'Timed out waiting for Nav2 to become active')
                    raise Exception("Timed out waiting for Nav2 to become active")
            except Exception as e:
                self.log_to_web('Server error', f'Failed to connect to Nav2: {e}')
                self.navigation_active = False
                return
            
            # Set up initial pose (wait for odometry if not available)
            self.log_to_web('Server info', 'Setting up initial pose...')
            max_attempts = 5
            attempts = 0
            
            while not self.latest_odom and attempts < max_attempts:
                self.log_to_web('Server info', f'Waiting for odometry data... (attempt {attempts+1}/{max_attempts})')
                time.sleep(1.0)
                attempts += 1
                
            self.log_to_web('Server info', f'Odometry wait complete. Odom available: {self.latest_odom is not None}')
            
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            
            if self.latest_odom:
                initial_pose.pose.position.x = self.latest_odom.pose.pose.position.x
                initial_pose.pose.position.y = self.latest_odom.pose.pose.position.y
                initial_pose.pose.position.z = self.latest_odom.pose.pose.position.z
                initial_pose.pose.orientation = self.latest_odom.pose.pose.orientation
                self.log_to_web('Server info', f'Using odometry for initial pose: x={initial_pose.pose.position.x}, y={initial_pose.pose.position.y}')
            else:
                initial_pose.pose.position.x = 0.0
                initial_pose.pose.position.y = 0.0
                initial_pose.pose.position.z = 0.0
                initial_pose.pose.orientation.w = 1.0
                self.log_to_web('Server warn', 'No odometry data available, using default initial pose (0,0)')
            
            # Set initial pose and start navigation    
            try:
                self.log_to_web('Server info', 'Setting initial pose on navigator...')
                self.navigator.setInitialPose(initial_pose)
                self.log_to_web('Server info', 'Initial pose set successfully')
                
                self.log_to_web('Server info', 'Starting waypoint navigation...')
                # Start navigation for all waypoints using a simple for loop
                self.navigate_to_waypoints()
                
            except Exception as e:
                self.log_to_web('Server error', f'Failed to set initial pose or start navigation: {e}')
                self.navigation_active = False
                return
            
        except Exception as e:
            self.log_to_web('Server error', f'Failed to initialize navigation: {e}')
            self.navigation_active = False
            return

    def navigate_to_waypoints(self):
        """
        Navigate through waypoints using a simple for loop iteration
        """
        try:
            waypoint_keys = list(self.waypoint_data.keys())
            self.log_to_web('Server info', f'Starting navigation to {len(waypoint_keys)} waypoints: {waypoint_keys}')
            
            for index, waypoint_key in enumerate(waypoint_keys):
                if not self.navigation_active:
                    self.log_to_web('Server info', 'Navigation was canceled, stopping waypoint iteration')
                    break
                    
                coords = self.waypoint_data[waypoint_key]
                self.log_to_web('Server info', f'Navigating to waypoint {index+1}/{len(waypoint_keys)}: {waypoint_key} at {coords}')
                
                # Validate coordinates
                if not isinstance(coords, list) or len(coords) < 2:
                    self.log_to_web('Server warn', f'Invalid coordinates for {waypoint_key}, skipping')
                    continue
                    
                try:
                    x_coord = float(coords[0])
                    y_coord = float(coords[1])
                    
                    # Check for unreasonable coordinates
                    if abs(x_coord) > 100 or abs(y_coord) > 100:
                        self.log_to_web('Server warn', f'Coordinates for {waypoint_key} seem extreme: [{x_coord}, {y_coord}], skipping')
                        continue
                        
                except (ValueError, TypeError) as e:
                    self.log_to_web('Server error', f'Invalid coordinate format for {waypoint_key}: {e}, skipping')
                    continue
                
                # Create goal pose
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                
                goal_pose.pose.position.x = x_coord
                goal_pose.pose.position.y = y_coord
                goal_pose.pose.position.z = 0.0
                
                # Set orientation
                if len(coords) > 2 and coords[2] != 0:
                    if coords[2] > 0:
                        goal_pose.pose.orientation.z = 0.707
                        goal_pose.pose.orientation.w = 0.707
                    else:
                        goal_pose.pose.orientation.z = -0.707
                        goal_pose.pose.orientation.w = 0.707
                else:
                    goal_pose.pose.orientation.z = 0.0
                    goal_pose.pose.orientation.w = 1.0
                
                # Send goal to navigator
                try:
                    self.navigator.goToPose(goal_pose)
                    self.log_to_web('Server info', f'Goal sent to BasicNavigator for {waypoint_key}')
                    
                    # Brief pause to allow navigation system to process the goal
                    time.sleep(0.5)
                    
                    # Wait for navigation to complete with safer monitoring
                    result = self.wait_for_navigation_completion(waypoint_key)
                    
                    if result == 'success':
                        self.log_to_web('Server info', f'Successfully reached {waypoint_key}!')
                        
                        # Send success message and wait for confirmation if there are more waypoints
                        if index < len(waypoint_keys) - 1:  # Not the last waypoint
                            self.send_success_message(waypoint_key, index, len(waypoint_keys))
                            
                            # Wait for confirmation to proceed
                            if not self.wait_for_confirmation():
                                self.log_to_web('Server info', 'Navigation canceled by user or timeout')
                                break
                        else:
                            self.log_to_web('Server info', 'All waypoints completed successfully!')
                            
                    elif result == 'failed':
                        self.log_to_web('Server error', f'Navigation to {waypoint_key} failed, continuing to next waypoint')
                        continue
                    elif result == 'canceled':
                        self.log_to_web('Server info', f'Navigation to {waypoint_key} was canceled')
                        break
                        
                except Exception as e:
                    self.log_to_web('Server error', f'Failed to send goal for {waypoint_key}: {e}')
                    continue
                    
            self.log_to_web('Server info', 'Waypoint navigation completed')
            self.navigation_active = False
            
        except Exception as e:
            self.log_to_web('Server error', f'Error in waypoint navigation: {e}')
            self.navigation_active = False
    
    def wait_for_navigation_completion(self, waypoint_key):
        """
        Wait for navigation to complete using safer, timeout-based approach
        """
        start_time = time.time()
        timeout = 300  # 5 minutes timeout
        check_interval = 2.0  # Check every 2 seconds instead of aggressive polling
        last_feedback_time = start_time
        consecutive_errors = 0
        max_consecutive_errors = 5
        
        self.log_to_web('Server info', f'Starting safe navigation monitoring for {waypoint_key}')
        
        while True:
            try:
                if not self.navigation_active:
                    self.log_to_web('Server info', f'Navigation was deactivated, returning canceled')
                    return 'canceled'
                    
                elapsed = time.time() - start_time
                
                # Timeout check
                if elapsed > timeout:
                    self.log_to_web('Server warn', f'Navigation to {waypoint_key} timed out after {timeout} seconds')
                    try:
                        self.navigator.cancelTask()
                    except Exception as e:
                        self.log_to_web('Server warn', f'Error canceling timed out task: {e}')
                    return 'failed'
                
                # Try to check completion status safely
                task_complete = False
                try:
                    task_complete = self.navigator.isTaskComplete()
                    consecutive_errors = 0  # Reset error counter on success
                except Exception as e:
                    consecutive_errors += 1
                    self.log_to_web('Server warn', f'Error checking task completion (attempt {consecutive_errors}): {e}')
                    
                    if consecutive_errors >= max_consecutive_errors:
                        self.log_to_web('Server error', f'Too many consecutive errors checking navigation status, assuming failure')
                        return 'failed'
                    
                    # Sleep longer on errors to avoid hammering the system
                    time.sleep(check_interval * 2)
                    continue
                
                # If task is complete, get the result
                if task_complete:
                    self.log_to_web('Server info', f'Navigation task completed for {waypoint_key}')
                    try:
                        result = self.navigator.getResult()
                        self.log_to_web('Server info', f'Navigation result for {waypoint_key}: {result}')
                        
                        if result == TaskResult.SUCCEEDED:
                            return 'success'
                        elif result == TaskResult.CANCELED:
                            return 'canceled'
                        else:
                            return 'failed'
                    except Exception as e:
                        self.log_to_web('Server error', f'Error getting navigation result: {e}')
                        return 'failed'
                
                # Get feedback periodically (but safely)
                if elapsed - last_feedback_time >= 10.0:  # Every 10 seconds
                    try:
                        feedback = self.navigator.getFeedback()
                        if feedback and hasattr(feedback, 'distance_remaining'):
                            distance = feedback.distance_remaining
                            self.log_to_web('Server info', f'Distance to {waypoint_key}: {distance:.2f}m (elapsed: {elapsed:.1f}s)')
                            
                            # Consider very close as success (backup detection)
                            if distance < 0.3:
                                self.log_to_web('Server info', f'Very close to {waypoint_key} ({distance:.2f}m), considering success')
                                return 'success'
                                
                            last_feedback_time = elapsed + start_time
                        else:
                            self.log_to_web('Server info', f'Navigation progress: {elapsed:.1f}s elapsed')
                            last_feedback_time = elapsed + start_time
                            
                    except Exception as e:
                        self.log_to_web('Server warn', f'Error getting feedback: {e}')
                        last_feedback_time = elapsed + start_time
                
                # Sleep before next check
                time.sleep(check_interval)
                
            except Exception as e:
                self.log_to_web('Server error', f'Unexpected error in navigation monitoring: {e}')
                consecutive_errors += 1
                if consecutive_errors >= max_consecutive_errors:
                    self.log_to_web('Server error', f'Too many errors in navigation monitoring, giving up')
                    return 'failed'
                time.sleep(check_interval * 2)

    def wait_for_confirmation(self):
        """
        Wait for user confirmation to proceed to next waypoint
        """
        self.waiting_for_confirmation = True
        confirmation_timeout = 30  # 30 seconds timeout
        start_time = time.time()
        
        while self.waiting_for_confirmation:
            if not self.navigation_active:
                return False
                
            elapsed = time.time() - start_time
            if elapsed > confirmation_timeout:
                self.log_to_web('Server info', 'Confirmation timeout, proceeding automatically')
                self.waiting_for_confirmation = False
                return True
                
            time.sleep(0.1)  # Check every 0.1 seconds
            
        return True  # Confirmation received

    def send_success_message(self, waypoint_key, current_index, total_waypoints):
        """
        Send a message to the web UI that navigation to a waypoint has succeeded
        """
        try:
            next_index = current_index + 1
            has_next = next_index < total_waypoints
            
            # Generate a unique confirmation ID
            confirmation_id = f"confirm_{int(time.time())}_{current_index}"
            self.current_confirmation_id = confirmation_id
            
            # Create the message
            message = {
                "current_waypoint": waypoint_key,
                "current_index": current_index,
                "navigation_success": True,
                "has_next_waypoint": has_next,
                "next_waypoint": f"waypoint_{next_index+1}" if has_next else "none",
                "awaiting_confirmation": has_next,
                "timestamp": time.time(),
                "confirmation_id": confirmation_id
            }
            
            # Send the message
            msg = String()
            msg.data = json.dumps(message)
            self.nav_success_publisher.publish(msg)
            
            self.log_to_web('Server info', f'Sent success message for waypoint {waypoint_key} ({current_index+1}/{total_waypoints})')
            
        except Exception as e:
            self.log_to_web('Server error', f'Error sending success message: {e}')

    # Create a subscriber to receive confirmation from web UI
    def create_confirmation_subscriber(self):
        """
        Create a subscription to receive confirmation messages from the web UI
        """
        self.confirmation_subscription = self.create_subscription(
            String,
            'navigation_confirmation',
            self.confirmation_callback,
            10
        )
        self.log_to_web('Server info', 'Created subscription for navigation confirmations')
    
    def confirmation_callback(self, msg):
        """
        Handle confirmation messages from the web UI
        """
        self.log_to_web('Server info', f'Received confirmation message: {msg.data}')
        
        try:
            data = json.loads(msg.data)
            self.log_to_web('Server info', f'Parsed confirmation data: {data}')
            
            # Check if we are waiting for confirmation
            if not self.waiting_for_confirmation:
                self.log_to_web('Server info', 'Received confirmation message but not waiting for confirmation')
                return
                
            # Validate the confirmation ID if present
            if 'confirmation_id' in data and hasattr(self, 'current_confirmation_id'):
                if data['confirmation_id'] != self.current_confirmation_id:
                    self.log_to_web('Server warn', f'Received confirmation with mismatched ID. Expected: {self.current_confirmation_id}, Got: {data["confirmation_id"]}')
                    return
            
            # Process the confirmation
            if 'confirmed' in data:
                if data['confirmed']:
                    self.log_to_web('Server info', 'Received confirmation to proceed to next waypoint')
                    self.waiting_for_confirmation = False
                else:
                    self.log_to_web('Server info', 'User canceled navigation to next waypoint')
                    self.waiting_for_confirmation = False
                    self.navigation_active = False  # This will stop the navigation loop
            else:
                self.log_to_web('Server warn', f'Confirmation message missing "confirmed" field: {data}')
                
        except json.JSONDecodeError as e:
            self.log_to_web('Server error', f'Error parsing confirmation message: {e}')
        except Exception as e:
            self.log_to_web('Server error', f'Error handling confirmation message: {e}')

    # Note: We no longer need _wait_for_nav2_with_timeout and _continue_navigation_setup
    # since we handle this directly in _process_navigation_request
    
    def cancel_navigation(self):
        """Cancel the current navigation task and clean up resources safely"""
        try:
            # First mark navigation as inactive to stop any ongoing navigation
            self.navigation_active = False
            self.log_to_web('Server info', 'Canceling navigation...')
            
            # Cancel any active navigation task
            if hasattr(self, 'navigator') and self.navigator is not None:
                try:
                    self.navigator.cancelTask()
                    self.log_to_web('info', 'Navigation task canceled')
                except Exception as cancel_error:
                    self.log_to_web('Server warn', f'Error canceling navigation task: {cancel_error}')
                
                # Clean up navigator
                try:
                    self.navigator.lifecycleShutdown()
                    self.log_to_web('info', 'Navigator lifecycle shutdown initiated.')
                except Exception as shutdown_error:
                    self.log_to_web('Server warn', f'Error during navigator lifecycle shutdown: {shutdown_error}')
                self.navigator = None
                self.log_to_web('info', 'Navigator cleaned up')
            
            # Reset confirmation state
            self.waiting_for_confirmation = False
            self.current_confirmation_id = None
            # Force cleanup
            self.navigation_active = False
            if hasattr(self, 'navigator') and self.navigator is not None:
                try:
                    self.navigator.lifecycleShutdown()
                except Exception:
                    pass  # Ignore errors during forced cleanup
            self.navigator = None
        except Exception as e:
            self.log_to_web('Server error', f'Error during navigation cancellation: {e}')
            # Force cleanup
            self.navigation_active = False
            self.navigator = None
    def log_to_web(self, level, message):
        """
        Send log messages to both the ROS logger and the web console
        level: 'info', 'warn', 'error', or 'debug'
        message: The message to log
        """
        # Log to ROS logger first
        if level == 'Server info':
            self.get_logger().info(message)
        elif level == 'Server warn':
            self.get_logger().warn(message)
        elif level == 'Server error':
            self.get_logger().error(message)
        elif level == 'Server debug':
            self.get_logger().debug(message)
        
        # Format a timestamped message for the web console (browser console)
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        web_message = {
            "timestamp": timestamp,
            "level": level,
            "message": message,
            "node": "launch_service_node"
        }
        
        # Publish the message to the web_console_logs topic
        msg = String()
        msg.data = json.dumps(web_message)
        self.web_log_publisher.publish(msg)

    def launch_with_streaming(self, cmd, process_name):
        """Launch a command with optional output streaming to terminal"""
        if self.stream_output:
            # Stream output directly to terminal for real-time debugging
            process = subprocess.Popen(cmd)
            self.get_logger().info(f'{process_name} launched with output streaming to terminal')
        else:
            # Capture output (old behavior)
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.get_logger().info(f'{process_name} launched with output capture')
        return process

    def start_robot_callback(self, request, response):
        if request.data:  # Start robot
            try:
                os.chdir('/home/viju/ros_ws')  # Change to ROS workspace directory
                
                # Launch robot bringup
                cmd = ['bash', '-c', 'source ~/.bashrc && source install/setup.bash && ros2 launch linorobot2_bringup bringup.launch.py magwick:=false orientation_stddev:=0.01']
                process = self.launch_with_streaming(cmd, 'Robot')
                
                self.processes['robot'] = process
                response.success = True
                response.message = "Server message: Robot launched successfully"
                self.get_logger().info('Robot launched')
                
            except Exception as e:
                response.success = False
                response.message = f"Server message: Failed to launch robot: {str(e)}"
                self.get_logger().error(f'Robot launch failed: {e}')
        
        else:  # Stop robot
            if 'robot' in self.processes:
                self.processes['robot'].terminate() #Ensure the process has terminated
                self.processes['robot'].send_signal(signal.SIGINT) # ctrl+c 
                del self.processes['robot']
            
            # Kill any remaining robot processes
            subprocess.run(['pkill', '-f', 'robot'], capture_output=True)
            
            response.success = True
            response.message = "Robot stopped"
            self.get_logger().info('Robot stopped')
        
        return response
    
    def slam_callback(self, request, response):
        if request.data:  # Launch SLAM
            try:
                # Change to ROS workspace directory
                os.chdir('/home/viju/linorobot2_ws')
                
                # Source ROS setup and launch SLAM
                cmd = ['bash', '-c', 'source ~/.bashrc && source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py']
                process = self.launch_with_streaming(cmd, 'SLAM')
                
                self.processes['slam'] = process
                response.success = True
                response.message = "SLAM launched successfully"
                self.get_logger().info('SLAM launched')
                
            except Exception as e:
                response.success = False
                response.message = f"Failed to launch SLAM: {str(e)}"
                self.get_logger().error(f'SLAM launch failed: {e}')
        
        else:  # Stop SLAM
            if 'slam' in self.processes:
                self.processes['slam'].terminate()
                del self.processes['slam']
            
            # Kill any remaining SLAM processes
            subprocess.run(['pkill', '-f', 'slam'], capture_output=True)
            
            response.success = True
            response.message = "SLAM stopped"
            self.get_logger().info('SLAM stopped')
        
        return response
    
    def nav_callback(self, request, response):
        if request.data:  # Launch Navigation
            try:
                os.chdir('/home/viju/ros_ws')  # Use ros_ws to match your environment
                
                # Check if map exists
                map_path = '/home/viju/ros_ws/maha_16m.yaml'  # Look for the map file you mentioned earlier
                map_exists = os.path.exists(map_path)
                
                if map_exists:
                    self.log_to_web('info', f'Using map: {map_path}')
                    # Launch navigation stack with map parameter
                    cmd = ['bash', '-c', f'source ~/.bashrc && source install/setup.bash && ros2 launch nav2_bringup navigation_launch.py map:={map_path}']
                else:
                    self.log_to_web('Server warn', f'Map file not found at {map_path}. Using default settings.')
                    # Launch navigation stack without map parameter - may fail
                    cmd = ['bash', '-c', 'source ~/.bashrc && source install/setup.bash && ros2 launch nav2_bringup navigation_launch.py']
                    
                process = self.launch_with_streaming(cmd, 'Navigation')
                
                self.processes['navigation'] = process
                response.success = True
                response.message = "Server message: Navigation launched successfully"
                self.get_logger().info('Navigation launched')
                
            except Exception as e:
                response.success = False
                response.message = f"Server message: Failed to launch Navigation: {str(e)}"
                self.get_logger().error(f'Navigation launch failed: {e}')
        
        else:  # Stop Navigation
            if 'navigation' in self.processes:
                self.processes['navigation'].terminate()
                del self.processes['navigation']
            
            # Kill navigation processes
            subprocess.run(['pkill', '-f', 'nav2'], capture_output=True)
            
            response.success = True
            response.message = "From server: -Navigation stopped"
            self.get_logger().info('Navigation stopped')
        
        return response
    
    def kill_callback(self, request, response):
        try:
            # Stop any active navigation
            self.navigation_active = False
            self.log_to_web('Server info', 'Stopping any active navigation')
            
            # Kill all tracked processes
            for name, process in self.processes.items():
                process.terminate()
            
            self.processes.clear()
            
            # Kill common ROS navigation processes
            processes_to_kill = ['slam', 'nav2', 'navigation', 'slam_toolbox']
            for proc in processes_to_kill:
                subprocess.run(['pkill', '-f', proc], capture_output=True)
            
            response.success = True
            response.message = "All launches killed"
            self.get_logger().info('All launches killed')
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to kill processes: {str(e)}"
            self.get_logger().error(f'Kill failed: {e}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LaunchServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Received keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            # Clean up processes on shutdown
            if hasattr(node, 'processes'):
                for process in node.processes.values():
                    try:
                        process.terminate()
                    except:
                        pass  # Ignore errors during cleanup
            
            # First shut down the node cleanly
            node.destroy_node()
            
            # Then attempt to shutdown rclpy if not already shut down
            try:
                rclpy.shutdown()
            except Exception as shutdown_err:
                print(f"Note: RCL already shut down: {shutdown_err}")
        except Exception as e:
            print(f"Error during cleanup: {e}")

if __name__ == '__main__':
    main()
