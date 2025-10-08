#!/usr/bin/env python3
"""
Service Manager for Robot Control Web Interface
Provides HTTP endpoints to start/stop/check system services
"""

import os
import json
import signal
import subprocess
import psutil
import csv
import io
import socket
import atexit
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import threading
import time

class ServiceManager:
    def __init__(self):
        self.processes = {}
        self.ros_workspace = "/home/viju/ros_ws"
        # Register cleanup function to run when program exits
        atexit.register(self.cleanup_all_processes)
        
        # Set up signal handlers for proper cleanup
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        """Handle termination signals to clean up processes"""
        print(f"\n🛑 Received signal {sig}, cleaning up processes...")
        self.cleanup_all_processes()
        # Let the default signal handler take over after cleanup
        signal.signal(sig, signal.SIG_DFL)
        os.kill(os.getpid(), sig)
        
    def cleanup_all_processes(self):
        """Clean up all managed processes"""
        print("🧹 Cleaning up all managed processes...")
        
        # First, try to terminate our tracked processes
        for name, process in list(self.processes.items()):
            try:
                print(f"📝 Terminating {name} process...")
                process.terminate()
                try:
                    process.wait(timeout=3)
                    print(f"✅ {name} terminated gracefully")
                except subprocess.TimeoutExpired:
                    print(f"⚠️ {name} didn't terminate, forcing kill...")
                    process.kill()
                    process.wait(timeout=2)
                    print(f"✅ {name} killed")
            except Exception as e:
                print(f"❌ Error terminating {name}: {e}")
            
            self.processes.pop(name, None)
        
        # Then find and kill any stray processes
        self.kill_stray_processes()
        
    def kill_stray_processes(self):
        """Find and kill any stray processes we might have started"""
        target_processes = [
            # Add process patterns to look for
            "rosbridge",
            "launch_service_node.py",
            "slam_toolbox"
        ]
        
        for pattern in target_processes:
            try:
                print(f"🔍 Looking for stray '{pattern}' processes...")
                for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                    try:
                        cmdline = ' '.join(proc.info['cmdline'] or [])
                        if pattern in cmdline and proc.pid != os.getpid():
                            print(f"🔥 Killing stray process: {proc.pid} - {cmdline[:60]}...")
                            proc.terminate()
                            try:
                                proc.wait(timeout=3)
                            except psutil.TimeoutExpired:
                                proc.kill()
                    except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                        print(f"⚠️ Error checking process: {e}")
            except Exception as e:
                print(f"❌ Error killing stray processes: {e}")
        
    def get_local_ip(self):
        """
        Get the local IP address of the machine in a more reliable way
        Tries multiple methods to detect the right IP address
        """
        # Method 1: Connect to external service to determine outgoing interface
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Doesn't actually connect but helps determine the outgoing IP
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            if not ip.startswith('127.'):
                # Make sure we don't have any unwanted characters in the IP
                ip = ip.strip().replace('/', '')
                print(f"Found IP using outgoing interface method: {ip}")
                return ip
        except Exception as e:
            print(f"Method 1 (outgoing interface) failed: {e}")
        
        # Method 2: Get all non-loopback IPv4 addresses
        try:
            for interface in socket.getaddrinfo(socket.gethostname(), None):
                # Filter for IPv4 addresses that are not loopback
                if interface[0] == socket.AF_INET and not interface[4][0].startswith('127.'):
                    ip = interface[4][0]
                    print(f"Found IP using hostname resolution: {ip}")
                    return ip
        except Exception as e:
            print(f"Method 2 (hostname resolution) failed: {e}")
        
        # Method 3: Get network interface information using subprocess
        try:
            # Try to get IP from 'ip' command
            cmd = "ip -4 addr | grep -v '127.0.0.1' | grep 'inet ' | awk '{print $2}' | cut -d/ -f1"
            result = subprocess.check_output(cmd, shell=True, text=True).strip().split('\n')[0]
            if result and not result.startswith('127.'):
                print(f"Found IP using ip command: {result}")
                return result
        except Exception as e:
            print(f"Method 3 (ip command) failed: {e}")
            
            # Fallback to ifconfig if ip command failed
            try:
                cmd = "ifconfig | grep 'inet ' | grep -v '127.0.0.1' | awk '{print $2}' | cut -d: -f2"
                result = subprocess.check_output(cmd, shell=True, text=True).strip().split('\n')[0]
                if result and not result.startswith('127.'):
                    print(f"Found IP using ifconfig command: {result}")
                    return result
            except Exception as e:
                print(f"Method 3b (ifconfig command) failed: {e}")
        
        # Final fallback: Try to get hostname-based IP
        try:
            ip = socket.gethostbyname(socket.getfqdn())
            if not ip.startswith('127.'):
                print(f"Found IP using FQDN resolution: {ip}")
                return ip
        except Exception as e:
            print(f"Method 4 (FQDN) failed: {e}")
        
        # Ultimate fallback
        print("All IP detection methods failed, defaulting to localhost")
        return "localhost"
        
    def source_ros_environment(self):
        """Get ROS environment variables"""
        env = os.environ.copy() #this give out put of environment variables of the current process ,for example PATH,HOME,USER etc.
        
        # Source ROS2 setup
        setup_files = [
            "/opt/ros/jazzy/setup.bash",
            "/home/viju/ros_ws/install/setup.bash"
        ]
        
        # Build the source command
        source_cmd = " && ".join([f"source {f}" for f in setup_files if os.path.exists(f)])
        
        # Get environment after sourcing
        if source_cmd:
            try:
                result = subprocess.run(
                    f"bash -c '{source_cmd} && env'",
                    shell=True, capture_output=True, text=True
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if '=' in line:
                            key, value = line.split('=', 1)
                            env[key] = value
            except Exception as e:
                print(f"Warning: Could not source ROS environment: {e}")
        
        return env
    
    def start_launch_service(self):
        """Start the launch service node"""
        try:
            # Check if already running using our better process check
            check_result = self.check_launch_service()
            if check_result.get('running', False):
                return {'success': True, 'pid': check_result.get('pid'), 'message': 'Launch service already running'}
            
            # Make sure any stray launch service is stopped first
            self.stop_launch_service(quiet=True)
            
            env = self.source_ros_environment()
            
            # Start launch service
            cmd = ['python3', '/home/viju/websocket_ws/launch_service_node.py']
            process = subprocess.Popen(
                cmd,
                env=env,
                cwd=self.ros_workspace,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # Wait a moment to check if it started successfully
            time.sleep(1)
            if process.poll() is None:  # Still running
                self.processes['launch_service'] = process
                return {'success': True, 'pid': process.pid, 'message': 'Launch service started successfully'}
            else:
                stdout, stderr = process.communicate()
                return {'success': False, 'error': f'Failed to start: {stderr.decode()}'}
                
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def stop_launch_service(self, quiet=False):
        """Stop the launch service node"""
        return self.stop_process('launch_service', quiet)
    
    def start_rosbridge(self):
        """Start rosbridge server"""
        try:
            # Check if already running using our process check
            check_result = self.check_rosbridge()
            if check_result.get('running', False):
                return {'success': True, 'pid': check_result.get('pid'), 'message': 'Rosbridge already running'}
            
            # Create a clean environment with just the system ROS setup
            env = os.environ.copy()
            
            # Use only system ROS setup to ensure rosbridge can be found
            ros_setup = "/opt/ros/jazzy/setup.bash"
            if os.path.exists(ros_setup):
                # Source ROS environment using a subprocess
                cmd = f"bash -c 'source {ros_setup} && env'"
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
                for line in proc.stdout:
                    (key, _, value) = line.decode().partition("=")
                    if key.strip():
                        env[key.strip()] = value.strip()
                proc.communicate()
            
            # First, make sure any old rosbridge processes are stopped
            self.stop_rosbridge(quiet=True)
            
            # Start rosbridge
            cmd = ['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml']
            process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # Wait a moment to check if it started successfully
            time.sleep(2)
            if process.poll() is None:  # Still running
                self.processes['rosbridge'] = process
                # Check if we can connect to the port to verify it's actually working
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(1)
                    s.connect(('localhost', 9090))
                    s.close()
                    return {'success': True, 'pid': process.pid, 'message': 'Rosbridge started successfully on port 9090'}
                except:
                    # Connection failed but process is running - return success but with warning
                    return {'success': True, 'pid': process.pid, 'message': 'Rosbridge process started, but port 9090 not responding yet'}
            else:
                stdout, stderr = process.communicate()
                return {'success': False, 'error': f'Failed to start: {stderr.decode()}'}
                
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def stop_rosbridge(self, quiet=False):
        """Stop rosbridge server"""
        return self.stop_process('rosbridge', quiet)
    
    def check_launch_service(self):
        """Check if launch service is running"""
        try:
            # First check if we have it in our tracked processes
            if 'launch_service' in self.processes:
                process = self.processes['launch_service']
                if process.poll() is None:  # Process is still running
                    return {'running': True, 'pid': process.pid, 'tracked': True}
                else:
                    # Process has terminated
                    del self.processes['launch_service']
            
            # If not in our tracked processes or our tracked process died, look for it in system processes
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    if 'launch_service_node.py' in cmdline:
                        # Found it in system processes
                        return {'running': True, 'pid': proc.info['pid'], 'tracked': False}
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            # Not found in our tracking or system processes
            return {'running': False}
        except Exception as e:
            return {'running': False, 'error': str(e)}
    
    def check_rosbridge(self):
        """Check if rosbridge is running"""
        try:
            # First check if we have it in our tracked processes
            if 'rosbridge' in self.processes:
                process = self.processes['rosbridge']
                if process.poll() is None:  # Process is still running
                    # Try to connect to verify it's actually working
                    try:
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.settimeout(1)
                        s.connect(('localhost', 9090))
                        s.close()
                        return {'running': True, 'pid': process.pid, 'status': 'responsive'}
                    except:
                        return {'running': True, 'pid': process.pid, 'status': 'process running but port not responding'}
                else:
                    # Process has terminated
                    del self.processes['rosbridge']
            
            # If not in our tracked processes or our tracked process died, look for it in system processes
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    if 'rosbridge' in cmdline.lower() and 'websocket' in cmdline.lower():
                        # Found rosbridge in system processes
                        # Try to connect to verify it's actually working
                        try:
                            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            s.settimeout(1)
                            s.connect(('localhost', 9090))
                            s.close()
                            return {'running': True, 'pid': proc.info['pid'], 'status': 'responsive'}
                        except:
                            return {'running': True, 'pid': proc.info['pid'], 'status': 'process running but port not responding'}
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            # Not found in our tracking or system processes
            return {'running': False}
        except Exception as e:
            return {'running': False, 'error': str(e)}
    
    def check_ros_topics(self):
        """Check ROS topics and their publishers/subscribers"""
        try:
            env = self.source_ros_environment()
            
            # Run ros2 topic list
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            if result.returncode != 0:
                return {'success': False, 'error': result.stderr}
                
            topics = result.stdout.strip().split('\n')
            
            # Get details for cmd_vel topic
            cmd_vel_info = {'publishers': [], 'subscribers': []}
            if '/cmd_vel' in topics:
                # Check publishers
                pub_result = subprocess.run(
                    ['ros2', 'topic', 'info', '-v', '/cmd_vel'],
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                
                if pub_result.returncode == 0:
                    info_lines = pub_result.stdout.strip().split('\n')
                    publishers = []
                    subscribers = []
                    
                    current_section = None
                    for line in info_lines:
                        if 'Publishers:' in line:
                            current_section = 'publishers'
                        elif 'Subscribers:' in line:
                            current_section = 'subscribers'
                        elif current_section and line.strip():
                            if current_section == 'publishers':
                                publishers.append(line.strip())
                            else:
                                subscribers.append(line.strip())
                    
                    cmd_vel_info = {
                        'publishers': publishers,
                        'subscribers': subscribers
                    }
            
            return {
                'success': True, 
                'topics': topics,
                'cmd_vel': cmd_vel_info
            }
            
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def echo_cmd_vel(self):
        """Echo the cmd_vel topic to see the most recent messages"""
        try:
            env = self.source_ros_environment()
            
            # Run ros2 topic echo with a timeout to get a sample of messages
            process = subprocess.Popen(
                ['ros2', 'topic', 'echo', '--once', '/cmd_vel'],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            try:
                stdout, stderr = process.communicate(timeout=3)
                if process.returncode == 0 and stdout.strip():
                    return {'success': True, 'message': stdout}
                elif stderr:
                    return {'success': False, 'error': stderr}
                else:
                    return {'success': False, 'error': 'No messages received on /cmd_vel'}
            except subprocess.TimeoutExpired:
                process.kill()
                return {'success': False, 'error': 'Timeout waiting for cmd_vel message'}
                
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def is_process_running(self, name_pattern):
        """Check if a process with given name pattern is running
        
        Args:
            name_pattern: String pattern to search for in command line
            
        Returns:
            dict: Status with 'running' flag and 'pid' if found
        """
        # First check if we have it in our tracked processes
        for process_name, process in self.processes.items():
            if name_pattern in process_name and process.poll() is None:
                return {'running': True, 'pid': process.pid, 'tracked': True}
        
        # If not in our tracked processes, look in system processes
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline'] or [])
                if name_pattern in cmdline:
                    return {'running': True, 'pid': proc.info['pid'], 'tracked': False}
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
                
        return {'running': False}
        
    def stop_process(self, process_name, quiet=False):
        """Generic method to stop a process by name
        
        Args:
            process_name: Name of the process to stop (used as key in self.processes)
            quiet: If True, suppress logging output
            
        Returns:
            dict: Result with success status and message
        """
        try:
            if not quiet:
                print(f"📝 Stopping {process_name} process...")
                
            # First check if we have it in our tracked processes
            if process_name in self.processes:
                process = self.processes[process_name]
                if process.poll() is None:  # Process is still running
                    if not quiet:
                        print(f"📝 Terminating tracked {process_name} process {process.pid}...")
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                        if not quiet:
                            print(f"✅ Terminated {process_name} process {process.pid}")
                        killed = True
                    except subprocess.TimeoutExpired:
                        if not quiet:
                            print(f"⚠️ {process_name} process {process.pid} didn't terminate, force killing...")
                        process.kill()
                        process.wait(timeout=2)
                        if not quiet:
                            print(f"✅ Force killed {process_name} process {process.pid}")
                        killed = True
                    
                    # Remove from our tracking
                    del self.processes[process_name]
                    return {'success': True, 'killed': True, 'message': f"{process_name} process stopped"}
                else:
                    # Process already terminated
                    if not quiet:
                        print(f"ℹ️ {process_name} process was already terminated")
                    del self.processes[process_name]
                    return {'success': True, 'killed': False, 'message': f"{process_name} process was already terminated"}
            
            # If not in our tracking, look for it by name pattern in system processes
            killed = False
            pattern = process_name
            
            # For specific process types, use custom patterns
            if process_name == 'navigation':
                pattern = 'slam_toolbox'
            elif process_name == 'rosbridge':
                pattern = 'rosbridge_websocket'
            elif process_name == 'launch_service':
                pattern = 'launch_service_node.py'
            
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    if pattern in cmdline:
                        if not quiet:
                            print(f"📝 Terminating {process_name} process {proc.pid}...")
                        proc.terminate()
                        try:
                            proc.wait(timeout=5)
                            if not quiet:
                                print(f"✅ Terminated {process_name} process {proc.pid}")
                            killed = True
                        except psutil.TimeoutExpired:
                            if not quiet:
                                print(f"⚠️ {process_name} process {proc.pid} didn't terminate, force killing...")
                            proc.kill()
                            proc.wait(timeout=2)
                            if not quiet:
                                print(f"✅ Force killed {process_name} process {proc.pid}")
                            killed = True
                except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                    if not quiet:
                        print(f"⚠️ Error accessing process: {e}")
                    
            if killed:
                if not quiet:
                    print(f"✅ {process_name} process stopped")
                return {'success': True, 'killed': killed, 'message': f"{process_name} process stopped"}
            else:
                if not quiet:
                    print(f"ℹ️ No running {process_name} process found")
                return {'success': True, 'killed': False, 'message': f"No running {process_name} process found"}
            
        except Exception as e:
            error_msg = f"Error stopping {process_name} process: {e}"
            if not quiet:
                print(f"❌ {error_msg}")
            return {'success': False, 'error': error_msg}
    
    # Supermarket items management functions
    def get_items_csv_path(self):
        """Get the path to the items CSV file"""
        return os.path.join('/home/viju/websocket_ws', 'supermarket_items.csv')
    
    def get_items(self):
        """Get all supermarket items from CSV file"""
        items_path = self.get_items_csv_path()
        
        # If file doesn't exist, create with default items
        if not os.path.exists(items_path):
            self.initialize_default_items()
        
        items = []
        try:
            with open(items_path, 'r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    # Convert numeric values
                    row['id'] = int(row['id'])
                    row['x'] = float(row['x'])
                    row['y'] = float(row['y'])
                    items.append(row)
        except Exception as e:
            print(f"Error reading items CSV: {e}")
            # Return empty list on error
            return []
            
        return items
    
    def initialize_default_items(self):
        """Create CSV file with default items"""
        items_path = self.get_items_csv_path()
        
        default_items = [
            {'id': 1, 'name': 'Salt', 'section': 'Spices', 'x': 2.5, 'y': 3.2},
            {'id': 2, 'name': 'Sugar', 'section': 'Baking', 'x': 3.8, 'y': 2.7},
            {'id': 3, 'name': 'Milk', 'section': 'Dairy', 'x': 5.2, 'y': 4.1},
            {'id': 4, 'name': 'Bread', 'section': 'Bakery', 'x': 1.5, 'y': 6.8},
            {'id': 5, 'name': 'Eggs', 'section': 'Dairy', 'x': 5.5, 'y': 4.5},
            {'id': 6, 'name': 'Rice', 'section': 'Grains', 'x': 7.2, 'y': 3.3},
            {'id': 7, 'name': 'Pasta', 'section': 'Grains', 'x': 7.8, 'y': 3.8},
            {'id': 8, 'name': 'Tomatoes', 'section': 'Produce', 'x': 2.2, 'y': 8.5},
            {'id': 9, 'name': 'Cereal', 'section': 'Breakfast', 'x': 4.6, 'y': 1.5},
            {'id': 10, 'name': 'Coffee', 'section': 'Beverages', 'x': 8.5, 'y': 2.2}
        ]
        
        try:
            with open(items_path, 'w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=['id', 'name', 'section', 'x', 'y'])
                writer.writeheader()
                writer.writerows(default_items)
        except Exception as e:
            print(f"Error creating default items CSV: {e}")
    
    def save_item(self, item_data):
        """Save or update an item in the CSV file"""
        items = self.get_items()
        
        # Validate required fields
        required_fields = ['name', 'section', 'x', 'y']
        for field in required_fields:
            if field not in item_data:
                return {'success': False, 'error': f'Missing required field: {field}'}
        
        try:
            # Convert to proper types
            if 'x' in item_data:
                item_data['x'] = float(item_data['x'])
            if 'y' in item_data:
                item_data['y'] = float(item_data['y'])
            
            # Check if item already exists
            existing_item = next((item for item in items if item['name'].lower() == item_data['name'].lower()), None)
            
            if existing_item:
                # Update existing item
                existing_item['section'] = item_data['section']
                existing_item['x'] = item_data['x']
                existing_item['y'] = item_data['y']
            else:
                # Add new item with next available ID
                next_id = 1
                if items:
                    next_id = max(item['id'] for item in items) + 1
                
                items.append({
                    'id': next_id,
                    'name': item_data['name'],
                    'section': item_data['section'],
                    'x': item_data['x'],
                    'y': item_data['y']
                })
            
            # Save all items back to CSV
            self._save_items_to_csv(items)
            
            return {'success': True, 'action': 'updated' if existing_item else 'added'}
            
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def delete_item(self, item_name):
        """Delete an item from the CSV file"""
        items = self.get_items()
        
        try:
            # Find item index
            item_index = next((i for i, item in enumerate(items) 
                              if item['name'].lower() == item_name.lower()), -1)
            
            if item_index == -1:
                return {'success': False, 'error': 'Item not found'}
            
            # Remove item
            items.pop(item_index)
            
            # Save remaining items
            self._save_items_to_csv(items)
            
            return {'success': True}
            
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def _save_items_to_csv(self, items):
        """Helper to save items list to CSV file"""
        items_path = self.get_items_csv_path()
        
        with open(items_path, 'w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=['id', 'name', 'section', 'x', 'y'])
            writer.writeheader()
            writer.writerows(items)

class ServiceHTTPHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, service_manager=None, **kwargs):
        self.service_manager = service_manager
        super().__init__(*args, **kwargs)
    
    def do_POST(self):
        """Handle POST requests for service management"""
        path = self.path
        
        if path == '/start_launch_service':
            result = self.service_manager.start_launch_service()
        elif path == '/stop_launch_service':
            result = self.service_manager.stop_launch_service()
        elif path == '/start_rosbridge':
            result = self.service_manager.start_rosbridge()
        elif path == '/stop_rosbridge':
            result = self.service_manager.stop_rosbridge()
        elif path == '/save_item':
            # Handle item save request
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length).decode('utf-8')
            try:
                item_data = json.loads(post_data)
                result = self.service_manager.save_item(item_data)
            except json.JSONDecodeError:
                result = {'success': False, 'error': 'Invalid JSON data'}
        elif path == '/delete_item':
            # Handle item delete request
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length).decode('utf-8')
            try:
                data = json.loads(post_data)
                if 'name' in data:
                    result = self.service_manager.delete_item(data['name'])
                else:
                    result = {'success': False, 'error': 'Item name not provided'}
            except json.JSONDecodeError:
                result = {'success': False, 'error': 'Invalid JSON data'}
        else:
            result = {'success': False, 'error': 'Unknown endpoint'}
        
        # Send response
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(result).encode())
    
    def do_GET(self):
        """Handle GET requests"""
        path = self.path
        
        if path == '/check_launch_service':
            result = self.service_manager.check_launch_service()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        elif path == '/check_rosbridge':
            result = self.service_manager.check_rosbridge()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        elif path == '/check_ros_topics':
            result = self.service_manager.check_ros_topics()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        elif path == '/echo_cmd_vel':
            result = self.service_manager.echo_cmd_vel()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        elif path == '/get_ip':
            # Return the server IP address for connecting from external devices
            ip = self.service_manager.get_local_ip()
            
            # Ensure the IP is clean without any trailing or extra characters
            # if ip:
            #     ip = ip.strip().replace('/', '')
            #     # Validate that it's a proper IP address
            #     try:
            #         socket.inet_aton(ip)  # This will raise an exception for invalid IPs
            #     except (socket.error, OSError):
            #         print(f"Invalid IP detected: '{ip}', falling back to localhost")
            #         ip = "localhost"
            
            print(f"Sending IP address to client: {ip}")
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({'ip': ip}).encode())
        elif path == '/get_items':
            # Serve supermarket items from CSV
            items = self.service_manager.get_items()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(items).encode())
        else:
            # Serve static files
            super().do_GET()
    
    def log_message(self, format, *args):
        """Override to reduce verbose logging"""
        if not any(x in self.path for x in ['/check_', '.css', '.js', '.ico']):
            super().log_message(format, *args)

def main():
    # Change to websocket_ws directory
    os.chdir('/home/viju/websocket_ws')
    
    # Create service manager
    service_manager = ServiceManager()
    
    # Auto-start rosbridge server on startup
    rosbridge_status = service_manager.check_rosbridge()
    if not rosbridge_status.get('running', False):
        print("🔄 Auto-starting rosbridge server...")
        result = service_manager.start_rosbridge()
        if result.get('success', False):
            print(f"✅ Rosbridge started automatically (PID: {result.get('pid')})")
        else:
            print(f"⚠️ Could not auto-start rosbridge: {result.get('error', 'Unknown error')}")
    else:
        print(f"ℹ️ Rosbridge already running (PID: {rosbridge_status.get('pid')})")
    
    # Try different ports if 8080 is unavailable
    port = 8080
    max_port_attempts = 10
    
    for attempt in range(max_port_attempts):
        try:
            # Create HTTP server with service management
            def handler(*args, **kwargs):
                return ServiceHTTPHandler(*args, service_manager=service_manager, **kwargs)
            
            server = HTTPServer(('0.0.0.0', port), handler)
            break
        except OSError as e:
            if "Address already in use" in str(e) and attempt < max_port_attempts - 1:
                print(f"⚠️ Port {port} is already in use, trying {port+1}...")
                port += 1
            else:
                raise
    
    print(f"🚀 Robot Control Service Manager started on http://0.0.0.0:{port}")
    
    # Get the local IP address for network access
    local_ip = service_manager.get_local_ip()
    print(f"🌐 Network access (local): http://localhost:{port}")
    print(f"🌐 Network access (LAN): http://{local_ip}:{port}/robot_control.html")
    
    print("📁 Serving files from: /home/viju/websocket_ws")
    print("🔧 Service management endpoints available")
    print(f"🛒 Supermarket item data stored in: {service_manager.get_items_csv_path()}")
    print("Press Ctrl+C to stop")
    
    # Create a pidfile to track this process
    try:
        with open('/tmp/service_manager.pid', 'w') as f:
            f.write(str(os.getpid()))
    except Exception as e:
        print(f"⚠️ Could not create PID file: {e}")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Shutting down service manager (KeyboardInterrupt)...")
    except Exception as e:
        print(f"\n🛑 Shutting down due to exception: {e}")
    finally:
        # Always clean up on exit
        print("🧹 Cleaning up before exit...")
        server.shutdown()
        service_manager.cleanup_all_processes()
        try:
            os.remove('/tmp/service_manager.pid')
        except:
            pass
        print("🏁 Service manager stopped")

if __name__ == '__main__':
    main()
