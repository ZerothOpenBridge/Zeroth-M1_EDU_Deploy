"""
UDP sender module for sending IMU data to PlotJuggler
"""

import time
import json
import socket
import threading
from queue import Queue
from pdb import set_trace as st

class UDPPlotJugglerSender:
    """UDP sender class for sending IMU data to PlotJuggler"""

    def __init__(self, host="192.168.51.48", port=9870, enable_udp=True):
        """
        Initialize UDP sender

        Args:
            host (str): Target host IP address
            port (int): Target port
            enable_udp (bool): Whether to enable UDP sending
        """
        self.host = host
        self.port = port
        self.enable_udp = enable_udp
        self.socket = None
        self.running = False
        self.send_thread = None
        self.data_queue = Queue()
        
        if self.enable_udp:
            self.init_socket()
    
    def init_socket(self):
        """Initialize UDP socket"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
            print(f"UDP socket initialized for {self.host}:{self.port}")
        except Exception as e:
            print(f"Failed to initialize UDP socket: {e}")
            self.socket = None
            self.enable_udp = False
    
    def start(self):
        """Start UDP sender"""
        if not self.enable_udp or not self.socket:
            print("UDP sender not enabled or socket not initialized")
            return False
        
        self.running = True
        self.send_thread = threading.Thread(target=self.send_thread_loop, daemon=True)
        self.send_thread.start()
        print("UDP sender started")
        return True
    
    def stop(self):
        """Stop UDP sender"""
        self.running = False
        
        if self.send_thread:
            self.send_thread.join(timeout=1.0)
        
        if self.socket:
            self.socket.close()
        
        print("UDP sender stopped")
    
    def send_imu_data(self, imu_data):
        """Queue IMU data for sending"""
        if not self.enable_udp:
            return
        
        try:
            # Add timestamp
            data_with_timestamp = {
                'timestamp': time.time(),
                'imu_data': imu_data
            }
            self.data_queue.put(data_with_timestamp)
        except Exception as e:
            print(f"Failed to queue IMU data: {e}")
    
    def send_thread_loop(self):
        """UDP send thread loop"""
        print(f"UDP send thread started for {self.host}:{self.port}")
        
        while self.running:
            try:
                # Fetch data from queue
                if not self.data_queue.empty():
                    data_with_timestamp = self.data_queue.get(timeout=0.1)
                    
                    # Build PlotJuggler-compatible data format
                    plotjuggler_data = self.format_for_plotjuggler(data_with_timestamp)
                    
                    # Send data
                    if self.socket:
                        self.socket.sendto(plotjuggler_data.encode('utf-8'), (self.host, self.port))
                
                time.sleep(0.01)  # 100Hz send rate
                
            except Exception as e:
                print(f"Error in UDP send thread: {e}")
                time.sleep(0.01)
    
    def format_for_plotjuggler(self, data_with_timestamp):
        """
        Format IMU data into PlotJuggler-compatible format

        Expected JSON format for PlotJuggler:
        {
            "topic": "imu_data",
            "data": {
                "timestamp": 1234567890.123,
                "roll": 0.1,
                "pitch": 0.2,
                "yaw": 0.3,
                "gyro_x": 0.01,
                "gyro_y": 0.02,
                "gyro_z": 0.03,
                "acc_x": 9.8,
                "acc_y": 0.1,
                "acc_z": 0.2,
                "quat_w": 1.0,
                "quat_x": 0.0,
                "quat_y": 0.0,
                "quat_z": 0.0
            }
        }
        """
        try:
            imu_data = data_with_timestamp['imu_data']
            timestamp = data_with_timestamp['timestamp']
            
            plotjuggler_msg = {
                "topic": "imu_data",
                "data": {
                    "timestamp": timestamp,
                    "imu_quat": [float(x) for x in imu_data.get('imu_quat', [1.0, 0.0, 0.0, 0.0])],
                    "rpy": [float(x) for x in imu_data['rpy']] if 'rpy' in imu_data else [],
                    "imu_angular_velocity_raw": [float(x) for x in imu_data.get('imu_angular_velocity_raw', [0.0, 0.0, 0.0])],
                    "joint_velocity_raw": [float(x) for x in imu_data['joint_velocity_raw']] if 'joint_velocity_raw' in imu_data else [],
                    "joint_velocity_filt": [float(x) for x in imu_data['joint_velocity_filt']] if 'joint_velocity_filt' in imu_data else [],
                    "joint_position": [float(x) for x in imu_data.get('joint_position', [])],
                    "gravity_in_body": [float(x) for x in imu_data['gravity_in_body']] if 'gravity_in_body' in imu_data else [],
                    "gyro_x_raw": float(imu_data.get('gyro_x_raw', 0.0)),
                    "gyro_y_raw": float(imu_data.get('gyro_y_raw', 0.0)),
                    "gyro_z_raw": float(imu_data.get('gyro_z_raw', 0.0)),
                    "gyro_x_filt": float(imu_data.get('gyro_x_filt', 0.0)),
                    "gyro_y_filt": float(imu_data.get('gyro_y_filt',0.0)),
                    "gyro_z_filt": float(imu_data.get('gyro_z_filt',0.0)),
                    "quat_w": float(imu_data.get('quat', imu_data.get('imu_quat', [1.0, 0.0, 0.0, 0.0]))[0]),
                    "quat_x": float(imu_data.get('quat', imu_data.get('imu_quat', [1.0, 0.0, 0.0, 0.0]))[1]),
                    "quat_y": float(imu_data.get('quat', imu_data.get('imu_quat', [1.0, 0.0, 0.0, 0.0]))[2]),
                    "quat_z": float(imu_data.get('quat', imu_data.get('imu_quat', [1.0, 0.0, 0.0, 0.0]))[3]),
                    "action": [float(x) for x in imu_data.get('action', [])] if 'action' in imu_data else [],
                }
            }
            
            return json.dumps(plotjuggler_msg)
            
        except Exception as e:
            print(f"Failed to format data for PlotJuggler: {e}")
            return "{}"


# Convenience function for sending a single IMU data packet
def send_imu_data_once(imu_data, host="192.168.28.118", port=9870):
    """
    Send IMU data once (without a persistent connection)

    Args:
        imu_data (dict): IMU data dictionary
        host (str): Target host IP
        port (int): Target port

    Returns:
        bool: Whether the send succeeded
    """
    try:
        # Create a temporary socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Format data
        data_with_timestamp = {
            'timestamp': time.time(),
            'imu_data': imu_data
        }

        # Use a sender instance to format the data
        temp_sender = UDPPlotJugglerSender(host=host, port=port, enable_udp=False)
        plotjuggler_data = temp_sender.format_for_plotjuggler(data_with_timestamp)
        
        # Send data
        sock.sendto(plotjuggler_data.encode('utf-8'), (host, port))
        sock.close()
        
        return True
        
    except Exception as e:
        print(f"Failed to send IMU data: {e}")
        return False


if __name__ == "__main__":
    # Module test
    print("UDP sender module test")

    # Test data
    test_imu_data = {
        'roll': 0.1,
        'pitch': 0.2,
        'yaw': 0.3,
        'gyro_x': 0.01,
        'gyro_y': 0.02,
        'gyro_z': 0.03,
        'acc_x': 9.8,
        'acc_y': 0.1,
        'acc_z': 0.2,
        'quat': [1.0, 0.0, 0.0, 0.0]
    }
    
    # Test one-shot send
    print("Testing one-shot send...")
    success = send_imu_data_once(test_imu_data)
    print(f"Send result: {'success' if success else 'failed'}")

    # Test continuous send
    print("Testing continuous send (5 seconds)...")
    sender = UDPPlotJugglerSender()
    if sender.start():
        start_time = time.time()
        while time.time() - start_time < 5:
            sender.send_imu_data(test_imu_data)
            time.sleep(0.1)
        sender.stop()
        print("Continuous send test complete")