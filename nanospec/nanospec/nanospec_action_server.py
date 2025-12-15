#!/usr/bin/env python3
"""
ros2 launch nanospec nanospec.launch.py
ros2 action send_goal /acquire_spectrum kortex_interfaces/action/AcquireSpectrum "{}"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import threading
import serial
import time

from kortex_interfaces.action import AcquireSpectrum
from kortex_interfaces.msg import NanoSpec
from nanospec.NanoLambdaNSP32 import NSP32, CmdCodeEnum


class NanoSpecActionServer(Node):
    """ROS2 Action Server for NSP32 spectrometer data acquisition."""

    def __init__(self):
        super().__init__('nanospec_action_server')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/nanospec')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_on_acquire', True)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.publish_on_acquire = self.get_parameter('publish_on_acquire').get_parameter_value().bool_value

        # Thread safety
        self.lock = threading.Lock()
        self.acquisition_complete = threading.Event()

        # Data storage
        self.sensor_id = None
        self.wavelengths = []
        self.spectrum_info = None

        # Serial connection
        try:
            self.ser = serial.Serial(
                serial_port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            self.get_logger().info(f'Connected to spectrometer on {serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {serial_port}: {e}')
            raise

        # NSP32 driver
        self.nsp32 = NSP32(self._send_data, self._on_packet_received)

        # Start serial receive thread
        self.serial_thread = threading.Thread(target=self._serial_receive_loop, daemon=True)
        self.serial_thread.start()

        # Callback group for concurrent action handling
        self.callback_group = ReentrantCallbackGroup()

        # Action server
        self.action_server = ActionServer(
            self,
            AcquireSpectrum,
            'acquire_spectrum',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group
        )

        # Publisher for spectrum data
        self.spectrum_pub = self.create_publisher(NanoSpec, 'spectral_data', 10)

        # Initialize sensor
        self._initialize_sensor()

        self.get_logger().info('NanoSpec Action Server ready!')
        self.get_logger().info('Trigger with: ros2 action send_goal /acquire_spectrum kortex_interfaces/action/AcquireSpectrum "{}"')

    def _initialize_sensor(self):
        """Initialize sensor and get wavelength calibration."""
        self.get_logger().info('Initializing sensor...')
        self.nsp32.Hello(0)
        time.sleep(0.1)
        self.nsp32.GetSensorId(0)
        time.sleep(0.1)
        self.nsp32.GetWavelength(0)
        time.sleep(0.5)  # Wait for wavelength data
        self.get_logger().info('Sensor initialization complete')

    def _send_data(self, data):
        """Send data to spectrometer via serial."""
        if self.ser.isOpen():
            self.ser.write(data)

    def _serial_receive_loop(self):
        """Background thread to receive serial data."""
        while rclpy.ok() and self.ser.isOpen():
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.nsp32.OnReturnBytesReceived(data)
                else:
                    time.sleep(0.001)  # Prevent busy loop
            except Exception as e:
                self.get_logger().error(f'Serial receive error: {e}')
                break

    def _on_packet_received(self, pkt):
        """Handle packets received from spectrometer."""
        with self.lock:
            if pkt.CmdCode == CmdCodeEnum.GetSensorId:
                self.sensor_id = pkt.ExtractSensorIdStr()
                self.get_logger().info(f'Sensor ID: {self.sensor_id}')

            elif pkt.CmdCode == CmdCodeEnum.GetWavelength:
                info = pkt.ExtractWavelengthInfo()
                self.wavelengths = list(info.Wavelength)
                self.get_logger().info(f'Wavelengths received: {len(self.wavelengths)} points')

            elif pkt.CmdCode == CmdCodeEnum.GetSpectrum:
                self.spectrum_info = pkt.ExtractSpectrumInfo()
                self.get_logger().info('Spectrum data received')
                self.acquisition_complete.set()

    def _goal_callback(self, goal_request):
        """Accept or reject incoming goal requests."""
        self.get_logger().info('Received spectrum acquisition request')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept cancel requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        """Execute the spectrum acquisition action."""
        self.get_logger().info('Starting spectrum acquisition...')

        result = AcquireSpectrum.Result()
        feedback = AcquireSpectrum.Feedback()

        # Get goal parameters
        goal = goal_handle.request
        integration_time = goal.integration_time if goal.integration_time > 0 else 1
        frame_avg_num = goal.frame_avg_num if goal.frame_avg_num > 0 else 20
        enable_ae = goal.enable_auto_exposure

        try:
            # Clear previous data
            with self.lock:
                self.spectrum_info = None
            self.acquisition_complete.clear()

            # Send feedback: starting
            feedback.status = 'Acquiring spectrum...'
            feedback.progress = 0.1
            goal_handle.publish_feedback(feedback)

            # Request spectrum acquisition
            self.nsp32.AcqSpectrum(0, integration_time, frame_avg_num, enable_ae)

            # Wait for acquisition with timeout
            feedback.status = 'Waiting for data...'
            feedback.progress = 0.5
            goal_handle.publish_feedback(feedback)

            timeout = 10.0  # seconds
            if not self.acquisition_complete.wait(timeout=timeout):
                self.get_logger().error('Spectrum acquisition timed out')
                result.success = False
                result.message = 'Acquisition timed out'
                goal_handle.abort()
                return result

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Acquisition cancelled'
                return result

            # Process results
            feedback.status = 'Processing data...'
            feedback.progress = 0.9
            goal_handle.publish_feedback(feedback)

            with self.lock:
                if self.spectrum_info is None:
                    result.success = False
                    result.message = 'No spectrum data received'
                    goal_handle.abort()
                    return result

                result.success = True
                result.message = 'Acquisition successful'
                result.wavelengths = self.wavelengths
                result.spectrum = list(self.spectrum_info.Spectrum)
                result.x = self.spectrum_info.X
                result.y = self.spectrum_info.Y
                result.z = self.spectrum_info.Z
                result.integration_time_used = self.spectrum_info.IntegrationTime
                result.was_saturated = self.spectrum_info.IsSaturated

            # Publish to topic if enabled
            if self.publish_on_acquire:
                msg = NanoSpec()
                msg.wavelengths = result.wavelengths
                msg.spectrum = result.spectrum
                self.spectrum_pub.publish(msg)
                self.get_logger().info('Published spectrum to /spectral_data')

            goal_handle.succeed()
            self.get_logger().info(
                f'Acquisition complete: {len(result.spectrum)} points, '
                f'integration_time={result.integration_time_used}, '
                f'saturated={result.was_saturated}'
            )

        except Exception as e:
            self.get_logger().error(f'Acquisition failed: {e}')
            result.success = False
            result.message = str(e)
            goal_handle.abort()

        return result

    def destroy_node(self):
        """Clean up resources."""
        if hasattr(self, 'ser') and self.ser.isOpen():
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = NanoSpecActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except Exception as e:
        print(f'Failed to start NanoSpec Action Server: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

