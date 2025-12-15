import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from kortex_interfaces.srv import GetSpectrum
from kortex_interfaces.msg import NanoSpec

class SpectrumClient(Node):
    def __init__(self):
        super().__init__('spectrum_client_triggered')

        self.client = self.create_client(GetSpectrum, 'get_spectrum')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to be available...')
        
        self.subscription = self.create_subscription(String, 'spec_service_trigger', self.listener_callback, 10)
        self.publisher = self.create_publisher(NanoSpec, 'spectral_data', 10)
        self.subscription  

    def listener_callback(self, msg):
        self.get_logger().info('Received trigger message: "%s"' % msg.data)
        # self.get_logger().info('Sending service request to get spectrum...')
        
        self.request = GetSpectrum.Request()
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response:
                self.publish_results(response.wavelengths, response.spectrum)
            else:
                self.get_logger().info('Service call failed')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

    def publish_results(self, wavelengths, spectrum):
        try:
            spectrum_msg = NanoSpec()
            spectrum_msg.wavelengths = wavelengths
            spectrum_msg.spectrum = spectrum
            
            # self.get_logger().info(f'Published data: wavelengths={wavelengths}, spectrum={spectrum}')
            self.publisher.publish(spectrum_msg)
            self.get_logger().info('Published wavelengths and spectrum.')
        except Exception as e:
            self.get_logger().error(f'Failed to publish results: {e}')

def main(args=None):
    rclpy.init(args=args)
    spectrum_client = SpectrumClient()
    rclpy.spin(spectrum_client)
    spectrum_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
