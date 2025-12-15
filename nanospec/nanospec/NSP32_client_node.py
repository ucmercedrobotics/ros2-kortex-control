import rclpy
from rclpy.node import Node
from kortex_interfaces.srv import GetSpectrum


class SpectrumClient(Node):
    def __init__(self):
        super().__init__('spectrum_client_node')
        self.client = self.create_client(GetSpectrum, 'get_spectrum')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to be available...')
        self.request = GetSpectrum.Request()

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    spectrum_client = SpectrumClient()
    response = spectrum_client.send_request()
    if response:
        spectrum_client.get_logger().info(f'Received wavelengths: {response.wavelengths}')
        spectrum_client.get_logger().info(f'Received spectrum: {response.spectrum}')
    else:
        spectrum_client.get_logger().info('Service call failed')
    spectrum_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()