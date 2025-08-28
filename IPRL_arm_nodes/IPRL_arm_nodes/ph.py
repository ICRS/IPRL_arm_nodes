import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger


class pHNode(Node):

    def __init__(self):
        super().__init__("ph_node")

        self.ph_raw_sub = self.create_subscription(Float32, "ph/raw", self.ph_measurement_received, 10)
        self.ph_request_pub = self.create_publisher(Float32, "ph/request", 10)

        # Calibration topics and variables
        self.ph_calibration_sub = self.create_subscription(Float32, "ph/calibrate", self.calibrate, 10)
        self.current_ph_actual = 0.0  # The buffer solution pH
        self.calibration_in_progress = False
        self.calibration_lut = {}  # Store the previous calibration results. This lookup table can be linearly extrapolated/interpolated to get the calibrated pH
        self.ph_calibrated_pub = self.create_publisher(Float32, "ph/calibrated", 10)

        self.calibrate_four_trigger = self.create_service(Trigger,"ph/calibrate_four_trigger",self.calibrate_four)
        self.calibrate_seven_trigger = self.create_service(Trigger,"ph/calibrate_seven_trigger",self.calibrate_seven)

    def calibrate(self, msg: Float32):

        # Start the calibration point
        self.current_ph_actual = msg.data
        self.calibration_in_progress = True

        # Request a measurement
        to_send = Bool()
        to_send.data = True
        self.ph_request_pub(to_send)

    def calibrate_four(self, request, response):
        msg = Float32()
        msg.data = 4.0
        self.calibrate(msg)
        response.success = True
        response.message = "Calibration in pH 4 buffer solution started"
        return response

    def calibrate_seven(self, request, response):
        msg = Float32()
        msg.data = 7.0
        self.calibrate(msg)
        response.success = True
        response.message = "Calibration in pH 7 buffer solution started"
        return response

    def apply_calibration(self, raw: float) -> float:

        calibrated = -1.0

        # Not calibrated
        if len(self.calibration_lut) < 2:
            return calibrated

        # Search for the interval we are in
        above_calibrated_point = None
        above_raw_point        = 100.0
        below_calibrated_point = None
        below_raw_point        = -100.0
        for raw_point, calibrated_point in self.calibration_lut.items():
            if raw_point < raw and raw_point > below_raw_point:
                below_raw_point = raw_point
                below_calibrated_point = calibrated_point
            elif raw_point > raw and raw_point < above_raw_point:
                above_raw_point = raw_point
                above_calibrated_point = calibrated_point

        # We are in an interval: interpolate
        if (above_calibrated_point is not None) and (below_calibrated_point is not None):
            raw_range = above_raw_point - below_raw_point
            calibrated_range = above_calibrated_point - below_calibrated_point
            calibrated = (((raw - below_raw_point) / (raw_range)) * (calibrated_range)) + below_calibrated_point

        # Extrapolate between the higest and lowest calibration points
        else:

            higest_raw = max(self.calibration_lut)
            lowest_raw = min(self.calibration_lut)

            # Calculate the gradient
            rise = self.calibration_lut[higest_raw] - self.calibration_lut[lowest_raw]
            run = higest_raw - lowest_raw
            m = rise / run

            # Calculate the y intercept
            c = self.calibration_lut[higest_raw] - m * higest_raw

            # Calculate the calibrated value
            calibrated = m * raw + c

        return calibrated

    def ph_measurement_received(self, msg: Float32):

        to_send = Float32()

        # Calibration measurement: store the measurement in the lookup table
        if self.calibration_in_progress:

            # Store in LUT
            ph_raw = msg.data
            self.calibration_lut[ph_raw] = self.current_ph_actual

            # Echo calibration value
            to_send.data = self.current_ph_actual
            self.ph_calibrated_pub.publish(to_send)

            # End calibration
            self.calibration_in_progress = False

        # Other measurement: apply calibration and publish
        else:
            to_send.data = self.apply_calibration(msg.data)
            self.ph_calibrated_pub(to_send)


def main(args=None):
    rclpy.init(args=args)
    node = pHNode()
    rclpy.spin(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
