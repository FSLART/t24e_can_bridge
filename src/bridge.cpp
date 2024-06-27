#include "t24e_can_bridge/bridge.hpp"

Bridge::Bridge() : Node("t24e_can_bridge") {

	// create a socket
	if((this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
		return;
	}

	// define the interface
	struct ifreq ifr;
	strcpy(ifr.ifr_name, T24E_CAN_INTERFACE);
	ioctl(this->s, SIOCGIFINDEX, &ifr);

	// bind the socket to the CAN interface
	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if(bind(this->s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
		return;
	}

	// initialize publisher and subscriber
	this->cmd_subscriber = this->create_subscription<lart_msgs::msg::DynamicsCMD>(
		DYNAMICS_CMD_TOPIC, 10, [this](lart_msgs::msg::DynamicsCMD::UniquePtr msg) {
			// TODO
		});

	this->dynamics_publisher = this->create_publisher<lart_msgs::msg::Dynamics>(DYNAMICS_TOPIC, 10);

	// read CAN frames
	struct can_frame frame;
	while (rclcpp::ok()) {
		int nbytes = read(s, &frame, sizeof(frame));
		if(nbytes < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to read CAN frame");
			return;
		}

		handle_can_frame(frame);
	}
}

void Bridge::send_can_frame(struct can_frame frame) {
	if(write(this->s, &frame, sizeof(frame)) < 0) {
		RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame");
	}
}

void Bridge::handle_can_frame(struct can_frame frame) {
	if(frame.can_id == VCU_PWT_CAN_ID) {
		RCLCPP_INFO(this->get_logger(), "Received VCU_PWT_CAN_ID frame");
		uint32_t data = int_from_bytes(frame.data, VCU_PWT_RPM_LEN);

		lart_msgs::msg::Dynamics dynamics;

		dynamics.rpm = data;
	}
}

uint32_t Bridge::int_from_bytes(uint8_t *bytes, size_t len) {
	uint32_t result = 0;
	for(size_t i = 0; i < len; i++) {
		result |= bytes[i] << (8 * i);
	}
	return result;
}

int main(int argc, char ** argv) {

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Bridge>());
	rclcpp::shutdown();
	return 0;
}
