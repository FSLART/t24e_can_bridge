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
			
			// create a CAN frame
			struct can_frame frame;
			frame.can_id = VCU_CMD_CAN_ID;
			frame.can_dlc = VCU_CMD_RPM_LEN;
			int_to_bytes(msg->rpm, frame.data, VCU_CMD_RPM_LEN);

			// send the CAN frame
			this->send_can_frame(frame);

			// update the last command message
			this->last_cmd = *msg;
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
		uint32_t data = int_from_bytes(frame.data, VCU_PWT_RPM_LEN);

		// update the RPM
		this->last_dynamics.rpm = data;

		// publish the dynamics message
		this->dynamics_publisher->publish(this->last_dynamics);
	}
}

uint32_t Bridge::int_from_bytes(uint8_t *bytes, size_t len) {
	uint32_t result = 0;
	for(size_t i = 0; i < len; i++) {
		result |= bytes[i] << (8 * i);
	}
	return result;
}

void Bridge::int_to_bytes(uint32_t value, uint8_t *bytes, size_t len) {
	for(size_t i = 0; i < len; i++) {
		bytes[i] = (value >> (8 * i)) & 0xFF;
	}
}

int main(int argc, char ** argv) {

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Bridge>());
	rclcpp::shutdown();
	return 0;
}
