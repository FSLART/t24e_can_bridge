#include "t24e_can_bridge/bridge.hpp"

Bridge::Bridge() : Node("t24e_can_bridge") {

	// create a socket
	if((this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
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
		RCLCPP_ERROR(this->get_logger(), "Failed to bind socket: %s", strerror(errno));
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

	// create a thread to read CAN frames
	std::thread can_thread(&Bridge::read_can_frames, this);
	can_thread.detach();
}

void Bridge::send_can_frame(struct can_frame frame) {
	if(write(this->s, &frame, sizeof(frame)) < 0) {
		RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame: %s", strerror(errno));
	}
}

void Bridge::read_can_frames() {
	while(rclcpp::ok()) {
		struct can_frame frame;
		int nbytes = read(this->s, &frame, sizeof(frame));
		if(nbytes < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to read CAN frame: %s", strerror(errno));
			return;
		}

		handle_can_frame(frame);
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
		result |= bytes[i] << ((len - i - 1) * 8);
	}
	return result;
}

void Bridge::int_to_bytes(uint32_t value, uint8_t *bytes, size_t len) {
	for(size_t i = 0; i < len; i++) {
		bytes[i] = (value >> ((len - i - 1) * 8)) & 0xFF;
	}
}

int main(int argc, char ** argv) {

	rclcpp::init(argc, argv);

	auto node = std::make_shared<Bridge>();

	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

	executor.add_node(node);

	executor.spin();

	rclcpp::shutdown();
	return 0;
}
