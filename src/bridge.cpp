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
	}

	//open maxon communication
	lResult = open_actuation();
	
	this->last_dynamics.rpm = 0;
	this->last_dynamics.steering_angle = 0;

	// initialize publisher and subscriber
	this->cmd_subscriber = this->create_subscription<lart_msgs::msg::DynamicsCMD>(
		DYNAMICS_CMD_TOPIC, 10, [this](lart_msgs::msg::DynamicsCMD::UniquePtr msg) {
			
			// create a CAN frame
			struct can_frame frame;
			frame.can_id = VCU_CMD_CAN_ID;
			frame.can_dlc = VCU_CMD_RPM_LEN;
			int rpm = msg->rpm;
			int max_variation = 30;
			//RCLCPP(this->get_logger(), "RPM: %d", rpm);
			//rpm = this->last_dynamics.rpm + std::clamp(rpm - this->last_dynamics.rpm, -max_variation, max_variation);
			//RCLCPP(this->get_logger(), "RPM: %d", rpm);
			int_to_bytes(rpm, frame.data, VCU_CMD_RPM_LEN);

			// send the CAN frame
			this->send_can_frame(frame);

			//TODO: DELETE THE COMMENT AFTER TESTING WITH MOTOR OFF
			//if(RPM_TO_MS((this->last_dynamics.rpm)*3.6) > 0.5){
				// send actuation to maxon
				float actuator_angle = RAD_ST_ANGLE_TO_ACTUATOR_POS(msg->steering_angle);
				actuate(actuator_angle);
			//}
			// update the last command message
			this->last_cmd = *msg;
		});

	this->dynamics_publisher = this->create_publisher<lart_msgs::msg::Dynamics>(DYNAMICS_TOPIC, 10);

	this->res_ready_publisher = this->create_publisher<std_msgs::msg::Bool>(RES_READY_TOPIC, 10);

	//TODO res responds with what?
	this->start_res();

	// create a thread to read CAN frames
	std::thread can_thread(&Bridge::read_can_frames, this);
	can_thread.detach();

	std::thread can_send_thread(&Bridge::send_can_frames, this);
	can_send_thread.detach();
}

void Bridge::send_can_frames() {
	// create a CAN frame
	struct can_frame frame;
	frame.can_id = CAN_AS_STATUS;
	frame.can_dlc = 8;
	{
		std::lock_guard<std::mutex> guard(this->state_mutex);
		this->last_state.data = lart_msgs::msg::State::READY;
	}
	std::chrono::time_point last_state_change = std::chrono::system_clock::now();

	while(rclcpp::ok()) {
		{
			std::lock_guard<std::mutex> guard2(this->state_mutex);
			//If it is in ready and it has been 6 seconds since the last state change and the ready res button is pressed
			if(this->last_state.data == lart_msgs::msg::State::READY && (std::chrono::system_clock::now() - last_state_change) > std::chrono::seconds(6) && this->res_ready.data) {
				this->last_state.data = lart_msgs::msg::State::DRIVING;	
				this->res_ready_publisher->publish(this->res_ready);
			}
			MAP_ENCODE_AS_STATE(frame.data, this->last_state.data);
			this->send_can_frame(frame);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
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
	if(frame.can_id == CAN_TOJAL_SEND_RPM) {
		uint32_t data = frame.data[0] << 8 | frame.data[1];
		// //RCLCPP(this->get_logger(), "received from can RPM: %d", data);
		// update the RPM
		this->last_dynamics.rpm = data;

		// publish the dynamics message
		this->dynamics_publisher->publish(this->last_dynamics);
	}
	if(frame.can_id == CAN_AS_STATUS) {
		uint8_t as_mission = MAP_DECODE_AS_MISSION(frame.data);
		
		//update mission
		this->last_mission.data = as_mission;
	}
	if(frame.can_id == RES_CAN_ID) {
		//TODO receive the ready button state from the res can open
		//TODO should we keep the true value if the car is not ready yet
		uint8_t res_response = frame.data[0];
		if(res_response == 0x05 || res_response == 0x07){
			RCLCPP_WARN(this->get_logger(), "RES ready button pressed");
			this->res_ready.data = true;
		}
		if(res_response == 0x00){
			RCLCPP_WARN(this->get_logger(), "RES emergency pressed");
			//Send Emergency to CAN
			//TODO WE NEED TO SEND THE EMERGENCY TO THE CAN BUT WE ALSO NEED TO STOP THE THREAD THAT IS SENDING THE STATE OF DRIVING OR THE LIGHT WILL BE SWITCHING 
			//I NEED TO HAVE IN ACCOUNT CONCURRENCY... THIS VARIABLE IS USED IN OTHER THREAD THAT SEND THE STATE OF THE CAR
			std::lock_guard<std::mutex> guard(this->state_mutex);
			this->last_state.data = lart_msgs::msg::State::EMERGENCY;
		}
	}
}

void Bridge::start_res() {
	// create a CAN frame
	struct can_frame frame;
	frame.can_id = RES_START_CAN_ID;
	frame.can_dlc = 2;
	frame.data[0] = 0x01;
	frame.data[1] = 0x00;

	// send the CAN frame
	this->send_can_frame(frame);
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
	close_actuation();
	return 0;
}
