#include "rclcpp/rclcpp.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/state.hpp"
#include <std_msgs/msg/bool.hpp>
#include "../can-header-map/CAN_asdb.h"
#include <chrono>
#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <thread>
#include <iostream>

#define T24E_CAN_INTERFACE "can0"

#define VCU_CMD_CAN_ID 0x500
#define VCU_CMD_RPM_BYTE 0
#define VCU_CMD_RPM_LEN 2

#define VCU_PWT_CAN_ID 0x510
#define VCU_PWT_RPM_BYTE 0
#define VCU_PWT_RPM_LEN 2

#define RES_CAN_ID 0x191
#define RES_START_CAN_ID 0x00

#define DYNAMICS_CMD_TOPIC "/pc_origin/dynamics"
#define DYNAMICS_TOPIC "/acu_origin/dynamics"
#define RES_READY_TOPIC "/acu_origin/res_ready"

class Bridge : public rclcpp::Node
{
	public:
		Bridge();

    private:
        
        /*! \brief CAN socket descriptor. */
        int s = -1;

        /*! \brief Subscriber from the control node. */
        rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr cmd_subscriber;

        /*! \brief Publisher to the control node with wheel-related information and RPM. */
        rclcpp::Publisher<lart_msgs::msg::Dynamics>::SharedPtr dynamics_publisher;

        /*! \brief Publisher to the control node with the RES ready button state. */
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr res_ready_publisher;

        /*! \brief Last command message. */
        lart_msgs::msg::DynamicsCMD last_cmd;

        /*! \brief Last dynamics message. */
        lart_msgs::msg::Dynamics last_dynamics;

        /*! \brief Last Mission selected message*/
        lart_msgs::msg::Mission last_mission;

        /*! \brief Last State selected message*/
        lart_msgs::msg::State last_state;

        /*! \brief state of the res ready button*/
        std_msgs::msg::Bool res_ready;

        /*! \brief Handle a CAN frame data. */
        void handle_can_frame(struct can_frame frame);

        /*! \brief Send a CAN frame. */
        void send_can_frame(struct can_frame frame);

        /*! \brief Read CAN frames. */
        void read_can_frames();

        /*! \brief Convert an array of bytes to an unsigned integer. */
        static uint32_t int_from_bytes(uint8_t *bytes, size_t len);

        /*! \brief Convert an unsigned integer to an array of bytes. */
        static void int_to_bytes(uint32_t value, uint8_t *bytes, size_t len);

        /*! \brief Send CAN frames. */
        void send_can_frames();

        /*! \brief Start the res. */
        void start_res();
};
