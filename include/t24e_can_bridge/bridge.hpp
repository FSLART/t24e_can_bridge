#include "rclcpp/rclcpp.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <thread>

#define T24E_CAN_INTERFACE "can0"

#define VCU_CMD_CAN_ID 0x500
#define VCU_CMD_RPM_BYTE 0
#define VCU_CMD_RPM_LEN 2

#define VCU_PWT_CAN_ID 0x510
#define VCU_PWT_RPM_BYTE 0
#define VCU_PWT_RPM_LEN 2


#define DYNAMICS_CMD_TOPIC "/pc_origin/dynamics"
#define DYNAMICS_TOPIC "/acu_origin/dynamics"

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

        /*! \brief Last command message. */
        lart_msgs::msg::DynamicsCMD last_cmd;

        /*! \brief Last dynamics message. */
        lart_msgs::msg::Dynamics last_dynamics;

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
};
