
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <iostream>

#include <Mavlink.hpp>

static void signal_handler(int signum);

volatile bool _should_exit = false;

int main(int argc, const char** argv)
{
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	setbuf(stdout, NULL); // Disable stdout buffering

	std::cout << "Hello mavlink" << std::endl;

	// Intialize mavlink
	mavlink::ConfigurationSettings mavlink_settings = {
		.connection_url = "udp://127.0.0.1:14561",
		.sysid = 1,
		.compid = 123,
		.mav_type = MAV_TYPE_WINCH,
		.mav_autopilot = MAV_AUTOPILOT_INVALID,
		.emit_heartbeat = true
	};

	auto mavlink = std::make_shared<mavlink::Mavlink>(mavlink_settings);

	// Subscribe to mavlink DISTANCE_SENSOR
	mavlink->subscribe_to_message(MAVLINK_MSG_ID_DISTANCE_SENSOR, [](const mavlink_message_t& message) { LOG("MAVLINK_MSG_ID_DISTANCE_SENSOR"); });
	mavlink->subscribe_to_message(MAVLINK_MSG_ID_HEARTBEAT, [](const mavlink_message_t& message) { LOG("MAVLINK_MSG_ID_HEARTBEAT"); });
	mavlink->subscribe_to_message(MAVLINK_MSG_ID_ATTITUDE, [](const mavlink_message_t& message) { LOG("MAVLINK_MSG_ID_ATTITUDE"); });

	while (!mavlink->connected()) {
		std::cout << "waiting for connection" << std::endl;

		usleep(1000000);
	}


	while (!_should_exit) {
		usleep(10000);
	}
}

static void signal_handler(int signum)
{
	LOG("signal_handler!");

	_should_exit = true;
}
