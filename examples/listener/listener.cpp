#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

#include <chrono>
#include <thread>

#include <Mavlink.hpp>

static void signal_handler(int signum);

volatile bool _should_exit = false;

int main(int argc, const char** argv)
{
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	setbuf(stdout, NULL); // Disable stdout buffering

	LOG("Hello mavlink");

	mavlink::ConfigurationSettings mavlink_settings = {
		.connection_url = "udp://127.0.0.1:14561",
		.sysid = 1,
		.compid = 123,
		.mav_type = MAV_TYPE_WINCH,
		.mav_autopilot = MAV_AUTOPILOT_INVALID,
		.emit_heartbeat = true
	};

	LOG("Creating mavlink interface");
	auto mavlink = std::make_shared<mavlink::Mavlink>(mavlink_settings);

	LOG("Subscribing to messages");
	mavlink->subscribe_to_message(MAVLINK_MSG_ID_DISTANCE_SENSOR, [](auto message) 	{ LOG("MAVLINK_MSG_ID_DISTANCE_SENSOR"); });
	mavlink->subscribe_to_message(MAVLINK_MSG_ID_HEARTBEAT, [](auto message) 		{ LOG("MAVLINK_MSG_ID_HEARTBEAT"); });
	mavlink->subscribe_to_message(MAVLINK_MSG_ID_ATTITUDE, [](auto message) 		{ LOG("MAVLINK_MSG_ID_ATTITUDE"); });

	// Waits for connection interface to discover an autopilot (sysid=1 && compid=1)
	while (!mavlink->connected() && !_should_exit) {
		LOG("Waiting for connection");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// Main loop
	while (!_should_exit) {
		// Do nothing -- message subscription callbacks are asynchronous and run in the connection receiver thread
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

	}

	LOG("Exiting...");
}

static void signal_handler(int signum)
{
	_should_exit = true;
}
