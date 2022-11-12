#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

#include <iostream>
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

	mavlink::ConfigurationSettings mavlink_settings = {
		.connection_url = "udp://127.0.0.1:14561",
		.sysid = 1,
		.compid = 123,
		.mav_type = MAV_TYPE_WINCH,
		.mav_autopilot = MAV_AUTOPILOT_INVALID,
		.emit_heartbeat = true
	};

	auto mavlink = std::make_shared<mavlink::Mavlink>(mavlink_settings);

	mavlink->subscribe_to_message(MAVLINK_MSG_ID_HEARTBEAT, [](auto message){
		LOG("MAVLINK_MSG_ID_HEARTBEAT -- %u/%u", message.sysid, message.compid);
	});

	auto result = mavlink->start();

	if (result != mavlink::ConnectionResult::Success) {
		std::cout << "Mavlink connection start failed: " << result << std::endl;
		return false;
	}

	LOG("Waiting for connection...");
	// Waits for connection interface to discover an autopilot (sysid=1 && compid=1)
	while (!mavlink->connected() && !_should_exit) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	static bool connected = true;
	// Main loop
	while (!_should_exit) {

		if (mavlink->connected()) {
			if (!connected) {
				LOG(GREEN_TEXT "Mavlink connection regained" NORMAL_TEXT);
				connected = true;
			}

		} else {
			if (connected) {
				LOG(RED_TEXT "Mavlink connection lost" NORMAL_TEXT);
				connected = false;
			}
		}

		// Do nothing -- message subscription callbacks are asynchronous and run in the connection receiver thread
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	LOG("Exiting...");
}

static void signal_handler(int signum)
{
	_should_exit = true;
}
