#pragma once

#include <mavlink.h>

class MessageParser
{
public:
	MessageParser(char* datagram, ssize_t length)
		: _datagram(datagram)
		, _length(length)
	{}

	// Parses a single mavlink message from the data buffer.
	// Note that one datagram can contain multiple mavlink messages.
	// It is OK if a message is fragmented because mavlink_parse_char places bytes into an internal buffer.
	bool parse(mavlink_message_t* message)
	{
		for (unsigned i = 0; i < _length; ++i) {

			// TODO: convenience function for reporting parser statistics
			mavlink_status_t status;

			if (mavlink_parse_char(0, _datagram[i], message, &status)) {
				// Move the pointer to the data forward by the amount parsed.
				_datagram += (i + 1);
				// And decrease the length, so we don't overshoot in the next round.
				_length -= (i + 1);
				// We have parsed one message, let's return so it can be handled.
				return true;
			}
		}

		// No more messages.
		return false;
	}
private:
	char* _datagram {};
	ssize_t _length {};
};