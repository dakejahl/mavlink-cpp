#pragma once

#include <sstream>

namespace mavlink
{

enum class ConnectionResult {
	Success = 0,
	Timeout,
	SocketError,
	BindError,
	SocketConnectionError,
	ConnectionError,
	NotImplemented,
	SystemNotConnected,
	SystemBusy,
	CommandDenied,
	DestinationIpUnknown,
	ConnectionsExhausted,
	ConnectionUrlInvalid,
	BaudrateUnknown
};

std::ostream& operator<<(std::ostream& str, const ConnectionResult& result);

} // namespace mavsdk