
## Description
A very lightweight and easy to use mavlink library designed for reusability. The goal is an interface that allows you to
quickly setup a mavlink connection and in only a few lines of code immediately begin sending and receiveing mavlink messages.

## Building the examples
First you need to build and install the library. Running `make` in the root directory will build the library and install it at
**install/**. The examples are built as their own CMake project and look for the library at **mavlink-cpp/install**.
```
make && make examples
```

## Design
- Register a message handler for a given mavlink message ID. The message handler function callbacks are executed in the receiving thread
context. This means the callbacks must be thread safe and non-blocking. You should use locks to modify shared data or leverage the included
ThreadSafeQueue class if you want to queue messages for processing later.

- Send any `mavlink_message_t` via `mavlink->send_message(msg)`. This function pushes the message into a threadsafe queue where the sending
thread will access it for asynchronous sending.

- Register function callbacks for PARAM_REQUEST_LIST and PARAM_SET. This allows decoupling of your applications parameter implementation.
```
void enable_parameters(std::function<std::vector<Parameter>(void)> request_list_cb,
		       std::function<bool(Parameter*)> set_cb);
```

- Automatically emit heartbeats at 1Hz if the `emit_heartbeat` flag is set in the constructor `MavlinkSettings` parameter.

- Specify the target sysid/compid to connect to.
