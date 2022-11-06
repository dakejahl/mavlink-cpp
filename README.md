
## Description
A very lightweight and easy to use mavlink library designed for reusability across all of your mavlink projects. The goal was to create
an interface that allows you to quickly setup a mavlink connection and in only a few lines of code immediately begin sending and receiveing
mavlink messages. The library implements thread safety by utilizing a message queue design with locks and condition variables.

- Register a function callback for a given mavlink message ID.
- Send a `mavlink_message_t` via the established connection.

TODO: finish explanation once library is finished

