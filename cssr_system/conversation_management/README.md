# Conversation Management ROS Node

This ROS node handles natural language understanding and dialogue management, facilitating the implementation of
conversational abilities for Pepper. It also handles intent detection to check for whether a tour should be given based
on a person's reply to Pepper's prompting.

## Installing sqlite3 version 3.51.0

1. `sudo apt-get update && sudo apt-get install build-essential tcl wget`
2. `wget https://sqlite.org/2025/sqlite-autoconf-3510000.tar.gz`
3. `tar -xvzf sqlite-autoconf-3510000.tar.gz`
4. `cd sqlite-autoconf-3510000`
5. `./configure`
6. `make`
7. `sudo make install`
8. `sudo ldconfig`
9. `sqlite3 --version`
