# You should only have to change these values
PORT = /dev/tty.usbserial-A100P01P

# You should set the path to your Arduino application if it is not the default one
# The default for Linux and Solaris is:
#ARD_HOME = /opt/Arduino
# The default for Mac is:
#ARD_HOME = /Applications/Arduino.app

# Leave these alone
BOARD = pro
BOARD_SUB = pro.menu.cpu.16MHzatmega328
MON_SPEED = 115200

include Arduino_Makefile_master/_Makefile.master