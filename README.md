# Welcome to CATCAR-X

* I'm not a python dev, I don't enjoy python. This code is not well tested and was written for expressly for UIUC CS437 IoT class
* It s based on SunFounder's [picar-x](https://docs.sunfounder.com/projects/picar-x/en/latest/) and requires setup to run
* It's called the catcar-x because I replaced the normal head with the head of the [Freenove robot dog kit](https://github.com/Freenove/Freenove_Robot_Dog_Kit_for_Raspberry_Pi) (which i argue look more cat like) so that I can have a panning and tilting ultrasonic sensor
   * the higher ultrasonic sensor can cause issues with sensing lower objects since the servo can't tilt down very well
   * you could theoretically install both sensors but I lacked an extra 4 pin cable that would work with the SunFounder robot hat

# TODOs:
1. ~~add css back to composite index~~
2. status messages
3. ~~loading spinners~~
4. ~~data chunking~~
5. test data chunking
6. make BT data transfer more efficient

# Running Code
* I don't have a requirements file set up. Client code should only require `bluedot` and `flask` on client machine or wherever you are sending commands from
* Server code requires PicarX base set up from above, `bluedot`, `flask`, `numpy`, and `matplotlib` on Raspberry Pi.
  * This has only been tested on a Raspberry Pi 4B running Python 3.12.x