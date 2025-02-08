# Welcome to CATCAR-X

* I'm not a python dev, I don't enjoy python. This code is not well tested and was written for expressly for UIUC CS437 IoT class
* It s based on SunFounder's [picar-x](https://docs.sunfounder.com/projects/picar-x/en/latest/) and requires setup to run
* It's called the catcar-x because I replaced the normal head with the head of the [Freenove robot dog kit](https://github.com/Freenove/Freenove_Robot_Dog_Kit_for_Raspberry_Pi) (which i argue look more cat like) so that I can have a panning and tilting ultrasonic sensor
   * the higher ultrasonic sensor can cause issues with sensing lower objects since the servo can't tilt down very well
   * you could theoretically install both sensors but I lacked an extra 4 pin cable that would work with the SunFounder robot hat