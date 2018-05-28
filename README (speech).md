# ROS Based Speech Interface

**This repository is for research onyl using!**

This repository contains an example for using a web interface to speak with the robot and listen to its voice.

The repository provides a web page based on the [Google Speach Demo](https://www.google.com/intl/en/chrome/demos/speech.html) for performing **speech-to-text**, and on [responsive voice API](https://responsivevoice.org/) for **text-to-speech**.

Such a web interface is implemented as a ROS node using the javascript [ROS Bridge](http://wiki.ros.org/rosbridge_suite). Which publish on the topic `/speech_to_text` the string of the user's sentence, a confidence value and the used language. 
Also, this javascript interfaces listens to the topic `\text_to_speech` topic in order to make the robot speaking. 
In this topic, you can also set the language, the volume, the rate, and the pitch.

In this repository, for showing purposes, we used the above ROS-web interface through a standard C++ ROS node, which repeats to the user what he/she said.

### Dependences

The web interface works only with the Chrome browser open with the `--disable-web-security` option.
Also, this repository depends on ROS and ROS Bridge, that you can obtain via `apt-get install`.
We tested the code with Ubuntu 16.04, ROS kinetic, and Chrome 66.

### Installation

Clone this repository on you catking workspace and run `catking_make` (be sure that all the depends are set on your machine first). 

### Execution

To execute the code run: 
```
    $ roslaunch speech_interaction speech_back_example.launch
```
A web page should happear with more information and dettails. Click on the micrhopone icon and listen the robot repeating what you said. For hardare trouble check the sentences of your Chrome.

Remember to check on the launcher file if you what to change the logging behaviour. By default the system write on the `/dialogs-log` folder all the sentences that the Human and Robot said.

### Author

[Luca Buoncompagni](mailto:luca.buoncompagni@edu.unige.it)
EMAROlab, DIBRIS department, University of Genoa, Italy.


Previous commit for this package can be [found here](https://gitlab.com/buoncubi/ros_verbal_interaction_node).
