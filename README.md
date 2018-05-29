# ROS based Speech Interface for Qualitative Teleoperation of the Miro Robot

This repository contains tree ROS packages for generate semantic spatial commands for qualitative spatial relation, given as sentences. 

In particular it contains the following ROS packages:
- **speech_interaction**: allows to use [Google Speech API](https://www.google.com/intl/it/chrome/demos/speech.html) for performing Text-to-Speech and [ResponsiveVoice](https://responsivevoice.org/) for Speech-to-Text translations.
- **ros_cagg_teleop**: allows to use the [CAGG API](https://github.com/EmaroLab/concept_action_grammar_generator) to evaluate sentences, based on BNF grammars, and extract semantic tags for words.
- **ros_cagg_msgs**: contains all the ROS messages used by `ros_cagg_teleop`.

For more references, please see [this repository](https://github.com/buoncubi/ros_cagg_pkgs) for `ros_cagg_teleop` and `ros_cagg_msgs`, as well as [this repository](https://github.com/buoncubi/ros_verbal_interaction_node) for `speech_interaction`.

## Packages Configuration

### Dependences

This repository as been tested with Ubuntu 16.04 and ROS Kinetic. 
It contains packages that depends on:
- Java 1.8,
- ROSjava bridge, 
- ROSBridge for using Java-script,
- Chrome browser.

### Installation

To install all the contents of this repository follow this instruction:
1. install ROSjava: `$ sudo apt-get install ros-kinetic-rosjava`
2. install ROSbride: `$ sudo apt-get install ros-kinetic-rosbridge-server` 
3. clone all the contents in the `/src` folder of your workspace.
4. do `$ catkin_make`
5. do `$roscd ros_cagg_teleop/ && ./gradlew deployApp`

### Execution

To execute the system simply use:
``
$ roslaunch speech_interaction speech_adapter.launch 
``

## Implementation

### Nodes

Above is shown the UML diagram of the architecture, which is composed by nodes:
- **web_interface_visualizer** is a javascript node (running under the `rosbridge_web_socket`) which shown an interface through which is possible to perform *text-to-speech* and *speech-to-text* translation.
- **ros_cagg_teleop** is a java node which depends on the CAGG API. It performs speech analysis based on specified grammars.
- **speech_teleop_adapter** is a C++ node which uses the nodes above, provides data, and logs experimental data on file.

### Architecture

![alt text](https://github.com/buoncubi/speech_based_teleop_commander/blob/developing/architecture.png "The speech_based_teleop_commander system architecture")

The figure above shows the UML diagram of the architecture implemented in this repository.
Developers that want to use this architecture should subscribe or publish messages only to the topics of the **speech_teleop_adapter** node.
While users that what to use the system should interact only with the **web_interface_visualizer** node.

In particular, when the robot should tell something, a string should be sent to the `/CAGG/adapted/text_to_specch` (that will be sent through the `/text_to_speech` topic of the **web_interface_visualizer** node). 

While when a user tells something through the web interface, the text translation of the user voice is send through the `/CAGG/input_text` topic.
Then, such string evaluated in order to compute and provide the semantic of the user's words, provided through the `/CAGG/semantic_tags` topic, which is propagated through the `/CAGG/adapted/semantic_tags` topic.

### Messages & Topics

Each topic of the architecture has a dedicated message format as following:
- the `/speech_to_text` topic uses the `speech_interaction/Speech2Text` message, which contains:
 - `std_msg::string language` (the speaking language used for the recognition)
 - `std_msg::string transcript` (the text generated from the user's voice)
 - `std_msg::float confidence` (the recognition confidence between 0 and 1)
- the `/text_to_speech` topic uses the `speech_interaction/Text2Speech` message, which contains:
 - `std_msg::string text` (the string that the robot should tell)
 - `std_msg::string language` (the speaking language used for voice synthesizes)
 - `std_msg::float volume` (the volume of the robot voice)
 - `std_msg::float pitch` (the pitch of the robot voice)
 - `std_msg::float rate` (the rate of the robot voice) 
- the `/CAGG/input_text` topic uses a `std_msg::string` message containing the user's sentence to translate.
- the `/CAGG/semantic_tags` topic uses the `ros_cagg_msgs/cagg_tags` message, which contains:
 - `Hearder` (which contains a time stamp)
 - `ros_cag_msgs::confidence` (the confidence of the semantic recognition, between 0 and 1)
 - `ros_cagg_msgs::computationTimeMs` (the computation time needed to evaluate the text, i.e., spent in **ros_cagg_teleop** in milliseconds)
 - `ros_cagg_msgs::cags_tags` (which contains a list of list of `std_msgs::string` representing the semantics recognized for specific words)
- the `CAGG/adapted/semantic_tags` is structured as `/CAGG/semantic_tags`, but returned by the adapter after the logging phase
- the `CAGG/adapted/text_to_speech` is structured as `/text_to_speech`, but returned by the adapter after the logging phase

### Parameters

The **ros_cagg_teleop** requires specific value on the ROS parameter server, in particular:
- `/cagg_log_config_path`: the absolute path to the Log4j configuration file,
- `/cagg_serialized_directive_grammar`: the absolute path tho the serialized CAGG grammar file about *directives* commands,
- `/cagg_serialized_go_grammar`: the absolute path tho the serialized CAGG grammar file about *go* commands.
All those paths are pointing on the folder `ros_cagg_teleop/ros_java_cagg_teleop_interface/res/`.

Also, you can set other further parameters of `CAGG_teleop` such as:
- `/cagg_timeout_ms`: the time (in milliseconds) to run the CAGG evaluation, after which the best result so far will be published in the output topic (remarkably, the evaluation start as soon as you send something on the input topic). By default set to 10000ms.
- `/cagg_stopping_check_frequency`: represent the frequency of stopping condition used before time-out. Remarkably, time-out will be applied after a multiple times of the `/cagg_stopping_check_frequency`. By default set to 1000ms.
- `/cagg_stopping_confidence_threshold`: represents the confidence threshold to stop searching for further results before time-out.
 - the *confidence* is computed as the ration of the word in a sentence at which CAGG attached at least a semantic tag, over the total number of words in a sentence. Therefore, if you grammar is not accurate (i.e., do not catch all the words in sentence), the confidence will be low even if a suitable recognition as been found.

Moreover, through the **speech_teleop_adapter** it is possible to set static values for the `/text_to_speech` topic (see the launch file for more information).

Also, the **web_interface_visualizer** has parameters to make the robot waiting for the user to finish speaking and in the opposite way round (see the `html` for more information). 

## Behavior

### Speech Recognition 

In this repository CAGG evaluator is used for identify *directives* tag streamed in the output topic. In particular:
- [**GO**]: is triggering by any sentences that contains the keywords: "go".
- [**STOP**]: is triggering by any sentences that contains the keywords: "stop", or "finish", or "done, or "ok".
- [**RESET**]: is triggering by any sentences that contains the keywords: "reset", or "forget", or "wrong", or "no";
This is defined in the grammar `ros_cagg_teleop/ros_java_cagg_teleop_interface/res/directive_grammar.cagg`.

Moreover, for the **GO** directive you can further specify sentences that follow the schema defined in the grammar `ros_cagg_teleop/ros_java_cagg_teleop_interface/res/go_grammar.cagg`. I.e.,:
``
!optional( <QUANTIFIER>) <relation> <object>
``
Where:
- the `QUANTIFIER` is such to be as `SLIGHTLY`, or `EXACTLY`,
- the `RELATION` can be: `RIGHT`, `LEFT`, `FRONT`, `BEHIND`, `NEAR` (or `CLOSE`), and
- the `OBJECT` is an integer number (i.e., unique identifier) of an object (supported number between `1` and `9`)
Note that in this case the `SUBJECT` is always the robot. 

For example: *"Miro, go slightly on the right-hand side of box number 3."* 
Will generate in the output topic a list of list as: 
``
{{GO,QUANTIFIER,SLIGHTLY},{GO,RELATION,right},{OBJECT,3}]
``
On the other had, sentences as: *"Miro stop!"*, *"forget all"*, will return: `{{STOP}}`, and `{{RESET}}` respectively

The package is such to call CAGG for checking the given directive, if it is of the *GO* type, than also the `go_grammar` is used. Otherwise the output will be only based on the `directive_grammar`.

Note that the input text is split by words such as *and*, and *also*.
Therefore, if you concatenate sentences through those keywords. The node will process them as two different string given in sequence.

Finally, consider that the sentence is not recognized if the node streams an empty message (i.e., `{{}}`) through the output topic.

### Data Logging

If you do not provide a logging path to **speech_teleop_adapter**, the system generates a log of the dialogue. 
By default those are saved in `speech_interaction/dialogs-log/` folder as a text file for each launch of the architecture.
Such a file contains tree possible types of lines as:
- time-stamp, "Human Voice", `language`, `confidence`, `transcript`
- time-stamp, "Robot Voice", `language`, `volume`, `pitch`, `rate`, `text`
- time-stamp, "CAGG Evaluation", `language`, `computation time`, `confidence`, `{{list, of, list},{of, semantic},{tags}}`


### Author

[Luca Buoncompagni](mailto:luca.buoncompagni@edu.unige.it)
EMAROlab, DIBRIS department, University of Genoa, Italy.

