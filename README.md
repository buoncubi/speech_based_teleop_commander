# ROS-CAGG Interpreter for Qualitiative Teleoperation of the Miro Robot

This repository contains a simple node that performs text recognition based on the Concept-Action Grammar Generator API ([CAGG](https://github.com/EmaroLab/concept_action_grammar_generator)).

### Dependences

It depends on ros-java, installable o via `apt-get`  (tested with Ubuntu 16.04 and ROS kinetic) and the CAGG API.
The latter is included as a jar file in the `/lib` folder, run it with a JVM if you what to use the CAGG grammar generator GUI.

This repository has been based on the ROS-java templated package [available here](https://github.com/buoncubi/ros_java_template_pkg) and on the CAGG templated interface [available here](https://github.com/buoncubi/ros_cagg_pkgs).

### Installation

Simpli copy this repository in the `src` folder of your *catking workspace*. Run `$ catkin_make`, and than run `./gradlew deployApp` in the `ros_cagg_teleop` package.

### Execution

An example can be executed using:
``
roslaunch ros_cagg_teleop ros_cagg_interface.launch
``

For make a test send to `ros_cagg_teleop` a sentence like:
``
rostopic pub /CAGG/input_text std_msgs/String "data: 'go on the right-hand side of box 1, also go slightly on front of 2, also left 9,  and stop.'"
``
and checkout the responce in the output topic.

## Architecture

The repository provides a node (called `CAGG_teleop` and implemented in `ros_java_cagg_teleop_interface`) which listens to a topic and publish in another. 
The definitions of the ROS messages are stored in a separate package called `ros_cagg_msgs`.
In particular,
- the input topic is named `/CAGG/input_text/` and is a string (i.e., a text provided by a user)
- the output topic is named `/CAGG/semantic_tags/` a custom message called `cagg_tags` and structured as:
 - `Header`,
 - `Time Stamp`,
 - `cagg_tag` (a list of list of string, where each recognized words in a sentences are defined through a list of semantic tag, based on the provided CAGG grammar),
 - `confidence` (a `float` value between 0 and 1).

### Parameter

The `CAGG_teleop` requires specific value on the ROS parameter server, in particular:
- `/cagg_log_config_path`: the absolute path to the Log4j configuration file,
- `/cagg_serialized_directive_grammar`: the absolute path tho the serialized CAGG grammar file about *directives* commands,
- `/cagg_serialized_go_grammar`: the absolute path tho the serialized CAGG grammar file about *go* commands,

Also, you can set other further parameters of `CAGG_teleop` such as:
- `/cagg_timeout_ms`: the time (in milliseconds) to run the CAGG evaluation, after which the best result so far will be published in the output topic (remarkably, the evaluation start as soon as you send something on the input topic). By default set to 10000ms.
- `/cagg_stopping_check_frequency`: represent the frequency of stopping condition used before time-out. Remarkably, time-out will be applied after a multiple times of the `/cagg_stopping_check_frequency`. By default set to 1000ms.
- `/cagg_stopping_confidence_threshold`: represents the confidence threshold to stop searching for further results before time-out.
 - the *confidence* is computed as the ration of the word in a sentence at which CAGG attached at least a semantic tag, over the total number of words in a sentence. Therefore, if you grammar is not accurate (i.e., do not catch all the words in sentence), the confidence will be low even if a suitable recognition as been found.

### Behavior

In this repository CAGG evaluator is used for identify *directives* tag streamed in the output topic. In particular:
- [**GO**]: is triggering by any sentences that contains the keywords: "go".
- [**STOP**]: is triggering by any sentences that contains the keywords: "stop", or "finish", or "done, or "ok".
- [**RESET**]: is triggering by any sentences that contains the keywords: "reset", or "forget", or "wrong", or "no";
This is defined in the grammar `res/directive_grammar.cagg`.

Moreover, for the **GO** directive you can further specify sentences that follow the schema defined in the grammar `res/go_grammar.cagg`. I.e.,:
``
!optional( <QUANTIFIER>) <relation> <object>
``
Where:
- the `QUANTIFIER` is such to be as `SLIGHTLY`, or `EXACTLY`,
- the `RELATION` can be: `right`, `left`, `front`, `bheind`, and
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

Finally, consider that the sentence is not recognized if the node streams an empty message (i.e., `{{}}`) throught the output topic.

### Author

[Luca Buoncompagni](mailto:luca.buoncompagni@edu.unige.it)
EMAROlab, DIBRIS department, University of Genoa, Italy.

Previous commit for this package can be [found here](https://gitlab.com/EMAROLabGroup/ros_cagg_miro_teleop).
