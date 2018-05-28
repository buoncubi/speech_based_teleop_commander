#!/bin/bash

# default values	
OPEN_PATH=~/ros_ws/src/speech_interaction/java-script/speech_web_interface.html # path to the visualiser to open
SECURITY=0; # if 1 open on security mode
DOSOMETHING=1; # do not do anithing if input parameter are wrong

# helper
usage=" PERHAPS SOMETHING WHERE WRONG ??!! Anyway, here is an helper:
	$(basename "$0") [-u] [-s] [-h]
This script runs the google speach API web interface for ROS.

where:
	-u  specifies the url (or path) to the web interface (html file) to open. Default value: "$OPEN_PATH".
	-s  flag that, if specfified, force the chrome security behaviour. Default value: "$SECURITY".
	-h  show this help text.
"


# get input parameter
while getopts ':h :s :u:' option; do
  case "$option" in
    h) echo "$usage"
       exit
	;;
    u) printf "opening $OPTARG" >&2
        OPEN_PATH=$OPTARG
	;;	
    s) SECURITY=1
	;;	
    :) printf "missing argument for -%s\n" "$OPTARG" >&2
       echo "$usage" >&2
       DOSOMETHING=0
       #exit 0
       ;;
    \?) printf "illegal option: -%s\n" "$OPTARG" >&2
       echo "$usage" >&2
       DOSOMETHING=0
       #exit 0
       ;;
    *)
       echo "$usage" >&2
       DOSOMETHING=0
       #exit 0
    ;;
  esac
done
shift $((OPTIND - 1))

# open interface
if [[ "$DOSOMETHING" == "1" ]]; then
	if [[ "$SECURITY" == "1" ]]; then
		google-chrome-stable --user-data-dir=/var/tmp/Chrome $OPEN_PATH #open in secure mode
	else
		google-chrome-stable --user-data-dir=/var/tmp/Chrome --disable-web-security $OPEN_PATH 
	fi
fi
