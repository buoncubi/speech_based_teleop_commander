#include "ros/ros.h"
#include "speech_interaction/Text2speech.h"
#include "speech_interaction/Speech2text.h"
#include "ros_cagg_msgs/cagg_tags.h"
#include "std_msgs/String.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <fstream>

using namespace speech_interaction;
using namespace std;
using namespace std_msgs;
using namespace ros_cagg_msgs;

ros::Publisher toSpeechPub;

ros::Publisher caggInputPublisher;
ros::Publisher caggResultPublisher;


// node parameters
std::string printDialogPath;
// for synthetiser (for specification see: http://responsivevoice.org/api/)
std::string language;
double volume;
double pitch;
double rate;
// node parameter default
const std::string DEFAULT_PRINT_DIALOG_PATH = ""; // empty string means that the dialog should not be printed
const std::string DEFAULT_LANGUAGE = "US English Female"; // for other voices see: http://responsivevoice.org/text-to-speech-languages/
const double DEFAULT_VOLUME = 1.0;
const double DEFAULT_PITCH = 2.0;
const double DEFAULT_RATE = 1.5;


// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

// returns unix time stamp as a formatted string to append to dialog printing lines
// Returns the current date and time formatted as %Y-%m-%d_%H.%M.%S
const std::string getData(){
	std::time_t t = std::time(NULL);
	char mbstr[20];
	std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d_%H.%M.%S", std::localtime(&t));
	std::string currentDate(mbstr);
	return currentDate;
}
// write to file a single string
void writeStrToFile( const std::string &toWrite){
	std::string formattedTime = "{" + getData() + "} ";
	ofstream file;
	file.open( printDialogPath.c_str(), std::ofstream::out | std::ofstream::app);
	file << formattedTime << toWrite << "\n";
	file.close();
}
// writes on the specified file path the data coming from the human
void printIncomingDialog( std::string language, double confidence, std::string transcript){
	std::string formattedDialog = "Human Voice, " + currentDateTime() + ", " + language + ", " + std::to_string( confidence) + ", " + transcript;
	writeStrToFile( formattedDialog);
}
// writes on the specified file path the data coming from the robot
void printOutcomingDialog( std::string language, double volume, double pitch, double rate, std::string text){
	std::string formattedDialog = "Robot Voice, " + currentDateTime() + ", " + language + ", " + std::to_string( volume) + ", " + std::to_string( pitch) + ", " + std::to_string( rate) + ", " + text;
	writeStrToFile( formattedDialog);
}
// write on the specified file path the CAGG evaluation oucomes
void printCAGGEvaluation( std::string tagsStr, int computationTime, double confidence){
    std::string formattedDialog = "CAGG Evaluation, " + currentDateTime() + ", " + std::to_string( computationTime) + ", " + std::to_string( confidence) + ", " + tagsStr;
    writeStrToFile( formattedDialog);
}
// return true if the dialog should be printed on the given path (from ros parameter)
bool shouldPrintDialog(){
	if( printDialogPath.empty())
		return false;
	return true;
}


// send a message to say something to the user
void sendTextToSpeech( std::string userSpeechTranscription){
	Text2speech toSpeech;
	// generate something that the robot should say
	toSpeech.text = "You said: " + userSpeechTranscription;
	// set the speaker parameters given during node launching
	toSpeech.language = language;
	toSpeech.volume = volume;
	toSpeech.pitch = pitch;
	toSpeech.rate = rate;

	// publish to java script in order to speech from text
	toSpeechPub.publish( toSpeech);

	// log for showing purposes
	ROS_INFO( " Publishing text: \"%s\"", toSpeech.text.c_str());
	// print dialog on file
	if( shouldPrintDialog())
		printOutcomingDialog( toSpeech.language, toSpeech.volume, toSpeech.pitch, toSpeech.rate, toSpeech.text);
}

// Receive the message containing the transcript of what the user said.
// Call CAGG for parse the sentene and obtain a result
void getSpokenText(const Speech2textPtr& toText){ // callback
	// log for showing purposes
	//ROS_INFO(" Speech to Text results.\n [language: %s] [transcript: %s] [confidence: %f]", toText->language.c_str(), toText->transcript.c_str(), toText->confidence);

	// print dialog on file
	if( shouldPrintDialog())
		printIncomingDialog( toText->language, toText->confidence, toText->transcript);

	// send sentence to CAGG for evaluation (it will trigger a callback)
    std_msgs::String msg;
    msg.data = toText->transcript;
	caggInputPublisher.publish( msg);
}

void getCAGGTags(const cagg_tagsPtr& tags){ // callback
    // print dialog on file
    if( shouldPrintDialog()){
        std::string tagsStr = "[";
        for (int i = 0; i < tags->cagg_tags.size(); ++i) {
            tagsStr += "[";
            for (int j = 0; j < tags->cagg_tags[i].cagg_tag.size(); ++j) {
                tagsStr += std::string(tags->cagg_tags[i].cagg_tag[j].c_str());
                if ( (j <= tags->cagg_tags[i].cagg_tag.size() - 2) & tags->cagg_tags[i].cagg_tag.size() != 1)
                    tagsStr += "; ";
            }
            tagsStr += "]";
            if( i <= tags->cagg_tags.size() - 2 & tags->cagg_tags.size() != 1)
                tagsStr += "; ";
        }
        tagsStr += "]";
        printCAGGEvaluation( tagsStr, tags->computationTimeMs, tags->confidence);
    }
            
    caggResultPublisher.publish( tags);
}

void getTextToSpeech(const std_msgs::StringConstPtr& text) { // callback
    Text2speech toSpeech;
    // generate something that the robot should say
    toSpeech.text = text->data;
    // set the speaker parameters given during node launching
    toSpeech.language = language;
    toSpeech.volume = volume;
    toSpeech.pitch = pitch;
    toSpeech.rate = rate;

    // log for showing purposes
    //ROS_INFO( " Publishing text: \"%s\"", toSpeech.text.c_str());

    // print dialog on file
    if( shouldPrintDialog())
        printOutcomingDialog( toSpeech.language, toSpeech.volume, toSpeech.pitch, toSpeech.rate, toSpeech.text);

    // publish to java script in order to speech from text
    toSpeechPub.publish( toSpeech);
}

int main(int argc, char **argv){
	// initialise the node
	ros::init(argc, argv, "speech_adapter");
	ros::NodeHandle node("~"); // it should be private for multiple node instance setting (independently)

	// get parameters or use defaults
	node.param( "speech_dialog_print", printDialogPath, DEFAULT_PRINT_DIALOG_PATH);
	if( shouldPrintDialog()){
		printDialogPath = printDialogPath + "dialog_" + getData() + ".log";
		ROS_INFO( " Setting dialog printing on file: %s", printDialogPath.c_str());
	} else ROS_INFO( " Setting to do not print dialogs. (empty dialog path:\"%s\"", printDialogPath.c_str());
	// robot speaker parameters
	node.param( "robot_language", language, DEFAULT_LANGUAGE);
	node.param( "robot_volume", volume, DEFAULT_VOLUME);
	node.param( "robot_pitch", pitch, DEFAULT_PITCH);
	node.param( "robot_rate", rate, DEFAULT_RATE);
	ROS_INFO( " Setting robot speaker parameters: [language: %s] [volume: %f] [pitch: %f] [rate: %f]", language.c_str(), volume, pitch, rate);

	// print dialog file hearder (tou must to have set the printDialogPath)
	if( shouldPrintDialog()){
		writeStrToFile( "Data header ...");
		writeStrToFile( ":Human: [language, confidence, \"transcript\"]");
		writeStrToFile( ":Robot: [language, volume, pitch, rate, \"text\"]\n");
		writeStrToFile( "Dialogs ...\n");
	}

	// initialise the subscriber to get text from speech (see callback)
	ros::Subscriber toTextSub = node.subscribe( "/speech_to_text", 10, getSpokenText);

	ros::Subscriber fromCAGGsub = node.subscribe( "/CAGG/semantic_tags", 10, getCAGGTags);

    ros::Subscriber fromTextSub = node.subscribe( "/CAGG/adapted/text_to_speech", 10, getTextToSpeech);

	// initialise the publisher to produce speech from text
	toSpeechPub = node.advertise< Text2speech>( "/text_to_speech", 10);

	caggInputPublisher = node.advertise< std_msgs::String>( "/CAGG/input_text", 10);
    caggResultPublisher = node.advertise< cagg_tags>( "/CAGG/adapted/semantic_tags", 10);

	// spin continuously
	while (ros::ok()){
		ros::spin(); // rate !! ??
	}
	return 0;
}
