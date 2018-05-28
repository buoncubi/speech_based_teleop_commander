#include "ros/ros.h"
#include "speech_interaction/Text2speech.h"
#include "speech_interaction/Speech2text.h"
#include "ros_cagg_msgs/cagg_tags.h"
#include "std_msgs/String.h"

#include <fstream>

using namespace speech_interaction;
using namespace std;
using namespace std_msgs;
using namespace ros_cagg_msgs;

ros::Publisher toSpeechPub;

ros::Publisher caggPublisher;


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
	std::string formattedDialog = ":Human: [" + language + ", " + std::to_string( confidence) + ", \"" + transcript + "\"]";
	writeStrToFile( formattedDialog);
}
// writes on the specified file path the data coming from the robot
void printOutcomingDialog( std::string language, double volume, double pitch, double rate, std::string text){
	std::string formattedDialog = ":Robot: [" + language + ", " + std::to_string( volume) + ", " + std::to_string( pitch) + ", " + std::to_string( rate) + ", \"" + text + "\"]";
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

// Receive the message containing the transcript of what the user said
void getSpokenText(const Speech2textPtr& toText){ // callback
	// log for showing purposes
	ROS_INFO(" Speech to Text results.\n [language: %s] [transcript: %s] [confidence: %f]", toText->language.c_str(), toText->transcript.c_str(), toText->confidence);
	// print dialog on file
	if( shouldPrintDialog())
		printIncomingDialog( toText->language, toText->confidence, toText->transcript);

	// make the machine saying something
	//sendTextToSpeech( toText->transcript.c_str());
	
	// send sentence to CAGG for evaluation (it will trigger a callback)
	caggPublisher.publish( toText);
}

void getCAGGTags(const cagg_tags& toText){ // callback
}

int main(int argc, char **argv){
	// initialise the node
	ros::init(argc, argv, "speech_back_example");
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

	// initialise the publisher to produce speech from text
	toSpeechPub = node.advertise< Text2speech>( "/text_to_speech", 10);
	
	caggPublisher = node.advertise< std_msgs::String>( "/CAGG/input_text", 10);

	// spin continuously
	while (ros::ok()){
		ros::spin(); // rate !! ??
	}
	return 0;
}
