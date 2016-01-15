#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <iostream>
#include "espeak/speak_lib.h"

// test using: rostopic pub -1 /text_to_speech std_msgs/String Hello

class EspeakSynth
{
public:
  EspeakSynth()
  {
    logIfError(espeak_Initialize(AUDIO_OUTPUT_SYNCH_PLAYBACK, 0,0,0));
  }
  ~EspeakSynth()
  {
    espeak_Terminate();
  }
  
  void synthesize(const char* msg)
  {
  }


private:
  // disable copying
  EspeakSynth(const EspeakSynth & );
  void logIfError(int errorCode) 
  {
    if (errorCode) {
      std::cout << "ERROR CODE " << errorCode << std::endl;
    }
  }
};

class Synthesizer {
  ros::NodeHandle n;
  ros::Subscriber sub;
  EspeakSynth synth;
public:
  Synthesizer() : 
    n(),
    sub(n.subscribe("text_to_speech", 300, &Synthesizer::textReceived, this)),
    synth()
  {
  }
  
  void textReceived(const std_msgs::String::ConstPtr &msg)
  {
    ROS_INFO("Received text_to_speech: [%s]", msg->data.c_str());
    std::cout << "RECEIVED " << msg->data.c_str() << std::endl;
    synth.synthesize(msg->data.c_str());
  }
private:
  // disable copying
  Synthesizer(const Synthesizer & );
};



int main(int argc, char **argv)
{
  std::cout << "STARTED" << std::endl;
  ros::init(argc, argv, "speech_synthesizer");

  Synthesizer s;

  ros::spin();

  return 0;
}