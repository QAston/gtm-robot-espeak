#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <iostream>
#include "espeak/speak_lib.h"
#include <actionlib/server/simple_action_server.h>
#include "gtm_speech_msgs/SpeakActionFeedback.h"
#include "gtm_speech_msgs/SpeakActionGoal.h"
#include "gtm_speech_msgs/SpeakAction.h"
#include "gtm_speech_msgs/SpeakActionResult.h"
#include "gtm_speech_msgs/SpeakFeedback.h"
#include "gtm_speech_msgs/SpeakGoal.h"
#include "gtm_speech_msgs/SpeakResult.h"
#include "boost/bind.hpp"
#include <stdexcept>
#include "gtm_speech_msgs/SpeechConfig.h"
#include "dynamic_reconfigure/server.h"

/**
 *actions served: speak - speak one-msg-at-a-time
 *has dynamic_reconfiguration for volume and stuff
 */


// example libespeak library usage:
// http://bazaar.launchpad.net/~rainct/python-espeak/trunk/view/head:/espeak/espeakmodulecore.cpp
// espeak can also take tags in text: http://espeak.sourceforge.net/ssml.html

class EspeakSynth
{
public:
  EspeakSynth(t_espeak_callback* synthCallback)
  {
    espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 2000, NULL, 0);
    espeak_SetSynthCallback(synthCallback);
    setVoice("pl");
  }
  ~EspeakSynth()
  {
    handleError(espeak_Terminate());
  }

  /**
   * msg: message to play
   * play_flags: espeakSSML   Elements within < > are treated as SSML elements, or if not recognised are ignored.

            espeakPHONEMES  Text within [[ ]] is treated as phonemes codes (in espeak's Hirshenbaum encoding).

            espeakENDPAUSE  If set then a sentence pause is added at the end of the text.  If not set then
              this pause is suppressed.
   * play_id: id helpful for identifying later data supplied to the callback.
   * user_data: pointer which will be passed to the callback function.
   */
  void asyncPlay(const char *msg, int play_flags = 0, unsigned int* play_id = NULL, void* user_data = NULL)
  {
    int len = strlen(msg);

    unsigned int flags = espeakCHARS_AUTO | play_flags;
    handleError(espeak_Synth(msg, len + 1, 0, POS_CHARACTER, 0, flags, play_id, user_data));
  }
  
  void stopPlaying()
  {
    handleError(espeak_Cancel());
  }
  
  bool isPlaying()
  {
    return espeak_IsPlaying();
  }
  
  void setVoice(const char* voice)
  {
    handleError(espeak_SetVoiceByName(voice));
  }
  
  /**
   * volume in range 0-200 or more. 0=silence, 100=normal full volume,
   */
  
  void setVolume(int value)
  {
    handleError(espeak_SetParameter(espeakVOLUME, value, 0));
  }
  
  int getVolume()
  {
    return espeak_GetParameter(espeakVOLUME, true);
  }
  
  /**
   *  base pitch, range 0-100.  50=normal
   */
  void setPitch(int value)
  {
    handleError(espeak_SetParameter(espeakPITCH, value, 0));
  }
  
  int getPitch()
  {
    return espeak_GetParameter(espeakPITCH, true);
  }
  
  /**
   *  speaking speed in word per minute.  Values 80 to 450.
   */
  void setRate(int value)
  {
    handleError(espeak_SetParameter(espeakRATE, value, 0));
  }
  
  int getRate()
  {
    return espeak_GetParameter(espeakRATE, true);
  }
  
   /**
   *  pitch range, range 0-100. 0-monotone, 50=normal
   */
  void setPitchRange(int value)
  {
    handleError(espeak_SetParameter(espeakRANGE, value, 0));
  }
  
  int getPitchRange()
  {
    return espeak_GetParameter(espeakRANGE, true);
  }
  
   /**
   *  which punctuation characters to announce
   */
  void setPunctuation(espeak_PUNCT_TYPE value)
  {
    handleError(espeak_SetParameter(espeakPUNCTUATION, value, 0));
  }
  
  int getPunctuation()
  {
    return espeak_GetParameter(espeakPUNCTUATION, true);
  }
  
   /**
   *  pause between words, units of 10mS (at the default speed)
   */
  void setWordGap(int value)
  {
    handleError(espeak_SetParameter(espeakWORDGAP, value, 0));
  }
  
  int getWordGap()
  {
    return espeak_GetParameter(espeakWORDGAP, true);
  }

private:
  // disable copying
  EspeakSynth(const EspeakSynth &);

  void handleError(int errorCode)
  {
    switch (errorCode)
    {
      case EE_OK:
        return;
      case EE_INTERNAL_ERROR:
        throw std::runtime_error("Espeak internal error.");
      case EE_BUFFER_FULL:
        throw std::runtime_error("Espeak buffer full error.");
      case EE_NOT_FOUND:
        throw std::runtime_error("Espeak not found error.");
      default:
        throw std::runtime_error("Unknown error.");
    }
  }
};

/* 
   wav:  is the speech sound data which has been produced.
      NULL indicates that the synthesis has been completed.

   numsamples: is the number of entries in wav.  This number may vary, may be less than
      the value implied by the buflength parameter given in espeak_Initialize, and may
      sometimes be zero (which does NOT indicate end of synthesis).

   events: an array of espeak_EVENT items which indicate word and sentence events, and
      also the occurance if <mark> and <audio> elements within the text. 
   Callback returns: 0=continue synthesis,  1=abort synthesis.
*/
int EspeakEventHandler(short *wav, int numsamples, espeak_EVENT *events);

class Synthesizer
{
  typedef actionlib::SimpleActionServer<gtm_speech_msgs::SpeakAction> ActionServer;
  typedef ActionServer::GoalConstPtr GoalConstPtr;
  typedef dynamic_reconfigure::Server<gtm_speech_msgs::SpeechConfig> ReconfigureServer;
  
  ros::NodeHandle n;
  EspeakSynth synth;
  ActionServer action_server;
  ReconfigureServer reconfigure_server;
  ros::Subscriber sub;

public:
  Synthesizer() : n(), synth(EspeakEventHandler), action_server(n, "speak", false),
    reconfigure_server()
  {
    reconfigure_server.setCallback(boost::bind(&Synthesizer::reconfigureCB, this, _1, _2));
    action_server.registerGoalCallback(boost::bind(&Synthesizer::goalCB, this));
    action_server.registerPreemptCallback(boost::bind(&Synthesizer::preemptCB, this));
    action_server.start();
  }
  
  ~Synthesizer()
  {
    action_server.shutdown();
  }
  
  void reconfigureCB(gtm_speech_msgs::SpeechConfig &config, uint32_t level)
  {
    synth.setPitch(config.pitch);
    synth.setPunctuation(config.announce_punctuation ? espeakPUNCT_ALL : espeakPUNCT_NONE);
    synth.setPitchRange(config.pitch_range);
    synth.setRate(config.rate);
    synth.setVolume(config.volume);
    synth.setWordGap(config.wordgap);
  }
  
  void goalCB()
  {
    boost::shared_ptr<const gtm_speech_msgs::SpeakGoal> goal = action_server.acceptNewGoal();
    if (synth.isPlaying()) {
      ROS_INFO("wait to stop goal");
      while (synth.isPlaying());
    }
    ROS_INFO("Received speak text: [%s]", goal->text.c_str());
    try
    {
      synth.asyncPlay(goal->text.c_str(), 0, 0, this);
    }
    catch(const std::runtime_error& ex)
    {
      ROS_ERROR("Failed speaking text %s", goal->text.c_str());
      gtm_speech_msgs::SpeakResult result;
      result.finished = false;
      action_server.setAborted(result);
    }
  }
  
  void preemptCB()
  {
    ROS_INFO("Speaking preempted");
    if (synth.isPlaying())
    {
      ROS_INFO("Stopping speech");
      synth.stopPlaying();
    }
    gtm_speech_msgs::SpeakResult result;
    result.finished = false;
    action_server.setPreempted(result);
  }
  
  void publishFeedback(int text_position)
  {
    if (!action_server.isPreemptRequested()) {
      gtm_speech_msgs::SpeakFeedback feedback;
      feedback.processed_characters = text_position;
      action_server.publishFeedback(feedback);
    }
  }
  
  //
  bool espeakPlayingWord(unsigned int play_id, int text_position, int length, int audio_position, int word_number)
  {
    ROS_INFO("Espeak playing a word");
    publishFeedback(text_position);
    return false;
  }
  
  bool espeakPlayingSentence(unsigned int play_id, int text_position, int audio_position, int sentence_number)
  {
    ROS_INFO("Espeak playing a sentence");
    publishFeedback(text_position);
    return false;
  }

  bool espeakPlayingSentenceEnd(unsigned int play_id, int text_position, int audio_position)
  {
    ROS_INFO("Espeak playing a sentence ending");
    publishFeedback(text_position);
    return false;
  }
  
  bool espeakPlayingEnd(unsigned int play_id, int text_position, int audio_position)
  {
    ROS_INFO("Espeak playing request ended");
    if (!action_server.isPreemptRequested()) {
      gtm_speech_msgs::SpeakResult result;
      result.finished = true;
      action_server.setSucceeded(result);
    }
    return false;
  }

private:
  // disable copying
  Synthesizer(const Synthesizer &);
};

int EspeakEventHandler(short *wav, int numsamples, espeak_EVENT *events)
{
  espeak_EVENT * event = events;
  Synthesizer* synthesizer = (Synthesizer*)events->user_data;
  switch (event->type)
  {
    case espeakEVENT_WORD:
      return synthesizer->espeakPlayingWord(event->unique_identifier, event->text_position, event->length, event->audio_position, event->id.number);
    case espeakEVENT_SENTENCE:
      return synthesizer->espeakPlayingSentence(event->unique_identifier, event->text_position, event->audio_position, event->id.number);
    case espeakEVENT_SAMPLERATE:
      break;
    case espeakEVENT_END:
      return synthesizer->espeakPlayingSentenceEnd(event->unique_identifier, event->text_position, event->audio_position);
    case espeakEVENT_MSG_TERMINATED:
      return synthesizer->espeakPlayingEnd(event->unique_identifier, event->text_position, event->audio_position);
    default:
      ROS_WARN("Unhandled espeak event: %i", event->type);
      break;
  }
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "speech_synthesizer");

  Synthesizer s;

  ros::spin();

  return 0;
}