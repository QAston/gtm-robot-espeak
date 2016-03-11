#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <iostream>
#include "espeak/speak_lib.h"

// test using: rostopic pub -1 /text_to_speech std_msgs/String "Witaj świecie"


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
    logIfError(espeak_Terminate());
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
    logIfError(espeak_Synth(msg, len + 1, 0, POS_CHARACTER, 0, flags, play_id, user_data));
  }
  
  void stopPlaying()
  {
    logIfError(espeak_Cancel());
  }
  
  bool isPlaying()
  {
    return espeak_IsPlaying();
  }
  
  void setVoice(const char* voice)
  {
    logIfError(espeak_SetVoiceByName(voice));
  }
  
  /**
   * volume in range 0-200 or more. 0=silence, 100=normal full volume,
   */
  
  void setVolume(int value)
  {
    logIfError(espeak_SetParameter(espeakVOLUME, value, 0));
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
    logIfError(espeak_SetParameter(espeakPITCH, value, 0));
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
    logIfError(espeak_SetParameter(espeakRATE, value, 0));
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
    logIfError(espeak_SetParameter(espeakRANGE, value, 0));
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
    logIfError(espeak_SetParameter(espeakPUNCTUATION, value, 0));
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
    logIfError(espeak_SetParameter(espeakWORDGAP, value, 0));
  }
  
  int getWordGap()
  {
    return espeak_GetParameter(espeakWORDGAP, true);
  }

private:
  // disable copying
  EspeakSynth(const EspeakSynth &);

  void logIfError(int errorCode)
  {
    if (errorCode)
    {
      ROS_ERROR("Espeak error, error code: %i", errorCode);
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
  ros::NodeHandle n;
  ros::Subscriber sub;
  EspeakSynth synth;

public:
  Synthesizer() : n(), sub(n.subscribe("text_to_speech", 300, &Synthesizer::textReceived, this)), synth(EspeakEventHandler)
  {
  }

  void textReceived(const std_msgs::String::ConstPtr &msg)
  {
    // don't change settings and stuff while playing because espeak doesn't work properly then
    // either wait or cancel
    while(synth.isPlaying()) {
    }
    ROS_INFO("Received text_to_speech: [%s]", msg->data.c_str());
    synth.asyncPlay(msg->data.c_str(), 0, 0, this);
  }
  
  //
  bool espeakPlayingWord(unsigned int play_id, int text_position, int length, int audio_position, int word_number)
  {

    ROS_INFO("Espeak playing a word");
    return false;
  }
  
  bool espeakPlayingSentence(unsigned int play_id, int text_position, int audio_position, int sentence_number)
  {
    ROS_INFO("Espeak playing a sentence");
    return false;
  }

  bool espeakPlayingSentenceEnd(unsigned int play_id, int text_position, int audio_position)
  {
    ROS_INFO("Espeak playing a sentence ending");
    return false;
  }
  
  bool espeakPlayingEnd(unsigned int play_id, int text_position, int audio_position)
  {
    ROS_INFO("Espeak playing ended");
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