/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <sstream>

#include "audio_common_msgs/AudioDataStamped.h"

namespace audio_transport {
  //
  // ---------- helper function for error handling
  //
  static gboolean  bus_call(GstBus  *bus, GstMessage *msg, gpointer data) {
    GMainLoop *loop = (GMainLoop *) data;

    switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      g_main_loop_quit(loop);
      break;
    case GST_MESSAGE_ERROR: {
      gchar  *debug;
      GError *error;
      gst_message_parse_error (msg, &error, &debug);
      g_free (debug);
      g_printerr ("Error: %s\n", error->message);
      g_error_free (error);
      g_main_loop_quit(loop);
      break; }
    default:
      break;
    }
    return TRUE;
  }

  class RawAudioPlayer  {
  public:
    RawAudioPlayer(const ros::NodeHandle &nh) : nh_(nh)  {
      // read ROS parameters
      nh_.param<std::string>("audio_in_format", audioInFormat_, "S24LE");
      nh_.param<int>("audio_in_depth", audioInDepth_, 24);
      nh_.param<int>("audio_in_rate",  audioInRate_, 48000);
      nh_.param<int>("num_audio_in_channels", numAudioInChannels_, 24);
      audioInBytes_          = audioInDepth_ / 8;
      audioInBytesPerSample_ = audioInBytes_ * numAudioInChannels_;
      nh_.param<std::vector<int>>("out_channels", outChannels_,
                                  std::vector<int> {0, 1});
      if (outChannels_.empty() || outChannels_.size() > 2) {
        ROS_ERROR_STREAM("must have 1 or 2 output channels");
        return;
      }
      if (outChannels_[0] < 0 || outChannels_[0] >= numAudioInChannels_) {
        ROS_ERROR_STREAM("out channel #1 out of range!");
        return;
      }
      if ((outChannels_.size() > 1) &&
          (outChannels_[1] < 0 || outChannels_[1] >= numAudioInChannels_)) {
        ROS_ERROR_STREAM("out channel #2 out of range!");
        return;
      }
      // initialize gst
      gst_init(NULL, NULL);
      // subscribe to ROS
      sub_      = nh_.subscribe("audio_stamped", 10,
                                &RawAudioPlayer::callback, this);
      // create a new loop object
      loop_     = g_main_loop_new(NULL, false);
      // create the complete pipeline (will link to it later)
      pipeline_ = gst_pipeline_new("raw_audio_pipeline");

      // create a handler to hear if the pipeline breaks
      GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
      gst_bus_add_watch(bus, bus_call, loop_);
      gst_object_unref(bus); // no longer needed I suppose...

      // make a gst "appsrc". This is a source where an applicatio
      // can directly stuff data in, exactly what we want
      source_ = gst_element_factory_make("appsrc", "raw_source");
     
      // since we are stuffing in raw bytes, we have to say
      // what format they have
      filter_ = gst_element_factory_make("capsfilter", "filter");
      std::stringstream fmt;
      fmt << "audio/x-raw,format=" << audioInFormat_ << ",channels="
          << outChannels_.size() << ",rate=" << audioInRate_;
      GstCaps *caps = gst_caps_from_string(fmt.str().c_str());
      g_object_set (filter_, "caps", caps, NULL);
      gst_caps_unref(caps);

      // no idea what the converter does, but I think it's needed
      conv_ = gst_element_factory_make("audioconvert", "convert");

      // this sink will be pulseaudio, or alsa, whatever the system finds
      sink_ = gst_element_factory_make("autoaudiosink", "sink");

      // now tie the whole pipeline together. A gst "bin" is
      // a single element that contains other elements. So
      // this is just a way of grouping the elements together
  
      gst_bin_add_many(GST_BIN(pipeline_), source_, filter_, conv_,
                       sink_, NULL);
      // link everything
      gst_element_link_many(source_, filter_, conv_, sink_, NULL);

      // switch on the pipeline
      gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_PLAYING);

      // kick off a new thread to drive the gst main loop
      gstThread_ = boost::thread(boost::bind(g_main_loop_run, loop_));
      ROS_INFO_STREAM("started gstreamer thread!");
    }

  private:

    // ROS callback function
    void callback(const audio_common_msgs::AudioDataStampedConstPtr &msg) {
      if (msg->format.rate != (unsigned int)audioInRate_) {
        ROS_ERROR_STREAM("incoming audio rate: " << msg->format.rate <<
                         " differs from config: " << audioInRate_);
        throw std::runtime_error("audio rate wrong!");
      }
      if (msg->format.channels != numAudioInChannels_) {
        ROS_ERROR_STREAM("incoming audio chans: " << msg->format.channels <<
                         " differs from config: " << numAudioInChannels_);
        throw std::runtime_error("audio channels wrong!");
      }
      if (msg->format.depth != audioInDepth_) {
        ROS_ERROR_STREAM("incoming audio depth: " << msg->format.depth <<
                         " differs from config: " <<  audioInDepth_);
        throw std::runtime_error("audio depth wrong!");
      }

      if (msg->format.id != "wave" && msg->format.id != audioInFormat_) {
        ROS_ERROR_STREAM("incoming audio format: " << msg->format.id <<
                         " differs from config: " <<  audioInFormat_);
        throw std::runtime_error("audio format wrong!");
      }
      const int numSamples = msg->data.size() / audioInBytesPerSample_;

      // assume that input format equals output format!
      std::vector<uint8_t> buf(numSamples * outChannels_.size() *
                               audioInBytes_);

      // go through the outut buffer in strides depending on format
      const int stride = audioInBytes_ * outChannels_.size();
      const int bps    = audioInBytesPerSample_;
      if (outChannels_.size() > 1) {
        unsigned int off(0);
        for (unsigned int i = 0; i < buf.size(); i += stride, off += bps) {
          const size_t off1 = off + outChannels_[0] * audioInBytes_;
          memcpy(&buf[i], &msg->data[off1], audioInBytes_);
          const size_t off2 = off + outChannels_[1] * audioInBytes_;
          memcpy(&buf[i] + audioInBytes_, &msg->data[off2], audioInBytes_);
        }
      } else {
        unsigned int off(0);
        for (unsigned int i = 0; i < buf.size(); i += stride, off += bps) {
          const size_t off1 = off + outChannels_[0] * audioInBytes_;
          memcpy(&buf[i], &msg->data[off1], audioInBytes_);
        }
      }
      // copy the modified data into the buffer
      GstBuffer *buffer = gst_buffer_new_and_alloc(buf.size());
      gst_buffer_fill(buffer, 0, &buf[0], buf.size());
      GstFlowReturn ret;
      g_signal_emit_by_name(source_, "push-buffer", buffer, &ret);
    }

    // ------- ROS stuff --------------
    ros::NodeHandle  nh_;
    ros::Subscriber  sub_;
    std::string      audioInFormat_;
    int              audioInDepth_;
    int              audioInRate_;
    int              audioInBytes_;
    int              numAudioInChannels_;
    int              audioInBytesPerSample_;
    std::vector<int> outChannels_;
    // gst stuff
    boost::thread   gstThread_; // thread that runs gst main loop
    GMainLoop       *loop_;
    GstElement      *pipeline_;
    GstElement      *source_;
    GstElement      *sink_;
    GstElement      *conv_;
    GstElement      *filter_;
  };
}


int main (int argc, char **argv) {
  ros::init(argc, argv, "raw_audio_player");
  ros::NodeHandle pnh("~");
  audio_transport::RawAudioPlayer node(pnh);
  ros::spin();
}
