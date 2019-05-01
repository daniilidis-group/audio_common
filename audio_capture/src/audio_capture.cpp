#include <stdio.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <thread>
#include <chrono>
#include <memory>

#include <ros/ros.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "audio_common_msgs/AudioData.h"
#include "audio_common_msgs/AudioDataStamped.h"

namespace audio_transport {
  class RosGstCapture {
  public:
    RosGstCapture() {
    }

    ~RosGstCapture() {
      shutDown();
    }

    void readParameters() {
      // Need to encoding or publish raw wave data
      ros::param::param<std::string>("~format", _format, "S24LE");
      // frame id to put on the message
      ros::param::param<std::string>("~frame_id", _frame_id, "audio");
      // The bitrate at which to encode the audio
      ros::param::param<int>("~bitrate", _bitrate, 192);
      // only available for raw data
      ros::param::param<int>("~channels", _channels, 1);
      ros::param::param<int>("~depth", _depth, 16);
      ros::param::param<int>("~sample_rate", _sample_rate, 16000);
      // The source of the audio
      ros::param::param<std::string>("~device", _device, "");
      // How long to sleep before retry
      ros::param::param<int>("~retry_sleep_seconds", _retry_sleep, 10);
    }

    void run() {
      ROS_INFO_STREAM("started gst thread!");
      while (true) {
        if (initGst()) {
          ROS_INFO_STREAM("capturing incoming packets.");
          g_main_loop_run(_loop);
          ROS_WARN_STREAM("encountered audio device issue!");
          shutDown();
          ROS_INFO_STREAM("sleeping " << _retry_sleep << "s before retry");
          std::this_thread::sleep_for(std::chrono::seconds(_retry_sleep));
        } else {
          ROS_ERROR_STREAM("cannot init GST!");
          throw std::runtime_error("cannot init GST!");
        }
      }
    }

    void startGst() {
      _gst_thread = std::thread(&RosGstCapture::run, this);
    }

    bool initGst() {
      ROS_INFO_STREAM("initializing gstreamer...");
      _loop = g_main_loop_new(NULL, false);
      _pipeline = gst_pipeline_new("ros_pipeline");
      _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
      gst_bus_add_signal_watch(_bus);
      g_signal_connect(_bus, "message::error",
                       G_CALLBACK(onMessage), this);
      g_object_unref(_bus);

      // We create the sink first, just for convenience
      _sink = gst_element_factory_make("appsink", "sink");
      g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
      g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
      g_signal_connect( G_OBJECT(_sink), "new-sample",
                        G_CALLBACK(onNewBuffer), this);
        
      _source = gst_element_factory_make("alsasrc", "source");
      // if device isn't specified, it will use the default which is
      // the alsa default source.
      // A valid device will be of the foram hw:0,0 with other numbers
      // than 0 and 0 as are available.
      if (_device != "") {
        // ghcar *gst_device = device.c_str();
        g_object_set(G_OBJECT(_source), "device", _device.c_str(), NULL);
      }

      _filter = gst_element_factory_make("capsfilter", "filter");
      {
        GstCaps *caps;
        caps = gst_caps_new_simple("audio/x-raw",
                                   //      "channels", G_TYPE_INT, _channels,
                                   //      "depth",    G_TYPE_INT, _depth,
                                   "rate",     G_TYPE_INT, _sample_rate,
                                   //       "signed",   G_TYPE_BOOLEAN, TRUE,
                                   NULL);
        g_object_set( G_OBJECT(_filter), "caps", caps, NULL);
        gst_caps_unref(caps);
      }

      _convert = gst_element_factory_make("audioconvert", "convert");
      if (!_convert) {
        ROS_ERROR_STREAM("Failed to create audioconvert element");
        return (false);
      }

      GstCaps *caps;
      caps = gst_caps_new_simple("audio/x-raw",
                                 "channels", G_TYPE_INT, _channels,
                                 "width",    G_TYPE_INT, _depth,
                                 "depth",    G_TYPE_INT, _depth,
                                 "rate",     G_TYPE_INT, _sample_rate,
                                 "signed",   G_TYPE_BOOLEAN, TRUE,
                                 NULL);
      g_object_set( G_OBJECT(_sink), "caps", caps, NULL);
      gst_caps_unref(caps);
      gst_bin_add_many( GST_BIN(_pipeline), _source, _sink, NULL);
      gboolean link_ok = gst_element_link_many( _source, _sink, NULL);

      if (!link_ok) {
        ROS_ERROR_STREAM("Unsupported media type.");
        return (false);
      }

      gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
      ROS_INFO_STREAM("gstreamer initialized successfully!");
      return (true);
    }

    void initializeROS() {
      double window_size, min_delay, max_delay, fcallback, ftol;
      ros::param::param<double>("~callback_frequency", fcallback, 100);
      ros::param::param<double>("~diagnostic_freq_tolerance", ftol, 0.05);
      ros::param::param<double>("~diagnostic_window_size", window_size, 10.0);
      ros::param::param<double>("~diagnostic_min_delay", min_delay, -2.0/fcallback);
      ros::param::param<double>("~diagnostic_max_delay", max_delay,  2.0/fcallback);
      _min_sample_rate = fcallback * (1.0 - ftol);
      _max_sample_rate = fcallback * (1.0 + ftol);
      _updater.setHardwareID(_device.empty() ? "none" : _device);
      _diagnostic.reset(
        new diagnostic_updater::TopicDiagnostic(
          "audio", _updater,
          diagnostic_updater::FrequencyStatusParam(&_min_sample_rate,
                                                   &_max_sample_rate, 0.0, window_size),
          diagnostic_updater::TimeStampStatusParam(min_delay, max_delay)));

      _pub = _nh.advertise<audio_common_msgs::AudioData>("audio", 10, true);
      _pub_stamped = _nh.advertise<audio_common_msgs::AudioDataStamped>("audio_stamped", 10, true);
    }
    

      void shutDown() {
        ROS_INFO_STREAM("cleaning up gst device state");
        g_main_loop_quit(_loop); // should be a no-op
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        gst_object_unref(_pipeline);
        g_main_loop_unref(_loop); // will this bring down the whole pipeline?
      }

      void publish(const ros::Time &t, const unsigned char *data, size_t size) {
        if (_pub.getNumSubscribers() > 0) {
          audio_common_msgs::AudioData msg;
          msg.data.resize(size);
          memcpy( &msg.data[0], data, size);
          _pub.publish(msg);
        }
        if (_pub_stamped.getNumSubscribers() > 0) {
          audio_common_msgs::AudioDataStamped msg;
          msg.header.stamp = t;
          msg.header.frame_id = _frame_id;
          msg.format.id       = _format;
          msg.format.rate     = _sample_rate;
          msg.format.channels = _channels;
          msg.format.depth    = _depth;
          msg.data.resize(size);
          memcpy(&msg.data[0], data, size);
          _pub_stamped.publish(msg);
        }
        _diagnostic->tick(t);
        _updater.update();
      }

      static GstFlowReturn onNewBuffer (GstAppSink *appsink, gpointer userData)
      {
        ros::Time t = ros::Time::now();
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GstMapInfo map;

        GstSample *sample;
        g_signal_emit_by_name(appsink, "pull-sample", &sample);

        GstBuffer *buffer = gst_sample_get_buffer(sample);

        gst_buffer_map(buffer, &map, GST_MAP_READ);
        server->publish(t, map.data, map.size);
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        return GST_FLOW_OK;
      }

      static gboolean onMessage (GstBus *bus, GstMessage *message, gpointer userData)
      {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GError *err;
        gchar *debug;

        gst_message_parse_error(message, &err, &debug);
        ROS_ERROR_STREAM("gstreamer: " << err->message);
        g_error_free(err);
        g_free(debug);

        g_main_loop_quit(server->_loop);
        return TRUE;
      }

  private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    ros::Publisher _pub_stamped;
    std::string    _frame_id;
    std::thread    _gst_thread;
    diagnostic_updater::Updater _updater;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> _diagnostic;
    double _min_sample_rate;
    double _max_sample_rate;
    GstElement *_pipeline, *_source, *_filter, *_sink, *_convert, *_encode;
    GstBus *_bus;
    int _bitrate{192};
    int _channels, _depth, _sample_rate;
    GMainLoop *_loop;
    std::string _format;
    std::string _device;
    int         _retry_sleep;
  };
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_capture");
  gst_init(&argc, &argv);
  try {
    audio_transport::RosGstCapture server;
    server.readParameters();
    server.initializeROS();
    server.startGst();
    ros::spin();
  } catch (const std::runtime_error &e) {
    ROS_WARN_STREAM("got runtime error!");
  }
}
