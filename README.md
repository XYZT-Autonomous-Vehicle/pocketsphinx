# ROS package for pocketsphinx
This package is an attempt to bring offline speech recognition to ROS. Pocketsphinx already offers many easy-to-use features in this domain, hence this package can be considered as an extension of pocketsphinx in the ROS world!

>This fork has been created to update the original repository to be
compatible with Python3 (used in ROS Noetic onwards)

>This fork has also been modified for the sound source to come from [odas](https://github.com/introlab/odas), but it can be easily modified to suit your needs

## Getting Started
> This guide assumes that you are starting from an Ubuntu computer that has ROS Noetic installed already. See http://wiki.ros.org/noetic/Installation/Ubuntu

### Install all necesary dependencies:
```
sudo apt install gstreamer1.0-plugins-base gstreamer1.0-pocketsphinx pocketsphinx-en-us python3-gi python3-gst-1.0
```
> `gstreamer1.0-tools` and `gstreamer1.0-plugins-good` are be helpful for debugging and using alternate audio sources respectively, if any issues arise when using this package

### Clone and build package
```
cd <your catkin_ws directory>/src
git clone https://github.com/XYZT-Autonomous-Vehicle/pocketsphinx.git
catkin build
source devel/setup.bash
```

### Launch Example
```
roslaunch pocketsphinx voice_cmd.launch
```

## Modifying the sound source
It is necesary to modify the `self.launch_config` variable in `recognizer.py`. The first two plugins are configured for use with odas and can be replaced with an alternate source.
```
tcpserversrc host=<host> port=<port> ! rawaudioparse num-channels=1
```

The `self.launch_config` string specifies a [pipeline description](https://gstreamer.freedesktop.org/documentation/tools/gst-launch.html?gi-language=python#audio-playback), which can be modifed to have different inputs. See the debugging section for an example that uses a file as input.

## Debugging
### Install additional packages
```
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good
```

### Verify Pocketphinx plugin functionality
```
gst-launch-1.0 fakesrc ! pocketsphinx ! fakesink
```
It should display the Pocketsphinx configuration then print `Pipeline is PREROLLING ...` (you may kill it with CTRL-C). If this didn't work, you may have forgotten to install `pocketsphinx-en-us`.

### Run Pocketphinx off an audio file
```
gst-launch-1.0 filesrc location=<location_of_your_audio_file> ! decodebin ! audioconvert ! audioresample ! pocketsphinx ! filesink location=text.txt
```
> This has been tested using a `.wav` input audio file, but other common formats should work too

It should display similar info to the previous command, then begin running. It will terminate when it has fully consumed the input file and transcribed it to `text.txt`. The output file should now contain the spoken contents of the input. If this didn't produce an output file, there may be larger issues in the GStreamer installation. If the output text is incorrect, the speech in the input file may not be understandable or the default Pocketsphinx language model may not match what was spoken.

## References
- [Pocketpshinx main page](https://cmusphinx.github.io/)
- [Pocketsphinx example code](https://cmusphinx.github.io/wiki/gstreamer/)
- [Gstreamer Documentation](https://gstreamer.freedesktop.org/documentation/)
- [Forked repository](https://github.com/Pankaj-Baranwal/pocketsphinx)
- [Original repository](https://github.com/mikeferguson/pocketsphinx)
- [Additional source for forked repository](https://github.com/gorinars/ros_voice_control)