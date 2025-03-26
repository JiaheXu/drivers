#!/bin/bash

. "$MMPUG_OPERATIONS/mmpug_utils/scripts/header.sh"
. "$MMPUG_OPERATIONS/mmpug_utils/scripts/formatters.sh"

if chk_flag --help $@ || chk_flag help $@ || chk_flag -h $@ || chk_flag -help $@; then
  title "$__file_name [ flags ] : Streams 6 camera gstreamer pipeline to local or remote host."
  text "Flags:"
  text "    -ip  	  < IP > 	                    : Local or remote system IP for the gstreamer pipeline stream"
	text "    -p  	  < port > 			              : Local or remote system port for the gstreamer pipeline stream."
  text "    -n  	  < num > 			              : Number of cameras to stream."
  text "    -fps  	< frames per second > 			: Number of frames per second to streamed."
  text "    -ds  	  < downsample rate > 			  : The downsample rate of frames stramed (exmaple -ds 4 means to downsample by 4)."
	text
	text " Streams camera gstreamer pipeline to local or remote host."
	text "	- example, localhost pipeline stream have the following setup: IP : 127.0.0.1 ,  Port : 5000. "
	text "For more help, please see the README.md or wiki."
  exit_success
fi

# //////////////////////////////////////////////////////////////////////////////
# @brief run the ansible robot playbook
# //////////////////////////////////////////////////////////////////////////////
main gstreamer 6 cameras pipeline

# get the system IP
if ! chk_flag -ip $@; then
  error "please specify hostname (-ip) of where to stream the gstreamer pipeline (see help for details)."
	exit_failure
fi
gst_ip=$(get_arg -ip $@)

# get the system port
if ! chk_flag -p $@; then
  error "please specify hostname (-p) of where to stream the gstreamer pipeline (see help for details)."
	exit_failure
fi
# get the docker container name
gst_port=$(get_arg -p $@)

# get the system port
if ! chk_flag -n $@; then
  error "please specify number of cameras (-n) to stream (see help for details)."
	exit_failure
fi
# get the docker container name
gst_ncameras=$(get_arg -n $@)

# get the system port
if ! chk_flag -fps $@; then
  error "please specify number frames per second (-fps) to stream (see help for details)."
	exit_failure
fi
# get the docker container name
gst_fps=$(get_arg -fps $@)

# TODO: automatic downsample... -- right now manual

# downsample rate x4
# dsp="sink_1::xpos=624 sink_2::xpos=1248 sink_3::ypos=520 sink_4::xpos=624 sink_4::ypos=520 sink_5::xpos=1248 sink_5::ypos=520"
dsp="sink_0::xpos=1248 sink_1::xpos=624 sink_2::xpos=0 sink_3::ypos=520 sink_4::xpos=624 sink_4::ypos=520 sink_5::xpos=1248 sink_5::ypos=520"
width=624
height=520

if chk_flag -ds $@; then
  # downsample rate x4
  # dsp="sink_1::xpos=312 sink_2::xpos=624 sink_3::ypos=260 sink_4::xpos=312 sink_4::ypos=260 sink_5::xpos=624 sink_5::ypos=260"
  dsp="sink_0::xpos=624 sink_1::xpos=312 sink_2::xpos=0 sink_3::ypos=260 sink_4::xpos=312 sink_4::ypos=260 sink_5::xpos=624 sink_5::ypos=260"
  width=312
  height=260
fi

# setup the bitrate options
opts=""
if chk_flag -opts $@; then
  gst_opts=$(get_arg -opts $@)

  # add bitrate
  # bitrate=20000000
  # opts="omxh264enc bitrate=500000 control-rate=constant vbv-size=1  EnableTwopassCBR=true iframeinterval=60"
  # opts="bitrate=500000 control-rate=constant vbv-size=1  EnableTwopassCBR=true iframeinterval=60"
  # opts="bitrate=700000 control-rate=constant vbv-size=1  EnableTwopassCBR=true iframeinterval=60"
  opts="bitrate=$gst_opts control-rate=constant vbv-size=1  EnableTwopassCBR=true iframeinterval=60"
fi

# -- gstreamer pipeline --

# setup the camera0 gstreamer pipeline
gst_main="gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), width=(int)$width, height=(int)$height, framerate=(fraction)$gst_fps/1' ! nvvidconv ! 'video/x-raw' ! queue ! videomixer name=m $dsp ! omxh264enc $opts ! 'video/x-h264,stream-format=byte-stream' ! h264parse ! rtph264pay ! udpsink host=$gst_ip port=$gst_port "

# add on all non-master cameras
x=1
gst_add_camera=""

# very ugly coding...
if [[ ${gst_ncameras} -eq 3 ]]; then
  # force a specific sequence
  gst_add_camera="${gst_add_camera} nvarguscamerasrc sensor-id=1 ! 'video/x-raw(memory:NVMM), width=(int)$width, height=(int)$height, framerate=(fraction)$gst_fps/1' ! nvvidconv ! 'video/x-raw' ! queue ! m. "
  gst_add_camera="${gst_add_camera} nvarguscamerasrc sensor-id=2 ! 'video/x-raw(memory:NVMM), width=(int)$width, height=(int)$height, framerate=(fraction)$gst_fps/1' ! nvvidconv ! 'video/x-raw' ! queue ! m. "
else
  while [ $x -lt $gst_ncameras ]; do
    gst_add_camera="${gst_add_camera} nvarguscamerasrc sensor-id=${x} ! 'video/x-raw(memory:NVMM), width=(int)$width, height=(int)$height, framerate=(fraction)$gst_fps/1' ! nvvidconv ! 'video/x-raw' ! queue ! m. "
    x=$(( $x + 1 ))
  done
fi

# run the gstreamer commands
echo "gstreamer command: $gst_main $gst_add_camera"
eval "$gst_main $gst_add_camera"

# exit and cleanup
exit_success

