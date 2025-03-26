#!/bin/bash

. "$MMPUG_OPERATIONS/mmpug_utils/scripts/header.sh"
. "$MMPUG_OPERATIONS/mmpug_utils/scripts/formatters.sh"

if chk_flag --help $@ || chk_flag help $@ || chk_flag -h $@ || chk_flag -help $@; then
  title "$__file_name [ flags ] : Client gstreamer pipeline to receive image streams from remote host."
  text "For more help, please see the README.md or wiki."
  exit_success
fi

# //////////////////////////////////////////////////////////////////////////////
# @brief run the ansible robot playbook
# //////////////////////////////////////////////////////////////////////////////
main gstreamer client pipeline

gst-launch-1.0 udpsrc multicast-group=239.255.100.2 port=5000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! avdec_h264 ! xvimagesink

# gst-launch-1.0 udpsrc multicast-group=239.255.100.2 port=5000 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96' ! rtph264depay ! avdec_h264 ! video/x-raw, format=BGRx ! videoscale ! video/x-raw,width=816,height=686 ! xvimagesink
# exit and cleanup
exit_success
