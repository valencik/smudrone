#!/usr/bin/env bash

#Get the target from supplied argument
if [ $# -ne 1 ];
then
  echo "Usage: $0 sample_led_animation.cpp"
  exit
fi
if [ ! -f $1 ];
then
  echo "Could not find the file..."
  exit
fi
filename=$(basename "$1")
extension="${filename##*.}"
filename="${filename%.*}"

#Compile
g++ -O2 -Wall -D__STDC_CONSTANT_MACROS `pkg-config --cflags opencv`   -c -o "$filename".o "$filename".cpp

#Link
g++ ardrone/ardrone.o ardrone/command.o ardrone/config.o ardrone/udp.o ardrone/tcp.o ardrone/navdata.o ardrone/version.o ardrone/video.o "$filename".o -o "$filename".a -O2 -Wall -D__STDC_CONSTANT_MACROS `pkg-config --cflags opencv` `pkg-config --libs opencv` -lm -lpthread -lavutil -lavformat -lavcodec -lswscale
