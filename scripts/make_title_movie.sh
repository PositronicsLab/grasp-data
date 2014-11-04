#!/bin/bash

### DESCRIPTION
# makes a movie from a single static image lasting for two seconds.  Intended
# to create a title card animation to be concatenated onto a longer movie

### REQUIREMENTS
# current distribution of ffmpeg (from source not apt) to be accessible in $PATH

### EXAMPLES
# assuming that title.png exists in the current directory
# bash make_title_movie.sh title.png title.mp4

### CODE
if [ "$#" -ne 2 ]; then
  echo "Invalid number of parameters provided"
  echo "Usage: ~ bash make_title_movie.sh <image> <film_name>"
  exit 1
fi

### PARAMETERS
image=$1         # path to image file to encode
film=$2          # movie file to produce as output

cmd="ffmpeg -loop 1 -i $image -t 00:00:02 -c:v libx264 $film"
eval $cmd

# - original command -
#cmd="/home/james/bin/ffmpeg -loop 1 -i $tmp_title_img -t 00:00:02 -c:v libx264 $tmp_title_name"

