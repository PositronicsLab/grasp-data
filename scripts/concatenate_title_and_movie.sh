#!/bin/bash

### DESCRIPTION
# concatenates one movie with another.  Intended to concatenate a title movie 
# with the main movie

### REQUIREMENTS
# current distribution of ffmpeg (from source not apt) to be accessible in $PATH
# movies need to have same resolution and codec.  Video only is handled

### EXAMPLES
# assuming that title.mp4 and movie.mp4 exist in the current directory
# bash concatenate_title_and_movie.sh title.mp4 movie.mp4 output.mp4

### CODE
if [ "$#" -ne 3 ]; then
  echo "Invalid number of parameters provided"
  echo "Usage: ~ bash make_title_movie.sh <title_movie> <main_movie> <output_movie>"
  exit 1
fi

### PARAMETERS
title=$1         # title movie path
film=$2          # main movie path
output=$3        # movie file to produce as output

cmd="ffmpeg -i $title -i $film -filter_complex "concat=n=2:v=1:a=0" -vn -y $output"
eval $cmd

# - original command -
#cmd="/home/james/bin/ffmpeg -i $tmp_title_name -i $tmp_film_name -filter_complex "concat=n=2:v=1:a=0" -vn -y $film"


