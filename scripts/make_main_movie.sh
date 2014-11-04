#!/bin/bash

### DESCRIPTION
# generates a movie from a set of images.  Chief capability is normalizing a 
# variable frame movie to the expected length, i.e. a simulation did not run in
# real time and produced too many frames but the produced movie length should 
# be equal to the length of the simulation and not based on any framerate 
# related to the encoding process.  In order to normalize, the script needs
# to have input the number of frames and the simulation (desired) time.

### REQUIREMENTS
# current distribution of ffmpeg (from source not apt) to be accessible in $PATH
# the number of digits indicating the frame number needs to be consistent across
# all frames, e.g. 4-digits implies zero padding up to four digits (typical), at# least one frame of 5-digits implies that all frames need zero padding up to 
# five digits.  Frame numbers must be contiguous (no gaps in numbering) or the
# movie will be truncated at the gap and the computed length will be wrong.

### EXAMPLES
# assuming that ../dispose/image_data/ode/highfriction-01-block contains 1144 
# jpg images named from 0000.jpg to 1143.jpg.  There is no prefix hence the 
# empty string in the second parameter, there is a 4 for the third parameter 
# because the frames have four numerics in title, jpg from the file names, the 
# frames are 1563x1075 but should be padded to 1600x1100 (note not scaled), and 
# the desired length of the movie is 100 seconds.

# bash make_main_movie.sh ../dispose/image_data/ode/highfriction-01-block "" 4 jpg 1563 1075 1600 1100 1144 100 ode.mp4

### CODE
if [ "$#" -ne 11 ]; then
  echo "Invalid number of parameters provided"
  echo "Usage: ~ bash make_main_movie.sh <path to frames> <prefix on frames> <number of digits in frame names> <frame name extension> <frame image width> <frame image height> <film width> <film height> <number of frames> <desired film length (s)> <output film name>"
  exit 1
fi

### PARAMETERS
path=$1             # path to the frames that compose the movie
prefix=$2           # prefix on the frames
digits=$3           # number of digits in the frame names
extension=$4        # file extension in the frame names
img_width=$5        # width of a frame image
img_height=$6       # height of a frame image
film_width=$7       # width of the film to output
film_height=$8      # height of the film to output
frames=$9           # number of frames in the set
desired_time=${10}  # desired length of the film in seconds
film=${11}          # name of the film to output

video_t=`bc -l <<<"scale=10;$frames/25"`
pts=`bc -l <<<"scale=10;$desired_time/$video_t"`

cmd="ffmpeg -i $path/$prefix%0${digits}d.$extension -vf scale=$img_width:$img_height,pad=$film_width:$film_height,setpts=$pts*PTS -c:v libx264 $film"
eval $cmd

# - original command -
#cmd="ffmpeg -i $trial_dir/%04d.jpg -vf scale=1563:1075,pad=1600:1100,setpts=$pts*PTS -c:v libx264 $tmp_film_name"


