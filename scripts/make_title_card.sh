#!/bin/bash

### DESCRIPTION 
# Programmatically generates a single image composed of a title and subtitle

### REQUIREMENTS
# current distribution of imagemagick to be accessible in $PATH

### EXAMPLES
# - equivalent command line to above test case -
# bash make_title_card.sh 1600 1100 "Title Text" "Subtitle Text" title.png

# - more extensive example with multiline title and multiline subtitle -
#bash make_title_card.sh 1600 1100 "Title Text\non two lines max" "Subtitle Text\nFor a multiline subtitle use slash- n to force newlines\nCan also be done in title as shown above\nSubtitle should probably be no more than four lines" title.png

### CODE
if [ "$#" -ne 5 ]; then
  echo "Invalid number of parameters provided"
  echo "Usage: ~ bash make_title_card.sh <width> <height> \"<title text>\" \"<subtitle text>\" <image_name>"
  exit 1
fi

### PARAMETERS
width=$1         # width of the image to produce
height=$2        # height of the image to produce
title=$3         # title to print onto the image
subtitle=$4      # subtitle to print onto the image
img_name=$5      # image file to produce as output

cmd="convert -size ${width}x${height} xc:Black \\( -gravity Center -fill white -weight Bold -font Times-New-Roman-Regular -pointsize 100 -annotate +0+0 \"$title\" \\) \\( -gravity Center -fill white -weight Bold -font Times-New-Roman-Regular -pointsize 60 -annotate +0+240 \"$subtitle\" \\) $img_name"

eval $cmd


# - original command -
#cmd="convert -size 1600x1100 xc:Black \\( -gravity Center -fill white -weight Bold -font Times-New-Roman-Regular -pointsize 100 -annotate +0+0 \"Dynamics: $dynamics_txt\\nExperiment: $experiment_txt\" \\) \\( -gravity Center -fill white -weight Bold -font Times-New-Roman-Regular -pointsize 60 -annotate +0+200 \"Trial: $trial_txt\" \\) \\( -gravity Center -fill white -weight Bold -font Times-New-Roman-Regular -pointsize 60 -annotate +0+280 \"$mu_txt\" \\) \\( -gravity Center -fill white -weight Bold -font Times-New-Roman-Regular -pointsize 60 -annotate +0+360 \"Simulated Time: $sim_t, Real Time: $real_t\" \\) $tmp_title_img"



