#!/bin/bash
for i in `seq 1 40`;
do
  command=( "python" "generate_plot.py" "$i" )
  "${command[@]}"
done
