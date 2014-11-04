#!/bin/bash

# NOTE : paths have changed.  This is an example only on how to use the 
# supplementary scripts.  Files were removed before this script was derived
# so it is partially untested

frame_width=1563
frame_height=1075
film_width=1600
film_height=1100
frame_extension="jpg"
frame_prefix="\"\""
frame_indices=4


root_dir="image_data"
# iterate through the root directory
for dynamics_dir in "$root_dir"/*; do
  # extract the dynamics subdirectory name from the full path
  dynamics="${dynamics_dir:${#root_dir}+1}"

  # iterate over all the trial subdirectories within the dynamics directory
  for trial_dir in "$dynamics_dir"/*; do

    # extract the trial name from the the full path
    trial="${trial_dir:${#dynamics_dir}+1}"

    # build a file name for the film
    file_name="${dynamics}-${trial}"

    # determine the appropriate log file name
    # ode (tuned) had a special path which needed correction to match logs
    if [ "$dynamics" = "ode_tuned" ]; then
      log_name="raw_data/$dynamics/ode-${trial}.log"
    else
      log_name="raw_data/$dynamics/$file_name.log"
    fi
    
    # set parameters used in the title text from the directory
    if [ "$dynamics" = "ode" ]; then
      dynamics_txt="ODE (Default)"
    elif [ "$dynamics" = "bullet" ]; then
      dynamics_txt="Bullet"
    elif [ "$dynamics" = "dart" ]; then
      dynamics_txt="DART"
    elif [ "$dynamics" = "ode_tuned" ]; then 
      dynamics_txt="ODE (Tuned)"
    fi

    # set parameters used in the subtitle text from the trial
    if [ "${trial:0:4}" = "high" ]; then
      mu_txt="High Friction (mu=100)"
    else
      mu_txt="'Infinite' Friction (mu=1e8)" 
    fi

    # set parameters used in the title text from the trial
    if [ "${trial:${#trial}-6}" = "-block" ]; then
      experiment_txt="Multiple Block Grasp" 
      num_blocks="${trial:${#trial}-8:2}"
      if [ "$num_blocks" = "01" ]; then
        trial_txt="1 Block"
      elif [ "$num_blocks" = "02" ]; then
        trial_txt="2 Block"
      elif [ "$num_blocks" = "03" ]; then
        trial_txt="3 Block"
      elif [ "$num_blocks" = "04" ]; then
        trial_txt="4 Block"
      elif [ "$num_blocks" = "05" ]; then
        trial_txt="5 Block"
      elif [ "$num_blocks" = "06" ]; then
        trial_txt="6 Block"
      elif [ "$num_blocks" = "07" ]; then
        trial_txt="7 Block"
      elif [ "$num_blocks" = "08" ]; then
        trial_txt="8 Block"
      elif [ "$num_blocks" = "09" ]; then
        trial_txt="9 Block"
      elif [ "$num_blocks" = "10" ]; then
        trial_txt="10 Block"
      elif [ "$num_blocks" = "11" ]; then
        trial_txt="11 Block"
      fi
    else
      experiment_txt="Industrial Arm Grasp"
      if [ "${trial:${#trial}-12}" = "primitiveall" ]; then
        trial_txt="Block (Primitive) & Hand (Primitive) Geometry"
      elif [ "${trial:${#trial}-14}" = "polygonalblock" ]; then
        trial_txt="Block (Tessellated) & Hand (Primitive) Geometry"
      elif [ "${trial:${#trial}-13}" = "polygonalhand" ]; then
        trial_txt="Block (Primitive & Hand (Tessellated) Geometry"
      elif [ "${trial:${#trial}-12}" = "polygonalall" ]; then
        trial_txt="Block (Tessellated) & Hand (Tessellated) Geometry"
      elif [ "${trial:${#trial}-14}" = "inertialmod_1x" ]; then
        trial_txt="Inertia Modification - Unit/Identity Inertia"
      elif [ "${trial:${#trial}-14}" = "inertialmod_2x" ]; then
        trial_txt="Inertia Modification - 1/2^n Inertia"
      elif [ "${trial:${#trial}-14}" = "inertialmod_3x" ]; then
        trial_txt="Inertia Modification - 1/3^n Inertia"
      elif [ "${trial:${#trial}-14}" = "inertialmod_4x" ]; then
        trial_txt="Inertia Modification - 1/4^n Inertia"
      elif [ "${trial:${#trial}-15}" = "inertialmod_10x" ]; then
        trial_txt="Inertia Modification - 1/10^n Inertia"
      fi
    fi


    # read the tail of the log file
    data=`tail -n 1 $log_name`
    if [ $? = 1 ]; then
      # if the read failed, then abort production of this movie
      continue
    fi

    # parse the log data by comma delimiter
    log_data=(${data//,/ })
    # extract the simulated time from the first log field
    sim_t=${log_data[0]}
    # extract the real time from the first log field
    real_t=${log_data[1]}
   
    # check for a real time less than zero
    is_bad_real_time=`bc <<< "$real_t < 0"`
    if [ "$is_bad_real_time" = "1" ]; then
      # arbitrarily correct the real time to zero if it was negative
      real_t=0
    fi

    # count the number of frames (jpg) in the directory
    frames=$(find $trial_dir/ -maxdepth 1 -type f -name "*.$frame_extension" | wc -l)

    # if the number of frames is zero then abort production of this movie
    if [ "$frames" = "0" ]; then
      continue
    fi

    # build parameters
    title="Dynamics: $dynamics_txt\\nExperiment: $experiment_txt"
    subtitle="Trial: $trial_txt\n$mu_txt\nSimulated Time: $sim_t, Real Time: $real_t"
    film="${file_name}.mp4"
    path="$trial_dir"

    # execute the produce movie script
    cmd="bash produce_movie.sh $path $frame_prefix $frame_indices $frame_extension $frame_width $frame_height $film_width $film_height $frames $sim_t \"$title\" \"$subtitle\" $film"
    eval $cmd

  done
done

