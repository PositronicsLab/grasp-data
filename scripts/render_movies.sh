#!bin/bash

# Note this is the original film production script.  Paths have changed so it
# is invalid.  Also, constituent operations have been separated into individual
# and more useful scripts.  Please refer to batch_produce_movies.sh as a new
# reference point

root_dir="image_data"
for dynamics_dir in "$root_dir"/*; do
  dynamics="${dynamics_dir:${#root_dir}+1}"
#  echo "$dynamics"
  for trial_dir in "$dynamics_dir"/*; do
#    echo "$trial_dir"
    trial="${trial_dir:${#dynamics_dir}+1}"
#    echo "$trial"
    file_name="${dynamics}-${trial}"

    if [ "$dynamics" = "ode_tuned" ]; then
      log_name="raw_data/$dynamics/ode-${trial}.log"
    else
      log_name="raw_data/$dynamics/$file_name.log"
    fi
    
    if [ "$dynamics" = "ode" ]; then
      dynamics_txt="ODE (Default)"
    elif [ "$dynamics" = "bullet" ]; then
      dynamics_txt="Bullet"
    elif [ "$dynamics" = "dart" ]; then
      dynamics_txt="DART"
    elif [ "$dynamics" = "ode_tuned" ]; then 
      dynamics_txt="ODE (Tuned)"
    fi

    if [ "${trial:0:4}" = "high" ]; then
      mu_txt="High Friction (mu=100)"
    else
      mu_txt="'Infinite' Friction (mu=1e8)" 
    fi

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

    #echo "$file_name $dynamics_txt $experiment_txt $trial_txt $mu_txt"
    #continue

    data=`tail -n 1 $log_name`
    if [ $? = 1 ]; then
      continue
    fi
    log_data=(${data//,/ })
    sim_t=${log_data[0]}
    real_t=${log_data[1]}
   
    bad_real_time=`bc <<< "$real_t < 0"`
    #echo $bad_real_time 
    if [ "$bad_real_time" = "1" ]; then
      real_t=0
    fi

    frames=$(find $trial_dir/ -maxdepth 1 -type f -name '*.jpg' | wc -l)

    #fps=`bc -l <<<"scale=10;$frames/$real_t"`
    #fpsceil=`bc -l <<<"scale=10;$fps+0.4"`

    #fps=${fps%.*}
    #fpsceil=${fpsceil%.*}

    #if [ "$fps" -le "0" ]; then
    #  fps=1
    #elif [ "$fpsceil" -gt "$fps" ]; then
    #  fps=$fpsceil
    #fi

    if [ "$frames" = "0" ]; then
      continue
    fi

    video_t=`bc -l <<<"scale=10;$frames/25"`
    pts=`bc -l <<<"scale=10;$sim_t/$video_t"`

    tmp_title_img="title.png"
    tmp_film_name="temp.mp4"
    tmp_title_name="title.mp4"

    film_name="${file_name}.mp4"
    film="raw_videos/${film_name}"

    # generate title card
    cmd="convert -size 1600x1100 xc:Black \\( -gravity Center -fill white -weight Bold -font Times-New-Roman-Regular -pointsize 100 -annotate +0+0 \"Dynamics: $dynamics_txt\\nExperiment: $experiment_txt\" \\) \\( -gravity Center -fill white -weight Bold -font Times-New-Roman-Regular -pointsize 60 -annotate +0+200 \"Trial: $trial_txt\" \\) \\( -gravity Center -fill white -weight Bold -font Times-New-Roman-Regular -pointsize 60 -annotate +0+280 \"$mu_txt\" \\) \\( -gravity Center -fill white -weight Bold -font Times-New-Roman-Regular -pointsize 60 -annotate +0+360 \"Simulated Time: $sim_t, Real Time: $real_t\" \\) $tmp_title_img"

    eval $cmd
    #`cp $tmp_title_img raw_videos/$file_name.png`
    #continue

    # generate title movie
    cmd="/home/james/bin/ffmpeg -loop 1 -i $tmp_title_img -t 00:00:02 -c:v libx264 $tmp_title_name"
    eval $cmd

    # generate main movie
    cmd="ffmpeg -i $trial_dir/%04d.jpg -vf scale=1563:1075,pad=1600:1100,setpts=$pts*PTS -c:v libx264 $tmp_film_name"
    #echo $cmd
    eval $cmd

    # concatenate title movie to main movie
    cmd="/home/james/bin/ffmpeg -i $tmp_title_name -i $tmp_film_name -filter_complex "concat=n=2:v=1:a=0" -vn -y $film"
    eval $cmd

    rm $tmp_title_name
    rm $tmp_title_img
    rm $tmp_film_name
  done
done

#find $1 -maxdepth 1 -name "img*.png" -type f -print | xargs rm


# number all images img0001.png, etc...
#ffmpeg -r 100 -i $1/img%04d.png -f mp4 -vcodec mpeg4 $1/movie.mp4
