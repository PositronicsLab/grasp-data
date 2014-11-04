from os import listdir
from os.path import isfile, join
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import re
import argparse

###########

###########
def process_file( path ):
  try:
    f = open( path )
  except Exception:  
    return []

  #with open( path ) as f:
  content = [x.strip('\n') for x in f.readlines() ]
  f.close()

  raw_data = []
  # raw_data = simt, realt, [idx, KE_1, KE_2, avgKE, (px py pz)]
  #             0     1       2    3     4     5         6

  for l in content:
    raw_data.append( re.split( ',', l ) )

  targets = len( raw_data[0] ) / 5

  data = []
  for i in raw_data:
    ke = 0
    j = 0
    while j < targets:
      idx = (j + 1) * 5
      ke = ke + float( i[idx] )
      j = j + 1
    data.append( [float( i[0]), float(i[1]), float(ke)] )
  return data

###########
def label_figure( plt, title, ode_handle, dart_handle, bullet_handle, tuned_ode_handle ):
  plt.xlabel( "Simulation Time", fontsize=24 )
  plt.ylabel( "Kinetic Energy", fontsize=24 )
  plt.title( title, fontsize=24 )
  #plt.legend( handles=[ode_handle, dart_handle, bullet_handle, tuned_ode_handle], labels=['ode','dart','bullet','ode tuned'] )
  plt.legend( [ode_handle, dart_handle, bullet_handle, tuned_ode_handle], ['ode (default)','dart','bullet','ode(tuned)'] )

###########
def plot_data( plt, data, name ):
  x = []
  y = []
  for i in data:
    x.append( i[0] )
    y.append( i[2] )

  #for l in data:
  #  print l

  #handle, = plt.plot( x, y, label=name, linewidth=2.0 )
  handle, = plt.semilogy( x,y, label=name, linewidth=2.5 )

  axes = plt.axis( [0,100,0.00000001,100000000] )

  return handle
###########

parser = argparse.ArgumentParser( description='File id (0-39)' )
parser.add_argument( 'file_id', metavar='N', type=int, help='an integer in the range 0-39' )
args = parser.parse_args();

file_index = args.file_id

#matplotlib.rcParams.update({'font.size':22})

titles=[["highfriction-inertialmod_10x", "Inertial 1/10 Modification (High Friction)"],["highfriction-inertialmod_1x", "Inertial 1/1 Modification (High Friction)"],["highfriction-inertialmod_2x", "Inertial 1/2 Modification (High Friction)"],["highfriction-inertialmod_3x", "Inertial 1/3 Modification (High Friction)"],["highfriction-inertialmod_4x", "Inertial 1/4 Modification (High Friction)"],["highfriction-polygonalall", "Polygonal Block/Polygonal Hand Collision Geometry (High Friction)"],["highfriction-polygonalblock", "Polygonal Block/Primitive Hand Collision Geometry (High Friction)"],["highfriction-polygonalhand", "Primitive Block/Polygonal Hand Collision Geometry (High Friction)"],["highfriction-primitiveall", "Primitive Block/Primitive Hand Collision Geometry (High Friction)"],["highfriction-01-block", "1 Block Grasp (High Friction)"],["highfriction-02-block", "2 Block Grasp (High Friction)"],["highfriction-03-block", "3 Block Grasp (High Friction)"],["highfriction-04-block", "4 Block Grasp (High Friction)"],["highfriction-05-block", "5 Block Grasp (High Friction)"],["highfriction-06-block", "6 Block Grasp (High Friction)"],["highfriction-07-block", "7 Block Grasp (High Friction)"],["highfriction-08-block", "8 Block Grasp (High Friction)"],["highfriction-09-block", "9 Block Grasp (High Friction)"],["highfriction-10-block", "10 Block Grasp (High Friction)"],["highfriction-11-block", "11 Block Grasp (High Friction)"],["veryhighfriction-inertialmod_10x", "Inertial 1/10 Modification ('Infinite' Friction)"],["veryhighfriction-inertialmod_1x", "Inertial 1/1 Modification ('Infinite' Friction)"],["veryhighfriction-inertialmod_2x", "Inertial 1/2 Modification ('Infinite' Friction)"],["veryhighfriction-inertialmod_3x", "Inertial 1/3 Modification ('Infinite' Friction)"],["veryhighfriction-inertialmod_4x", "Inertial 1/4 Modification ('Infinite' Friction)"],["veryhighfriction-polygonalall", "Polygonal Block/Polygonal Hand Collision Geometry ('Infinite' Friction)"],["veryhighfriction-polygonalblock", "Polygonal Block/Primitive Hand Collision Geometry ('Infinite' Friction)"],["veryhighfriction-polygonalhand", "Primitive Block/Polygonal Hand Collision Geometry ('Infinite' Friction)"],["veryhighfriction-primitiveall", "Primitive Block/Primitive Hand Collision Geometry ('Infinite' Friction)"],["veryhighfriction-01-block", "1 Block Grasp ('Infinite' Friction)"],["veryhighfriction-02-block", "2 Block Grasp ('Infinite' Friction)"],["veryhighfriction-03-block", "3 Block Grasp ('Infinite' Friction)"],["veryhighfriction-04-block", "4 Block Grasp ('Infinite' Friction)"],["veryhighfriction-05-block", "5 Block Grasp ('Infinite' Friction)"],["veryhighfriction-06-block", "6 Block Grasp ('Infinite' Friction)"],["veryhighfriction-07-block", "7 Block Grasp ('Infinite' Friction)"],["veryhighfriction-08-block", "8 Block Grasp ('Infinite' Friction)"],["veryhighfriction-09-block", "9 Block Grasp ('Infinite' Friction)"],["veryhighfriction-10-block", "10 Block Grasp ('Infinite' Friction)"],["veryhighfriction-11-block", "11 Block Grasp ('Infinite' Friction)"]]

desired_title = titles[file_index]
desired_log = desired_title[0] + '.log'
title = desired_title[1]
image_name = desired_title[0] + '.png'

#print args

result_path = '../results/data/'

ode_dir = 'ode/'
tuned_ode_dir = 'tuned_ode/'
dart_dir = 'dart/'
bullet_dir = 'bullet/'

ode_path = result_path + ode_dir
dart_path = result_path + dart_dir
bullet_path = result_path + bullet_dir
tuned_ode_path = result_path + tuned_ode_dir

ode_file = ode_path + desired_log
tuned_ode_file = tuned_ode_path + desired_log
dart_file = dart_path + desired_log
bullet_file = bullet_path + desired_log

ode_data = process_file( ode_file )
tuned_ode_data = process_file( tuned_ode_file )
dart_data = process_file( dart_file )
bullet_data = process_file( bullet_file )

fig = plt.figure(1,(16,9))

#mng = plt.get_current_fig_manager()
#mng.full_screen_toggle()
#mng.resize(*mng.window.maxsize())

ode_handle = plot_data( plt, ode_data, 'ode' )
dart_handle = plot_data( plt, dart_data, 'dart' )
bullet_handle = plot_data( plt, bullet_data, 'bullet' )
tuned_ode_handle = plot_data( plt, tuned_ode_data, 'ode(tuned)' )

label_figure( plt, title, ode_handle, dart_handle, bullet_handle, tuned_ode_handle )

#plt.show()

plt.savefig( image_name )
