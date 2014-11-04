import string
import os
from collections import OrderedDict
#from os import walk

# python script that renames frames in the directory tree and
# removes many unnecessary/redundant portions of the file names

def process_dir( path ):
  all_dirs = []
  for( root, dirs, files ) in os.walk( path ):
    for name in dirs:
      all_dirs.append( name )

  all_dirs = list(OrderedDict.fromkeys( all_dirs ) )
  all_dirs.sort()

  #print all_dirs

  for d in all_dirs:
    file_map = []
    subpath = path + d

    files = [f for f in os.listdir(subpath) if os.path.isfile(os.path.join(subpath,f))]
    strip = d + '_gzclient_camera(1)-'

    for f in files:
      if f != 'time.log':
        fnew = string.replace( f, strip, '' )
        fnew = string.replace( fnew, ".jpg", '' )
        file_map.append( (f,fnew) )

    file_map = list(OrderedDict.fromkeys(file_map))
    file_map.sort()
  #  print file_map
    i = 0
    for f in file_map:
      k = int( f[1] )
      if i != k:
        print 'bad sequence '
      i = i + 1

    for f in file_map:
      old_file = os.path.join( subpath, f[0] )
      new_file = os.path.join( subpath, f[1] + '.jpg' )
      #print old_file + ' ' + new_file
      os.rename( old_file, new_file )

root_dir = 'image_data/'

bullet_dir = 'bullet'
dart_dir = 'dart'
ode_dir = 'ode'
tuned_ode_dir = 'ode_tuned'

path = root_dir + ode_dir + '/'
process_dir( path )

path = root_dir + tuned_ode_dir + '/'
process_dir( path )

path = root_dir + dart_dir + '/'
process_dir( path )

path = root_dir + bullet_dir + '/'
process_dir( path )

