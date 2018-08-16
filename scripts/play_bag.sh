bagfile=$1
cwd=$(dirname "$0")

if [ "$bagfile" == '' ]; then
  # use the latest
  bagfile=$($cwd/../ros1_ws/src/util/src/util/latest_file.py $cwd/../data/'*.bag')
else
  # absolute
  bagfile=$($cwd/../ros1_ws/src/util/src/util/abs_path.py $bagfile)
fi

rosbag play -l -d 2 $bagfile
