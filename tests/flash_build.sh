fileDir=$(dirname $0)
fileName=$(basename $0)
[ $fileDir == . ] && fileDir=$(pwd)

[ "$fileDir" != "/data/canfilter/tests" ] && {
  echo  "$fileName: please make sure this script is in /data/canfilter/tests"
  exit 1
}

fwFile="$fileDir/../board/build/CanFilter.bin.signed"
[ -f "$fwFile" ] || {
  echo "$fileName: $fileDir/../board/build/CanFilter.bin.signed not exist"
  exit 2
}

echo "stop comma service"
if [ -f /EON ]; then
  setprop ctl.stop comma
else
  sudo systemctl stop comma
fi

echo "start the car to power radar ..."
sleep 5

echo "flash"
$fileDir/enter_canloader.py $fwFile
