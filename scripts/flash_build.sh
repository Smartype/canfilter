fileDir=$(dirname $0)
fileName=$(basename $0)
[ $fileDir == . ] && fileDir=$(pwd)

fwFile="$fileDir/../board/build/CanFilter.bin.signed"
[ -f "$fwFile" ] || {
  echo "$fileName: $fileDir/../board/build/CanFilter.bin.signed not exist"
  exit 2
}

echo "stop comma service"
op-stop

echo "start the car to power radar ..."
sleep 5

echo "flash"
$fileDir/enter_canloader.py $fwFile
