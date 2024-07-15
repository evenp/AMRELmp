#/bin/bash
baseDir=`pwd`
make config="release"

if [[ $? != 0 ]]; then
	echo "compile failed"
	return
fi

echo "==== Starting AMREL ($@) ===="
cd ./resources/
../binaries/AMREL/Release/AMREL $@
cd $baseDir
