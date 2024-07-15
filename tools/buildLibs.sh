 #!/usr/bin/sh
baseDir=`pwd`
echo BUILDING SHAPELIB...
cd ../deps/shapelib/
mkdir build 2> /dev/null
cd build
cmake ../ -Wno-dev
sudo make install
cd "$baseDir"
echo DONE!
