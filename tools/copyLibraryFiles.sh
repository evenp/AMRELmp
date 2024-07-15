baseDir=`pwd`
mkdir ../src/Libs/ 2> /dev/null

echo -------STBI-------
rmdir ../src/Libs/stbi/ /s /q 2> /dev/null
mkdir ../src/Libs/stbi/ 2> /dev/null
cp ../deps/stb/stb_image.h ../src/Libs/stbi
cp ../deps/stb/stb_image_write.h ../src/Libs/stbi
cd "$baseDir"
