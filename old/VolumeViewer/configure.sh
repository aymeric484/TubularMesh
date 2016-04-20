#!/bin/sh
echo ============= Checking for cmake ============
if (cmake --version)
then
   echo "found CMake"
else
   echo "cmake not found, please install it (see http://www.cmake.org/)" 
   exit
fi
os=`uname`
for config in Release Debug
do
	echo
	echo ============= Creating makefiles for $config mode =============
	mkdir -p build/$os-$config
	(cd build/$os-$config; cmake ../../ -DCMAKE_BUILD_TYPE:STRING=$config -DCGoGN_BUILD_PATH:STRING=../../../cgogn/build/$os-$config)
done
echo
echo ============= CGoGN build configured =============
cat << EOF
to build:
 go to build/$os-Release or build/$os-Debug
 and make
EOF
