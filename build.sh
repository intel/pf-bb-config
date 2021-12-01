#!/bin/bash

VERSION_STRING=`git describe --tags --long`
echo ${VERSION_STRING}
sed -i "s/#VERSION_STRING#/${VERSION_STRING}/g" config_app.c
make clean
make
