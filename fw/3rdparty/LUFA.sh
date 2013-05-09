#!/bin/sh

VERSION=130303

rm -rf LUFA LUFA-${VERSION} LUFA-${VERSION}.zip
wget http://lufa-lib.googlecode.com/files/LUFA-${VERSION}.zip
unzip LUFA-${VERSION}.zip
ln -s LUFA-${VERSION} LUFA
