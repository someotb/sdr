#!/bin/bash
set -e  # остановка при любой ошибке

sudo apt-get update
sudo apt-get install -y \
  python3-pip python3-setuptools python3-numpy swig python3-matplotlib \
  cmake g++ libpython3-dev libxml2 libxml2-dev bison flex libcdk5-dev \
  libusb-1.0-0-dev libaio-dev pkg-config libavahi-common-dev libavahi-client-dev \
  libsdl2-dev libgl1-mesa-dev libglew-dev
sudo apt install -y libfftw3-dev
