#!/bin/sh

sudo ifconfig can0 down
sudo pkill ux2cand
sudo rmmod ux2can
sudo rmmod can-raw
sudo rmmod can-dev
sudo rmmod can
sudo insmod net/can/can.ko
sudo insmod drivers/net/can/can-dev.ko
sudo insmod net/can/can-raw.ko
sudo insmod drivers/net/can/ux2can.ko
sudo ../../can-utils/ux2cand 1 1000
sudo ifconfig can0 up

