#!/bin/sh
./imagelist_creator imagelist.yaml *.jpg
./calibration -w=7 -h=7 imagelist.yaml
sudo rm -rf imagelist.yaml
