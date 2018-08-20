#!/bin/sh
cd ../imgs
./imagelist_creator imagelist.yaml *.jpg
./calibration -w=7 -h=7 imagelist.yaml
sudo mv out_camera_data.yml ../
sudo rm -rf imagelist.yaml
