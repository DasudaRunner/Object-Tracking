#!/bin/sh
sudo chmod 777 demo
cd ../imgs
sudo chmod 777 imagelist_creator calibration
./imagelist_creator imagelist.yaml *.jpg
./calibration -w=7 -h=7 imagelist.yaml
sudo mv out_camera_data.yml ../
sudo rm -rf imagelist.yaml
