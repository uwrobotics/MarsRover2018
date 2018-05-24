# Panorama Code

## Install
With a ROS install opencv3 should already by bundled with `ros-kinetic-opencv3`. In case it is not run
```
sudo apt-get install python-opencv
```
## Usage
```
python pano.py txtlists/<your_image_list.txt>
```

The txtlists is structured so that each image to combine is listed per line, from left-most image to right-most image

```
test-images/S1.jpg
test-images/S2.jpg
test-images/S3.jpg
test-images/S5.jpg
test-images/S6.jpg
```

There are currently images in the `test-images` folder that can be used to test out the panorama code

Cloned from https://github.com/kushalvyas/Python-Multiple-Image-Stitching
