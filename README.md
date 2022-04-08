# Installation
Install opencv
```
git clone git@github.com:opencv/opencv.git
cmake
sudo make install
```
If runtime linking error, find libopencv\*so\*. Maybe in /usr/local/lib/

Then create file `/etc/ld.so.conf.d/opencv.conf`
And write in it:
```
/usr/local/lib/
```
Then run:
```
sudo ldconfig -v
```
