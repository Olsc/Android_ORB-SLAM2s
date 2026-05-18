# Ubuntu-20.04-GUI

It is not yet fully refined and requires extensive testing!!!!


```bash
sudo apt-get update
sudo apt-get install -y build-essential

sudo apt-get install -y libopencv-dev

mkdir build
cd build

cmake ..

make -j$(nproc)
```