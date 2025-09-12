# csc376

# Installation

Might need linux dependencies, if a fresh install:
```bash
sudo apt -y install gcc g++ unzip python3-pybind11
```

Installing python robotics toolbox:
```bash 
conda create -n csc376 python==3.12
conda activate csc376
pip install roboticstoolbox-python ruckig
pip install numpy==1.26.4 matplotlib==3.8.4 websockets==13.0.1
```

Install our own version of swift:
```bash
git clone https://github.com/ContinuumRoboticsLab/swift-csc376
cd swift-csc376
pip install -e .
```

Install libfranka 0.9.2 (teaching lab), 
Taken from, https://github.com/frankarobotics/libfranka, partially modified because 0.9.2 build instructions was from libfranka website that is down:

```bash
sudo apt-get update && sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev

git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git
cd libfranka
git checkout 0.9.2 && git submodule update
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && make
cpack -G DEB &&  sudo dpkg -i libfranka*.deb
cd ../.. # Get out of libfranka lib
```

Install csc376 franky:
```bash
conda activate csc376
pip install pybind11
cd csc376_franky
pip install .
```

# Run

Prerequisites:
1. Make sure you are running a linux real-time kernel: `uname -r` should show "rt" in the output
2. Robot will move forward 10cm, please put in a possible position
3. Put the Franka robot to FCI mode in Franka Desk

```bash
conda activate csc376
cd csc376_franky
python3 test/visualize_then_execute.py  
```

# Realtime Kernel Settings
because libfranka website is down

```bash
sudo nano /etc/security/limits.conf

```

Add these:
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400

```bash
sudo groupadd -r realtime
sudo usermod -aG realtime $USER
```

Logout and login again
