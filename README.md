# csc376

# Installation

Might need linux dependencies, if a fresh install:
```bash
sudo apt install gcc g++ unzip 
```

Installing python robotics toolbox:
```bash 
conda create -n csc376 python==3.10
conda activate csc376
pip install roboticstoolbox-python ruckig
pip install websockets==10.4 # Fix asyncio run error
pip install numpy==1.25.0 # Fix numpy > 2 error
```

Installing franky:
```bash
conda activate csc376
pip uninstall franky-control # (added by us) remove previously installed
VERSION=0-9-2 # (added by us) Teachinglab frankas use 0.9.2
wget https://github.com/TimSchneider42/franky/releases/latest/download/libfranka_${VERSION}_wheels.zip
unzip libfranka_${VERSION}_wheels.zip
# pip install numpy # (added by us) Should already be done from above
pip install --no-index --find-links=./dist franky-control
```

If outdated, the above was taken and modified from: https://github.com/TimSchneider42/franky?tab=readme-ov-file#installing-franky. 

# Run

