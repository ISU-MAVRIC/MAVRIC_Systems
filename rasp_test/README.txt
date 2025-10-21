# Set up CAN
```
sudo ip link set can0 type can bitrate 1000000 && \
sudo ip link set up can0
```

# Install Python Dependencies
```
python3 -m venv .venv && \
source .venv/bin/activate && \
pip3 install -r requirements.txt
```