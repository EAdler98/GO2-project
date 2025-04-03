#!/bin/bash

# Step 1: Run RS_API.py (if needed)
echo "Running RS_API.py..."
python3 RS_API10.py

# Check if RS_API.py executed successfully
if [ $? -ne 0 ]; then
  echo "RS_API.py execution failed. Exiting..."
  exit 1
fi

# Step 2: Get coordinates from main.py
coordinates="$(python3 main.py | tail -n 1)"
echo "Coordinates: $coordinates"

# Step 3: SSH into the robot, select ROS Foxy (1), set environment variables, and run start.py
echo "Start SSH"

# Use the -t flag to force allocation of a pseudo-terminal, ensuring all output is shown
sshpass -p "123" ssh -tt -o PubkeyAuthentication=no unitree@192.168.123.18 << EOF
echo '1'
cd ~/Documents/project/from_lola/FINAL
python3 start.py "$coordinates"
EOF

# Check if the SSH command executed successfully
if [ $? -ne 0 ]; then
  echo "SSH command execution failed. Exiting..."
  exit 1
fi

echo "Process completed."
