#!bin/bash

# Test the server with TCP requests
ipv4="127.0.0.1"
port="8080"

# Generate test file
echo "abcdefghijklmnopqrstuvwxyz" > "test.txt"

# Test TCP
cat < "test.txt" > /dev/tcp/$ipv4/$port
echo "Sent test file to $ipv4:$port"

# Delete test file
rm "test.txt"
