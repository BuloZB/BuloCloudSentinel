#!/bin/bash
# Script to generate self-signed certificates for Federated Learning

# Create certs directory if it doesn't exist
mkdir -p certs

# Change to certs directory
cd certs

# Generate CA key and certificate
openssl genrsa -out ca.key 2048
openssl req -new -x509 -days 365 -key ca.key -out ca.crt -subj "/CN=BuloCloudSentinel CA"

# Generate server key and certificate signing request
openssl genrsa -out server.key 2048
openssl req -new -key server.key -out server.csr -subj "/CN=fl_server"

# Sign server certificate with CA
openssl x509 -req -days 365 -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out server.crt

# Generate client key and certificate signing request
openssl genrsa -out client.key 2048
openssl req -new -key client.key -out client.csr -subj "/CN=fl_client"

# Sign client certificate with CA
openssl x509 -req -days 365 -in client.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out client.crt

# Set permissions
chmod 644 *.crt
chmod 600 *.key

# Clean up
rm *.csr
rm *.srl

echo "Certificates generated successfully in the certs directory."
