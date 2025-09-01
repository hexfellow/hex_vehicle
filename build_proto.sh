#!/bin/bash
protoc -I=proto-public-api --python_out=hex_vehicle/ proto-public-api/*.proto