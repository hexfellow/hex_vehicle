#!/bin/bash
protoc -I=proto-public-api --python_out=robot_interface/ proto-public-api/*.proto