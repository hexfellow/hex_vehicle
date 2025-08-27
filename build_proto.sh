#!/bin/bash
protoc --experimental_allow_proto3_optional -I=proto-public-api --python_out=robot_interface/proto/ proto-public-api/*.proto