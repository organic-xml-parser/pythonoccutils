#!/usr/bin/env bash

cp ../requirements.txt ./requirements.txt

docker build . -t pythonoccutils-img:latest
