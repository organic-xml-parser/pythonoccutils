#!/usr/bin/env bash

docker container rm pythonoccutils-container

vals=($(xauth list $DISPLAY | head -n 1))

XAUTH_ADD_ARG="${vals[1]} ${vals[2]}" docker compose run --name pythonoccutils-container app

