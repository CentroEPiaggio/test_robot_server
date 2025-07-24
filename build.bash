#!/bin/bash

export MYUID=$(id -u)
export MYGID=$(id -g)

docker compose -f .devcontainer/docker-compose.yml build