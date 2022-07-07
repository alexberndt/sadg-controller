# Oneshell means I can run multiple lines in a recipe in the same shell, so I don't have to
# chain commands together with semicolon
.ONESHELL:

# Need to specify bash in order for conda activate to work.
SHELL=/bin/bash

all: dev

dev:
	poetry install
	mkdir third_party/libMultiRobotPlanning/build
	cd third_party/libMultiRobotPlanning/build
	cmake ..
	make