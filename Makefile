# Oneshell means I can run multiple lines in a recipe in the same shell, so I don't have to
# chain commands together with semicolon
.ONESHELL:

# Need to specify bash in order for conda activate to work.
SHELL=/bin/bash

all: setup

setup:
	poetry install
	poetry build
	poetry run poetry-lock-package --build
	mkdir third_party/libmultirobotplanning/build
	cd third_party/libmultirobotplanning/build
	cmake ..
	make

dev: setup
	poetry run pre-commit install

docker:
	docker build -t sadg-controller:dev .
