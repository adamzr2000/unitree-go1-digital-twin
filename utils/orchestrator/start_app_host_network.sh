#!/bin/bash

export FLASK_APP=app_host_network.py
export FLASK_ENV=development
flask run --host=0.0.0.0 --port=9999
