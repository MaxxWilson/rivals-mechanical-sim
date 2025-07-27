#!/bin/bash
#
# Installs Python dependencies for the dyno_eval project.
# It's highly recommended to run this inside a Python virtual environment.
#
# To create a venv: python3 -m venv venv
# To activate it:  source venv/bin/activate
#

echo ">>> Installing dependencies from requirements.txt..."
python3 -m pip install -r requirements.txt
echo ">>> Dependencies installed successfully."