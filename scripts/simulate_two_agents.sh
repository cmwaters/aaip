#!/bin/bash

gnome-terminal -e scripts/start_massim_src.sh

sleep 5

gnome-terminal -e scripts/run_two_agents.sh