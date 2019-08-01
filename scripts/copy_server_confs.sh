#!/bin/bash

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$SCRIPT_DIRECTORY/../third-party/massim/server/conf"

rm *Config.json

cp -av "$SCRIPT_DIRECTORY/../server_configs/." "$SCRIPT_DIRECTORY/../third-party/massim/server/conf"




