#!/bin/bash

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$SCRIPT_DIRECTORY/../third-party/massim/server"

#IT IS NECESSARY TO RUN ONCE BEFORE "mvn install" IN THE ROOT PROJECT OF MASSIM

mvn exec:java -Dexec.args="--monitor"

#Monitor available at http://localhost:8000/
