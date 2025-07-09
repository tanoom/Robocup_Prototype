#!/bin/bash

# Run build.sh
./scripts/build.sh && echo "success build"

# Run start.sh once
./scripts/start.sh && echo "success start"

# Run build.sh again
./scripts/build.sh && echo "success build"

# Run start.sh again
./scripts/start.sh && echo "success start"
