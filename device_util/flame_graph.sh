#!/bin/bash
# Ref:
#   1. https://askubuntu.com/a/753796
#   2. https://www.brendangregg.com/FlameGraphs/cpuflamegraphs.html
#   3. https://github.com/brendangregg/FlameGraph
# Note:
#   1. Scripts like ./stackcollapse-perf.pl and ./flamegraph.pl can be downloaded raw from github
#   2. In some cases executing perf requires root privilege

set -x

monitoring_PID=1937542
monitoring_TIME=600 # in seconds
chmod +x *.pl

sudo perf record -F 99 -a -g -p $monitoring_PID -- sleep $monitoring_TIME
sudo perf script | ./stackcollapse-perf.pl > out.perf-folded
sudo ./flamegraph.pl out.perf-folded > perf.svg
