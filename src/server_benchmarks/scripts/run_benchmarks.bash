#!/usr/bin/env bash
# Runs the Robot Server timing benchmarks and writes the raw CSVs to $1.
#   ros2 run server_benchmarks run_benchmarks.bash <output_dir> [seconds]
# Each case: server + client (+ N load subscribers), separate processes, as in
# the experiment. Server CPU time is read from /proc over the run.
# The i9-12900H mixes performance (CPUs 0-11) and efficiency cores (12-19):
# every process is pinned to its own performance core, or the scheduler moving
# the server between the two kinds dominates the measurement.
set -u
OUT=${1:?output directory}
SECONDS_PER_CASE=${2:-30}
REPETITIONS=${3:-3}
SERVER_CPU=4
CLIENT_CPU=6
LOAD_CPUS=(8 10 0 2 9 11 1)
mkdir -p "$OUT"
LIB=$(ros2 pkg prefix server_benchmarks)/lib/server_benchmarks

taskset -c $SERVER_CPU "$LIB/eval_benchmark" 20000 > "$OUT/eval.csv"

cpu_ticks() { awk '{print $14 + $15}' "/proc/$1/stat"; }

run_case() {
  local mode=$1 load=$2 name="${1}_load${2}_rep${3}"
  echo "== $name"
  taskset -c $SERVER_CPU "$LIB/bench_server_node" --ros-args -p output:="$OUT/${name}_server.csv" > "$OUT/${name}_server.log" 2>&1 &
  local server=$!
  local loads=()
  for ((i = 0; i < load; ++i)); do
    taskset -c ${LOAD_CPUS[$i]} "$LIB/load_subscriber_node" --ros-args -r __node:=load_$i > "$OUT/${name}_load$i.log" 2>&1 &
    loads+=($!)
  done
  sleep 3
  taskset -c $CLIENT_CPU "$LIB/bench_client_node" --ros-args -p mode:="$mode" -p output:="$OUT/${name}_client.csv" > "$OUT/${name}_client.log" 2>&1 &
  local client=$!
  sleep 2
  local cpu0; cpu0=$(cpu_ticks $server); local t0; t0=$(date +%s.%N)
  sleep "$SECONDS_PER_CASE"
  local cpu1; cpu1=$(cpu_ticks $server); local t1; t1=$(date +%s.%N)
  echo "server_cpu_percent,$(python3 -c "print(100 * ($cpu1 - $cpu0) / $(getconf CLK_TCK) / ($t1 - $t0))")" > "$OUT/${name}_cpu.csv"
  kill -INT $client; wait $client
  for pid in "${loads[@]}"; do kill -INT $pid; done
  for pid in "${loads[@]}"; do wait $pid; done
  kill -INT $server; wait $server
}

for ((rep = 0; rep < REPETITIONS; ++rep)); do
  for load in 0 1 3 7; do run_case topic $load $rep; done
  for load in 0 3; do run_case service $load $rep; done
done
