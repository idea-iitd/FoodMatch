#!/bin/bash
echo "Running Instance $1"
./main.o public_instances/$1 $2 > results_$2/$1/sim.results
python3 evaluation_scripts/mdrp_format.py $1 $2
python3 evaluation_scripts/compute_performance_summary.py instance_dir=public_instances/$1 input_dir=results_$2/$1 output_dir=results_$2/$1
