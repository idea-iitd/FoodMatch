#!/bin/bash
echo "Running All Instance for Algorithm $1"
for f in $(ls public_instances/);
do 
    echo "Running Instance $f"
    ./main.o public_instances/$f $1 > results_$1/$f/sim.results
    python3 evaluation_scripts/mdrp_format.py $f $1 0
    python3 evaluation_scripts/compute_performance_summary.py instance_dir=public_instances/$f input_dir=results_$1/$f output_dir=results_$1/$f 0
done

python3 evaluation_scripts/aggregate_results.py $1
