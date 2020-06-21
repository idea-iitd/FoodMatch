## Quick Start

1. Compile the project:
```bash
    make main.o
```
2. [public_instances](public_instances) contains 240 different instances described in detail in the [data repository](https://github.com/grubhub/mdrplib/blob/master/MDRPInstances.pdf). Run the code on one instance and report metrics:
    ```bash
        bash evaluation_scripts/run_instance.sh [instance_name] [algorithm={FM|MDRP}] 
    ```
    - The parameters are explained below:
        -   **instance_name**: Name of dataset directory in [public_instances](public_instances) to run code on
        -   **algorithm**: Algorithm to use for assignment
        ```
        'FM' = FoodMatch algorithm
        'MDRP' = Default algorithm proposed by Reyes et. al. [1,2]
        ```
    - Sample command:
    ```bash
        bash evaluation_scripts/run_instance.sh 0o100t100s1p100 FM
    ```
3. Run the code on all instances and report mean metrics:
    ```bash
        bash evaluation_scripts/run_all.sh [algorithm={FM|MDRP}]
    ```
    - Sample command:
    ```bash
        bash evaluation_scripts/run_all.sh FM 
    ```        

## Code Description
- `main.cpp` - contains the code that drives the simulation.<br>
    It produces a log of simulation on stdout. This is piped to a file for evaluation.
- `include/` - contains code for the simulation framework and algorithms.
- `include/constants.cpp` - contains the default parameters used.
- `include/vehicle_assignment.cpp` - contains the code for FoodMatch algorithm.
- `include/mdrp_assignment.cpp` - contains the code for default algorithm proposed by Reyes et. al. \[1,2\].
- `evaluation_scripts/mdrp_format.py` - converts the simulation log into solution format of MDRP evaluation script and reports metrics defined in our paper.
- `evaluation_scripts/compute_performance_summary.py` - reports metrics defined by Reyes et. al. \[1,2\].
- `evaluation_scripts/aggregate_results.py` - aggregates metrics across all instances for an algorithm.

- Running the simulation
```
    ./main.o [path to instance directory] [algorithm={FM|MDRP}] > [result_directory]/[instance name]/sim.results

```
- Generating MDRP solution files and FoodMatch metrics:
```
    python3 evaluation_scripts/mdrp_format.py [instance name] [algorithm={FM|MDRP}]
```
- Get MDRP metrics:
```
python3 evaluation_scripts/compute_performance_summary.py instance_dir=[path to instance directory] input_dir=[path to solution directory] output_dir=[path to output folder]
```

## References

[1] Reyes, Damián et al. “The Meal Delivery Routing Problem.” (2018). <br>
[2] Reyes, Damián. Innovations in last-mile delivery systems. PhD Dissertion. Georgia Institute of Technology, May 2018.
