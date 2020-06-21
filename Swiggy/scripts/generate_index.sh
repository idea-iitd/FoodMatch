#!/bin/bash
cd hl/
make clean
make 
cd ../
city=("A" "B" "C")
for c in "${city[@]}"
do
    for (( i = 1; i <= 6; i++ ))
    do
        for (( j = 0 ; j <= 23; j++ ))
        do
            ./hl/akiba -o data/data_"$c"_anonymized/map/$i/per_hour_edges/dimacs_0.order data/data_"$c"_anonymized/map/$i/per_hour_edges/dimacs_$j -l data/data_"$c"_anonymized/map/$i/per_hour_edges/dimacs_$j.label
        done
    done
done
