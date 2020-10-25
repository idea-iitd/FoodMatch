# Batching and Matching for Food Delivery in Dynamic Road Networks

This repository contains official implementation of the algorithms defined in our paper.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine.

### Prerequisites

The algorithms are implemented in C++11 (GCC 7.4.0) and evaluation scripts are implemented in Python 3 (Python 3.6)

### Installation

Setup a conda environment which includes packages required to run evaluation scripts:

```bash
conda env create -f environment.yml
conda activate fm_evn
```

### Datasets and evaluation procedure
We use two datasets for evaluation of algorithms:
- Dataset provided by [Swiggy](https://www.swiggy.com/) : The code for simulation and algorithms defined in our paper is provided in [./Swiggy](Swiggy). An anonymized version of the proprietary dataset will be made available once an agreement is signed. The request can be made by emailing the authors of the paper: [Prof. Sayan Ranu](mailto:sayanranu@cse.iitd.ac.in), [Prof. Amitabha Bagchi](mailto:bagchi@cse.iitd.ac.in), [Manas Joshi](mailto:manasjoshi241@gmail.com), [Arshdeep Singh](mailto:arshdeep50625@gmail.com)

- [GrubHub dataset](https://github.com/grubhub/mdrplib) : We implement a simulation framework conforming to the structural assumptions made by Reyes et. al.\[1,2\] and compare our algorithm FoodMatch(adapted to work without road networks). The code for this is provided in [./GrubHub](GrubHub).

## References
[1] Reyes, Damián et al. “The Meal Delivery Routing Problem.” (2018). <br>
[2] Reyes, Damián. Innovations in last-mile delivery systems. PhD Dissertion. Georgia Institute of Technology, May 2018. <br>
[3] Hungarian Implementation: https://bougleux.users.greyc.fr/lsape/
