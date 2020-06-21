import numpy as np
import pandas as pd
import os
import warnings
import sys

warnings.filterwarnings("ignore")

result_folder = "results_" + sys.argv[1] + '/'

print ("------------------------- FM METRICS -------------------------")
dfs = []
for f in os.listdir(result_folder):
    df = pd.read_csv(result_folder + f + "/fm_metrics.csv", names=['orders_per_km', 'XDT', 'DE_wait_time', 'Rejected'])
    dfs.append(df)
df = pd.concat(dfs)
print(df.describe().loc[['count', 'mean', 'std']].T)

print ("------------------------- MDRP METRICS -------------------------")
dfs = []
for f in os.listdir(result_folder):
    df = pd.read_csv(result_folder + f + "/mdrp_metrics.csv", names=["%% undelivered","CtoD mean","CtoD 90%%","CtoD overage","RtoP mean","RtoP 90%%","utilization mean","utilization 10%%","cost per order","orders per bundle"])
    dfs.append(df)
df = pd.concat(dfs)
print(df.describe().loc[['count', 'mean', 'std']].T)