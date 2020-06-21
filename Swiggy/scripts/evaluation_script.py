import argparse
import numpy as np
import pandas as pd
import pytz
import datetime
import os
import re
import time
from datetime import datetime
import random
import copy
import itertools
from collections import Counter
import warnings

warnings.filterwarnings("ignore")

parser = argparse.ArgumentParser()
parser.add_argument("--input_file", type=str, required=True)
parser.add_argument("--output_file", type=str, required=True)
parser.add_argument("--start", type=int, default=0)
parser.add_argument("--end", type=int, default=28)
parser.add_argument("--delta", type=int, default=180)
args = parser.parse_args()

delta = args.delta

result_file = args.input_file
output_file = args.output_file
st_hr = args.start
end_hr = args.end

st = st_hr*3600
end = end_hr*3600

data = pd.read_csv(result_file, names=["a", "b", "c", "d", "e", "f", "g", "h"])

data_sdt = data[data['a'] == "SDT"].drop(['a', 'g', 'h'], axis = 1)
data_sdt.columns = ['order_id', 'sdt', 'ordered_time', 'prep_time', 'sla']
data_sdt = data_sdt.set_index('order_id')
data_sdt = data_sdt[(data_sdt.ordered_time >= st) & (data_sdt.ordered_time <= end)]

data_assign = data[data['a'] == "ASSIGN"].drop(['a', 'c', 'e', 'f', 'g', 'h'], axis = 1)
data_assign.columns = ['order_id', 'assigned_time']
data_assign = data_assign.groupby('order_id').max()

data_reject = data[data['a'] == 'REJECT'].drop(['a', 'c', 'd', 'e', 'f', 'g', 'h'], axis = 1)
data_reject.columns = ['order_id']

data_deliver = data[data['a'] == "DELIVER"].drop(['a', 'e', 'f', 'g', 'h'], axis = 1)
data_deliver.columns = ['order_id', 'delivered_time', 'vehicle_id']
data_deliver = data_deliver.set_index('order_id')

data_picked = data[data['a'] == "PICKEDUP"].drop(['a', 'd', 'e', 'f', 'g', 'h'], axis = 1)
data_picked.columns = ['order_id', 'picked_time']
data_picked = data_picked.set_index('order_id')

data_reached = data[data['a'] == "REACHED"].drop(['a', 'd', 'e', 'f', 'g', 'h'], axis = 1)
data_reached.columns = ['order_id', 'reached_time']
data_reached = data_reached.groupby('order_id').min()

data_move = data[data['a'] == "MOVE"].drop(['a'], axis = 1)
data_move.columns = ['vehicle_id', 'to_order', 'curr_time', 'dist_travelled',
                        'carrying_orders', 'time_travelled', 'event']
data_move['to_order'] = data_move['to_order'].astype('int')

df = pd.concat([data_sdt, data_deliver, data_picked, data_reached, data_assign], join='inner', axis=1)

df = df[(df.ordered_time >= st) & (df.ordered_time <= end)]
data_move = data_move[(data_move.curr_time >= st) & (data_move.curr_time <= end)]

df['total_del'] = (df['delivered_time'] - df['ordered_time'])/60
df['sdt'] = df['sdt']/60
df['extra_del_time'] = df['total_del'] - df['sdt']
df['de_wait_time'] = (df['picked_time'] - df['reached_time'])/60
df = df.sort_values(by='ordered_time')
data_move['weighted_dist'] = data_move['carrying_orders']*data_move['dist_travelled']

try:
    ovral = [   result_file,
                df.describe().loc['mean']['extra_del_time'],
                df['de_wait_time'].mean(),
                df.shape[0],
                data_move['weighted_dist'].sum()/data_move['dist_travelled'].sum(),
                len(data_reject),
            ]
except:
    print("*****Error in file {}********".format(result_file))

with open(result_file, "r") as myf:
    data = myf.readlines()

full_time = [int(line.split(",")[1][:-1]) for line in data if line.startswith("full_time")]
total_time = np.array(full_time)

div = int((end-st)/args.delta)

total_time = total_time[1:div+1]
total_time = total_time/1000000;

mean_time = np.mean(total_time)
std_time = np.std(total_time)
percent_violations = np.sum(total_time > delta)/len(total_time) * 100

if st_hr==12 or st_hr==20:
    peak_violations = percent_violations
else:
    peak_violations = -1

print("Extra Delivery Time = " + str(df.describe().loc['mean']['extra_del_time']))
print("DE Wait Time = " + str(df['de_wait_time'].mean()))
print("Number of Orders Delivered = " + str(df.shape[0]))
print("Orders per Km = " + str(data_move['weighted_dist'].sum()/data_move['dist_travelled'].sum()))
print("Rejected Orders = " + str(len(data_reject)))
print("Average Runtime = " + str(mean_time))
print("Percent Violations = " + str(percent_violations))

ovral.extend([mean_time, std_time, peak_violations])
df = pd.DataFrame([ovral])
df.to_csv(output_file, header=False)



