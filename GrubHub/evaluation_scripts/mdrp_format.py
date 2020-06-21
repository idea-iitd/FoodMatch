import numpy as np
import pandas as pd
import os
import itertools
import warnings
import sys
from collections import Counter

warnings.filterwarnings("ignore")

instance_folder = "public_instances/"
instance_name = sys.argv[1]
result_folder = "results_" + sys.argv[2] + '/'
verbosity = 1
if (len(sys.argv) == 4):
    verbosity = int(sys.argv[3])

ainfo_file = result_folder+instance_name+"/solution_info_assignments.txt"
osol_file = result_folder+instance_name+"/solution_info_orders.txt"
csol_file = result_folder+instance_name+"/solution_info_couriers.txt"
sim_file = result_folder+instance_name+"/sim.results"
metric_file = result_folder+instance_name+"/fm_metrics.csv"

data = pd.read_csv(sim_file, names=["a", "b", "c", "d", "e", "f", "g", "h"])

data_sdt = data[data['a'] == "SDT"].drop(['a', 'f', 'g', 'h'], axis = 1)
data_sdt.columns = ['order_id', 'sdt', 'ordered_time', 'ready_time']
data_sdt = data_sdt.set_index('order_id')
data_sdt = data_sdt.astype('float')

data_assign = data[data['a'] == "ASSIGN"].drop(['a', 'c', 'e', 'f', 'g', 'h'], axis = 1)
data_assign.columns = ['order_id', 'assigned_time']
data_assign.assigned_time = data_assign.assigned_time.astype('float')
data_assign = data_assign.groupby('order_id').max()

data_reject = data[data['a'] == 'REJECT'].drop(['a', 'c', 'd', 'e', 'f', 'g', 'h'], axis = 1)
data_reject.columns = ['order_id']

data_deliver = data[data['a'] == "DELIVER"].drop(['a', 'e', 'f', 'g', 'h'], axis = 1)
data_deliver.columns = ['order_id', 'delivered_time', 'vehicle_id']
data_deliver = data_deliver.set_index('order_id')
data_deliver.delivered_time = data_deliver.delivered_time.astype('float')

data_reached = data[data['a'] == "REACHED"].drop(['a', 'd', 'e', 'f', 'g', 'h'], axis = 1)
data_reached.columns = ['order_id', 'reached_time']
data_reached.reached_time = data_reached.reached_time.astype('float')
data_reached = data_reached.groupby('order_id').min()

data_picked = data[data['a'] == "PICKEDUP"].drop(['a', 'e', 'f', 'g', 'h'], axis = 1)
data_picked.columns = ['order_id', 'picked_time', 'rest_id']
data_picked = data_picked.set_index('order_id')
data_picked.picked_time = data_picked.picked_time.astype('float')

data_move = data[data['a'] == "MOVE"].drop(['a'], axis = 1)
data_move.columns = ['vehicle_id', 'to_order', 'curr_time', 'dist_travelled',
                        'carrying_orders', 'time_travelled', 'event']
data_move[['curr_time', 'dist_travelled', 'carrying_orders', 'time_travelled', 'event']] = \
        data_move[['curr_time', 'dist_travelled', 'carrying_orders', 'time_travelled', 'event']].astype('float')

df = pd.concat([data_sdt, data_deliver, data_reached, data_picked, data_assign], join='inner', axis=1)

df['total_del'] = (df['delivered_time'] - df['ordered_time'])/60.0
df['extra_del_time'] = (df['delivered_time'] - df['sdt'])/60.0
df['de_wait_time'] = (df['picked_time'] - df['reached_time'])/60.0
data_move['weighted_dist'] = data_move['carrying_orders']*data_move['dist_travelled']

batch_df = (df[['picked_time', 'delivered_time', 'vehicle_id']].sort_values(by='picked_time'))
bundles = []
cnt = 0
for name, grp in batch_df.groupby('vehicle_id'):
    # name ='c103'
    # grp = batch_df.groupby('vehicle_id').get_group(name)
    temp2 = (grp.reset_index().sort_values(by='picked_time')).values.tolist()
    colour = [-1]*len(temp2)
    colour[0] = 0
    for i in range(1,len(temp2)):
        for j in range(i):
            if temp2[j][2] >= temp2[i][1]:
                colour[i]= colour[j]
                break

        if colour[i] == -1:
            colour[i] = colour[i-1] + 1

    for k, v in Counter(colour).items():
        if v > 1:
            cnt += v
        new_bun = []
        for i in range(len(colour)):
            if (colour[i] == k):
                new_bun.append(temp2[i][0])
        bundles.append(new_bun)

with open(metric_file, 'w') as myf:
    myf.write(str(data_move['weighted_dist'].sum()/data_move['dist_travelled'].sum()))
    myf.write(',')
    myf.write(str((df['extra_del_time'].sum() + len(data_reject)*120.0)/len(data_sdt)))
    myf.write(',')
    myf.write(str(df['de_wait_time'].mean()))
    myf.write(',')
    myf.write(str(len(data_reject)))

if verbosity==1:
    print ("------------------------- FM METRICS -------------------------")
    print( "Orders per Km = " + str(data_move['weighted_dist'].sum()/data_move['dist_travelled'].sum()))
    print( "XDT = " + str((df['extra_del_time'].sum() + len(data_reject)*120.0)/len(data_sdt)))
    print( "DE Wait Time = " + str(df['de_wait_time'].mean()))
    print( "Rejected Orders = " + str(len(data_reject)))
    print ("--------------------------------------------------------------")

ainfo_data = [['assignment_time' , 'pickup_time' , 'courier_id', 'order_list']]
for bun in bundles:
    temp = df.loc[bun]
    for name, grp in temp.groupby('rest_id'):
        picked_max = grp.picked_time.max()
        assigned_max = grp.assigned_time.max()
        df.loc[grp.index, 'picked_time'] = picked_max
        df.loc[grp.index, 'assigned_time'] = assigned_max

        ainfo_row = [assigned_max/60.0, picked_max/60.0, grp.vehicle_id[0]]
        ainfo_row.extend(grp.sort_values(by="delivered_time").index.values.tolist())

        ainfo_data.append(ainfo_row)

with open(ainfo_file, 'w') as myf:
    for d in ainfo_data:
        myf.write(' '.join([str(i) for i in d]) + '\n')


data_courier = data[data['a'] == "MDRP MOVE"].drop(['a', 'f', 'g', 'h'], axis = 1)
data_courier.columns = ['courier_id', 'dept_time', 'origin', 'destination']
data_courier.dept_time = data_courier.dept_time.astype('float')/60.0
data_courier.sort_values(by=['courier_id', 'dept_time']).to_csv(csol_file, sep=' ', index=False)

output_df = df[['ordered_time', 'ready_time', 'picked_time', 'delivered_time', 'vehicle_id']].reset_index()
output_df.columns = ['order', 'placement_time', 'ready_time', 'pickup_time', 'dropoff_time', 'courier']
output_df.placement_time /= 60.0
output_df.ready_time /= 60.0
output_df.pickup_time /= 60.0
output_df.dropoff_time /= 60.0
output_df.to_csv(osol_file, sep=" ")
