#!/usr/bin/env python3

import csv
from stand import stand

# NOTE: You need to import the wanted file e.g. pick_up.py

def convert_time_array(arr):
    out = []
    pose = 1
    for el in arr:
        webots_time = convert_time_to_webots_time(el, pose)
        out.append(webots_time)
        pose += 1

    return out


def convert_time_to_webots_time(time, pose):
    tmp = (time % 1)
    milisec = tmp * 1000
    sec = time - tmp

    return '00:' + ("%02d" % (sec,)) + ':' + ("%03d" % (milisec,)) + ',' + ('Pose%d' % pose)


all_names, all_times, all_keys = stand() # NOTE: Python function should be called here!
first_row = ['#WEBOTS_MOTION','V1.0'] + all_names

if __name__ == "__main__":
    where_to = input('Where to save the motion file: ')
    motion = where_to
    time_set = set()
    with open(motion, 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(first_row)

        # Get all times
        for times in all_times:
            for time in times:
                time_set.add(time)

        time_array = sorted(time_set)
        time_array_webots = convert_time_array(time_array)

        for name,times,keys in zip(all_names,all_times,all_keys):
            indexes = []
            for time,key in zip(times,keys):
                index = time_array.index(time)
                indexes.append(index)
                time_array_webots[index] += (',' + str(key))

            time_array_webots = [(x + ',*') if (i not in indexes) else time_array_webots[i] for i,x in enumerate(time_array_webots) ] 
            print(time_array_webots)
                   
        out_csv = []
        for row in time_array_webots:
            out = row.split(',')
            out_csv.append(out)
                
        writer.writerows(out_csv)
                
