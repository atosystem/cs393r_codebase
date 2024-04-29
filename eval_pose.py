import argparse
import csv
import numpy as np
from dtw import dtw

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimated", type=str, default="estimated_pose.csv")
    parser.add_argument("--reference", type=str, default="reference_pose.csv")
    args = parser.parse_args()

    estimated_pose = []
    with open(args.estimated, "r", newline='') as f:
        reader =  csv.reader(f)
        next(reader)
        for row in reader:
            estimated_pose.append([float(row[0]), float(row[1]), float(row[2])])

    reference_pose = []
    with open(args.reference, "r", newline='') as f:
        reader =  csv.reader(f)
        next(reader)
        for row in reader:
            reference_pose.append([float(row[0]), float(row[1]), float(row[2])])

    estimated_pose = np.array(estimated_pose)
    reference_pose = np.array(reference_pose)

    se2_dist = lambda x, y: ((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2) ** 0.5 + 1 - np.cos(x[2] - y[2])

    d, cost_matrix, acc_cost_matrix, path = dtw(
        reference_pose, estimated_pose, dist=se2_dist)

    xy = []
    for i in range(len(path[0])):
        xy.append((path[0][i], path[1][i]))
    cost = 0
    acc_cost = 0
    for i, (x, y) in enumerate(xy):
        if i < len(xy) - 1 and xy[i + 1][1] == y:
            continue
        cost += cost_matrix[x, y]
        acc_cost += acc_cost_matrix[x, y]
    print(f"Comparing {args.estimated} and {args.reference} using DTW")
    print("Distance:", d)
    print("Cost:", cost)
    print("Accumulated cost:", acc_cost)

if __name__ == "__main__":
    main()
