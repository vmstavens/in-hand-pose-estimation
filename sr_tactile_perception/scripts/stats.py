#!/usr/bin/env python3

from geometry_msgs.msg import Vector3
from typing import List
import json
import numpy as np
import matplotlib.pyplot as plt

def normality(data: List) -> None:
	plt.hist(data)
 
 
def main():
	f = open("data/stat_20230227_000000.json", "r")
	data = json.load(f)

	nc: List[int] = []
	mean_e: List[float] = []
	for t in range(len(data.keys())):
		for fn in data[f"t{t}"].keys():
			nc.append(data[f"t{t}"][fn]["nc"])
			mean = 0 if len(data[f"t{t}"][fn]["theta_e"]) == 0 else np.mean(data[f"t{t}"][fn]["theta_e"])
			mean_e.append(mean)

	# print(f"{nc=}")
	# print(f"{mean_e=}")
	
	fig, axs = plt.subplots(1, 2, figsize=[12,8])

	axs[0].hist(nc,bins=[i for i in range(-3,13)])
	axs[0].set_title("Number of Contact Points")
	axs[0].set_xlabel("Number of contact points")
	axs[0].set_ylabel("Number of instances of number of contact points")
	axs[0].set_ylim(0,430)
 
 
	axs[1].hist(mean_e, bins=[i for i in range(-3,13)])
	axs[1].set_title("Mean Angle Error Between (0,1,0) and Normals")
	axs[1].set_xlabel("Mean angle error")
	axs[1].set_ylabel("Number of instances of mean angle error")
	axs[1].set_ylim(0,430)
	plt.savefig("stats.png")
	plt.show()
 
 
	return


if __name__ == "__main__":
	main()