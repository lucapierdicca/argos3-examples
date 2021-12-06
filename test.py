from p5 import *
import numpy as np
from Utils import loadDataset
import matplotlib.pyplot as plt
import csv



def setup():
	size(500, 800)
	no_loop()

def draw():
	background(160)
	translate(250.0,600.0,0.0)
	rotate_z(3.1415)
	rotate_y(3.1415)

	
	for classlbl,data in map_ground_truth.items():

		for bb in data['bb']:
			bb_vertices = [(bb[0][0]*50,bb[0][1]*50), 
							(bb[1][0]*50,bb[0][1]*50),
							(bb[1][0]*50,bb[1][1]*50),
							(bb[0][0]*50,bb[1][1]*50)]

			begin_shape()
			for v in bb_vertices:
				vertex(v[0],v[1])
			end_shape("CLOSE")

def extractFeature(measurement):

	poly_v = []
	for a,d in measurement.items():
		x = d*np.cos(a)
		y = d*np.sin(a)

		poly_v.append((x,y))

	poly_v.append(poly_v[0])

	A = 0.0
	P = 0.0
	for i in range(len(poly_v)-1):
		A += poly_v[i][0]*poly_v[i+1][1] - poly_v[i][1]*poly_v[i+1][0]
		P += np.linalg.norm(np.array(poly_v[i+1])-np.array(poly_v[i]))

	return 0.5*A, P






if __name__ == '__main__':

	dyn_dataset = loadDataset("dynamic_dataset.csv")
	print("Dataset: ", len(dyn_dataset))

	x,y,c = [],[],[]
	for step in dyn_dataset:
		A,P = extractFeature(step['world_model_long'])
		x.append(A)
		y.append(P)
		c.append((step['color'][0]/255.0, step['color'][1]/255.0, step['color'][2]/255.0))


	plt.scatter(x, [5]*len(x), s=3.0, c=c, alpha=0.2)
	plt.show()


