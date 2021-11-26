import csv
from pprint import pprint
from p5 import *
import numpy as np


PI = 3.14159265358979323846264338327950288

def toRadians(degree):
	return degree*PI/180.0

def accuracy(y_pred,y_true):
	n = len(y_true)
	counter = 0

	for p,t in zip(y_pred,y_true):
		if p == t:
			counter+=1

	return counter/n

def loadDataset(path="dynamic_dataset.csv"):
	file = open(path)
	reader = csv.reader(file, delimiter='|')

	dataset = []


	for row in reader:
		step = {'clock':0, 
		'x':0, 'y':0, 
		'theta':0, 
		'v_left':0, 
		'v_right':0, 
		'world_model_long':{}, 
		'world_model_short':{}}
		
		step['clock'] = int(row[0])-10
		step['x'] = float(row[1].replace(",","."))
		step['y'] = float(row[2].replace(",","."))
		step['theta'] = float(row[3].replace(",","."))
		step['v_left'] = float(row[4].replace(",","."))
		step['v_right'] = float(row[5].replace(",","."))

		step['world_model_long'] = {float(row[i].replace(",",".")):float(row[i+1].replace(",",".")) for i in range(6,120*2+6,2)}
		step['world_model_short'] = {float(row[i].replace(",",".")):float(row[i+1].replace(",",".")) for i in range(120*2,120*2+120*2+6,2)}

		dataset.append(step)

	return dataset

def map_():
	vertices =  [[0.75,-2.0],
				 [0.75,2.0],
				 [4.0,2.0],
				 [4.0,3.5],
				 [0.75,3.5],
				 [0.75,7.5],
				 [4.0,7.5],
				 [4.0,9.0],
				 [-4.0,9.0],
				 [-4.0,7.5],
				 [-0.75,7.5],
				 [-0.75,3.5],
				 [-4.0,3.5],
				 [-4.0,2.0],
				 [-0.75,2.0],
				 [-0.75,-2.0],
				 [0.75,-2.0]]

	map_ = PShape()

	with map_.edit():
		for v in vertices:
			map_.add_vertex(v)

	map_.scale(50,50)

	draw_shape(map_)


def origin():
	circle((0,0), 5)

def preProcess(measurement, w_len):
	d = list(measurement.values())
	d_pre = [0.0]*len(d)

	for i in range(len(d)):
		avg = 0
		for j in range(w_len):
			avg = avg + d[(i+j) % len(d)]

		avg = avg/w_len

		d_pre[(i+int(w_len/2)) % len(d_pre)] = avg


	return {a:d for a,d in zip(measurement.keys(), d_pre)}

def extractFeature(measurement):
	
	local_min_angles = []
	a = list(measurement.keys())
	d = list(measurement.values())

	for i in range(len(d)-4):
		ll = d[i]
		l = d[i+1]
		c = d[i+2]
		r = d[i+3]
		rr = d[i+4]

		if ll>l and l>c and c<r and r<rr:
			local_min_angles.append(a[i+2])

	
	n_intervals = 8
	aperture = toRadians(360.0/n_intervals)
	start = -PI + aperture/2
	interval_ids = []
	feature = [0]*4
	placed = False

	for lm_angle in local_min_angles:
		placed = False
		for i in range(n_intervals-1):
			if lm_angle > start + i*aperture and lm_angle <= start + (i+1)*aperture:
				interval_ids.append(i)
				placed = True

	# handling the last sector (sector id 7 range [180-aperture/2, -180+aperture/2])
	if not placed: interval_ids.append(n_intervals-1) 

	# niente multiple interval_ids
	interval_ids = list(set(interval_ids))

	if(len(interval_ids) <= 1):
		return local_min_angles, feature

	for i in range(len(interval_ids)-1):
		distance_in_intervals = abs(interval_ids[i+1] - interval_ids[i])
		if(distance_in_intervals > n_intervals/2):
			distance_in_intervals = n_intervals - distance_in_intervals

		feature[distance_in_intervals-1]+=1


	# handling the last and first interval 
	distance_in_intervals = abs(interval_ids[len(interval_ids)-1] - interval_ids[0])
	if(distance_in_intervals > n_intervals/2):
		distance_in_intervals = n_intervals - distance_in_intervals

	feature[distance_in_intervals-1]+=1


	return local_min_angles, feature

def predict(feature):
	
	classLbl_to_template = {'I':[0,4,0,0],
							'C':[0,0,0,2],
							'V':[0,2,0,1],
							'G':[0,1,2,0]}

	classLbls = list(classLbl_to_template.keys())
	templates = list(classLbl_to_template.values())

	norms = [norm(np.array(feature) - np.array(t)) for t in templates]

	argmin = np.argmin(np.array(norms))

	return classLbls[argmin]

def norm(vector):
	return np.linalg.norm(vector)




def setup():
	size(500, 800)
	no_loop()


def draw():
	background(80)
	translate(250.0,600.0,0.0)
	rotate_z(3.1415)
	rotate_y(3.1415)

	no_fill()
	stroke(0)
	stroke_weight(4)
	map_()

	y_pred,x_pred,y_true = [],[],[]
	for step in dyn_dataset:
		if step['clock']%10 == 0:
			x = [step['x']*50, step['y']*50]
			for c,data in map_ground_truth.items():
				for bb in data['bb']:
					if x[0] > bb[0][0]*50 and x[0] < bb[1][0]*50 and x[1] > bb[1][1]*50 and x[1] < bb[0][1]*50:
						x_pred.append(x)
						y_true.append(c)
						z = preProcess(step['world_model_long'],3)
						local_min_angles, feature = extractFeature(z)
						y_pred.append(predict(feature))



	acc = accuracy(y_pred, y_true)

	print(acc)

	no_stroke()
	for c,p in zip(x_pred, y_pred):
		color = map_ground_truth[p]['color']
		fill(color[0],color[1],color[2],color[3])
		circle((c[0],c[1]),4)


if __name__ == '__main__':
	map_ground_truth = {'I':{'bb':[[(-0.75,3.5),(0.75,2.0)]],'color':(255,0,0,120)},
						'C':{'bb':[[(-2.5,9.0),(-0.75,7.5)],
							 [(0.75,9.0),(2.5,7.5)],
							 [(-0.75,7.5),(0.75,3.5)],
							 [(-2.5,3.5),(-0.75,2.0)],
							 [(0.75,3.5),(2.5,2.0)],
							 [(-0.75,2.0),(0.75,-0.5)]], 'color':(0,255,0,120)},
						'V':{'bb':[[(-4.0,9.0),(-2.5,7.5)],
							 [(2.5,9.0),(4.0,7.5)],
							 [(-4.0,3.5),(-2.5,2.0)],
							 [(2.5,3.5),(4.0,2.0)],
							 [(-0.75,-0.5),(0.75,-2.0)]], 'color':(0,0,255,120)},
						'G':{'bb':[[(-0.75,9.0),(0.75,7.5)]],'color':(255,255,0,120)}}
	

	dyn_dataset = loadDataset()
	run()