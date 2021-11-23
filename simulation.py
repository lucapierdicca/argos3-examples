import csv
from pprint import pprint
from p5 import *



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
		
		step['clock'] = int(row[0])
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

def robot():

	x = dyn_dataset[step]['x']*50
	y = dyn_dataset[step]['y']*50
	theta = dyn_dataset[step]['theta']

	circle((x,y), 15)

	with push_matrix():
		translate(x,y,0)
		rotate_z(theta)
		
		no_stroke()
		stroke_weight(2)
		fill(255)
		circle((3,0), 5)


def origin():
	circle((0,0), 5)


def setup():
	global step
	
	size(500, 800)
	

def draw():
	global step
	background(160)

	translate(250.0,600.0,0.0)
	rotate_z(3.1415)
	rotate_y(3.1415)

	fill(0)
	no_stroke()
	origin()

	no_fill()
	stroke(255)
	stroke_weight(4)
	map_()

	no_fill()
	stroke(255)
	stroke_weight(2)
	robot()


	step+=1



if __name__ == '__main__':
	step = 0
	dyn_dataset = loadDataset()
	run()