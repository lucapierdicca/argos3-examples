import csv
from p5 import *
from Utils import Classifier
from Utils import Filter




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

	no_stroke()
	for c,p in zip(x_pred, y_pred):
		color = classifier.map_ground_truth[p]['color']
		fill(color[0],color[1],color[2],color[3])
		circle((c[0],c[1]),4)





if __name__ == '__main__':

	
	dyn_dataset = loadDataset()
	print("Dataset: ", len(dyn_dataset))

	classifier = Classifier()

	y_pred,x_pred,y_true = [],[],[]
	for step in dyn_dataset:
		if step['clock']%10 == 0:
			x = [step['x']*50, step['y']*50]
			classlbl, _ = classifier.getClassTrue(x)
			if not classlbl == -1: 
				x_pred.append(x)
				y_true.append(classlbl)
				z = classifier.preProcess(step['world_model_long'],3)
				_, feature = classifier.extractFeature(z)
				y_pred.append(classifier.predict(feature))

	print("Test set: ", len(y_true))


	report = classifier.classification_report_(y_true, y_pred)
	confusion = classifier.confusion_matrix_(y_true, y_pred)

	print(report)
	print()
	print(confusion)


	bayes_filter = Filter(dyn_dataset)

	belief = [0.0, 0.0, 1.0, 0.0] # parte in V [I C V G]

	y_pred,x_pred,y_true = [],[],[]
	for step in dyn_dataset:
		if step['clock']%10 == 0:
			x = [step['x']*50, step['y']*50]
			classlbl, _ = classifier.getClassTrue(x)
			if not classlbl == -1: 
				x_pred.append(x)
				y_true.append(classlbl)
				z = classifier.preProcess(step['world_model_long'],3)
				_, feature = classifier.extractFeature(z)

				belief = bayes_filter.update(belief, feature)
				y_pred.append(bayes_filter.predict(feature))

	report = classifier.classification_report_(y_true, y_pred)
	confusion = classifier.confusion_matrix_(y_true, y_pred)

	print(report)
	print()
	print(confusion)	


	#run()