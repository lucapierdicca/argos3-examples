import csv
from p5 import *
from pprint import pprint
from Utils import Classifier, DiscreteFilter, GaussianFilter, loadDataset, map_ground_truth



def map_(classifier):
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

	no_stroke()
	for classlbl,data in map_ground_truth.items():
		color = data['color']
		fill(color[0],color[1],color[2],40)
		for bb in data['bb']:
			bb_vertices = [(bb[0][0]*50,bb[0][1]*50), 
							(bb[1][0]*50,bb[0][1]*50),
							(bb[1][0]*50,bb[1][1]*50),
							(bb[0][0]*50,bb[1][1]*50)]

			begin_shape()
			for v in bb_vertices:
				vertex(v[0],v[1])
			end_shape("CLOSE")


def origin():
	circle((0,0), 5)


def setup():
	size(500, 800)
	no_loop()


def draw():
	background(150)
	translate(250.0,600.0,0.0)
	rotate_z(3.1415)
	rotate_y(3.1415)

	#draw the map + ground truth
	no_fill()
	stroke(0)
	stroke_weight(4)
	map_(classifier)

	#draw the predictions
	no_stroke()
	for c,p in zip(x_pred, y_pred):
		color = map_ground_truth[p]['color']
		fill(color[0],color[1],color[2],color[3])
		circle((c[0],c[1]),4)



if __name__ == '__main__':
	
	wallfollowing_dataset = loadDataset("wallfollowing_dataset.csv")
	print("wallfollowing_dataset: ", len(wallfollowing_dataset))

	randomwalk_dataset = loadDataset("randomwalk_dataset.csv")
	print("randomwalk_dataset: ", len(randomwalk_dataset))

	template_dataset = loadDataset("template.csv")
	print("template_dataset: ", len(template_dataset))

	

	# # no filter---------------------------------------------------

	# classifier = Classifier()

	# y_pred,x_pred,y_true = [],[],[]
	# for step in dyn_dataset:
	# 	if step['clock']%10 == 0:
	# 		x = [step['x']*50, step['y']*50]
	# 		x_pred.append(x)
	# 		y_true.append(step['true_class'])
	# 		z = classifier.preProcess(step['world_model_long'],3)
	# 		_, feature = classifier.extractFeature(z)
	# 		y_pred.append(classifier.predict(feature))

	# print("Test set: ", len(y_true))


	# report = classifier.classification_report_(y_true, y_pred)
	# confusion = classifier.confusion_matrix_(y_true, y_pred)

	# print(report)
	# print()
	# print(confusion)


	# #discrete filter----------------------------------------------

	# discrete_filter = DiscreteFilter(dyn_dataset[:12000])
	# print("Vocabulary: ", discrete_filter.observation_model.shape[0])

	# belief = [0.0, 0.0, 1.0, 0.0] # parte in V [I C V G]

	# y_pred,x_pred,y_true = [],[],[]
	# for step in dyn_dataset[12001:]:
	# 	if step['clock']%10 == 0:
	# 		x = [step['x']*50, step['y']*50]
	# 		x_pred.append(x)
	# 		y_true.append(step['true_class'])
	# 		z = classifier.preProcess(step['world_model_long'],3)
	# 		_, feature = classifier.extractFeatureRaw(z)

	# 		belief = discrete_filter.update(belief, feature)
	# 		y_pred.append(discrete_filter.predict(belief))


	# print("Test set: ", len(y_true))

	# report = classifier.classification_report_(y_true, y_pred)
	# confusion = classifier.confusion_matrix_(y_true, y_pred)

	# print(report)
	# print()
	# print(confusion)



	#Gaussian filter----------------------------------------------

	classifier = Classifier()
	gaussian_filter = GaussianFilter(wallfollowing_dataset, randomwalk_dataset, template_dataset)
	pprint(gaussian_filter.transition_model)
	print("Parameters: ", gaussian_filter.parameters)


	belief = [0.0, 0.0, 1.0, 0.0] # parte in V [I C V G]

	y_pred,x_pred,y_true = [],[],[]
	for step in wallfollowing_dataset:
		if step['clock']%10 == 0:
			x = [step['x']*50, step['y']*50]
			x_pred.append(x)
			y_true.append(step['true_class'])
			z = classifier.preProcess(step['world_model_long'],3)
			feature = gaussian_filter.extractFeature(z)

			belief = gaussian_filter.update(belief, feature)
			y_pred.append(gaussian_filter.predict(belief))


	print("Test set: ", len(y_true))

	report = classifier.classification_report_(y_true, y_pred)
	confusion = classifier.confusion_matrix_(y_true, y_pred)

	print(report)
	print()
	print(confusion)
	
	run()