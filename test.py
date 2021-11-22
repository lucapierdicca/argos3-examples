import csv
from pprint import pprint

def accuracy(y_pred,y_true):
	n = len(y_true)
	counter = 0

	for p,t in zip(y_pred,y_true):
		if p == t:
			counter+=1

	return counter/n

file = open('dynamic_dataset.csv')
reader = csv.reader(file, delimiter='|')


dataset = []
for row in reader:
	step = {'clock':0, 'x':0, 'y':0, 'theta':0, 'v_left':0, 'v_right':0, 'world_model_long':{}, 'world_model_short':{}}
	
	step['clock'] = int(row[0])
	step['x'] = float(row[1].replace(",","."))
	step['y'] = float(row[2].replace(",","."))
	step['theta'] = float(row[3].replace(",","."))
	step['v_left'] = float(row[4].replace(",","."))
	step['v_right'] = float(row[5].replace(",","."))

	step['world_model_long'] = {float(row[i].replace(",",".")):float(row[i+1].replace(",",".")) for i in range(6,120*2+6,2)}
	step['world_model_short'] = {float(row[i].replace(",",".")):float(row[i+1].replace(",",".")) for i in range(120*2,120*2+120*2+6,2)}

	dataset.append(step)


	

pprint(dataset[0])