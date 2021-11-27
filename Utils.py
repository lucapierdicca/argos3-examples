import numpy as np
from sklearn.metrics import confusion_matrix, classification_report

PI = 3.14159265358979323846264338327950288

def toRadians(degree):
	return degree*PI/180.0

def norm(vector):
	return np.linalg.norm(vector)


class Classifier:
	def __init__(self):
		self.map_ground_truth = {'I':{'bb':[[(-0.75,3.5),(0.75,2.0)]],'color':(255,0,0,120)},
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

		self.classlbl_to_template = {'I':[0,4,0,0],
							'C':[0,0,0,2],
							'V':[0,2,0,1],
							'G':[0,1,2,0]}

		self.classlbl_to_id = {c:i for i,c in enumerate(list(self.map_ground_truth.keys()))}
		self.id_to_classlbl = {i:c for c,i in self.classlbl_to_id.items()}

	def preProcess(self, measurement, w_len):
		d = list(measurement.values())
		d_pre = [0.0]*len(d)

		for i in range(len(d)):
			avg = 0
			for j in range(w_len):
				avg = avg + d[(i+j) % len(d)]

			avg = avg/w_len

			d_pre[(i+int(w_len/2)) % len(d_pre)] = avg


		return {a:d for a,d in zip(measurement.keys(), d_pre)}

	def extractFeature(self, measurement):
		
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

	def predict(self, feature):
		
		classlbls = list(self.classlbl_to_template.keys())
		templates = list(self.classlbl_to_template.values())

		norms = [norm(np.array(feature) - np.array(t)) for t in templates]

		argmin = np.argmin(np.array(norms))

		return classlbls[argmin]

	def getClassTrue(self, robot_position):
		for classlbl, data in self.map_ground_truth.items():
			for bb in data['bb']:
				if robot_position[0] > bb[0][0]*50 and robot_position[0] < bb[1][0]*50 and robot_position[1] > bb[1][1]*50 and robot_position[1] < bb[0][1]*50:
					return classlbl, self.classlbl_to_id[classlbl]
		return -1, -1

	def classification_report_(self, y_true,y_pred):
		return classification_report(y_true, y_pred, labels=list(self.classlbl_to_id.keys()))

	def confusion_matrix_(self, y_true, y_pred):
		return confusion_matrix(y_true, y_pred, labels=list(self.classlbl_to_id.keys()))



class Filter:

	def __init__(self, dataset):
		self.state_dim = 4
		self.classifier = Classifier()
		self.estimateTransitionModel(dataset)
		self.estimateObservationModel(dataset)



	def estimateTransitionModel(self, dataset):

		joint = np.ones((self.state_dim, self.state_dim))

		for i in range(len(dataset)-1):
			robot_position = [dataset[i]['x']*50, dataset[i]['y']*50]
			_, classid = self.classifier.getClassTrue(robot_position)
			xt_1 = classid

			robot_position = [dataset[i+1]['x']*50, dataset[i+1]['y']*50]
			_, classid = self.classifier.getClassTrue(robot_position)
			xt = classid

			if not (xt_1 == -1 or xt == -1):
				joint[xt,xt_1] += 1.0

		self.transition_model = joint/np.sum(joint, axis=0)

	def estimateObservationModel(self, dataset):
		
		joint_dict = {}

		for step in dataset:
			z = self.classifier.preProcess(step['world_model_long'],3)
			_, feature = self.classifier.extractFeature(z)

			if tuple(feature) not in joint_dict:
				joint_dict[tuple(feature)] = [0.0]*self.state_dim

			robot_position = [step['x']*50, step['y']*50]
			_, classid = self.classifier.getClassTrue(robot_position)
			
			if classid != -1:
				joint_dict[tuple(feature)][classid] += 1.0

			self.feature_to_id = {feature:i for i,feature in enumerate(list(joint_dict.keys()))}

			joint = np.ones((len(joint_dict),self.state_dim))

			for feature,state_counts in joint_dict.items():
				for i,counts in enumerate(state_counts):
					joint[self.feature_to_id[feature], i] += counts

			self.observation_model = joint/np.sum(joint, axis=0)


	def transitionModel(self, xt, xt_1):
		return self.transition_model[xt, xt_1]


	def observationModel(self, zt, xt):
		return self.observation_model[feature_to_id[tuple(feature)], xt]

	def update(self, belief, feature):
		bt_t_1 = [0.0]*self.state_dim
		for i in range(self.state_dim):
			for j in range(self.state_dim):
				bt_t_1[j] += self.transition_model[j, i]*belief[i]




		btt = [0.0]*self.state_dim
		den = 0.0
		for i in range(self.state_dim):
			btt[i] = self.observation_model[self.feature_to_id[tuple(feature)],i]*bt_t_1[i]
			den+=btt[i]

		return [b/den for b in btt]

	def predict(self, belief):
		argmax = np.argmax(np.array(belief))
		return self.classifier.id_to_classlbl[argmax]





