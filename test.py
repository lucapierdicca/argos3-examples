from p5 import *



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

def setup():
	size(500, 800)
	rect_mode(CORNER)

	no_loop()

	

def draw():
	background(160)
	translate(250.0,600.0,0.0)
	rotate_z(3.1415)
	rotate_y(3.1415)

	for class_lbl,data in map_ground_truth.items():
		fill(data['color'][0],data['color'][1],data['color'][2],data['color'][3])
		for bbox in data['bb']:
			rect_origin = (bbox[0][0]*50, bbox[0][1]*50)
			print(class_lbl, rect_origin)
			w = (bbox[1][0] - bbox[0][0])*50
			h = (bbox[0][1] - bbox[1][1])*50
			rect(rect_origin,w,h)

	rect((-37.5,175),20,20),



if __name__ == '__main__':
	run()