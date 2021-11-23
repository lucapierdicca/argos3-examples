from p5 import *


def setup():
	size(500, 500)
	background(0)

	no_loop()

	

def draw():
	background(200)
	stroke_weight(1)	
	circle((100,100), 20)
	stroke_weight(5)
	circle((300,300), 20)



if __name__ == '__main__':
	run()