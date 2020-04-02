import math

class PDController:

	def __init__(self,direction,distance_to_wall,Kp=10.0,Kd=5.0, angle_coefficient=1.0):

		self.direction = direction
		self.distance_desired = distance_to_wall
		self.Kp = Kp
		self.Kd = Kd
		self.angle_coefficient = angle_coefficient

		self.err_curr = 0.0
		self.err_prev = 0.0
		self.err_deriv  = 0.0
		self.c = 0.0


	def step(self,distance_curr,angle_with_closest_obstacle):

		self.err_curr = distance_curr-self.distance_desired
		self.err_deriv = self.err_curr - self.err_prev

		self.c = self.direction*(self.Kp*self.err_curr + self.Kd*self.err_deriv) * self.angle_coefficient*(angle_with_closest_obstacle-math.pi*self.direction/2)

		self.err_prev = self.err_curr

		return self.c


	def print_status(self):
		print("Error current :"+str(self.err_curr))
		print("Error previous :"+str(self.err_prev))
		print("Error derivative :"+str(seld.err_deriv))
		print("Calculated angular z :"+str(self.c))
		print("******************************************")
		print()