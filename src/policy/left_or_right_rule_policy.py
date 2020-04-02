import math

class LeftOrRightRule:

	def __init__(self,type,turn_speed = -2.0):

		self.type = type
		self.direction = 1 if type=='LHR' else -1
		self.turn_speed = turn_speed
		self.decision = ""

	def step(self,space_ahead,angle_with_closest_obstacle):
		linear_x_factor = 0.0
		angular_z = 0.0

		if(space_ahead<2):
			angular_z = self.direction*self.turn_speed
			self.decision="Turn"

		elif(space_ahead<4):
			linear_x_factor = 0.5
			self.decision = "Ahead"

		elif(math.fabs(angle_with_closest_obstacle)>1.75):
			linear_x_factor = 0.4
			self.decision = "Go ahead slowly"
		else:
			linear_x_factor = 1
			self.decision = "Go ahead with full speed"

		self.print_status()
		return linear_x_factor,angular_z

	def print_status(self):
		print("Decision:")
		print(self.decision)