from mp import *

class resources:
	def __init__(self):
		self.hostname = "hai-1095.local"
		self.mp_ = Mindprobe()
		self.mp_.init(self.hostname)

r = resources()
