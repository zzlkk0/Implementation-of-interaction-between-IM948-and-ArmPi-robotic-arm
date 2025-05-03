from Board import getBusServoPulse
from time import sleep

while True:

	for id in range(1,7):
		pulse=getBusServoPulse(id)
		print(pulse)
	sleep(1)

	

