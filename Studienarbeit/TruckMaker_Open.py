import subprocess
import time
from pycarmaker import CarMaker, Quantity

# 1 - Open CarMaker with option -cmdport
# For example: on a Windows system with CarMaker 8.0.2 installed on the default
# folder send the command C:\IPG\carmaker\win64-8.0.2\bin\CM.exe -cmdport 16660

# 2 - Start any TesRun

# 3 - Initialize pyCarMaker
IP_ADDRESS = "localhost"
PORT = 16660
cm = carMaker(IP_ADDRESS, PORT)

# 4 - Connect to CarMaker
cm.connect()

# 5 - Subscribe to vehicle speed
# Create a Quantity instance for vehicle speed (vehicle speed is a float type variable)
vehspd = Quantity("Car.v", Quantity.FLOAT)

# Initialize with negative speed to indicate that value was not read
vehspd.data = -1.0

# Subscribe (TCP socket need to be connected)
cm.subscribe(vehspd)

# 6 - Read all subscribed quantities. In this example, vehicle speed and simulation status
# For some reason, the first two reads will be incomplete and must be ignored
# You will see 2 log errors like this: [ ERROR]   CarMaker: Wrong read
cm.read()
cm.read()
time.sleep(0.1)
c = 5
while(c > 0):
    c = c - 1
    # Read data from carmaker
    cm.read()
    print()
    print("Vehicle speed: " + str(vehspd.data * 3.6) + " km/h")
    print("Simulation status: " + ("Running" if sim_status.data >= 0 else tm.status_dic.get(sim_status.data)))
    time.sleep(1)


# Johannes script
# subprocess.call(["taskkill", "/F", "/IM", "CarlaUE4-Win64-Shipping.exe"])
# time.sleep(3)
# subprocess.Popen([r"C:\ProgramData\Microsoft\Windows\Start Menu\Programs\IPG\CarMaker 11.0.1"])