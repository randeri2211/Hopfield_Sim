from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from time import sleep
from math import pi

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(True)


world = sim.getObject('/')
print(world)