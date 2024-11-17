import random
from xml.etree import ElementTree as ET
import traci
from acoTrips import CAMOACO

# Generate trips with customizable parameters
num_vehicles = 100
trip_length = 10  # Average trip length in km
speed_range = (20, 30)  # Average speed range in km/h

# Create the root element for the trips.xml file
root = ET.Element('trips')

for i in range(num_vehicles):
    # Generate random start and end points
    start_x = random.uniform(0, 10)
    start_y = random.uniform(0, 10)
    end_x = random.uniform(0, 10)
    end_y = random.uniform(0, 10)

    # Calculate the trip distance
    trip_distance = ((start_x - end_x) ** 2 + (start_y - end_y) ** 2) ** 0.5

    # Generate a random speed within the specified range
    avg_speed = random.uniform(*speed_range)

    # Calculate the trip duration
    trip_duration = trip_distance / avg_speed * 3600  # Convert to seconds

    # Create the vehicle element
    vehicle = ET.SubElement(root, 'vehicle', attrib={
        'id': 'vehicle_{}'.format(i),
        'depart': '{}'.format(i * 10)
    })

    # Add the route element
    route = ET.SubElement(vehicle, 'route')
    route.text = ' '.join(['edge_{}'.format(j) for j in range(int(trip_distance // 100))])

# Write the trips.xml file
tree = ET.ElementTree(root)
tree.write('trips.xml', encoding='utf-8', xml_declaration=True)

# Load the SUMO network and trips
net = traci.load(['--net-file', 'nycmap.net.xml', '--route-files', 'trips.xml'])

camo_aco = CAMOACO(num_ants=50, alpha=1, beta=2, rho=0.5, q0=0.8, max_iterations=100)

for step in range(net.getMinExpectedNumber()):
    traci.simulationStep()

    # Retrieve current vehicle positions and states
    vehicles = traci.vehicle.getIDList()
    for vehicle_id in vehicles:
        current_edge = traci.vehicle.getRoadID(vehicle_id)
        destination_edge = traci.vehicle.getRoute(vehicle_id)[-1]

        # Run CAMO-ACO to find the optimal route for the vehicle
        best_path, best_cost = camo_aco.run(current_edge, destination_edge)

        # Update the vehicle's route in the SUMO simulation
        traci.vehicle.setRoute(vehicle_id, best_path)

traci.close()
