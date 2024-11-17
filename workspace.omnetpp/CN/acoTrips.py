import os
import sys
import random
import sumolib
import traci
import math
import numpy as np
import logging
from collections import defaultdict
from typing import List, Dict, Tuple

class VANETMetrics:
    def __init__(self, net: sumolib.net.Net):
        self.net = net
        # Configuration parameters
        self.rssi_threshold = -85  # dBm
        self.snr_threshold = 10    # dB
        self.optimal_density = 20  # vehicles per km
        self.max_speed = 30       # m/s
        self.communication_range = 300  # meters
        
        # Initialize logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("VANETMetrics")

    def get_rssi(self, edge_id: str) -> float:
        """Simulate RSSI based on vehicle positions and density"""
        try:
            vehicles = traci.edge.getLastStepVehicleIDs(edge_id)
            if not vehicles:
                return self.rssi_threshold
            
            # Simulate RSSI based on vehicle density and positions
            vehicle_positions = [traci.vehicle.getPosition(v) for v in vehicles]
            avg_distance = self._calculate_average_distance(vehicle_positions)
            
            # Path loss model: RSSI = -20log₁₀(d) - 20log₁₀(f) + 27.55
            rssi = -20 * math.log10(max(avg_distance, 1)) - 40 + random.uniform(-5, 5)
            return min(max(rssi, self.rssi_threshold), -40)
        except traci.TraCIException as e:
            self.logger.warning(f"Error getting RSSI for edge {edge_id}: {e}")
            return self.rssi_threshold

    def get_snr(self, edge_id: str) -> float:
        """Calculate SNR based on vehicle density and traffic conditions"""
        try:
            density = traci.edge.getLastStepVehicleNumber(edge_id)
            edge_length = self.net.getEdge(edge_id).getLength()
            
            # SNR decreases with higher vehicle density
            base_snr = self.snr_threshold
            density_factor = min(1, density * edge_length / 1000)
            snr = base_snr * (1 - 0.5 * density_factor) + random.uniform(-2, 2)
            
            return max(snr, 0)
        except Exception as e:
            self.logger.warning(f"Error calculating SNR for edge {edge_id}: {e}")
            return 0

    def calculate_link_reliability(self, edge_id: str) -> float:
        """Calculate comprehensive link reliability"""
        try:
            rssi = self.get_rssi(edge_id)
            snr = self.get_snr(edge_id)
            stability = self.get_connection_stability(edge_id)
            
            # Normalize metrics
            norm_rssi = (rssi - self.rssi_threshold) / (-40 - self.rssi_threshold)
            norm_snr = snr / self.snr_threshold
            
            # Weighted combination
            reliability = (
                0.4 * norm_rssi +
                0.3 * norm_snr +
                0.3 * stability
            )
            return max(min(reliability, 1.0), 0.0)
        except Exception as e:
            self.logger.error(f"Error calculating link reliability: {e}")
            return 0.0

    def get_connection_stability(self, edge_id: str) -> float:
        """Calculate connection stability based on vehicle movements"""
        try:
            vehicles = traci.edge.getLastStepVehicleIDs(edge_id)
            if not vehicles:
                return 1.0
            
            speeds = [traci.vehicle.getSpeed(v) for v in vehicles]
            directions = [traci.vehicle.getAngle(v) for v in vehicles]
            
            # Calculate stability based on speed and direction variations
            speed_var = np.var(speeds) if len(speeds) > 1 else 0
            dir_var = self._calculate_angular_variance(directions)
            
            stability = math.exp(-speed_var/100) * math.exp(-dir_var/90)
            return stability
        except Exception as e:
            self.logger.warning(f"Error calculating connection stability: {e}")
            return 1.0

    def calculate_vehicle_density(self, edge_id: str) -> float:
        """Calculate normalized vehicle density"""
        try:
            edge_length = self.net.getEdge(edge_id).getLength() / 1000  # km
            num_vehicles = traci.edge.getLastStepVehicleNumber(edge_id)
            density = num_vehicles / max(edge_length, 0.001)
            
            return min(density / self.optimal_density, 1.0)
        except Exception as e:
            self.logger.warning(f"Error calculating vehicle density: {e}")
            return 0.0

    def _calculate_average_distance(self, positions: List[Tuple[float, float]]) -> float:
        """Calculate average distance between vehicles"""
        if len(positions) < 2:
            return 1.0
        
        distances = []
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = math.sqrt(
                    (positions[i][0] - positions[j][0])**2 +
                    (positions[i][1] - positions[j][1])**2
                )
                distances.append(dist)
        
        return sum(distances) / len(distances)

    def _calculate_angular_variance(self, angles: List[float]) -> float:
        """Calculate variance of angular values"""
        if len(angles) < 2:
            return 0.0
        
        # Convert angles to radians and calculate circular variance
        angles_rad = np.radians(angles)
        sin_sum = sum(np.sin(angles_rad))
        cos_sum = sum(np.cos(angles_rad))
        R = math.sqrt(sin_sum**2 + cos_sum**2) / len(angles)
        
        return 1 - R

class VANETACO:
    def __init__(self, net: sumolib.net.Net, step_length: float = 0.1):
        self.net = net
        self.metrics = VANETMetrics(net)
        self.step_length = step_length
        
        # ACO parameters
        self.alpha = 1.0    # Pheromone influence
        self.beta = 2.0     # Distance influence
        self.gamma = 1.5    # Reliability influence
        self.delta = 1.0    # Density influence
        self.epsilon = 0.8  # Speed influence
        self.q0 = 0.9      # Exploitation vs exploration parameter
        
        # Initialize pheromone matrix and parameters
        self.pheromone_matrix = defaultdict(lambda: 1.0)
        self.min_pheromone = 0.1
        self.max_pheromone = 5.0
        self.evaporation_rate = 0.1
        
        # Setup logging
        self.logger = logging.getLogger("VANETACO")

    def select_route(self, start_edge_id: str, dest_edge_id: str) -> List[str]:
        """Select optimal route using VANET-ACO algorithm"""
        try:
            current_edge_id = start_edge_id
            route = [current_edge_id]
            
            while current_edge_id != dest_edge_id:
                next_edge_id = self._select_next_edge(current_edge_id, dest_edge_id)
                if not next_edge_id or next_edge_id in route:  # Avoid loops
                    break
                
                route.append(next_edge_id)
                current_edge_id = next_edge_id
                
                if len(route) > 100:  # Prevent infinite loops
                    self.logger.warning("Route length exceeded maximum limit")
                    break
            
            return route
        except Exception as e:
            self.logger.error(f"Error selecting route: {e}")
            return [start_edge_id]

    def _select_next_edge(self, current_edge_id: str, dest_edge_id: str) -> str:
        """Select next edge based on VANET metrics and pheromone levels"""
        try:
            current_edge = self.net.getEdge(current_edge_id)
            outgoing_edges = [conn.getTo() for conn in current_edge.getOutgoing()]
            
            if not outgoing_edges:
                return None
            
            # Calculate probabilities for each outgoing edge
            probabilities = {}
            total_prob = 0
            
            for edge in outgoing_edges:
                prob = self._calculate_edge_probability(
                    current_edge_id, 
                    edge.getID(), 
                    dest_edge_id
                )
                probabilities[edge.getID()] = prob
                total_prob += prob
            
            # Normalize probabilities
            if total_prob > 0:
                for edge_id in probabilities:
                    probabilities[edge_id] /= total_prob
            
            # Select next edge using pseudo-random proportional rule
            if random.random() < self.q0:
                # Exploitation: choose best edge
                return max(probabilities.items(), key=lambda x: x[1])[0]
            else:
                # Exploration: probabilistic selection
                return random.choices(
                    list(probabilities.keys()),
                    weights=probabilities.values()
                )[0]
        except Exception as e:
            self.logger.error(f"Error selecting next edge: {e}")
            return None

    def _calculate_edge_probability(self, current_id: str, next_id: str, dest_id: str) -> float:
        """Calculate transition probability for an edge"""
        try:
            # Get edge metrics
            pheromone = self.pheromone_matrix[next_id]
            distance = self.net.getEdge(next_id).getLength()
            reliability = self.metrics.calculate_link_reliability(next_id)
            density = self.metrics.calculate_vehicle_density(next_id)
            
            # Calculate heuristic information (inverse distance to destination)
            dest_distance = self._calculate_distance_to_destination(next_id, dest_id)
            heuristic = 1.0 / (dest_distance + 1.0)
            
            # Combined probability calculation
            probability = (
                (pheromone ** self.alpha) *
                (heuristic ** self.beta) *
                (reliability ** self.gamma) *
                (density ** self.delta)
            )
            
            return max(probability, 1e-10)
        except Exception as e:
            self.logger.error(f"Error calculating edge probability: {e}")
            return 0.0

    def update_pheromones(self, route: List[str], quality: float):
        """Update pheromone levels for the route"""
        try:
            # Calculate dynamic evaporation rate based on network conditions
            network_dynamics = self._calculate_network_dynamics(route)
            dynamic_evap_rate = self.evaporation_rate * (1 + 0.5 * network_dynamics)
            
            # Update pheromones for each edge in the route
            for edge_id in route:
                current_pheromone = self.pheromone_matrix[edge_id]
                
                # Apply evaporation
                new_pheromone = (1 - dynamic_evap_rate) * current_pheromone
                
                # Add new pheromone
                deposit = quality * (1 + self.metrics.calculate_link_reliability(edge_id))
                new_pheromone += deposit
                
                # Apply bounds
                self.pheromone_matrix[edge_id] = max(
                    min(new_pheromone, self.max_pheromone),
                    self.min_pheromone
                )
        except Exception as e:
            self.logger.error(f"Error updating pheromones: {e}")

    def calculate_route_quality(self, route: List[str]) -> float:
        """Calculate comprehensive route quality"""
        try:
            if not route:
                return 0.0
                
            # Calculate various metrics
            total_length = sum(self.net.getEdge(edge_id).getLength() for edge_id in route)
            avg_reliability = np.mean([
                self.metrics.calculate_link_reliability(edge_id)
                for edge_id in route
            ])
            avg_density = np.mean([
                self.metrics.calculate_vehicle_density(edge_id)
                for edge_id in route
            ])
            
            # Combined quality metric
            quality = 1.0 / (
                total_length * 
                (1 - avg_reliability) ** self.gamma * 
                (1 - avg_density) ** self.delta
            )
            
            return quality
        except Exception as e:
            self.logger.error(f"Error calculating route quality: {e}")
            return 0.0

    def _calculate_network_dynamics(self, route: List[str]) -> float:
        """Calculate network dynamics factor"""
        try:
            reliability_changes = []
            density_changes = []
            
            for edge_id in route:
                reliability = self.metrics.calculate_link_reliability(edge_id)
                density = self.metrics.calculate_vehicle_density(edge_id)
                reliability_changes.append(reliability)
                density_changes.append(density)
            
            # Calculate variability in network conditions
            rel_var = np.var(reliability_changes) if len(reliability_changes) > 1 else 0
            den_var = np.var(density_changes) if len(density_changes) > 1 else 0
            
            return (rel_var + den_var) / 2
        except Exception as e:
            self.logger.error(f"Error calculating network dynamics: {e}")
            return 0.0

    def _calculate_distance_to_destination(self, current_id: str, dest_id: str) -> float:
        """Calculate Euclidean distance to destination"""
        try:
            current_edge = self.net.getEdge(current_id)
            dest_edge = self.net.getEdge(dest_id)
            
            current_pos = current_edge.getFromNode().getCoord()
            dest_pos = dest_edge.getToNode().getCoord()
            
            return math.sqrt(
                (current_pos[0] - dest_pos[0])**2 +
                (current_pos[1] - dest_pos[1])**2
            )
        except Exception as e:
            self.logger.error(f"Error calculating distance to destination: {e}")
            return float('inf')

def main(net_file: str, num_vehicles: int, output_file: str):
    """Main function to generate routes using VANET-ACO"""
    try:
        # Initialize SUMO and load network
        if 'SUMO_HOME' not in os.environ:
            raise EnvironmentError("Please set SUMO_HOME environment variable")
            
        traci.start(["sumo", "-n", net_file])
        net = sumolib.net.readNet(net_file)
        
        # Initialize VANET-ACO
        aco = VANETACO(net)
        logging.info("Initialized VANET-ACO algorithm")
        
        # Generate routes for vehicles
        routes = []
        edges = net.getEdges()
        
        for i in range(num_vehicles):
            # Select random start and destination edges
            start_edge = random.choice(edges)
            dest_edge = random.choice(edges)
            
            # Generate route using VANET-ACO
            route = aco.select_route(start_edge.getID(), dest_edge.getID())
            
            # Calculate route quality and update pheromones
            quality = aco.calculate_route_quality(route)
            aco.update_pheromones(route, quality)
            
            routes.append(route)
            
            if i % 10 == 0:
                logging.info(f"Generated {i + 1}/{num_vehicles} routes")
        
        # Write routes to output file
        with open(output_file, 'w') as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            f.write('<routes>\n')
            
            for i, route in enumerate(routes):
                route_str = ' '.join(route)
                f.write(f'    <vehicle id="veh{i}" depart="0">\n')
                f.write(f'        <route edges="{route_str}"/>\n')
                f.write('    </vehicle>\n')
            
            f.write('</routes>\n')
        
        logging.info(f"Successfully wrote {num_vehicles} routes to {output_file}")
        
    except Exception as e:
        logging.error(f"Error in main function: {e}")
        raise
    finally:
        traci.close()

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python script.py <net_file> <num_vehicles> <output_file>")
        sys.exit(1)
        
    net_file = sys.argv[1]
    num_vehicles = int(sys.argv[2])
    output_file = sys.argv[3]
    
    main(net_file, num_vehicles, output_file)
