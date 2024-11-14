#include "CAMOACORouting.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include <vector>
#include <map>
#include <string>
#include <cmath>

Define_Module(CAMOACORouting);

simsignal_t CAMOACORouting::routeUpdateSignal = registerSignal("routeUpdate");

// Forward declarations of helper functions
bool CAMOACORouting::needsRerouting() {
    // Implement rerouting logic here
    return false; // placeholder
}

double CAMOACORouting::calculateLocalDensity() {
    // Calculate density of traffic around the current vehicle
    return 1.0; // Placeholder value
}

double CAMOACORouting::calculatePathQuality(const std::string& current, const std::string& next) {
    double pheromone = pheromoneMatrix[current];
    double traffic = trafficDensity[current];
    return pheromone / (1.0 + traffic);
}

std::string CAMOACORouting::selectNextEdge(const std::vector<RouteSegment>& availableEdges) {
    if (availableEdges.empty()) {
        return "";
    }

    // Calculate selection probabilities based on pheromone and heuristic
    std::vector<double> probabilities;
    double totalProbability = 0.0;

    for (const auto& segment : availableEdges) {
        double pheromone = pheromoneMatrix[segment.edgeId];
        double heuristic = 1.0 / (1.0 + trafficDensity[segment.edgeId]);
        double probability = pow(pheromone, alpha) * pow(heuristic, beta);
        probabilities.push_back(probability);
        totalProbability += probability;
    }

    // Select edge based on probabilities
    double random = uniform(0, totalProbability);
    double sum = 0.0;

    for (size_t i = 0; i < availableEdges.size(); i++) {
        sum += probabilities[i];
        if (sum >= random) {
            return availableEdges[i].edgeId;
        }
    }

    return availableEdges[0].edgeId;
}

void CAMOACORouting::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    
    if (stage == 0) {
        // Initialize parameters
        evaporationRate = par("evaporationRate").doubleValue();
        alpha = par("alpha").doubleValue();
        beta = par("beta").doubleValue();
        numAnts = par("numAnts").intValue();
        
        // Get TraCI interfaces
        mobility = TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        
        // Initialize update timer
        updateTimer = new cMessage("updateTimer");
        scheduleAt(simTime() + 1, updateTimer);
    }
}

void CAMOACORouting::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);
    
    // Update current position and check if rerouting is needed
    std::string currentEdge = mobility->getRoadId();
    if (needsRerouting()) {
        std::string destination = traciVehicle->getPlannedRoadIds().back();
        std::vector<std::string> newRouteVector = findOptimalPath(currentEdge, destination);
        
        if (!newRouteVector.empty()) {
            // Convert vector to list for TraCI compatibility
            std::list<std::string> newRoute(newRouteVector.begin(), newRouteVector.end());
            traciVehicle->changeVehicleRoute(newRoute);
            emit(routeUpdateSignal, 1);
        }
    }
}

void CAMOACORouting::handleLowerMsg(cMessage* msg) {
    DemoSafetyMessage* wsm = dynamic_cast<DemoSafetyMessage*>(msg);
    if (wsm) {
        onBeacon(wsm);
    }
    delete msg;
}

void CAMOACORouting::onBeacon(DemoSafetyMessage* wsm) {
    // Extract traffic information from beacon
    std::string edge = mobility->getRoadId();
    updateTrafficDensity(edge, calculateLocalDensity());
}

void CAMOACORouting::updateTrafficDensity(const std::string& edge, double density) {
    trafficDensity[edge] = density;
}

std::vector<std::string> CAMOACORouting::findOptimalPath(std::string start, std::string destination) {
    std::vector<std::string> bestPath;
    double bestQuality = -1;
    
    // Run ant colony optimization
    for (int i = 0; i < numAnts; i++) {
        std::vector<std::string> currentPath;
        std::string current = start;
        double pathQuality = 0;
        
        while (current != destination) {
            if (!roadNetwork.count(current) || roadNetwork[current].empty()) {
                break;
            }

            std::string next = selectNextEdge(roadNetwork[current]);
            if (next.empty()) {
                break;
            }
            
            currentPath.push_back(next);
            pathQuality += calculatePathQuality(current, next);
            current = next;
        }
        
        if (pathQuality > bestQuality && current == destination) {
            bestQuality = pathQuality;
            bestPath = currentPath;
        }
    }
    
    updatePheromones();
    return bestPath;
}

void CAMOACORouting::updatePheromones() {
    // Evaporation
    for (auto& entry : pheromoneMatrix) {
        entry.second *= (1.0 - evaporationRate);
    }
    
    // Update based on current traffic conditions
    for (const auto& edge : roadNetwork) {
        double traffic = trafficDensity[edge.first];
        double quality = 1.0 / (1.0 + traffic);
        pheromoneMatrix[edge.first] += quality;
    }
}
