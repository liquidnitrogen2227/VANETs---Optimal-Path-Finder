#ifndef __VEINS_CAMOAROUTING_H_
#define __VEINS_CAMOAROUTING_H_

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include <map>
#include <vector>
#include <string>
#include <list>   // Added for std::list

using namespace veins;

class CAMOACORouting : public DemoBaseApplLayer {
public:
    virtual void initialize(int stage) override;
    virtual void handlePositionUpdate(cObject* obj) override;
    virtual void handleLowerMsg(cMessage* msg) override;

protected:
    // Route segment definition
    struct RouteSegment {
        std::string edgeId;
        double pheromone;
        double traffic;
    };

    // CAMO-ACO parameters
    double evaporationRate;
    double alpha; // pheromone importance
    double beta;  // heuristic importance
    int numAnts;
    
    // Helper functions
    bool needsRerouting();
    double calculateLocalDensity();
    double calculatePathQuality(const std::string& current, const std::string& next);
    std::string selectNextEdge(const std::vector<RouteSegment>& availableEdges);
    
    // Data structures
    std::map<std::string, std::vector<RouteSegment>> roadNetwork;
    std::map<std::string, double> pheromoneMatrix;
    std::map<std::string, double> trafficDensity;
    
    // Beacon handling
    void onBeacon(DemoSafetyMessage* wsm);
    void updateTrafficDensity(const std::string& edge, double density);
    void updatePheromones();
    
    // Route calculation
    std::vector<std::string> findOptimalPath(std::string start, std::string destination);
    
    // TraCI interface
    TraCIMobility* mobility;
    TraCICommandInterface* traci;
    TraCICommandInterface::Vehicle* traciVehicle;
    
    // Timer messages
    cMessage* updateTimer;
    
private:
    static simsignal_t routeUpdateSignal;
};

#endif // __VEINS_CAMOAROUTING_H_
