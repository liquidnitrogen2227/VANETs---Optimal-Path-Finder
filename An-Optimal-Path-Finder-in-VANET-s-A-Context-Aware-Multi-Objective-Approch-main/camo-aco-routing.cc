#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/energy-module.h"
#include "ns3/wifi-module.h"
#include "ns3/camo-aco-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CamoAcoContextRoutingExample");

// Custom function to simulate different network contexts
void SetupNetworkContexts(NodeContainer &nodes) {
    // Simulate different node characteristics
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        // Example context parameters
        Ptr<CamoAcoRouting> routing = 
            DynamicCast<CamoAcoRouting>(nodes.Get(i)->GetObject<Ipv4RoutingProtocol>());
        
        if (routing) {
            // Simulate different contexts:
            // 1. Node energy level
            // 2. Node mobility
            // 3. Traffic load
            double energyLevel = 1.0 - (i * 0.2); // Decreasing energy from node 0 to node 4
            double mobilitySpeed = i * 2.0; // Increasing mobility
            double trafficLoad = (i % 2 == 0) ? 0.5 : 0.8; // Alternating traffic loads

            // You would implement methods in your CamoAcoRouting class to set these contexts
            // routing->SetEnergyContext(energyLevel);
            // routing->SetMobilityContext(mobilitySpeed);
            // routing->SetTrafficContext(trafficLoad);
        }
    }
}

int main(int argc, char *argv[]) {
    // Enable logging
    LogComponentEnable("CamoAcoRouting", LOG_LEVEL_ALL);
    LogComponentEnable("CamoAcoContextRoutingExample", LOG_LEVEL_ALL);

    // Configuration parameters
    uint32_t nNodes = 5;
    double simulationTime = 20.0; // seconds
    std::string phyMode("OfdmRate6Mbps");

    // Parse command-line arguments
    CommandLine cmd;
    cmd.AddValue("nNodes", "Number of nodes", nNodes);
    cmd.AddValue("SimTime", "Simulation time", simulationTime);
    cmd.Parse(argc, argv);

    // Create nodes
    NodeContainer nodes;
    nodes.Create(nNodes);

    // Setup wireless network
    WifiHelper wifi;
    wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
    
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    // Wifi MAC
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    // Install wireless devices
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    // Install Internet stack with CAMO-ACO routing
    InternetStackHelper stack;
    CamoAcoHelper camoAcoRouting;
    stack.SetRoutingHelper(camoAcoRouting);
    stack.Install(nodes);

    // Assign IP addresses
    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // Setup mobility
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                  "X", StringValue("100.0"),
                                  "Y", StringValue("100.0"),
                                  "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=30]"));
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                              "Speed", StringValue("ns3::UniformRandomVariable[Min=0|Max=10]"),
                              "Pause", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"));
    mobility.Install(nodes);

    // Simulate different network contexts
    SetupNetworkContexts(nodes);

    // Create multi-objective traffic patterns
    // 1. Delay-sensitive traffic
    OnOffHelper delayTraffic("ns3::UdpSocketFactory", 
        InetSocketAddress(interfaces.GetAddress(nNodes-1), 9));
    delayTraffic.SetConstantRate(DataRate("200kbps"));
    delayTraffic.SetAttribute("PacketSize", UintegerValue(256));
    ApplicationContainer delayApps = delayTraffic.Install(nodes.Get(0));
    delayApps.Start(Seconds(1.0));
    delayApps.Stop(Seconds(simulationTime));

    // 2. Bandwidth-intensive traffic
    OnOffHelper bandwidthTraffic("ns3::UdpSocketFactory", 
        InetSocketAddress(interfaces.GetAddress(nNodes-1), 10));
    bandwidthTraffic.SetConstantRate(DataRate("500kbps"));
    bandwidthTraffic.SetAttribute("PacketSize", UintegerValue(1024));
    ApplicationContainer bandwidthApps = bandwidthTraffic.Install(nodes.Get(1));
    bandwidthApps.Start(Seconds(2.0));
    bandwidthApps.Stop(Seconds(simulationTime));

    // Packet sinks
    PacketSinkHelper sink1("ns3::UdpSocketFactory", 
        InetSocketAddress(Ipv4Address::GetAny(), 9));
    PacketSinkHelper sink2("ns3::UdpSocketFactory", 
        InetSocketAddress(Ipv4Address::GetAny(), 10));
    
    ApplicationContainer sinkApps;
    sinkApps.Add(sink1.Install(nodes.Get(nNodes-1)));
    sinkApps.Add(sink2.Install(nodes.Get(nNodes-1)));
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(simulationTime));

    // NetAnim configuration
    AnimationInterface anim("camo-aco-context-animation.xml");

    // Run simulation
    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();

    // Collect and print results
    // Here you would add methods to extract routing performance metrics
    // such as path optimality, context-awareness effectiveness, etc.

    Simulator::Destroy();

    return 0;
}
