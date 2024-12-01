#ifndef CAMO_ACO_ROUTING_H
#define CAMO_ACO_ROUTING_H

#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4.h"
#include "ns3/net-device.h"
#include "ns3/ptr.h"
#include "ns3/socket.h"
#include <map>
#include <vector>

namespace ns3 {

class CamoAcoRouting : public Ipv4RoutingProtocol {
public:
  static TypeId GetTypeId(void);
  CamoAcoRouting();
  virtual ~CamoAcoRouting();

  // Overridden methods from Ipv4RoutingProtocol
  virtual Ptr<Ipv4Route> RouteOutput(
      Ptr<Packet> packet, const Ipv4Header &header, Ptr<NetDevice> oif, 
      Socket::SocketErrno &sockerr) override;

  virtual bool RouteInput(
      Ptr<const Packet> packet, const Ipv4Header &header, Ptr<const NetDevice> idev, 
      UnicastForwardCallback ucb, MulticastForwardCallback mcb, 
      LocalDeliverCallback lcb, ErrorCallback ecb) override;

  virtual void NotifyInterfaceUp(uint32_t interface) override;
  virtual void NotifyInterfaceDown(uint32_t interface) override;
  virtual void NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address) override;
  virtual void NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address) override;
  virtual void SetIpv4(Ptr<Ipv4> ipv4) override;

  // CAMO-ACO specific methods
  void UpdatePheromoneLevels();
  void CalculateHeuristics();

private:
  void MonitorNetworkConditions();
  void ScheduleUpdate();

  Ptr<Ipv4> m_ipv4;
  std::map<Ipv4Address, double> m_pheromoneTable;  // Pheromone levels
  std::map<Ipv4Address, double> m_heuristicTable;  // Heuristic values
};

} // namespace ns3

#endif // CAMO_ACO_ROUTING_H

