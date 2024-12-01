#ifndef CAMO_ACO_HELPER_H
#define CAMO_ACO_HELPER_H

#include "ns3/ipv4-routing-helper.h"
#include "ns3/camo-aco-routing.h"

namespace ns3 {

class CamoAcoHelper : public Ipv4RoutingHelper {
public:
  CamoAcoHelper();
  virtual ~CamoAcoHelper();

  virtual Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const override;
};

} // namespace ns3

#endif // CAMO_ACO_HELPER_H

