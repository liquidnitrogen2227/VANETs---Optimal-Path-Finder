#include "camo-aco-helper.h"
#include "ns3/camo-aco-routing.h"
#include "ns3/log.h"

namespace ns3 {

CamoAcoHelper::CamoAcoHelper() {
  NS_LOG_FUNCTION(this);
}

CamoAcoHelper::~CamoAcoHelper() {
  NS_LOG_FUNCTION(this);
}

Ptr<Ipv4RoutingProtocol> CamoAcoHelper::Create(Ptr<Node> node) const {
  NS_LOG_FUNCTION(this << node);
  return CreateObject<CamoAcoRouting>();
}

} // namespace ns3

