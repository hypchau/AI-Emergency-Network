#ifndef __AIEN_GROUNDMOBILITY_H
#define __AIEN_GROUNDMOBILITY_H

#include "inet/mobility/base/StationaryMobilityBase.h"

namespace aien {

using namespace omnetpp;
using inet::Coord;

/**
 * Stationary node standing on the DEM. See GroundMobility.ned.
 */
class GroundMobility : public inet::StationaryMobilityBase
{
  protected:
    virtual void setInitialPosition() override;
};

} // namespace aien

#endif
