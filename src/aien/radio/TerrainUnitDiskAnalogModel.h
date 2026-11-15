#ifndef __AIEN_TERRAINUNITDISKANALOGMODEL_H
#define __AIEN_TERRAINUNITDISKANALOGMODEL_H

#include <map>
#include <tuple>

#include "inet/physicallayer/wireless/unitdisk/UnitDiskAnalogModel.h"

namespace aien {

using namespace omnetpp;
class TerrainModel;

/**
 * Unit-disk analog model with phone range limitation and relief blocking.
 * See TerrainUnitDiskAnalogModel.ned.
 */
class TerrainUnitDiskAnalogModel : public inet::physicallayer::UnitDiskAnalogModel
{
  protected:
    TerrainModel *terrain = nullptr;
    std::string phonePrefix;
    double phoneRange = 2000;
    bool useLineOfSight = true;
    double cacheGrid = 25;
    typedef std::tuple<long, long, long, long, long, long> Key;
    mutable std::map<Key, bool> losCache;

  protected:
    virtual void initialize() override;
    bool isPhone(const cModule *radio) const;
    bool lineOfSight(const inet::Coord& a, const inet::Coord& b) const;

  public:
    virtual const inet::physicallayer::IReception *computeReception(const inet::physicallayer::IRadio *radio,
            const inet::physicallayer::ITransmission *transmission, const inet::physicallayer::IArrival *arrival) const override;
};

} // namespace aien

#endif
