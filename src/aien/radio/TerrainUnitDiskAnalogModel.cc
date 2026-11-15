#include "TerrainUnitDiskAnalogModel.h"

#include <cmath>

#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IArrival.h"
#include "inet/physicallayer/wireless/unitdisk/UnitDiskReception.h"
#include "inet/physicallayer/wireless/unitdisk/UnitDiskTransmission.h"

#include "../terrain/TerrainModel.h"

namespace aien {

Define_Module(TerrainUnitDiskAnalogModel);

using namespace inet;
using namespace inet::physicallayer;

void TerrainUnitDiskAnalogModel::initialize()
{
    terrain = dynamic_cast<TerrainModel *>(findModuleByPath(par("terrainModule")));
    phonePrefix = par("phoneNodePrefix").stdstringValue();
    phoneRange = par("phoneRange").doubleValueInUnit("m");
    useLineOfSight = par("useLineOfSight");
    cacheGrid = par("losCacheGrid").doubleValueInUnit("m");
}

bool TerrainUnitDiskAnalogModel::isPhone(const cModule *radio) const
{
    cModule *node = findContainingNode(radio);
    return node != nullptr && strncmp(node->getName(), phonePrefix.c_str(), phonePrefix.size()) == 0;
}

bool TerrainUnitDiskAnalogModel::lineOfSight(const Coord& a, const Coord& b) const
{
    if (!useLineOfSight || terrain == nullptr || !terrain->hasDem())
        return true;
    auto q = [&](double v) { return (long)std::floor(v / cacheGrid); };
    Key k1(q(a.x), q(a.y), q(a.z), q(b.x), q(b.y), q(b.z));
    auto it = losCache.find(k1);
    if (it != losCache.end())
        return it->second;
    bool los = terrain->hasLineOfSight(a, b);
    Key k2(q(b.x), q(b.y), q(b.z), q(a.x), q(a.y), q(a.z));
    if (losCache.size() > 2000000)
        losCache.clear();
    losCache[k1] = los;
    losCache[k2] = los;
    return los;
}

const IReception *TerrainUnitDiskAnalogModel::computeReception(const IRadio *receiverRadio, const ITransmission *transmission, const IArrival *arrival) const
{
    auto unitDisk = check_and_cast<const UnitDiskTransmission *>(transmission);
    const Coord& from = transmission->getStartPosition();
    const Coord& to = arrival->getStartPosition();
    double distance = from.distance(to);

    double communicationRange = unitDisk->getCommunicationRange().get();
    auto receiverModule = check_and_cast<const cModule *>(receiverRadio);
    auto transmitterModule = dynamic_cast<const cModule *>(transmission->getTransmitter());
    if (transmitterModule == nullptr)
        transmitterModule = getSimulation()->getModule(transmission->getTransmitterId());
    if (isPhone(receiverModule) || (transmitterModule != nullptr && isPhone(transmitterModule)))
        communicationRange = std::min(communicationRange, phoneRange);

    UnitDiskReception::Power power;
    if (distance > unitDisk->getDetectionRange().get() && distance > communicationRange)
        power = UnitDiskReception::POWER_UNDETECTABLE;
    else if (!lineOfSight(from, to))
        power = UnitDiskReception::POWER_UNDETECTABLE;   // hidden by the relief
    else if (distance <= communicationRange)
        power = UnitDiskReception::POWER_RECEIVABLE;
    else if (distance <= unitDisk->getInterferenceRange().get())
        power = UnitDiskReception::POWER_INTERFERING;
    else
        power = UnitDiskReception::POWER_DETECTABLE;
    return new UnitDiskReception(receiverRadio, transmission, arrival->getStartTime(), arrival->getEndTime(),
            arrival->getStartPosition(), arrival->getEndPosition(), arrival->getStartOrientation(),
            arrival->getEndOrientation(), power);
}

} // namespace aien
