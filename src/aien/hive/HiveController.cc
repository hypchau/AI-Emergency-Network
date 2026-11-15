#include "HiveController.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include "inet/common/geometry/common/CanvasProjection.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadio.h"

#include "../drone/DroneBattery.h"
#include "../mobility/DroneMobility.h"
#include "../terrain/TerrainModel.h"

namespace aien {

Define_Module(HiveController);

simsignal_t HiveController::coverageSignal = cComponent::registerSignal("coverage");
simsignal_t HiveController::coveredUsersSignal = cComponent::registerSignal("coveredUsers");
simsignal_t HiveController::dronesAirborneSignal = cComponent::registerSignal("dronesAirborne");
simsignal_t HiveController::dronesOnStationSignal = cComponent::registerSignal("dronesOnStation");
simsignal_t HiveController::dronesConnectedSignal = cComponent::registerSignal("dronesConnected");
simsignal_t HiveController::postsHeldSignal = cComponent::registerSignal("postsHeld");
simsignal_t HiveController::batterySwapsSignal = cComponent::registerSignal("batterySwaps");

const char *HiveController::stateName(DroneState s)
{
    switch (s) {
        case READY: return "PRET";
        case OUTBOUND: return "ALLER";
        case ON_STATION: return "POSTE";
        case RETURNING: return "RETOUR";
        case SWAPPING: return "BATTERIE";
        case FAILED: return "PANNE";
    }
    return "?";
}

HiveController::~HiveController()
{
    cancelAndDelete(tickTimer);
    cancelAndDelete(launchTimer);
    cancelAndDelete(failureTimer);
}

static double horizontalDistance(const Coord& a, const Coord& b)
{
    double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

void HiveController::initialize(int stage)
{
    if (stage == inet::INITSTAGE_LOCAL) {
        terrain = check_and_cast<TerrainModel *>(getModuleByPath(par("terrainModule")));
        tickInterval = par("tickInterval");
        startTime = par("startTime");
        launchInterval = par("launchInterval");
        accessRange = par("accessRange").doubleValueInUnit("m");
        meshRange = par("meshRange").doubleValueInUnit("m");
        motherRange = par("motherRange").doubleValueInUnit("m");
        motherHeight = par("motherHeight").doubleValueInUnit("m");
        useLineOfSight = par("useLineOfSight");
        reserveRatio = par("reserveRatio");
        handoverMargin = par("handoverMargin").doubleValueInUnit("s");
        batterySwapTime = par("batterySwapTime").doubleValueInUnit("s");
        fullChargeTime = par("fullChargeTime").doubleValueInUnit("s");
        minStationTime = par("minStationTime").doubleValueInUnit("s");
        gridStep = par("gridStep").doubleValueInUnit("m");
        planningMargin = par("planningMargin").doubleValueInUnit("m");
        minPostSpacing = par("minPostSpacing").doubleValueInUnit("m");
        maxPosts = par("maxPosts");
        failureTime = par("failureTime");
        tickTimer = new cMessage("tick");
        launchTimer = new cMessage("launch");
        failureTimer = new cMessage("failure");
        WATCH(numSwaps);
        WATCH(numFailures);
        WATCH(numGaps);
        WATCH(numLaunches);
        WATCH(lastCoverage);
        WATCH(plannedCoverage);
    }
    else if (stage == inet::INITSTAGE_LAST) {
        collectNodes();
        planPosts();
        pruneUselessBridges();
        drawPlan();
        lastTick = simTime();
        scheduleAt(simTime() + tickInterval, tickTimer);
        if (!posts.empty())
            scheduleAt(simTime() + startTime, launchTimer);
        if (failureTime >= SIMTIME_ZERO)
            scheduleAt(failureTime, failureTimer);
    }
}

void HiveController::collectNodes()
{
    cModule *network = getSystemModule();
    truck = getModuleByPath(par("truckModule"));
    auto truckMobility = check_and_cast<inet::IMobility *>(truck->getSubmodule("mobility"));
    motherPosition = terrain->aboveScenePosition(truckMobility->getCurrentPosition(), motherHeight);

    const char *droneName = par("droneModule");
    int numDrones = network->hasSubmoduleVector(droneName) ? network->getSubmoduleVectorSize(droneName) : 0;
    for (int i = 0; i < numDrones; i++) {
        Drone d;
        d.node = network->getSubmodule(droneName, i);
        d.mobility = check_and_cast<DroneMobility *>(d.node->getSubmodule("mobility"));
        d.battery = check_and_cast<DroneBattery *>(d.node->getSubmodule("battery"));
        drones.push_back(d);
    }
    if (drones.empty())
        throw cRuntimeError("No relay drone in the network: set numDrones >= 1");

    const char *userName = par("userModule");
    int numUsers = network->hasSubmoduleVector(userName) ? network->getSubmoduleVectorSize(userName) : 0;
    for (int i = 0; i < numUsers; i++) {
        cModule *u = network->getSubmodule(userName, i);
        users.push_back(u);
        userPositions.push_back(check_and_cast<inet::IMobility *>(u->getSubmodule("mobility"))->getCurrentPosition());
    }
    userCovered.assign(users.size(), false);
    userCoverable.assign(users.size(), false);

    int spares = par("sparePacks");
    if (spares < 0)
        spares = 2 * (int)drones.size();
    packReadyAt.assign(spares, SIMTIME_ZERO);
}

double HiveController::airbornePower() const
{
    return drones[0].battery->getAirbornePower();
}

double HiveController::usableEndurance() const
{
    return drones[0].battery->getFullEndurance() * (1 - reserveRatio);
}

bool HiveController::linked(const Coord& a, const Coord& b, double range) const
{
    if (a.distance(b) > range)
        return false;
    return !useLineOfSight || terrain->hasLineOfSight(a, b);
}

void HiveController::planPosts()
{
    struct Candidate {
        Coord position;
        double outboundTime, returnTime, stationTime;
        int needed;
        std::vector<int> users;
        int parent = -2; // -2 not linked, -1 mother, >= 0 post index
        bool used = false;
    };

    DroneMobility *mob = drones[0].mobility;
    double flightHeight = mob->getFlightHeight();
    double usable = usableEndurance();
    EV_INFO << "Planning: full-pack endurance " << drones[0].battery->getFullEndurance() / 60 << " min, usable "
            << usable / 60 << " min (reserve " << reserveRatio * 100 << " %)" << endl;

    // phones covered by the mother antenna
    for (size_t u = 0; u < users.size(); u++)
        if (linked(motherPosition, userPositions[u], motherRange))
            userCovered[u] = userCoverable[u] = true;

    // candidate grid over the area of interest
    double minX = motherPosition.x, maxX = motherPosition.x, minY = motherPosition.y, maxY = motherPosition.y;
    for (auto& p : userPositions) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    minX -= planningMargin; maxX += planningMargin; minY -= planningMargin; maxY += planningMargin;
    std::vector<Candidate> candidates;
    int unreachable = 0;
    for (double x = minX; x <= maxX; x += gridStep) {
        for (double y = minY; y <= maxY; y += gridStep) {
            Candidate c;
            c.position = terrain->aboveScenePosition(Coord(x, y, 0), flightHeight);
            c.outboundTime = mob->estimateOutboundTime(c.position);
            c.returnTime = mob->estimateReturnTime(c.position);
            c.stationTime = usable - c.outboundTime - c.returnTime - handoverMargin;
            if (c.stationTime < minStationTime) {
                unreachable++;
                continue;
            }
            c.needed = (int)std::ceil((usable + batterySwapTime) / c.stationTime);
            for (size_t u = 0; u < users.size(); u++)
                if (linked(c.position, userPositions[u], accessRange)) {
                    c.users.push_back(u);
                    userCoverable[u] = true;
                }
            if (linked(motherPosition, c.position, meshRange))
                c.parent = -1;
            candidates.push_back(c);
        }
    }
    int coverable = std::count(userCoverable.begin(), userCoverable.end(), true);
    EV_INFO << "Planning: " << candidates.size() << " reachable candidate posts (" << unreachable
            << " too far for the battery), " << coverable << "/" << users.size() << " phones coverable at all" << endl;

    int budget = drones.size();
    int used = 0;
    auto tooClose = [&](const Candidate& c) {
        if (horizontalDistance(c.position, motherPosition) < minPostSpacing)
            return true;
        for (auto& p : posts)
            if (horizontalDistance(c.position, p.position) < minPostSpacing)
                return true;
        return false;
    };
    auto uncoveredTargets = [&]() {
        std::vector<int> v;
        for (size_t u = 0; u < users.size(); u++)
            if (!userCovered[u] && userCoverable[u])
                v.push_back(u);
        return v;
    };

    while ((int)posts.size() < maxPosts) {
        auto targets = uncoveredTargets();
        if (targets.empty())
            break;
        int best = -1;
        double bestScore = -std::numeric_limits<double>::infinity();
        bool bridge = false;
        // 1) best phones-per-drone ratio among linked, affordable candidates
        for (size_t i = 0; i < candidates.size(); i++) {
            auto& c = candidates[i];
            if (c.used || c.parent == -2 || tooClose(c))
                continue;
            if (!posts.empty() && used + c.needed > budget)
                continue;
            int gain = 0;
            for (int u : c.users)
                if (!userCovered[u])
                    gain++;
            if (gain == 0)
                continue;
            double score = (double)gain / c.needed - 1e-3 * c.needed;
            if (score > bestScore) {
                bestScore = score;
                best = i;
            }
        }
        // 2) otherwise a bridge towards the nearest uncovered phone
        if (best == -1) {
            auto distanceToTargets = [&](const Coord& p) {
                double d = std::numeric_limits<double>::infinity();
                for (int u : targets)
                    d = std::min(d, horizontalDistance(p, userPositions[u]));
                return d;
            };
            double current = distanceToTargets(motherPosition);
            for (auto& p : posts)
                current = std::min(current, distanceToTargets(p.position));
            double bestDistance = current - gridStep; // a bridge must bring us closer
            for (size_t i = 0; i < candidates.size(); i++) {
                auto& c = candidates[i];
                if (c.used || c.parent == -2 || tooClose(c))
                    continue;
                // a bridge is only useful if the fleet can also hold a post beyond it,
                // which is farther from the hive, hence at least as expensive
                if (used + 2 * c.needed > budget)
                    continue;
                double d = distanceToTargets(c.position);
                if (d < bestDistance) {
                    bestDistance = d;
                    best = i;
                }
            }
            bridge = true;
        }
        if (best == -1)
            break;

        auto& c = candidates[best];
        c.used = true;
        Post p;
        p.id = posts.size() + 1;
        p.position = c.position;
        p.outboundTime = c.outboundTime;
        p.returnTime = c.returnTime;
        p.stationTime = c.stationTime;
        p.dronesNeeded = c.needed;
        p.parent = c.parent;
        p.bridge = bridge;
        for (int u : c.users)
            if (!userCovered[u]) {
                p.users.push_back(u);
                userCovered[u] = true;
            }
        posts.push_back(p);
        used += c.needed;
        int newIndex = posts.size() - 1;
        // candidates that can now link to the mesh through the new post
        for (auto& o : candidates)
            if (o.parent == -2 && linked(p.position, o.position, meshRange))
                o.parent = newIndex;
        EV_INFO << "Planning: post P" << p.id << (bridge ? " (bridge)" : "") << " at " << p.position
                << ", " << horizontalDistance(p.position, motherPosition) / 1000 << " km from the hive, "
                << p.users.size() << " new phones, " << p.stationTime / 60 << " min on post per sortie, "
                << p.dronesNeeded << " drones to hold it" << endl;
        if (used >= budget)
            break;
    }

    int planned = std::count(userCovered.begin(), userCovered.end(), true);
    plannedCoverage = users.empty() ? 0 : (double)planned / users.size();
    EV_INFO << "Planning: " << posts.size() << " posts, " << used << " drones needed for continuous cover ("
            << budget << " available), planned coverage " << planned << "/" << users.size() << endl;
    if (used > budget)
        EV_WARN << "Not enough drones to hold the first post continuously: expect coverage gaps" << endl;
    std::fill(userCovered.begin(), userCovered.end(), false);
}

void HiveController::pruneUselessBridges()
{
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = (int)posts.size() - 1; i >= 0; i--) {
            if (!posts[i].bridge)
                continue;
            bool hasChild = false;
            for (auto& p : posts)
                if (p.parent == i)
                    hasChild = true;
            if (hasChild)
                continue;
            EV_INFO << "Planning: removing bridge P" << posts[i].id << " (it leads to no phone)" << endl;
            posts.erase(posts.begin() + i);
            for (auto& p : posts)
                if (p.parent > i)
                    p.parent--;
            changed = true;
            break;
        }
    }
    for (size_t i = 0; i < posts.size(); i++)
        posts[i].id = i + 1;
}

void HiveController::drawPlan()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    auto projection = inet::CanvasProjection::getCanvasProjection(canvas);
    planLayer = new cGroupFigure("hivePlan");
    planLayer->setZIndex(-1);
    canvas->addFigure(planLayer);

    auto circle = [&](const Coord& center, double radius, const cFigure::Color& color, const char *name) {
        auto c = projection->computeCanvasPoint(center);
        auto e = projection->computeCanvasPoint(center + Coord(radius, 0, 0));
        double r = std::sqrt((e.x - c.x) * (e.x - c.x) + (e.y - c.y) * (e.y - c.y));
        auto oval = new cOvalFigure(name);
        oval->setBounds(cFigure::Rectangle(c.x - r, c.y - r, 2 * r, 2 * r));
        oval->setLineColor(color);
        oval->setLineWidth(2);
        oval->setLineStyle(cFigure::LINE_DASHED);
        oval->setFilled(true);
        oval->setFillColor(color);
        oval->setFillOpacity(0.06);
        oval->setZoomLineWidth(false);
        planLayer->addFigure(oval);
    };
    circle(motherPosition, motherRange, cFigure::Color(230, 120, 0), "motherRange");
    for (auto& p : posts) {
        circle(p.position, accessRange, cFigure::Color(30, 110, 200), "postRange");
        auto c = projection->computeCanvasPoint(p.position);
        Coord parentPos = p.parent < 0 ? motherPosition : posts[p.parent].position;
        auto line = new cLineFigure("meshLink");
        line->setStart(projection->computeCanvasPoint(parentPos));
        line->setEnd(c);
        line->setLineColor(cFigure::Color(30, 110, 200));
        line->setLineStyle(cFigure::LINE_DOTTED);
        line->setLineWidth(2);
        line->setZoomLineWidth(false);
        planLayer->addFigure(line);
        auto icon = new cIconFigure("post");
        icon->setImageName("aien/post");
        icon->setPosition(c);
        icon->setAnchor(cFigure::ANCHOR_CENTER);
        planLayer->addFigure(icon);
        auto label = new cLabelFigure("postLabel");
        char text[64];
        snprintf(text, sizeof(text), "P%d%s  %d drone%s", p.id, p.bridge ? " relais" : "", p.dronesNeeded,
                 p.dronesNeeded > 1 ? "s" : "");
        label->setText(text);
        label->setPosition(c + cFigure::Point(0, 14));
        label->setAnchor(cFigure::ANCHOR_N);
        label->setColor(cFigure::Color(20, 60, 120));
        planLayer->addFigure(label);
    }
    auto status = new cTextFigure("hiveStatus");
    status->setText("");
    status->setPosition(cFigure::Point(20, 20));
    status->setAnchor(cFigure::ANCHOR_NW);
    status->setFont(cFigure::Font("", 16, cFigure::FONT_BOLD));
    status->setColor(cFigure::Color(20, 20, 20));
    planLayer->addFigure(status);
}

void HiveController::handleMessage(cMessage *msg)
{
    if (msg == tickTimer) {
        tick();
        scheduleAt(simTime() + tickInterval, tickTimer);
    }
    else if (msg == launchTimer)
        launchNextInitial();
    else if (msg == failureTimer)
        injectFailure();
    else
        throw cRuntimeError("Unexpected message %s", msg->getName());
}

void HiveController::launchNextInitial()
{
    if (nextInitialLaunch >= posts.size())
        return;
    auto& p = posts[nextInitialLaunch];
    if (p.active == -1 && p.inbound == -1 && !launch(nextInitialLaunch)) {
        // no drone ready (tiny fleet): retry later
        scheduleAt(simTime() + launchInterval, launchTimer);
        return;
    }
    nextInitialLaunch++;
    if (nextInitialLaunch < posts.size())
        scheduleAt(simTime() + launchInterval, launchTimer);
}

int HiveController::pickReadyDrone() const
{
    for (size_t i = 0; i < drones.size(); i++)
        if (drones[i].state == READY)
            return i;
    return -1;
}

bool HiveController::launch(int postIndex)
{
    int i = pickReadyDrone();
    if (i == -1)
        return false;
    auto& d = drones[i];
    auto& p = posts[postIndex];
    d.battery->update(); // account the ground time before taking off
    d.mobility->flyTo(p.position);
    d.state = OUTBOUND;
    d.post = postIndex;
    d.sorties++;
    p.inbound = i;
    numLaunches++;
    EV_INFO << d.node->getFullName() << " takes off for P" << p.id << " (battery "
            << 100 * d.battery->getResidualRatio() << " %)" << endl;
    return true;
}

void HiveController::startSwap(int i)
{
    auto& d = drones[i];
    double removed = d.battery->getResidualRatio();
    simtime_t now = simTime();
    if (!packReadyAt.empty()) {
        auto k = std::min_element(packReadyAt.begin(), packReadyAt.end()) - packReadyAt.begin();
        simtime_t start = std::max(now, packReadyAt[k]);
        d.swapDoneAt = start + batterySwapTime;
        packReadyAt[k] = d.swapDoneAt + fullChargeTime * (1 - removed); // the removed pack goes on the charger
    }
    else
        d.swapDoneAt = now + fullChargeTime * (1 - removed); // no spare: recharge in place
    d.state = SWAPPING;
    EV_INFO << d.node->getFullName() << " landed with " << 100 * removed << " %, ready again at "
            << d.swapDoneAt << endl;
}

void HiveController::updateDrone(int i)
{
    auto& d = drones[i];
    if (d.state == FAILED)
        return;
    double energy = d.battery->update();
    auto phase = d.mobility->getPhase();
    bool airborne = phase != DroneMobility::PARKED && phase != DroneMobility::CRASHED;

    if (airborne && d.battery->isEmpty()) {
        failDrone(i, "battery empty in flight");
        return;
    }

    switch (d.state) {
        case READY:
        case FAILED:
            break;
        case OUTBOUND:
            if (phase == DroneMobility::HOVER) {
                auto& p = posts[d.post];
                d.state = ON_STATION;
                p.inbound = -1;
                if (p.active >= 0 && p.active != i) {
                    auto& old = drones[p.active];
                    old.mobility->returnHome();
                    old.state = RETURNING;
                    EV_INFO << "Handover on P" << p.id << ": " << d.node->getFullName() << " relieves "
                            << old.node->getFullName() << endl;
                }
                p.active = i;
            }
            break;
        case ON_STATION: {
            auto& p = posts[d.post];
            double power = airbornePower();
            double reserve = reserveRatio * d.battery->getCapacity();
            double back = d.mobility->estimateReturnTime(d.mobility->getCurrentPosition()) * power;
            double timeLeft = (energy - reserve - back) / power;
            if (timeLeft <= 0) {
                d.mobility->returnHome();
                d.state = RETURNING;
                if (p.active == i)
                    p.active = -1;
                numGaps++;
                EV_WARN << d.node->getFullName() << " must leave P" << p.id << " before being relieved: coverage gap" << endl;
            }
            else if (p.inbound == -1 && timeLeft <= p.outboundTime + handoverMargin)
                launch(d.post);
            break;
        }
        case RETURNING:
            if (phase == DroneMobility::PARKED)
                startSwap(i);
            break;
        case SWAPPING:
            if (simTime() >= d.swapDoneAt) {
                d.battery->swap(1.0);
                d.state = READY;
                d.post = -1;
                numSwaps++;
                emit(batterySwapsSignal, numSwaps);
            }
            break;
    }
}

void HiveController::injectFailure()
{
    std::vector<int> candidates;
    for (size_t i = 0; i < drones.size(); i++)
        if (drones[i].state == ON_STATION)
            candidates.push_back(i);
    if (candidates.empty()) {
        EV_WARN << "Failure injection: no drone on post" << endl;
        return;
    }
    failDrone(candidates[intuniform(0, candidates.size() - 1)], "injected failure");
}

void HiveController::failDrone(int i, const char *reason)
{
    auto& d = drones[i];
    d.mobility->crash();
    if (auto radio = dynamic_cast<inet::physicallayer::IRadio *>(d.node->findModuleByPath(".wlan[0].radio")))
        radio->setRadioMode(inet::physicallayer::IRadio::RADIO_MODE_OFF);
    if (d.post >= 0) {
        auto& p = posts[d.post];
        if (p.active == i) p.active = -1;
        if (p.inbound == i) p.inbound = -1;
    }
    d.state = FAILED;
    numFailures++;
    bubble("Panne drone !");
    EV_WARN << d.node->getFullName() << " FAILED (" << reason << "): its post will be refilled from the hive" << endl;
}

void HiveController::tick()
{
    for (size_t i = 0; i < drones.size(); i++)
        updateDrone(i);
    // refill posts that are empty (after a failure or a gap), once the initial wave has been sent
    for (size_t k = 0; k < posts.size() && k < nextInitialLaunch; k++)
        if (posts[k].active == -1 && posts[k].inbound == -1)
            launch(k);
    evaluateCoverage();
}

void HiveController::evaluateCoverage()
{
    // airborne drones form a mesh rooted at the mother antenna
    std::vector<Coord> nodes{motherPosition};
    std::vector<int> nodeDrone{-1};
    int airborne = 0, onStation = 0;
    for (size_t i = 0; i < drones.size(); i++) {
        if (drones[i].state == FAILED)
            continue;
        if (drones[i].state == ON_STATION)
            onStation++;
        if (drones[i].mobility->isAirborne()) {
            airborne++;
            nodes.push_back(drones[i].mobility->getCurrentPosition());
            nodeDrone.push_back(i);
        }
    }
    std::vector<bool> connected(nodes.size(), false);
    std::deque<int> queue{0};
    connected[0] = true;
    while (!queue.empty()) {
        int a = queue.front();
        queue.pop_front();
        for (size_t b = 1; b < nodes.size(); b++)
            if (!connected[b] && linked(nodes[a], nodes[b], meshRange)) {
                connected[b] = true;
                queue.push_back(b);
            }
    }
    int numConnected = std::count(connected.begin() + 1, connected.end(), true);

    int covered = 0;
    for (size_t u = 0; u < users.size(); u++) {
        bool ok = linked(motherPosition, userPositions[u], motherRange);
        for (size_t k = 1; !ok && k < nodes.size(); k++)
            ok = connected[k] && linked(nodes[k], userPositions[u], accessRange);
        userCovered[u] = ok;
        covered += ok;
    }
    double coverage = users.empty() ? 0 : (double)covered / users.size();

    simtime_t now = simTime();
    coverageIntegral += lastCoverage * (now - lastTick).dbl();
    lastTick = now;
    lastCoverage = coverage;
    lastConnected = numConnected;
    if (timeTo50 < SIMTIME_ZERO && coverage >= 0.5) timeTo50 = now;
    if (timeTo90 < SIMTIME_ZERO && coverage >= 0.9) timeTo90 = now;
    if (timeToPlanned < SIMTIME_ZERO && plannedCoverage > 0 && coverage >= plannedCoverage - 1e-9) timeToPlanned = now;

    int held = 0;
    for (auto& p : posts)
        held += p.active >= 0;
    emit(coverageSignal, coverage);
    emit(coveredUsersSignal, (long)covered);
    emit(dronesAirborneSignal, (long)airborne);
    emit(dronesOnStationSignal, (long)onStation);
    emit(dronesConnectedSignal, (long)numConnected);
    emit(postsHeldSignal, (long)held);
}

std::string HiveController::droneLabel(int i) const
{
    auto& d = drones[i];
    char buf[64];
    double pct = 100.0 * d.battery->peekResidualRatio();
    if (d.post >= 0 && d.state != READY)
        snprintf(buf, sizeof(buf), "%s P%d %.0f%%", stateName(d.state), posts[d.post].id, pct);
    else
        snprintf(buf, sizeof(buf), "%s %.0f%%", stateName(d.state), pct);
    return buf;
}

void HiveController::refreshDisplay() const
{
    static const char *icons[] = {"aien/drone_parked", "aien/drone_outbound", "aien/drone_onstation",
                                  "aien/drone_returning", "aien/drone_swapping", "aien/drone_failed"};
    for (size_t i = 0; i < drones.size(); i++) {
        auto& ds = drones[i].node->getDisplayString();
        ds.setTagArg("i", 0, icons[drones[i].state]);
        ds.setTagArg("t", 0, droneLabel(i).c_str());
        ds.setTagArg("t", 1, "t");
    }
    for (size_t u = 0; u < users.size(); u++)
        users[u]->getDisplayString().setTagArg("i", 0, userCovered[u] ? "aien/victim_ok" : "aien/victim_sos");

    int charged = 0;
    for (auto t : packReadyAt)
        charged += t <= simTime();
    int held = 0;
    for (auto& p : posts)
        held += p.active >= 0;
    char text[256];
    snprintf(text, sizeof(text), "Couverture %.0f %%  |  postes tenus %d/%d  |  drones relies %d/%d  |  "
             "batteries chargees %d/%zu  |  echanges %ld  |  pannes %ld",
             100 * lastCoverage, held, (int)posts.size(), lastConnected, (int)drones.size(), charged,
             packReadyAt.size(), numSwaps, numFailures);
    truck->getDisplayString().setTagArg("t", 0, text);
    truck->getDisplayString().setTagArg("t", 1, "t");
    if (planLayer)
        if (auto status = dynamic_cast<cTextFigure *>(planLayer->getFigure("hiveStatus")))
            status->setText(text);
    getDisplayString().setTagArg("t", 0, str().c_str()); // shown above the truck by the info visualizer (2D and 3D)
}

std::string HiveController::str() const
{
    char buf[128];
    snprintf(buf, sizeof(buf), "Couverture %.0f %% | drones relies %d/%zu", 100 * lastCoverage, lastConnected, drones.size());
    return buf;
}

void HiveController::finish()
{
    coverageIntegral += lastCoverage * (simTime() - lastTick).dbl();
    recordScalar("numDrones", drones.size());
    recordScalar("numUsers", users.size());
    recordScalar("numPosts", posts.size());
    recordScalar("plannedCoverage", plannedCoverage);
    recordScalar("meanCoverage", simTime() > SIMTIME_ZERO ? coverageIntegral / simTime().dbl() : 0);
    recordScalar("timeTo50pctCoverage", timeTo50 < SIMTIME_ZERO ? -1 : timeTo50.dbl(), "s");
    recordScalar("timeTo90pctCoverage", timeTo90 < SIMTIME_ZERO ? -1 : timeTo90.dbl(), "s");
    recordScalar("timeToPlannedCoverage", timeToPlanned < SIMTIME_ZERO ? -1 : timeToPlanned.dbl(), "s");
    recordScalar("numLaunches", numLaunches);
    recordScalar("numBatterySwaps", numSwaps);
    recordScalar("numCoverageGaps", numGaps);
    recordScalar("numFailures", numFailures);
    recordScalar("usableEndurance", usableEndurance(), "s");
}

} // namespace aien
