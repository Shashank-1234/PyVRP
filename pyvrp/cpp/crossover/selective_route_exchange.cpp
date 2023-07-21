#include "crossover.h"

#include "DynamicBitset.h"

#include <cmath>

using Client = int;
using Clients = std::vector<Client>;
using Route = pyvrp::Solution::Route;
using Routes = std::vector<Route>;

// TODO get rid of these?
using DynamicBitset = pyvrp::DynamicBitset;
using CostEvaluator = pyvrp::CostEvaluator;

namespace
{
// Angle of the given route w.r.t. the centroid of all client locations.
double routeAngle(pyvrp::ProblemData const &data, Route const &route)
{
    // This computes a pseudo-angle that sorts roughly equivalently to the atan2
    // angle, but is much faster to compute. See the following post for details:
    // https://stackoverflow.com/a/16561333/4316405.
    auto const [dataX, dataY] = data.centroid();
    auto const [routeX, routeY] = route.centroid();
    auto const dx = routeX - dataX;
    auto const dy = routeY - dataY;
    return std::copysign(1. - dx / (std::fabs(dx) + std::fabs(dy)), dy);
}

Routes sortByAscAngle(pyvrp::ProblemData const &data,
                      Routes routes,
                      bool perVehicleType = false)
{
    // Sort the routes by ascending angle, optionally first sort by vehicle
    // type (sort by angle per vehicle type)
    auto cmp = [&data, perVehicleType](Route a, Route b) {
        // First sort by vehicle type
        if (perVehicleType && a.vehicleType() != b.vehicleType())
            return a.vehicleType() < b.vehicleType();
        return routeAngle(data, a) < routeAngle(data, b);
    };

    std::sort(routes.begin(), routes.end(), cmp);
    return routes;
}
}  // namespace

bool isValidAssignment_(pyvrp::ProblemData const &data,
                        std::vector<size_t> const &usedVehicles)
{
    for (size_t typeIdx = 0; typeIdx < data.numVehicleTypes(); typeIdx++)
        if (usedVehicles[typeIdx] > data.vehicleType(typeIdx).numAvailable)
            return false;
    return true;
}

std::vector<size_t> assignVehicles_(pyvrp::ProblemData const &data,
                                    Routes const &routesA,
                                    Routes const &routesB,
                                    std::vector<int> const &exchanges,
                                    std::vector<Clients> const &visits)
{
    // Simple heuristic to assign vehicles to routes in offspring. In principle
    // each route keeps its original type, but we reassign some route types if
    // doing so exceeds the numAvailable for a route type.

    auto const nRoutes = routesA.size();
    auto assignments = std::vector<size_t>(nRoutes, 0);

    // If we don't have more than one vehicle type, we're done
    if (data.numVehicleTypes() <= 1)
        return assignments;

    // Simply assign all to their new vehicle types, if this works we're done
    std::vector<size_t> used = std::vector<size_t>(data.numVehicleTypes(), 0);
    for (size_t r = 0; r < nRoutes; r++)
        if (!visits[r].empty())
        {
            assignments[r] = exchanges[r] < 0
                                 ? routesA[r].vehicleType()
                                 : routesB[exchanges[r]].vehicleType();
            used[assignments[r]]++;
        }

    // Small optimization, check if we were succesful
    if (isValidAssignment_(data, used))
        return assignments;

    // If not, loop once more and reassign vehicles that exceed their
    // availability by greedily re-assigning to a type that is still available
    size_t curType = 0;
    for (size_t r = 0; r < nRoutes; r++)
        if (!visits[r].empty())
        {
            auto const vehType = assignments[r];
            if (vehType != routesA[r].vehicleType()
                && used[vehType] > data.vehicleType(vehType).numAvailable)
            {
                // Find first one that we can use
                while (used[curType] >= data.vehicleType(curType).numAvailable)
                    curType++;
                // Reassign to curType
                assignments[r] = curType;
                used[vehType]--;
                used[curType]++;
            }
        }

    return assignments;
}

pyvrp::Solution routeExchange_(pyvrp::ProblemData const &data,
                               Routes const &routesA,
                               Routes const &routesB,
                               CostEvaluator const &costEvaluator,
                               std::vector<int> const &exchanges,
                               DynamicBitset const &selectedA,
                               DynamicBitset const &selectedB)
{
    // Exchanges is a vector where exchanges[i] = j means that route i
    // from parent A is exchanged with route j from parent B unless j = -1

    // We create two candidate offsprings, both based on parent A:
    // Let A and B denote the set of customers selected from parents A and B
    // Ac and Bc denote the complements: the customers not selected
    // Let v denote union and ^ intersection
    // Parent A: A v Ac
    // Parent B: B v Bc

    // Offspring 1:
    // B and Ac\B, remainder A\B unplanned
    // (note B v (Ac\B) v (A\B) = B v ((Ac v A)\B) = B v Bc = all)
    // Note Ac\B = (A v B)c

    // Offspring 2:
    // A^B and Ac, remainder A\B unplanned
    // (note A^B v Ac v A\B = (A^B v A\B) v Ac = A v Ac = all)
    auto const nRoutes = routesA.size();

    // Identify differences between route sets
    auto const selectedBNotA = selectedB & ~selectedA;

    std::vector<Clients> visits1(nRoutes);
    std::vector<Clients> visits2(nRoutes);

    // Loop over routesA and exchange or keep each route
    for (size_t r = 0; r < nRoutes; r++)
    {
        if (exchanges[r] >= 0)
        {
            // Replace selected routes from parent A with routes from parent B
            for (Client c : routesB[exchanges[r]])
            {
                visits1[r].push_back(c);  // c in B

                if (!selectedBNotA[c])
                    visits2[r].push_back(c);  // c in A^B
            }
        }
        else
        {
            // Move routes from parent A that are kept
            for (Client c : routesA[r])
            {
                if (!selectedBNotA[c])
                    visits1[r].push_back(c);  // c in Ac\B

                visits2[r].push_back(c);  // c in Ac
            }
        }
    }

    // Insert unplanned clients (those that were in the removed routes of A, but
    // not the inserted routes of B).
    auto const unplanned = selectedA & ~selectedB;
    pyvrp::crossover::greedyRepair(visits1, unplanned, data, costEvaluator);
    pyvrp::crossover::greedyRepair(visits2, unplanned, data, costEvaluator);

    // Assign correct types to routes (from parents) and filter empty
    std::vector<pyvrp::Solution::Route> routes1;
    routes1.reserve(nRoutes);
    std::vector<pyvrp::Solution::Route> routes2;
    routes2.reserve(nRoutes);

    // Vehicles get assigned the types of the original routes, but after
    // exchange we may exceed numAvailable so we need to take some care
    auto const vehicleTypes1
        = assignVehicles_(data, routesA, routesB, exchanges, visits1);
    auto const vehicleTypes2
        = assignVehicles_(data, routesA, routesB, exchanges, visits2);
    for (size_t r = 0; r < nRoutes; r++)
    {
        if (!visits1[r].empty())
            routes1.emplace_back(data, visits1[r], vehicleTypes1[r]);

        if (!visits2[r].empty())
            routes2.emplace_back(data, visits2[r], vehicleTypes2[r]);
    }

    pyvrp::Solution sol1{data, routes1};
    pyvrp::Solution sol2{data, routes2};

    auto const cost1 = costEvaluator.penalisedCost(sol1);
    auto const cost2 = costEvaluator.penalisedCost(sol2);
    return cost1 < cost2 ? sol1 : sol2;
}

pyvrp::Solution routeExchange_(pyvrp::ProblemData const &data,
                               Routes const &routesA,
                               Routes const &routesB,
                               CostEvaluator const &costEvaluator,
                               std::vector<int> const &exchanges)
{
    DynamicBitset selectedA(data.numClients() + 1);
    DynamicBitset selectedB(data.numClients() + 1);

    // Determine clients in routes involved in exchange
    for (size_t r = 0; r < routesA.size(); r++)
        if (exchanges[r] >= 0)
        {
            // auto const &routeA = routesA[r];
            // selectedA.insert(routeA.begin(), routeA.end());

            // auto const &routeB = routesB[exchanges[r]];
            // selectedB.insert(routeB.begin(), routeB.end());
        }
    return routeExchange_(
        data, routesA, routesB, costEvaluator, exchanges, selectedA, selectedB);
}

std::pair<size_t, size_t> optimizeStartIndices_(Routes const &routesA,
                                                Routes const &routesB,
                                                size_t startA,
                                                size_t startB,
                                                size_t numMovedRoutes,
                                                DynamicBitset &selectedA,
                                                DynamicBitset &selectedB)
{
    size_t nRoutesA = routesA.size();
    size_t nRoutesB = routesB.size();

    if (startA >= nRoutesA)
        throw std::invalid_argument("Expected startA < nRoutes.");

    if (startB >= nRoutesB)
        throw std::invalid_argument("Expected startB < nRoutesB.");

    if (numMovedRoutes < 1 || numMovedRoutes > std::min(nRoutesA, nRoutesB))
    {
        auto msg = "Expected numMovedRoutes in [1, min(nRoutes, nRoutesB)]";
        throw std::invalid_argument(msg);
    }

    // Routes are sorted on polar angle, so selecting adjacent routes in both
    // parents should result in a large overlap when the start indices are
    // close to each other.
    for (size_t r = 0; r < numMovedRoutes; r++)
    {
        for (Client c : routesA[(startA + r) % nRoutesA])
            selectedA[c] = true;

        for (Client c : routesB[(startB + r) % nRoutesB])
            selectedB[c] = true;
    }

    // For the selection, we want to minimize |A\B| as these need replanning
    while (true)
    {
        // Difference for moving 'left' in parent A
        int differenceALeft = 0;

        for (Client c : routesA[(startA - 1 + nRoutesA) % nRoutesA])
            differenceALeft += !selectedB[c];

        for (Client c : routesA[(startA + numMovedRoutes - 1) % nRoutesA])
            differenceALeft -= !selectedB[c];

        // Difference for moving 'right' in parent A
        int differenceARight = 0;

        for (Client c : routesA[(startA + numMovedRoutes) % nRoutesA])
            differenceARight += !selectedB[c];

        for (Client c : routesA[startA])
            differenceARight -= !selectedB[c];

        // Difference for moving 'left' in parent B
        int differenceBLeft = 0;

        for (Client c : routesB[(startB - 1 + numMovedRoutes) % nRoutesB])
            differenceBLeft += selectedA[c];

        for (Client c : routesB[(startB - 1 + nRoutesB) % nRoutesB])
            differenceBLeft -= selectedA[c];

        // Difference for moving 'right' in parent B
        int differenceBRight = 0;

        for (Client c : routesB[startB])
            differenceBRight += selectedA[c];

        for (Client c : routesB[(startB + numMovedRoutes) % nRoutesB])
            differenceBRight -= selectedA[c];

        int const bestDifference = std::min({differenceALeft,
                                             differenceARight,
                                             differenceBLeft,
                                             differenceBRight});

        if (bestDifference >= 0)  // there are no further improving moves
            break;

        if (bestDifference == differenceALeft)
        {
            for (Client c : routesA[(startA + numMovedRoutes - 1) % nRoutesA])
                selectedA[c] = false;

            startA = (startA - 1 + nRoutesA) % nRoutesA;
            for (Client c : routesA[startA])
                selectedA[c] = true;
        }
        else if (bestDifference == differenceARight)
        {
            for (Client c : routesA[startA])
                selectedA[c] = false;

            startA = (startA + 1) % nRoutesA;
            for (Client c : routesA[(startA + numMovedRoutes - 1) % nRoutesA])
                selectedA[c] = true;
        }
        else if (bestDifference == differenceBLeft)
        {
            for (Client c : routesB[(startB + numMovedRoutes - 1) % nRoutesB])
                selectedB[c] = false;

            startB = (startB - 1 + nRoutesB) % nRoutesB;
            for (Client c : routesB[startB])
                selectedB[c] = true;
        }
        else if (bestDifference == differenceBRight)
        {
            for (Client c : routesB[startB])
                selectedB[c] = false;

            startB = (startB + 1) % nRoutesB;
            for (Client c : routesB[(startB + numMovedRoutes - 1) % nRoutesB])
                selectedB[c] = true;
        }
    }
    return std::make_pair(startA, startB);
}

pyvrp::Solution heterogeneousSelectiveRouteExchange(
    std::pair<pyvrp::Solution const *, pyvrp::Solution const *> const &parents,
    pyvrp::ProblemData const &data,
    CostEvaluator const &costEvaluator,
    std::vector<size_t> const startIndicesPerVehicleType,
    std::vector<size_t> const numMovedRoutesPerVehicleType)
{
    // Heterogeneous version of selective route exchange. Routes will be
    // grouped by vehicle type and the exchange of routes will happen per
    // vehicle type. The repair phase is done globally after all routes of all
    // types have been exchanged.
    auto const routesA = sortByAscAngle(data, parents.first->getRoutes(), true);
    auto const routesB
        = sortByAscAngle(data, parents.second->getRoutes(), true);

    DynamicBitset selectedA(data.numClients() + 1);
    ;
    DynamicBitset selectedB(data.numClients() + 1);
    ;
    // // Vector to keep track of which routes to exchange, initialize at -1.
    std::vector<int> exchanges(routesA.size(), -1);

    // SortByAscAngle first sorts by vehicle type, so we can loop over the
    // vehicle types and find the ranges of routes for that vehicle type in
    // the sorted routes for both parents by advancing pointers.

    auto beginA = routesA.begin(), beginB = routesB.begin();

    for (size_t vehType = 0; vehType < data.numVehicleTypes(); vehType++)
    {
        // Find range of routes with this vehicle type for first parent
        while (beginA != routesA.end() && beginA->vehicleType() < vehType)
            beginA++;
        auto endA = beginA;
        while (endA != routesA.end() && endA->vehicleType() <= vehType)
            endA++;

        // Find range of routes with this vehicle type for second parent
        while (beginB != routesB.end() && beginB->vehicleType() < vehType)
            beginB++;
        auto endB = beginB;
        while (endB != routesB.end() && endB->vehicleType() <= vehType)
            endB++;

        auto numMovedRoutes = numMovedRoutesPerVehicleType[vehType];

        if (numMovedRoutes > 0)
        {
            // Find optimal start indices
            std::vector<Route> typeRoutesA(beginA, endA);
            std::vector<Route> typeRoutesB(beginB, endB);

            if (typeRoutesA.size() < numMovedRoutes
                || typeRoutesB.size() < numMovedRoutes)
            {
                auto msg = "Expected numMovedRoutes in [0, min(nRoutesA, "
                           "nRoutesB)] for each vehicle type.";
                throw std::invalid_argument(msg);
            }
            auto startA = startIndicesPerVehicleType[vehType];
            auto startB = startA < typeRoutesB.size() ? startA : 0;

            std::tie(startA, startB) = optimizeStartIndices_(typeRoutesA,
                                                             typeRoutesB,
                                                             startA,
                                                             startB,
                                                             numMovedRoutes,
                                                             selectedA,
                                                             selectedB);

            // Fill the exchanges vector
            auto const offsetA = std::distance(routesA.begin(), beginA);
            auto const offsetB = std::distance(routesB.begin(), beginB);
            for (size_t r = 0; r < numMovedRoutes; r++)
            {
                size_t indexA = (startA + r) % typeRoutesA.size();
                size_t indexB = (startB + r) % typeRoutesB.size();
                exchanges[indexA + offsetA] = indexB + offsetB;
            }
        }

        beginA = endA;
        beginB = endB;
    }

    return routeExchange_(
        data, routesA, routesB, costEvaluator, exchanges, selectedA, selectedB);
}

pyvrp::Solution selectiveRouteExchange(
    std::pair<pyvrp::Solution const *, pyvrp::Solution const *> const &parents,
    pyvrp::ProblemData const &data,
    CostEvaluator const &costEvaluator,
    std::pair<size_t, size_t> const startIndices,
    size_t const numMovedRoutes)
{
    // Standard version of selective route exchange. The vehicle type will not
    // be considered in the exchange, and the resulting routes get assigned the
    // type of the route in the first parent, even if it is exchanged with a
    // route of a different type (otherwise satisfying the number available
    // per vehicle type is not guaranteed).
    auto const routesA = sortByAscAngle(data, parents.first->getRoutes());
    auto const routesB = sortByAscAngle(data, parents.second->getRoutes());
    auto const nRoutes = routesA.size();
    auto const nRoutesB = routesB.size();

    DynamicBitset selectedA(data.numClients() + 1);
    DynamicBitset selectedB(data.numClients() + 1);
    auto [startA, startB] = optimizeStartIndices_(routesA,
                                                  routesB,
                                                  startIndices.first,
                                                  startIndices.second,
                                                  numMovedRoutes,
                                                  selectedA,
                                                  selectedB);

    // // Vector to keep track of which routes to exchange, initialize at -1.
    std::vector<int> exchanges(nRoutes, -1);
    for (size_t r = 0; r < numMovedRoutes; r++)
        exchanges[(startA + r) % nRoutes] = (startB + r) % nRoutesB;

    return routeExchange_(data, routesA, routesB, costEvaluator, exchanges);
}

pyvrp::Solution routeExchange(
    std::pair<pyvrp::Solution const *, pyvrp::Solution const *> const &parents,
    pyvrp::ProblemData const &data,
    CostEvaluator const &costEvaluator,
    std::vector<int> const &exchanges)
{
    // Performs route exchange given an externally provided set of exchanges.
    // The vehicle type will not be considered in the exchange, and the
    // resulting routes get assigned the type of the route in the first parent,
    // even if it is exchanged with a route of a different type (otherwise
    // satisfying the number available per vehicle type is not guaranteed).
    if (exchanges.size() != parents.first->numRoutes())
        throw std::invalid_argument(
            "Expected exchanges to have length parentA.numRoutes().");
    auto const routesA = parents.first->getRoutes();
    auto const routesB = parents.second->getRoutes();
    return routeExchange_(data, routesA, routesB, costEvaluator, exchanges);
}
