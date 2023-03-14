#include "RelocateStar.h"

int RelocateStar::evaluate(Route *U,
                           Route *V,
                           PenaltyManager const &penaltyManager)
{
    move = {};

    for (auto *nodeU = n(U->depot); !nodeU->isDepot(); nodeU = n(nodeU))
    {
        // Test inserting U after V's depot
        int deltaCost = relocate.evaluate(nodeU, V->depot, penaltyManager);

        if (deltaCost < move.deltaCost)
            move = {deltaCost, nodeU, V->depot};

        for (auto *nodeV = n(V->depot); !nodeV->isDepot(); nodeV = n(nodeV))
        {
            // Test inserting U after V
            deltaCost = relocate.evaluate(nodeU, nodeV, penaltyManager);

            if (deltaCost < move.deltaCost)
                move = {deltaCost, nodeU, nodeV};

            // Test inserting V after U
            deltaCost = relocate.evaluate(nodeV, nodeU, penaltyManager);

            if (deltaCost < move.deltaCost)
                move = {deltaCost, nodeV, nodeU};
        }
    }

    return move.deltaCost;
}

void RelocateStar::apply(Route *U, Route *V) const
{
    move.from->insertAfter(move.to);
}
