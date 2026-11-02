#include "search/neighbourhood.h"

#include "Matrix.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>

using pyvrp::Matrix;
using pyvrp::ProblemData;
using pyvrp::search::NeighbourhoodParams;

namespace
{

Matrix<double> computeProximity(ProblemData const &data,
                                NeighbourhoodParams const &params)
{
    Matrix<double> prox(data.numClients(),
                        data.numClients(),
                        std::numeric_limits<double>::max());

    std::set<std::tuple<pyvrp::Cost, pyvrp::Cost, size_t>> seen = {};
    for (auto const &vehType : data.vehicleTypes())
    {
        auto const key = std::make_tuple(vehType.unitDistanceCost,
                                         vehType.unitDurationCost,
                                         vehType.profile);

        if (seen.contains(key))
            continue;

        seen.insert(key);
        auto const &dists = data.distanceMatrix(vehType.profile);
        auto const &durs = data.durationMatrix(vehType.profile);

        for (size_t frm = 0; frm != data.numClients(); ++frm)
        {
            auto const &frmData = data.client(frm);
            auto const frmServ = static_cast<double>(frmData.serviceDuration);
            auto const frmEarly = static_cast<double>(frmData.twEarly);
            auto const frmLate = static_cast<double>(frmData.twLate);

            for (size_t to = 0; to != data.numClients(); ++to)
            {
                auto const &toData = data.client(to);
                auto const toEarly = static_cast<double>(toData.twEarly);
                auto const toLate = static_cast<double>(toData.twLate);

                auto const dur = durs(frmData.location, toData.location);
                auto const edgeDur = static_cast<double>(dur);

                if (frmEarly + frmServ + edgeDur > toLate)
                    continue;

                auto const dist = dists(frmData.location, toData.location);
                auto const distance = static_cast<double>(dist);

                auto const minWait = toEarly - edgeDur - frmServ - frmLate;
                auto const duration = edgeDur + std::max(minWait, 0.0);

                auto const cost
                    = static_cast<double>(vehType.unitDistanceCost) * distance
                      + static_cast<double>(vehType.unitDurationCost) * duration
                      + params.weightWaitTime * std::max(minWait, 0.0);

                prox(frm, to) = std::min(cost, prox(frm, to));
            }
        }
    }

    return prox;
}
}

NeighbourhoodParams::NeighbourhoodParams(double weightWaitTime,
                                         size_t numNeighbours,
                                         bool symmetricProximity)
    : weightWaitTime(weightWaitTime),
      numNeighbours(numNeighbours),
      symmetricProximity(symmetricProximity)
{
    if (numNeighbours == 0)
        throw std::invalid_argument("num_neighbours == 0 not understood.");
}

std::vector<std::vector<size_t>>
pyvrp::search::computeNeighbours(ProblemData const &data,
                                 NeighbourhoodParams const &params)
{
    auto prox = computeProximity(data, params);

    if (params.symmetricProximity)
        for (size_t frm = 0; frm != data.numClients(); ++frm)
            for (size_t to = frm; to != data.numClients(); ++to)
                prox(frm, to) = prox(to, frm)
                    = std::min(prox(frm, to), prox(to, frm));

    for (auto const &group : data.groups())
        for (auto const frmClient : group)
            for (auto const toClient : group)

                prox(frmClient, toClient) = std::numeric_limits<double>::max();

    for (size_t idx = 0; idx != data.numClients(); ++idx)
        prox(idx, idx) = std::numeric_limits<double>::infinity();

    size_t const numClients = std::max<size_t>(data.numClients(), 1);
    size_t const numNeighbours = std::min(params.numNeighbours, numClients - 1);

    std::vector<std::vector<size_t>> neighbours(data.numClients());

    std::vector<size_t> indices(data.numClients());
    for (size_t client = 0; client != data.numClients(); ++client)
    {
        auto const comp = [&](auto const a, auto const b)
        { return prox(client, a) < prox(client, b); };

        std::iota(indices.begin(), indices.end(), 0);
        std::stable_sort(indices.begin(), indices.end(), comp);

        neighbours[client] = {indices.begin(), indices.begin() + numNeighbours};
    }

    return neighbours;
}
