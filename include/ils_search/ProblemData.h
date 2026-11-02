#include <optional>

#ifndef PYVRP_PROBLEMDATA_H
#define PYVRP_PROBLEMDATA_H

#include "Client.h"
#include "ClientGroup.h"
#include "Depot.h"
#include "Location.h"
#include "Matrix.h"
#include "Measure.h"
#include "VehicleType.h"

#include <cassert>
#include <vector>

namespace pyvrp
{

class ProblemData
{
    std::vector<Matrix<Distance>> const dists_;
    std::vector<Matrix<Duration>> const durs_;
    std::vector<Location> const locations_;
    std::vector<Client> const clients_;
    std::vector<Depot> const depots_;
    std::vector<VehicleType> const vehicleTypes_;
    std::vector<ClientGroup> const groups_;

    size_t const numVehicles_;
    size_t const numLoadDimensions_;
    bool const hasTimeWindows_;

    void validate() const;

public:
    bool operator==(ProblemData const &other) const = default;

    [[nodiscard]] std::vector<Location> const &locations() const;

    [[nodiscard]] std::vector<Client> const &clients() const;

    [[nodiscard]] std::vector<Depot> const &depots() const;

    [[nodiscard]] std::vector<ClientGroup> const &groups() const;

    [[nodiscard]] std::vector<VehicleType> const &vehicleTypes() const;

    [[nodiscard]] std::vector<Matrix<Distance>> const &distanceMatrices() const;

    [[nodiscard]] std::vector<Matrix<Duration>> const &durationMatrices() const;

    [[nodiscard]] Location const &location(size_t location) const;

    [[nodiscard]] inline Client const &client(size_t client) const;

    [[nodiscard]] inline Depot const &depot(size_t depot) const;

    [[nodiscard]] ClientGroup const &group(size_t group) const;

    [[nodiscard]] VehicleType const &vehicleType(size_t vehicleType) const;

    [[nodiscard]] inline Matrix<Distance> const &
    distanceMatrix(size_t profile) const;

    [[nodiscard]] inline Matrix<Duration> const &
    durationMatrix(size_t profile) const;

    [[nodiscard]] inline bool hasTimeWindows() const;

    [[nodiscard]] size_t numClients() const;

    [[nodiscard]] size_t numDepots() const;

    [[nodiscard]] size_t numGroups() const;

    [[nodiscard]] size_t numLocations() const;

    [[nodiscard]] size_t numVehicleTypes() const;

    [[nodiscard]] size_t numVehicles() const;

    [[nodiscard]] size_t numLoadDimensions() const;

    ProblemData replace(std::optional<std::vector<Location>> &locations,
                        std::optional<std::vector<Client>> &clients,
                        std::optional<std::vector<Depot>> &depots,
                        std::optional<std::vector<VehicleType>> &vehicleTypes,
                        std::optional<std::vector<Matrix<Distance>>> &distMats,
                        std::optional<std::vector<Matrix<Duration>>> &durMats,
                        std::optional<std::vector<ClientGroup>> &groups) const;

    ProblemData(std::vector<Location> locations,
                std::vector<Client> clients,
                std::vector<Depot> depots,
                std::vector<VehicleType> vehicleTypes,
                std::vector<Matrix<Distance>> distMats,
                std::vector<Matrix<Duration>> durMats,
                std::vector<ClientGroup> groups = {});

    ProblemData() = delete;
};

Client const &ProblemData::client(size_t client) const
{
    assert(client < numClients());
    return clients_[client];
}

Depot const &ProblemData::depot(size_t depot) const
{
    assert(depot < numDepots());
    return depots_[depot];
}

Matrix<Distance> const &ProblemData::distanceMatrix(size_t profile) const
{
    assert(profile < dists_.size());
    return dists_[profile];
}

Matrix<Duration> const &ProblemData::durationMatrix(size_t profile) const
{
    assert(profile < durs_.size());
    return durs_[profile];
}

bool ProblemData::hasTimeWindows() const { return hasTimeWindows_; }
}

#endif
