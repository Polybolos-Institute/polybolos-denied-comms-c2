#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace Polybolos {
namespace Fusion {

/// Bayesian network node (random variable).
struct BayesianNode {
    std::string id;
    std::vector<std::string> parents;
    std::vector<std::string> states;

    /// CPT: Conditional Probability Table.
    /// indexed as cpt[parent_combo][this_state] = probability
    std::map<std::string, std::vector<double>> cpt;
};

/// Directed acyclic graph for sensor dependencies.
class BayesianNetwork {
public:
    BayesianNetwork();

    // Build network
    void addNode(const BayesianNode& node);
    void addEdge(const std::string& from, const std::string& to);

    // Belief propagation
    void updateBelief(const std::string& node_id, const std::string& evidence);
    std::map<std::string, double> queryBelief(const std::string& node_id) const;

    // Inference algorithms
    void inferenceExact();           // Exact (tree-structured networks)
    void inferenceApproximate();     // Loopy belief propagation

    // MCMC inference for complex posteriors
    void inferenceMCMC(int iterations = 1000);

private:
    std::unordered_map<std::string, BayesianNode> nodes_;
    std::map<std::string, std::vector<std::string>> graph_;
    std::map<std::string, std::map<std::string, double>> beliefs_;
};

}  // namespace Fusion
}  // namespace Polybolos
