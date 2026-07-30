#ifndef GRAPH_CLASS_HPP
#define GRAPH_CLASS_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <set>
#include <iterator>
#include <Eigen/Dense>

class Graph {
public:
    // 构造函数，接受 Eigen::MatrixXd 类型的邻接矩阵
    Graph(const Eigen::MatrixXd& adjacency_matrix)
        : adjacency_matrix(adjacency_matrix), n(adjacency_matrix.rows()) {}

    int compute_max_k_core() {
        std::vector<int> degree(n);
        for (int i = 0; i < n; ++i) {
            degree[i] = static_cast<int>(adjacency_matrix.row(i).sum());
        }

        int max_k = 0;

        while (true) {
            bool changed = false;
            for (int v = 0; v < n; ++v) {
                if (degree[v] < max_k) {
                    changed = true;
                    degree[v] = -1; // 标记为已删除
                    // 减少邻居的度数
                    for (int u = 0; u < n; ++u) {
                        if (adjacency_matrix(v, u) == 1 && degree[u] != -1) {
                            --degree[u];
                        }
                    }
                }
            }
            if (!changed) break;

            ++max_k;
        }

        return max_k - 1; // 返回找到的最大 K 值
    }

    std::vector<int> get_k_core_nodes(int k) {
        std::vector<int> degree(n);
        for (int i = 0; i < n; ++i) {
            degree[i] = static_cast<int>(adjacency_matrix.row(i).sum());
        }
        std::vector<int> k_core_nodes;

        for (int v = 0; v < n; ++v) {
            if (degree[v] >= k) {
                k_core_nodes.push_back(v);
            }
        }

        return k_core_nodes;
    }

    bool is_clique(const std::vector<int>& vertices) {
        for (size_t i = 0; i < vertices.size(); ++i) {
            for (size_t j = i + 1; j < vertices.size(); ++j) {
                if (adjacency_matrix(vertices[i], vertices[j]) == 0) {
                    return false;
                }
            }
        }
        return true;
    }

    std::vector<int> find_max_clique_in_k_core() {
        int max_k = compute_max_k_core();

        std::vector<int> k_core_nodes = get_k_core_nodes(max_k);
        std::vector<int> max_clique;

        int size = k_core_nodes.size();

        // 检查所有k_core_nodes的组合
        for (int r = 1; r <= size; ++r) {
            std::vector<bool> v(size);
            std::fill(v.begin(), v.begin() + r, true);

            do {
                std::vector<int> combo;
                for (int i = 0; i < size; ++i) {
                    if (v[i]) {
                        combo.push_back(k_core_nodes[i]);
                    }
                }
                if (is_clique(combo) && combo.size() > max_clique.size()) {
                    max_clique = combo;
                }
            } while (std::prev_permutation(v.begin(), v.end()));
        }

        return max_clique;
    }

private:
    Eigen::MatrixXd adjacency_matrix; // 使用 Eigen 的 MatrixXd 来表示邻接矩阵
    int n;
};

#endif