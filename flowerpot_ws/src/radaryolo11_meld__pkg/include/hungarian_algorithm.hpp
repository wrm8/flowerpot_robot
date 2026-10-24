#ifndef HUNGARIAN_ALGORITHM_HPP_
#define HUNGARIAN_ALGORITHM_HPP_

#include <vector>
#include <cmath>
#include <limits>

namespace hungarian {

// 简化的点结构
struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double _x, double _y) : x(_x), y(_y) {}
};

/**
 * 匈牙利算法求解器
 */
class HungarianSolver {
public:
    HungarianSolver() : inf_(1e9) {}
    ~HungarianSolver() {}
    
    /**
     * 求解最小化分配问题
     */
    std::vector<int> solveMinimization(const std::vector<std::vector<double>>& cost_matrix) {
        if (cost_matrix.empty()) return {};
        
        int n = cost_matrix.size();      // 任务数（相机点数）
        int m = cost_matrix[0].size();   // 工人数（雷达点数）
        int size = std::max(n, m);
        
        // 构建方阵
        std::vector<std::vector<double>> matrix(size, std::vector<double>(size, 0.0));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                matrix[i][j] = cost_matrix[i][j];
            }
        }
        
        std::vector<double> u(size + 1, 0.0);
        std::vector<double> v(size + 1, 0.0);
        std::vector<int> p(size + 1, 0);
        std::vector<int> way(size + 1, 0);
        
        for (int i = 1; i <= size; i++) {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(size + 1, inf_);
            std::vector<bool> used(size + 1, false);
            
            do {
                used[j0] = true;
                int i0 = p[j0];
                double delta = inf_;
                int j1 = 0;
                
                for (int j = 1; j <= size; j++) {
                    if (!used[j]) {
                        double cur = matrix[i0 - 1][j - 1] - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }
                
                for (int j = 0; j <= size; j++) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);
            
            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0);
        }
        
        // 提取匹配结果
        std::vector<int> assignment(n, -1);
        for (int j = 1; j <= size; j++) {
            if (p[j] <= n && j <= m) {
                assignment[p[j] - 1] = j - 1;
            }
        }
        
        return assignment;
    }
    
    /**
     * 计算总代价
     */
    double getTotalCost(const std::vector<std::vector<double>>& cost_matrix,
                        const std::vector<int>& assignment) {
        double total = 0.0;
        for (size_t i = 0; i < assignment.size(); i++) {
            if (assignment[i] != -1) {
                total += cost_matrix[i][assignment[i]];
            }
        }
        return total;
    }
    
private:
    double inf_;
};

/**
 * 计算距离矩阵
 */
inline std::vector<std::vector<double>> computeDistanceMatrix(
    const std::vector<Point>& points1,
    const std::vector<Point>& points2) {
    
    std::vector<std::vector<double>> dist_matrix(
        points1.size(), std::vector<double>(points2.size(), 0.0));
    
    for (size_t i = 0; i < points1.size(); i++) {
        for (size_t j = 0; j < points2.size(); j++) {
            double dx = points1[i].x - points2[j].x;
            double dy = points1[i].y - points2[j].y;
            dist_matrix[i][j] = std::hypot(dx, dy);
        }
    }
    
    return dist_matrix;
}

/**
 * 根据阈值获取有效匹配
 */
inline std::vector<std::pair<int, int>> getValidMatchesByThreshold(
    const std::vector<int>& assignment,
    const std::vector<std::vector<double>>& distance_matrix,
    double threshold) {
    
    std::vector<std::pair<int, int>> valid_matches;
    for (size_t i = 0; i < assignment.size(); i++) {
        if (assignment[i] != -1 && distance_matrix[i][assignment[i]] <= threshold) {
            valid_matches.push_back({i, assignment[i]});
        }
    }
    return valid_matches;
}

} // namespace hungarian

#endif