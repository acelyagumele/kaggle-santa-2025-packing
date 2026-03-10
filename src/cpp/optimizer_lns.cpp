#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <random>
#include <iomanip>
#include <set>

constexpr double PI = 3.14159265358979323846;
constexpr double COLLISION_TOLERANCE = 1e-11;

// GEOMETRIC STRUCTURES
struct Point { double x, y; };

struct Tree {
    std::string original_id;
    int group_id;
    double x, y, deg;
    std::vector<Point> polygon;
    bool locked = false; // LNS mechanism: True means this tree acts as a static obstacle

    // Memory reuse optimization for high-frequency mutation calls
    void update_polygon(const std::vector<Point>& template_poly) {
        if (polygon.size() != template_poly.size()) {
            polygon.resize(template_poly.size());
        }
        double rad = deg * PI / 180.0;
        double c = std::cos(rad);
        double s = std::sin(rad);

        for (size_t i = 0; i < template_poly.size(); ++i) {
            polygon[i].x = (template_poly[i].x * c - template_poly[i].y * s) + x;
            polygon[i].y = (template_poly[i].x * s + template_poly[i].y * c) + y;
        }
    }
};

// HELPER FUNCTIONS
std::vector<Point> get_tree_template() {
    constexpr double tw=0.15, th=0.2, bw=0.7, mw=0.4, ow=0.25;
    constexpr double tip=0.8, t1=0.5, t2=0.25, base=0.0, tbot=-th;
    std::vector<double> x = {0, ow/2, ow/4, mw/2, mw/4, bw/2, tw/2, tw/2, -tw/2, -tw/2, -bw/2, -mw/4, -mw/2, -ow/4, -ow/2};
    std::vector<double> y = {tip, t1, t1, t2, t2, base, base, tbot, tbot, base, base, t2, t2, t1, t1};
    std::vector<Point> pts;
    pts.reserve(x.size());
    for(size_t i=0; i<x.size(); ++i) pts.push_back({x[i], y[i]});
    return pts;
}

double parse_val(const std::string& s) {
    if (!s.empty() && s[0] == 's') return std::stod(s.substr(1));
    return std::stod(s);
}

int get_group_id(const std::string& id) {
    size_t pos = id.find('_');
    if (pos != std::string::npos) return std::stoi(id.substr(0, pos));
    return 0;
}

// COLLISION CHECKS (FAIL-SAFE) 
inline bool onSegment(Point p, Point a, Point b) {
    return p.x <= std::max(a.x, b.x) && p.x >= std::min(a.x, b.x) && 
           p.y <= std::max(a.y, b.y) && p.y >= std::min(a.y, b.y);
}

inline int orientation(Point p, Point q, Point r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (std::abs(val) < 1e-9) return 0;
    return (val > 0) ? 1 : 2;
}

bool segmentsIntersect(Point p1, Point q1, Point p2, Point q2) {
    int o1 = orientation(p1, q1, p2); int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1); int o4 = orientation(p2, q2, q1);
    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && onSegment(p2, p1, q1)) return true;
    if (o2 == 0 && onSegment(q2, p1, q1)) return true;
    if (o3 == 0 && onSegment(p1, p2, q2)) return true;
    if (o4 == 0 && onSegment(q1, p2, q2)) return true;
    return false;
}

bool isInside(const std::vector<Point>& poly, Point p) {
    int n = poly.size();
    if (n < 3) return false;
    Point extreme = {1e10, p.y};
    int count = 0, i = 0;
    do {
        int next = (i + 1) % n;
        if (segmentsIntersect(poly[i], poly[next], p, extreme)) {
            if (orientation(poly[i], p, poly[next]) == 0) return onSegment(p, poly[i], poly[next]);
            count++;
        }
        i = next;
    } while (i != 0);
    return count % 2 == 1;
}

bool check_overlap(const Tree& t1, const Tree& t2) {
    double minX1=1e10, maxX1=-1e10, minY1=1e10, maxY1=-1e10;
    for(const auto& p : t1.polygon) { minX1=std::min(minX1, p.x); maxX1=std::max(maxX1, p.x); minY1=std::min(minY1, p.y); maxY1=std::max(maxY1, p.y); }
    double minX2=1e10, maxX2=-1e10, minY2=1e10, maxY2=-1e10;
    for(const auto& p : t2.polygon) { minX2=std::min(minX2, p.x); maxX2=std::max(maxX2, p.x); minY2=std::min(minY2, p.y); maxY2=std::max(maxY2, p.y); }

    if (maxX1 < minX2 - COLLISION_TOLERANCE || maxX2 < minX1 - COLLISION_TOLERANCE ||
        maxY1 < minY2 - COLLISION_TOLERANCE || maxY2 < minY1 - COLLISION_TOLERANCE) return false;

    for (size_t i = 0; i < t1.polygon.size(); i++) {
        for (size_t j = 0; j < t2.polygon.size(); j++) {
            if (segmentsIntersect(t1.polygon[i], t1.polygon[(i+1)%t1.polygon.size()], 
                                  t2.polygon[j], t2.polygon[(j+1)%t2.polygon.size()])) return true;
        }
    }
    if (isInside(t2.polygon, t1.polygon[0])) return true;
    if (isInside(t1.polygon, t2.polygon[0])) return true;
    return false;
}

// --- LNS (LARGE NEIGHBORHOOD SEARCH) BOTTLENECK DETECTOR ---
// I implemented this function to autonomously detect the "culprit" polygons that dictate 
// the bounding box limits, and mark them (along with their neighbors) for localized destruction.
std::vector<int> get_critical_indices(const std::vector<Tree*>& trees, double radius_multiplier = 1.5) {
    double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300;
    for (const auto* t : trees) {
        for (const auto& p : t->polygon) {
            if (p.x < min_x) min_x = p.x; if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y; if (p.y > max_y) max_y = p.y;
        }
    }

    double width = max_x - min_x;
    double height = max_y - min_y;

    // Determine the problematic edge that prevents the bounding box from shrinking
    bool fix_width = width > height;

    std::set<int> critical_set;
    std::vector<int> result;

    constexpr double threshold = 0.5; // Proximity threshold to be considered a bottleneck
    std::vector<Point> trouble_centers;

    for (size_t i=0; i<trees.size(); i++) {
        bool is_culprit = false;
        double t_min_x = 1e300, t_max_x = -1e300, t_min_y = 1e300, t_max_y = -1e300;
        for(const auto& p : trees[i]->polygon) {
            t_min_x = std::min(t_min_x, p.x); t_max_x = std::max(t_max_x, p.x);
            t_min_y = std::min(t_min_y, p.y); t_max_y = std::max(t_max_y, p.y);
        }

        if (fix_width) {
            if (t_max_x >= max_x - threshold || t_min_x <= min_x + threshold) is_culprit = true;
        } else {
            if (t_max_y >= max_y - threshold || t_min_y <= min_y + threshold) is_culprit = true;
        }

        if (is_culprit) {
            critical_set.insert(i);
            trouble_centers.push_back({trees[i]->x, trees[i]->y});
        }
    }

    // Domino Effect: Also include neighbors of the culprits to create enough empty space for re-packing
    double search_radius = 4.0 * radius_multiplier;
    for (size_t i=0; i<trees.size(); i++) {
        if (critical_set.count(i)) continue;
        for (const auto& center : trouble_centers) {
            double dist = std::sqrt(std::pow(trees[i]->x - center.x, 2) + std::pow(trees[i]->y - center.y, 2));
            if (dist < search_radius) {
                critical_set.insert(i);
                break;
            }
        }
    }

    for (int idx : critical_set) result.push_back(idx);
    return result;
}

double calculate_group_score(const std::vector<Tree*>& group_trees) {
    double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300;
    for (const auto* t : group_trees) {
        for (const auto& p : t->polygon) {
            if (p.x < min_x) min_x = p.x; if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y; if (p.y > max_y) max_y = p.y;
        }
    }
    double side = std::max(max_x - min_x, max_y - min_y);
    return side * side;
}

// MAIN OPTIMIZATION LOOP
int main(int argc, char* argv[]) {
    int target_n = 0;
    int seed = 42;
    int iterations = 200000;
    std::string output_file = "partial_out.csv";

    for(int i=1; i<argc; i++) {
        std::string arg = argv[i];
        if(arg == "-n" && i+1 < argc) target_n = std::stoi(argv[i+1]);
        if(arg == "-r" && i+1 < argc) seed = std::stoi(argv[i+1]);
        if(arg == "-o" && i+1 < argc) output_file = argv[i+1];
        if(arg == "-i" && i+1 < argc) iterations = std::stoi(argv[i+1]);
    }

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist_01(0.0, 1.0);
    std::uniform_real_distribution<double> dist_sym(-0.5, 0.5);
    std::uniform_real_distribution<double> dist_deg(0.0, 360.0);

    std::vector<Point> template_poly = get_tree_template();

    std::ifstream file("submission.csv");
    if (!file.is_open()) return 1;

    std::string line;
    std::vector<Tree> all_trees;
    if (std::getline(file, line)) {}

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        std::vector<std::string> cols;
        while (std::getline(ss, val, ',')) cols.push_back(val);
        if (cols.size() < 4) continue;

        Tree t;
        t.original_id = cols[0];
        t.group_id = get_group_id(t.original_id);
        t.x = parse_val(cols[1]);
        t.y = parse_val(cols[2]);
        t.deg = parse_val(cols[3]);
        t.update_polygon(template_poly);
        all_trees.push_back(t);
    }
    file.close();

    std::vector<Tree*> active_group;
    for(auto& t : all_trees) {
        if (t.group_id == target_n) active_group.push_back(&t);
    }

    if (!active_group.empty()) {
        double current_score = calculate_group_score(active_group);
        std::vector<Tree> best_group_state;
        best_group_state.reserve(active_group.size());
        for(Tree* t : active_group) best_group_state.push_back(*t);
        double global_best_score = current_score;

        // LNS INITIALIZATION  
        // 1. Identify critical structural elements preventing optimization
        std::vector<int> mutable_indices = get_critical_indices(active_group);

        // Fallback: If too few trees are selected, randomly inject entropy (10-20% mutation rate)
        if (mutable_indices.size() < active_group.size() * 0.1) {
            std::uniform_int_distribution<int> tree_dist(0, active_group.size() - 1);
            int to_add = active_group.size() * 0.2;
            for(int i=0; i<to_add; i++) mutable_indices.push_back(tree_dist(rng));
        }

        // 2. Lock the well-placed structures to preserve known good patterns
        for(auto* t : active_group) t->locked = true;
        for(int idx : mutable_indices) active_group[idx]->locked = false;

        // 3. RUIN Phase: Aggressively displace the selected bottleneck elements
        for(int idx : mutable_indices) {
            active_group[idx]->x += dist_sym(rng) * 1.0;
            active_group[idx]->y += dist_sym(rng) * 1.0;
            active_group[idx]->deg = dist_deg(rng); // Randomize orientation completely
            active_group[idx]->update_polygon(template_poly);
        }

        // RECREATE PHASE (Simulated Annealing on mutable trees) 
        double temp = 1.5;
        constexpr double cooling_rate = 0.99995;
        constexpr double gravity_strength = 0.002; // Stronger center-gravity to force re-packing

        std::uniform_int_distribution<int> mutable_dist(0, mutable_indices.size() - 1);
        std::uniform_int_distribution<int> move_dist(0, 2);

        for (int k = 0; k < iterations; k++) {
            // Only manipulate the unlocked (ruined) trees
            int rand_idx = mutable_dist(rng);
            Tree* t = active_group[mutable_indices[rand_idx]];
            Tree backup = *t;

            double step = 0.0001 + (temp * 0.01);
            int move_type = move_dist(rng);

            // GRAVITY: Pull aggressively towards the center to fill the gaps
            double pull_x = -1.0 * t->x * gravity_strength;
            double pull_y = -1.0 * t->y * gravity_strength;

            if (move_type == 0) t->x += (dist_sym(rng) * step) + pull_x;
            else if (move_type == 1) t->y += (dist_sym(rng) * step) + pull_y;
            else t->deg += dist_sym(rng) * 15.0 * step;

            t->update_polygon(template_poly);

            // COLLISION CHECK against ALL trees (both locked and unlocked)
            bool overlap = false;
            for (const Tree* other : active_group) {
                if (other == t) continue;
                if (check_overlap(*t, *other)) { overlap = true; break; }
            }

            if (overlap) {
                *t = backup;
            } else {
                double new_score = calculate_group_score(active_group);
                bool accept = false;

                if (new_score < global_best_score - 1e-9) {
                    global_best_score = new_score;
                    best_group_state.clear();
                    for(const Tree* gt : active_group) best_group_state.push_back(*gt);
                    accept = true;
                } else if (new_score < current_score) {
                    current_score = new_score;
                    accept = true;
                } else {
                    double p = std::exp((current_score - new_score) / temp);
                    if (dist_01(rng) < p) {
                        current_score = new_score;
                        accept = true;
                    }
                }
                if (!accept) *t = backup;
            }
            temp *= cooling_rate;
        }

        // Restore global best state
        for(size_t i=0; i<active_group.size(); i++) {
            *(active_group[i]) = best_group_state[i];
        }
    }

    std::ofstream outfile(output_file);
    outfile << std::fixed << std::setprecision(18);
    for(const auto* t : active_group) {
        outfile << t->original_id << ",s" << t->x << ",s" << t->y << ",s" << t->deg << "\n";
    }
    outfile.close();

    return 0;
}