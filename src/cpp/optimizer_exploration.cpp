#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <random>
#include <iomanip>

using namespace std;

const double PI = 3.14159265358979323846;

// --- GEOMETRIC STRUCTURES ---
struct Point { double x, y; };

struct Tree {
    string original_id;
    int group_id;
    double x, y, deg;
    vector<Point> polygon;

    // Calculate Polygon
    void update_polygon(const vector<Point>& template_poly) {
        polygon.clear();
        double rad = deg * PI / 180.0;
        double c = cos(rad);
        double s = sin(rad);

        for (const auto& p : template_poly) {
            double nx = (p.x * c - p.y * s) + x;
            double ny = (p.x * s + p.y * c) + y;
            polygon.push_back({nx, ny});
        }
    }
};

// --- HELPER FUNCTIONS ---
vector<Point> get_tree_template() {
    double tw=0.15, th=0.2, bw=0.7, mw=0.4, ow=0.25;
    double tip=0.8, t1=0.5, t2=0.25, base=0.0, tbot=-th;
    vector<double> x = {0, ow/2, ow/4, mw/2, mw/4, bw/2, tw/2, tw/2, -tw/2, -tw/2, -bw/2, -mw/4, -mw/2, -ow/4, -ow/2};
    vector<double> y = {tip, t1, t1, t2, t2, base, base, tbot, tbot, base, base, t2, t2, t1, t1};
    vector<Point> pts;
    for(size_t i=0; i<x.size(); ++i) pts.push_back({x[i], y[i]});
    return pts;
}

double parse_val(string s) {
    if (s.size() > 0 && s[0] == 's') return stod(s.substr(1));
    return stod(s);
}

int get_group_id(const string& id) {
    size_t pos = id.find('_');
    if (pos != string::npos) return stoi(id.substr(0, pos));
    return 0;
}

// --- COLLISION CHECKS (OLD RELIABLE LOGIC) ---
// We use the exact logic from your "Old Code" that was working fine.
bool onSegment(Point p, Point a, Point b) {
    return p.x <= max(a.x, b.x) && p.x >= min(a.x, b.x) && p.y <= max(a.y, b.y) && p.y >= min(a.y, b.y);
}
int orientation(Point p, Point q, Point r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (abs(val) < 1e-9) return 0;
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
bool isInside(const vector<Point>& poly, Point p) {
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
    // 1. BOUNDING BOX CHECK with SAFETY TOLERANCE
    // This is the FIX. We force C++ to be stricter than Python.
    double minX1=1e10, maxX1=-1e10, minY1=1e10, maxY1=-1e10;
    for(auto& p : t1.polygon) { minX1=min(minX1, p.x); maxX1=max(maxX1, p.x); minY1=min(minY1, p.y); maxY1=max(maxY1, p.y); }
    double minX2=1e10, maxX2=-1e10, minY2=1e10, maxY2=-1e10;
    for(auto& p : t2.polygon) { minX2=min(minX2, p.x); maxX2=max(maxX2, p.x); minY2=min(minY2, p.y); maxY2=max(maxY2, p.y); }

    // Tolerance: 1e-11 prevents "touching" errors in Python
    double tolerance = 1e-11;

    if (maxX1 < minX2 - tolerance || maxX2 < minX1 - tolerance ||
        maxY1 < minY2 - tolerance || maxY2 < minY1 - tolerance) return false;

    // 2. Detailed Segment Check
    for (size_t i = 0; i < t1.polygon.size(); i++) {
        for (size_t j = 0; j < t2.polygon.size(); j++) {
            if (segmentsIntersect(t1.polygon[i], t1.polygon[(i+1)%t1.polygon.size()], t2.polygon[j], t2.polygon[(j+1)%t2.polygon.size()])) return true;
        }
    }

    // 3. Inside Check
    if (isInside(t2.polygon, t1.polygon[0])) return true;
    if (isInside(t1.polygon, t2.polygon[0])) return true;
    return false;
}

double calculate_group_score(const vector<Tree*>& group_trees) {
    double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300;
    for (const auto* t : group_trees) {
        for (const auto& p : t->polygon) {
            if (p.x < min_x) min_x = p.x; if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y; if (p.y > max_y) max_y = p.y;
        }
    }
    double side = max(max_x - min_x, max_y - min_y);
    return side * side;
}

// --- MAIN PROGRAM ---
int main(int argc, char* argv[]) {
    int target_n = 0;
    int seed = 42;
    int iterations = 200000;
    string output_file = "partial_out.csv";

    // Read Arguments
    for(int i=1; i<argc; i++) {
        string arg = argv[i];
        if(arg == "-n" && i+1 < argc) target_n = stoi(argv[i+1]);
        if(arg == "-r" && i+1 < argc) seed = stoi(argv[i+1]);
        if(arg == "-o" && i+1 < argc) output_file = argv[i+1];
        if(arg == "-i" && i+1 < argc) iterations = stoi(argv[i+1]);
    }

    srand(seed);
    vector<Point> template_poly = get_tree_template();

    // Read Input File
    ifstream file("submission.csv");
    if (!file.is_open()) return 1;

    string line;
    vector<Tree> all_trees;
    if (getline(file, line)) {}

    while (getline(file, line)) {
        stringstream ss(line);
        string val;
        vector<string> cols;
        while (getline(ss, val, ',')) cols.push_back(val);
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

    // Select Target Group
    vector<Tree*> active_group;
    for(int i=0; i<all_trees.size(); i++) {
        if (all_trees[i].group_id == target_n) {
            active_group.push_back(&all_trees[i]);
        }
    }

    if (!active_group.empty()) {
        double current_score = calculate_group_score(active_group);

        // Save Best State
        vector<Tree> best_group_state;
        for(Tree* t : active_group) best_group_state.push_back(*t);
        double global_best_score = current_score;

        // --- SMART STRATEGY SETTINGS (Vibration/Shake) ---
        int switch_iteration = iterations * 0.30;
        double temp = 2.0;
        double cooling_rate = 0.99999;
        double gravity_strength = 0.0001;

        for (int k = 0; k < iterations; k++) {
            // Dynamic Gravity
            int gravity_mode = 0; // 0: Center, 3: South-West
            double target_x = 0;
            double target_y = 0;
            double current_gravity_str = gravity_strength;

            if (k < switch_iteration) {
                // PHASE 1: SHAKE to South-West
                gravity_mode = 3;
                target_x = -10000.0;
                target_y = -10000.0;
                current_gravity_str = 0.0005;
            } else {
                // PHASE 2: COMPRESS to Center
                gravity_mode = 0;
                current_gravity_str = 0.0001;
            }

            int idx = rand() % active_group.size();
            Tree* t = active_group[idx];
            Tree backup = *t;

            double step = 0.0001 + (temp * 0.005);
            int move_type = rand() % 3;

            double pull_x = 0, pull_y = 0;
            if (gravity_mode == 0) { // Center
                pull_x = -1.0 * t->x * current_gravity_str;
                pull_y = -1.0 * t->y * current_gravity_str;
            } else { // South-West
                pull_x = (target_x - t->x) * current_gravity_str;
                pull_y = (target_y - t->y) * current_gravity_str;
            }

            if (move_type == 0) { // X
                double random_push = (rand()/(double)RAND_MAX - 0.5) * step;
                t->x += random_push + pull_x;
            }
            else if (move_type == 1) { // Y
                double random_push = (rand()/(double)RAND_MAX - 0.5) * step;
                t->y += random_push + pull_y;
            }
            else { // Angle
                t->deg += (rand()/(double)RAND_MAX - 0.5) * 5.0 * step;
            }

            t->update_polygon(template_poly);

            // COLLISION CHECK (Using strict bounding box)
            bool overlap = false;
            for (Tree* other : active_group) {
                if (other == t) continue;
                if (check_overlap(*t, *other)) { overlap = true; break; }
            }

            if (overlap) {
                *t = backup; // Revert
            } else {
                double new_score = calculate_group_score(active_group);

                // Acceptance Logic
                bool accept = false;
                if (new_score < global_best_score - 1e-9) {
                    global_best_score = new_score;
                    best_group_state.clear();
                    for(Tree* gt : active_group) best_group_state.push_back(*gt);
                    accept = true;
                }
                else if (new_score < current_score) {
                    current_score = new_score;
                    accept = true;
                }
                else {
                    double p = exp((current_score - new_score) / temp);
                    if ((rand()/(double)RAND_MAX) < p) {
                        current_score = new_score;
                        accept = true;
                    }
                }

                if (!accept) *t = backup;
            }
            temp *= cooling_rate;
        }

        // Restore Best State
        for(size_t i=0; i<active_group.size(); i++) {
            *(active_group[i]) = best_group_state[i];
        }
    }

    // Write Output
    ofstream outfile(output_file);
    outfile << fixed << setprecision(18);
    for(const auto* t : active_group) {
        outfile << t->original_id << ",s" << t->x << ",s" << t->y << ",s" << t->deg << endl;
    }
    outfile.close();

    return 0;
}