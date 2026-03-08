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

using namespace std;

const double PI = 3.14159265358979323846;

// --- GEOMETRIC STRUCTURES ---
struct Point { double x, y; };

struct Tree {
    string original_id;
    int group_id;
    double x, y, deg;
    vector<Point> polygon;
    bool locked = false; // LNS: Eğer true ise bu ağaç hareket etmez

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

// --- COLLISION CHECKS ---
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
    double minX1=1e10, maxX1=-1e10, minY1=1e10, maxY1=-1e10;
    for(auto& p : t1.polygon) { minX1=min(minX1, p.x); maxX1=max(maxX1, p.x); minY1=min(minY1, p.y); maxY1=max(maxY1, p.y); }
    double minX2=1e10, maxX2=-1e10, minY2=1e10, maxY2=-1e10;
    for(auto& p : t2.polygon) { minX2=min(minX2, p.x); maxX2=max(maxX2, p.x); minY2=min(minY2, p.y); maxY2=max(maxY2, p.y); }

    double tolerance = 1e-10; // Biraz daha gevşek tolerans
    if (maxX1 < minX2 - tolerance || maxX2 < minX1 - tolerance ||
        maxY1 < minY2 - tolerance || maxY2 < minY1 - tolerance) return false;

    for (size_t i = 0; i < t1.polygon.size(); i++) {
        for (size_t j = 0; j < t2.polygon.size(); j++) {
            if (segmentsIntersect(t1.polygon[i], t1.polygon[(i+1)%t1.polygon.size()], t2.polygon[j], t2.polygon[(j+1)%t2.polygon.size()])) return true;
        }
    }
    if (isInside(t2.polygon, t1.polygon[0])) return true;
    if (isInside(t1.polygon, t2.polygon[0])) return true;
    return false;
}

// --- LNS HELPERS ---
// Hangi ağaçlar sınırları belirliyor ve onlara yakın olanlar hangileri?
vector<int> get_critical_indices(const vector<Tree*>& trees, double radius_multiplier = 1.5) {
    double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300;
    for (const auto* t : trees) {
        for (const auto& p : t->polygon) {
            if (p.x < min_x) min_x = p.x; if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y; if (p.y > max_y) max_y = p.y;
        }
    }

    double width = max_x - min_x;
    double height = max_y - min_y;

    // Problemli kenarı bul (Kareyi bozan taraf)
    bool fix_width = width > height;

    set<int> critical_set;
    vector<int> result;

    // Sınıra çok yakın ağaçları bul (Critical Trees)
    double threshold = 0.5; // Sınıra ne kadar yakınsa "suçlu" sayılır
    vector<Point> trouble_centers;

    for (int i=0; i<trees.size(); i++) {
        bool is_culprit = false;
        double t_min_x = 1e300, t_max_x = -1e300, t_min_y = 1e300, t_max_y = -1e300;
        for(auto& p : trees[i]->polygon) {
            t_min_x = min(t_min_x, p.x); t_max_x = max(t_max_x, p.x);
            t_min_y = min(t_min_y, p.y); t_max_y = max(t_max_y, p.y);
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

    // Neighbors: Suçlu ağaçlara yakın olanları da ekle (Domino etkisi için)
    // Yarıçapı biraz büyük tutuyoruz ki hareket alanı açılsın
    double search_radius = 4.0 * radius_multiplier;

    for (int i=0; i<trees.size(); i++) {
        if (critical_set.count(i)) continue;
        for (auto& center : trouble_centers) {
            double dist = sqrt(pow(trees[i]->x - center.x, 2) + pow(trees[i]->y - center.y, 2));
            if (dist < search_radius) {
                critical_set.insert(i);
                break;
            }
        }
    }

    for (int idx : critical_set) result.push_back(idx);
    return result;
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

int main(int argc, char* argv[]) {
    int target_n = 0;
    int seed = 42;
    int iterations = 200000;
    string output_file = "partial_out.csv";

    for(int i=1; i<argc; i++) {
        string arg = argv[i];
        if(arg == "-n" && i+1 < argc) target_n = stoi(argv[i+1]);
        if(arg == "-r" && i+1 < argc) seed = stoi(argv[i+1]);
        if(arg == "-o" && i+1 < argc) output_file = argv[i+1];
        if(arg == "-i" && i+1 < argc) iterations = stoi(argv[i+1]);
    }

    srand(seed);
    vector<Point> template_poly = get_tree_template();

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

    vector<Tree*> active_group;
    for(int i=0; i<all_trees.size(); i++) {
        if (all_trees[i].group_id == target_n) {
            active_group.push_back(&all_trees[i]);
        }
    }

    if (!active_group.empty()) {
        double current_score = calculate_group_score(active_group);
        vector<Tree> best_group_state;
        for(Tree* t : active_group) best_group_state.push_back(*t);
        double global_best_score = current_score;

        // --- LNS INITIALIZATION ---
        // 1. Sadece kritik ağaçları seç
        vector<int> mutable_indices = get_critical_indices(active_group);

        // Eğer çok az ağaç seçildiyse (nadiren olur), rastgele %20 ekle
        if (mutable_indices.size() < active_group.size() * 0.1) {
            int to_add = active_group.size() * 0.2;
            for(int i=0; i<to_add; i++) mutable_indices.push_back(rand() % active_group.size());
        }

        // 2. Diğerlerini kilitle (Locked)
        for(auto* t : active_group) t->locked = true;
        for(int idx : mutable_indices) active_group[idx]->locked = false;

        // 3. YIKIM (Destruction) - Seçilenleri sars
        // Bu ağaçları hafifçe dışarı/boşluğa atıyoruz ki tekrar yerleşsinler
        for(int idx : mutable_indices) {
            active_group[idx]->x += (rand()/(double)RAND_MAX - 0.5) * 1.0;
            active_group[idx]->y += (rand()/(double)RAND_MAX - 0.5) * 1.0;
            active_group[idx]->deg = rand() % 360; // Açıyı sıfırla/karıştır
            active_group[idx]->update_polygon(template_poly);
        }

        // --- SIMULATED ANNEALING ---
        double temp = 1.5;
        double cooling_rate = 0.99995;
        double gravity_strength = 0.002; // LNS'de daha güçlü gravity lazım

        for (int k = 0; k < iterations; k++) {
            // Sadece mutable (kilitli olmayan) ağaçlardan seç
            int rand_idx = rand() % mutable_indices.size();
            Tree* t = active_group[mutable_indices[rand_idx]];
            Tree backup = *t;

            double step = 0.0001 + (temp * 0.01);
            int move_type = rand() % 3;

            // GRAVITY: Merkeze çek (Sadece hareketli olanları)
            double pull_x = -1.0 * t->x * gravity_strength;
            double pull_y = -1.0 * t->y * gravity_strength;

            if (move_type == 0) t->x += ((rand()/(double)RAND_MAX - 0.5) * step) + pull_x;
            else if (move_type == 1) t->y += ((rand()/(double)RAND_MAX - 0.5) * step) + pull_y;
            else t->deg += (rand()/(double)RAND_MAX - 0.5) * 15.0 * step;

            t->update_polygon(template_poly);

            // COLLISION CHECK
            bool overlap = false;
            // Çarpışma kontrolü TÜM ağaçlarla yapılmalı (Sabitler dahil)
            for (Tree* other : active_group) {
                if (other == t) continue;
                // bounding box ön elemesi (hız için)
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
                    for(Tree* gt : active_group) best_group_state.push_back(*gt);
                    accept = true;
                } else if (new_score < current_score) {
                    current_score = new_score;
                    accept = true;
                } else {
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

        // En iyi hali geri yükle
        for(size_t i=0; i<active_group.size(); i++) {
            *(active_group[i]) = best_group_state[i];
        }
    }

    ofstream outfile(output_file);
    outfile << fixed << setprecision(18);
    for(const auto* t : active_group) {
        outfile << t->original_id << ",s" << t->x << ",s" << t->y << ",s" << t->deg << endl;
    }
    outfile.close();

    return 0;
}
