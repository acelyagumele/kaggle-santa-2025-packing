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

// --- GEOMETRİK YAPILAR ---
struct Point { double x, y; };

struct Tree {
    string original_id;
    int group_id;
    double x, y, deg;
    vector<Point> polygon;

    // Poligonu hesapla
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

// --- YARDIMCI FONKSİYONLAR ---
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

int get_group_id(string id) {
    size_t pos = id.find('_');
    if (pos != string::npos) return stoi(id.substr(0, pos));
    return 0;
}

// --- ÇARPIŞMA KONTROLLERİ (Optimize Edilmiş) ---
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
    if (t1.group_id != t2.group_id) return false;

    // Bounding Box Check (Hız için kritik)
    double minX1=1e10, maxX1=-1e10, minY1=1e10, maxY1=-1e10;
    for(auto& p : t1.polygon) { minX1=min(minX1, p.x); maxX1=max(maxX1, p.x); minY1=min(minY1, p.y); maxY1=max(maxY1, p.y); }
    double minX2=1e10, maxX2=-1e10, minY2=1e10, maxY2=-1e10;
    for(auto& p : t2.polygon) { minX2=min(minX2, p.x); maxX2=max(maxX2, p.x); minY2=min(minY2, p.y); maxY2=max(maxY2, p.y); }

    if (maxX1 < minX2 || maxX2 < minX1 || maxY1 < minY2 || maxY2 < minY1) return false;

    // Detaylı Kontrol
    for (size_t i = 0; i < t1.polygon.size(); i++) {
        for (size_t j = 0; j < t2.polygon.size(); j++) {
            if (segmentsIntersect(t1.polygon[i], t1.polygon[(i+1)%t1.polygon.size()], t2.polygon[j], t2.polygon[(j+1)%t2.polygon.size()])) return true;
        }
    }
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

// --- MAIN SOLVER ---
int main(int argc, char* argv[]) {
    int target_n = 0;
    int seed = 42;
    // İterasyonu çok yüksek tutuyoruz çünkü mikro adımlar atacağız
    int iterations = 200000;

    for(int i=1; i<argc; i++) {
        string arg = argv[i];
        if(arg == "-n" && i+1 < argc) target_n = stoi(argv[i+1]);
        if(arg == "-r" && i+1 < argc) seed = stoi(argv[i+1]);
    }

    srand(seed);
    vector<Point> template_poly = get_tree_template();

    // --- DOSYA OKUMA ---
    ifstream file("submission.csv");
    string line;
    vector<Tree> all_trees;
    if (getline(file, line)) {}

    while (getline(file, line)) {
        stringstream ss(line);
        string val;
        vector<string> cols;
        while (getline(ss, val, ',')) cols.push_back(val);

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

    // Hedef Grubu Seç
    vector<Tree*> active_group;
    for(int i=0; i<all_trees.size(); i++) {
        if (all_trees[i].group_id == target_n) {
            active_group.push_back(&all_trees[i]);
        }
    }

    if (!active_group.empty()) {
        double current_score = calculate_group_score(active_group);

        // EN İYİ SKORU SAKLA
        vector<Tree> best_group_state;
        for(Tree* t : active_group) best_group_state.push_back(*t);
        double global_best_score = current_score;

        // PRO AYARLAR: MİKRO ADIMLAR ve GRAVITY
        // Başlangıç sıcaklığı çok düşük, çünkü 70 zaten "soğuk" bir yapı.
        double temp = 0.05;
        double cooling_rate = 0.99999; // Çok yavaş soğuma

        // Gravity Gücü: Ağaçları (0,0)'a çeken güç
        double gravity_strength = 0.00005;

        for (int k = 0; k < iterations; k++) {
            int idx = rand() % active_group.size();
            Tree* t = active_group[idx];
            Tree backup = *t;

            // MİKRO ADIM HESABI (Step Size)
            // 157'de step 0.1 idi, şimdi 0.0001 seviyelerine iniyoruz
            double step = 0.0001 + (temp * 0.001);

            // HAREKET MANTIĞI: GRAVITY + RANDOM
            int move_type = rand() % 3;

            if (move_type == 0) { // X Hareketi
                // Rastgelelik + Merkeze çekim (Gravity)
                // Eğer x pozitifse, negatif yönlü bir çekim uygula (-x)
                double gravity_pull = -1.0 * t->x * gravity_strength;
                double random_push = (rand()/(double)RAND_MAX - 0.5) * step;
                t->x += random_push + gravity_pull;
            }
            else if (move_type == 1) { // Y Hareketi
                double gravity_pull = -1.0 * t->y * gravity_strength;
                double random_push = (rand()/(double)RAND_MAX - 0.5) * step;
                t->y += random_push + gravity_pull;
            }
            else { // Açı Hareketi (Sadece titreşim)
                t->deg += (rand()/(double)RAND_MAX - 0.5) * 2.0 * step;
            }

            t->update_polygon(template_poly);

            // ÇAKIŞMA KONTROLÜ
            bool overlap = false;
            // Sadece bu ağaca yakın olanları kontrol etsek iyi olurdu ama
            // 200 ağaç için Brute Force hala yeterince hızlıdır.
            for (Tree* other : active_group) {
                if (other == t) continue;
                if (check_overlap(*t, *other)) { overlap = true; break; }
            }

            if (overlap) {
                *t = backup; // Geri al
            } else {
                double new_score = calculate_group_score(active_group);

                // REKOR KIRILDI MI?
                if (new_score < global_best_score - 1e-9) { // Floating point hata payı
                    global_best_score = new_score;
                    best_group_state.clear();
                    for(Tree* gt : active_group) best_group_state.push_back(*gt);
                }

                // Kabul Kriteri
                if (new_score < current_score) {
                    current_score = new_score;
                } else {
                    double p = exp((current_score - new_score) / temp);
                    if ((rand()/(double)RAND_MAX) >= p) *t = backup;
                }
            }
            temp *= cooling_rate;
        }

        // EN İYİ HALİ GERİ YÜKLE
        cout << "Bulunan En Iyi Skor: " << global_best_score << endl;
        for(size_t i=0; i<active_group.size(); i++) {
            *(active_group[i]) = best_group_state[i];
        }
    }

    // DOSYAYI YAZ (Yüksek Hassasiyet)
    ofstream outfile("submission.csv");
    outfile << "id,x,y,deg" << endl;
    outfile << fixed << setprecision(18); // 18 basamak hassasiyet şart
    for(const auto& t : all_trees) {
        outfile << t.original_id << ",s" << t.x << ",s" << t.y << ",s" << t.deg << endl;
    }
    outfile.close();

    return 0;
}
