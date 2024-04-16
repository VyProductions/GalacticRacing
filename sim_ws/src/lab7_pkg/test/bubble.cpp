#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using std::cout;
using std::endl;
using std::setw;
using std::sort;
using std::sqrt;
using std::vector;

struct vec2 {
    int x{};
    int y{};
};

double magnitude(const vec2& p);

int main() {
    vector<vec2> offs;

    for (int x = -5; x <= 5; ++x) {
        for (int y = -5; y <= 5; ++y) {
            if (magnitude({x, y}) <= 4.0) {
                offs.push_back({x, y});
            }
        }
    }

    sort(offs.begin(), offs.end(), [](vec2 a, vec2 b) {
        return magnitude(a) < magnitude(b);
    });

    bool done = false;
    for (size_t r = 0; !done; ++r) {
        for (size_t c = 0; !done && c < 6; ++c) {
            size_t pos = r * 6 + c;

            if (pos >= offs.size()) {
                done = true;
            } else if (pos >= offs.size() - 1) {
                done = true;

                cout << "Vec2(" << setw(2) << offs.at(pos).x
                     << ", " << setw(2) << offs.at(pos).y << ")";
            } else {
                cout << "Vec2(" << setw(2) << offs.at(pos).x
                     << ", " << setw(2) << offs.at(pos).y << "), ";
            }
        }

        cout << endl;
    }

    return 0;
}

double magnitude(const vec2& p) {
    return sqrt(p.x * p.x + p.y * p.y);
}
