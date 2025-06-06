#include <tuple>
#include <string>
using namespace std;

class Compare {
    public: 
        bool operator()(const tuple<string, string, double>& a, const tuple<string, string, double>& b) {
            return get<2>(a) > get<2>(b);
        }
};