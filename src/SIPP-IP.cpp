#include <iostream>
#include <algorithm>
#include <set>
#include <utility>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cassert>
#include "../include/constants.h"
using namespace std;
using namespace std::chrono;



class Bot{
public:
    int x, y, o, v, t_lower, t_upper, parentID;
    Bot(int x, int y, int o, int v, int t_lower, int t_upper):x(x),y(y),o(o),v(v),t_lower(t_lower),t_upper(t_upper){
        parentID = -1;
    }
    Bot(int x, int y, int o, int v, int t_lower, int t_upper, int parentID):x(x),y(y),o(o),v(v),t_lower(t_lower),t_upper(t_upper),parentID(parentID){}
    Bot(){
        x = y = o = v = t_lower = t_upper = 0;
        parentID=-1;
    }
    Bot(const Bot&) = default;
    Bot &operator=(const Bot&other)= default;
    friend bool operator == (const Bot& a, const Bot& b);
    friend bool operator < (const Bot& a, const Bot& b);
    friend ostream &operator<<( ostream &output, const Bot &p ) {
        output<<"x="<<p.x<<", y="<<p.y<<", o="<<p.o<<", v="<<p.v<<", t_lower="<<p.t_lower<<", t_upper="<<p.t_upper<<", parentID="<<p.parentID;
        return output;
    }
};
int end_x, end_y, end_v, minimalTransitionCost;
bool operator < (const Bot& a, const Bot& b){
    if(a.t_lower+(abs(a.x-end_x)+abs(a.y-end_y))*minimalTransitionCost != b.t_lower+(abs(b.x-end_x)+abs(b.y-end_y))*minimalTransitionCost) {
        return a.t_lower + (abs(a.x - end_x) + abs(a.y - end_y))*minimalTransitionCost < b.t_lower + (abs(b.x - end_x) + abs(b.y - end_y))*minimalTransitionCost;
    }
    if(abs(a.x-end_x)+abs(a.y-end_y) != abs(b.x-end_x)+abs(b.y-end_y)) {
        return abs(a.x - end_x) + abs(a.y - end_y) < abs(b.x - end_x) + abs(b.y - end_y);
    }
    if(a.t_upper != b.t_upper){
        return a.t_upper > b.t_upper;
    }
    return ((a.x*MXW+a.y)*4+a.o)*MXV+a.v
           < ((b.x*MXW+b.y)*4+b.o)*MXV+b.v;
}
bool operator == (const Bot& a, const Bot& b){
    return a.x == b.x && a.y == b.y && a.o == b.o && a.v == b.v && a.t_lower == b.t_lower && a.t_upper == b.t_upper && a.parentID==b.parentID;
}

class Primitive{
public:
    struct move{
        // ftt (first-touch-time) = (t_lower) in the paper. 
        // swt (sweeping time) = (t_upper - t_lower) in the paper.
        move(int dx, int dy, int ftt, int swt, bool isEndCell):dx(dx),dy(dy),ftt(ftt),swt(swt), isEndCell(isEndCell){}
        int dx, dy, ftt, swt;
        bool isEndCell; // flag if this cell is still touched at the end of the primitive. `swt` in this case is equal to the remaining time until the end of the primitive.
    };
    vector<move> mvs;
    int o, v;
    Primitive(){
        mvs.clear();
        o = v = 0;
    }
    Primitive(vector<move> mvs, int o, int v):
    mvs(std::move(mvs)),o(o),v(v){}
    friend ostream &operator<<( ostream &output, const Primitive &p ) {
        output << "Moves:"<<endl;
        for(auto it:p.mvs){
            output << "  dx:"<<it.dx<<", dy:"<<it.dy<<", ftt:"<<it.ftt<<", swt:"<<it.swt<<endl;
        }
        output<<"Final_o:"<<p.o<<", Final_v:"<<p.v<<endl;
        return output;
    }
};

int H, W;
set<pair<int, int>> rsrv_tbl[MXH][MXW];
vector<Primitive> motion_primitives[MXO][MXV];

set<Bot> OPEN;
vector<Bot> CLOSED_vec;
set<pair<int, int>> CLOSED[MXH][MXW][MXO][MXV];

bool outMap(int x, int y) {
    return x<0 || x>=H || y<0 || y>=W;
}

// Check the CLOSED set if any state intersect with the given interval.
// return new t_lower (because the intersection (if exists) will be from lower bound always).
int checkCLOSED(int x, int y, int o, int v, int t_lower){
    if(CLOSED[x][y][o][v].empty())
        return t_lower;
    // there shouldn't be any interval in CLOSED with lower_bound > t_lower.
    auto it = --CLOSED[x][y][o][v].end();
    return max(t_lower, it->second+1);
}

vector<Bot> applyPrimitive(int x, int y, int t_lower, int t_upper, Primitive mp) {
    vector<pair<int, int>> timeIntervals, tmp;
    timeIntervals.push_back({t_lower, t_upper});
    bool endCellTouched = false;
    int xx = x, yy = y, past_time = 0;
    for(auto mv:mp.mvs){
        xx = x + mv.dx;
        yy = y + mv.dy;
        if (outMap(xx, yy)) {
            return {};
        }
        for(auto &it:timeIntervals){
            int t_l = min(INF, it.first + (mv.ftt - past_time));
            int t_u = min(INF, it.second + (mv.ftt - past_time));
            auto itr= rsrv_tbl[xx][yy].lower_bound({t_l, INF - 1});
            auto itr_prev = itr;
            --itr_prev;
            while (itr_prev->second < t_u) {
                int new_tlower = max(t_l, itr_prev->second + 1);
                int new_tupper = min(t_u, itr->first - 1 - mv.swt);
                if (new_tlower <= new_tupper && (mv.isEndCell && !endCellTouched) &&
                    mp.v == 0) { // if there is at least one timestep to get there and vel there is zero and this is the first end-cell in the primitive.
                    new_tupper = itr->first - 1 - mv.swt;
                }
                if (new_tlower <= new_tupper){
                    tmp.push_back({new_tlower, new_tupper});
                }
                ++itr;
                ++itr_prev;
            }
        }
        timeIntervals = tmp;
        tmp.clear();
        past_time = mv.ftt;
        if(mv.isEndCell){
            endCellTouched = true;
        }
    }
    vector<Bot> vBot;
    for(auto it:timeIntervals){
        vBot.push_back(Bot(xx, yy, mp.o, mp.v, min(INF, it.first+mp.mvs.back().swt), min(INF,it.second+mp.mvs.back().swt)));
    }
    return vBot;
}
void generateSuccessors(int x, int y, int o, int v, int t_lower, int t_upper, vector<Bot>& succs) {
    for (auto &mp: motion_primitives[o][v]) {
        auto vBots = applyPrimitive(x, y, t_lower, t_upper, mp);
        move(vBots.begin(), vBots.end(), back_inserter(succs));
    }
}

void getSolutionStates(int goalID, vector<Bot> &vecStates){
    vecStates.clear();
    while(goalID!=-1){
        vecStates.push_back(CLOSED_vec[goalID]);
        goalID = CLOSED_vec[goalID].parentID;
    }
    reverse(vecStates.begin(), vecStates.end());
}

pair<int, bool> search(int st_x, int st_y, int st_o, int st_v, int st_tlower=0){
    auto it = rsrv_tbl[st_x][st_y].lower_bound({st_tlower,INF});
    int st_tupper = it->first - 1;
    --it;
    if (st_tupper < st_tlower || it->second >= st_tlower) {
        cerr<<"Error, the initial state falls in an obstacle!!"<<endl;
        exit(0);
    }
    OPEN.clear();
    CLOSED_vec.clear();
    OPEN.insert(Bot(st_x, st_y, st_o, st_v, st_tlower, st_tupper, -1));
    int x, y, o, v, t_lower, t_upper, id;
    int cntNodes = 0;
    while(!OPEN.empty() && cntNodes<MAX_NUM_NODES){
        auto tmp = *OPEN.begin();
        OPEN.erase(OPEN.begin());
        --cntNodes;
        x = tmp.x, y = tmp.y, o = tmp.o, v = tmp.v, t_lower = tmp.t_lower, t_upper = tmp.t_upper;
        auto new_t_lower = checkCLOSED(x, y, o, v, t_lower);
        if(new_t_lower > t_upper){
            continue;
        }
        if(t_lower!=new_t_lower){ // if the state got trimmed, insert it again in OPEN.
            tmp.t_lower = new_t_lower;
            OPEN.insert(tmp);
            continue;
        }
        CLOSED[x][y][o][v].insert({t_lower, t_upper});
        ++cntNodes;
        id = CLOSED_vec.size();
        CLOSED_vec.emplace_back(tmp);
        if(x == end_x && y == end_y && v == end_v){
            return {id, true};
        }
        vector<Bot> succs;
        generateSuccessors(x, y, o, v, t_lower, t_upper, succs);
        for(auto &it:succs){
            it.parentID = id;
            OPEN.insert(it);
            ++cntNodes;
        }
    }
    return {0, false};
}


void clr(){
    for(int i=0; i<MXH; ++i){
        for(int j=0; j<MXW; ++j){
            rsrv_tbl[i][j].clear();
            rsrv_tbl[i][j].insert({-1, -1});
            rsrv_tbl[i][j].insert({INF+1, INF+1});
            for(int k=0; k<MXO; ++k){
                for(int l=0; l<MXV; ++l){
                    CLOSED[i][j][k][l].clear();
                }
            }
        }
    }
}

int testNum = 0, num_of_obstacles;
void readInputs(int &stx, int &sty, int &sto) {
    ifstream in;
    in.open("../maps/"+map+".txt");
    in>>H>>W;
    for(int i=0; i<H; ++i){
        for(int j=0; j<W; ++j){
            bool tmp;
            in>>tmp;
            if(tmp)
                rsrv_tbl[i][j].insert({0,INF});
        }
    }
    in.close();
    in.open("../tests/"+map+"/"+map+"-test-"+ to_string(testNum)+"-"+to_string(num_of_obstacles)+".txt");
    string ignore_string;
    int x, y, t1, t2;
    in>>ignore_string>>ignore_string>>stx>>ignore_string>>sty>>ignore_string>>sto;
    in>>ignore_string>>ignore_string>>end_x>>ignore_string>>end_y;
    while(in>>ignore_string){
        in>>ignore_string>>x>>ignore_string>>y>>ignore_string>>t1>>ignore_string>>t2;
        t1 *= F;
        t2 *= F;
        rsrv_tbl[x][y].insert({t1,t2});
    }
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            for (auto it = rsrv_tbl[i][j].begin(); it != rsrv_tbl[i][j].end(); ++it) {
                auto it1 = it;
                ++it1;
                if (it1 == rsrv_tbl[i][j].end())
                    break;
                if(it1->first <= it->second){
                    cerr<<"Error: Blocked intervals on cell ("<<i<<","<<j<<") are intersecting!"<<endl;
                }
                assert(it1->first > it->second);
            }
        }
    }
    in.close();
}



void fillActions() {
    minimalTransitionCost = 5; // the minimum cost to go from one cell to next (0.5s = 5 timesteps).
    // turn actions
    for (int o = 0; o < MXO; ++o) {
        Primitive tmp;
        tmp.mvs = {Primitive::move(0, 0, 0, 20, 1)}; // two seconds (20 timesteps) to rotate.
        tmp.o = (o + 1) % MXO;
        tmp.v = 0;
        motion_primitives[o][0].emplace_back(tmp);
        tmp.o = (o + 3) % MXO;
        motion_primitives[o][0].emplace_back(tmp);
    }
    int dx[4] = {0, -1, 0, 1}, dy[4] = {1, 0, -1, 0};
    // Acceleration and Deceleration primitives:
    // take 4 cells and 4 seconds (40 timesteps) to finish - a=+-0.5m/s^2, v_max=2m/s.
    vector<pair<int, int>> costs = {make_pair(0,20), make_pair(0,29), make_pair(20, 15), make_pair(28,12), make_pair(34,6)};
    int full_cost = 40;
    int sz = costs.size();
    for (int o = 0; o < MXO; ++o) {
        Primitive tmp;
        tmp.v = 1;
        tmp.o=o;
        for(int j=0; j<sz; ++j){
            tmp.mvs.push_back(Primitive::move(dx[o]*j,dy[o]*j,costs[j].first, costs[j].second,int(j==sz-1)));
        }
        motion_primitives[o][0].push_back(tmp);
        tmp.mvs.clear();
        tmp.v = 0;
        for(int j=0; j<sz; ++j){
            tmp.mvs.push_back(Primitive::move(dx[o]*j,dy[o]*j,full_cost-(costs[sz-1-j].first+costs[sz-1-j].second), costs[sz-1-j].second,int(j==sz-1)));
        }
        motion_primitives[o][1].push_back(tmp);
        tmp.mvs = {Primitive::move(0, 0, 0, 5, 0), Primitive::move(dx[o], dy[o], 0, 5, 1)};
        tmp.o = o;
        tmp.v = 1;
        motion_primitives[o][1].emplace_back(tmp);
    }
}

int numOfFreeCells(){
    bool mp[600][600];
    int sm = 0;
    ifstream in;
    in.open("../maps/" + map + ".txt");
    in >> H >> W;
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            in >> mp[i][j];
            sm += mp[i][j]==0;
        }
    }
    in.close();
    return sm;
}
void printSolutionStates(int goal_id){
    vector<Bot> v;
    getSolutionStates(goal_id, v);
    cout<<"Solution states:";
    for (auto it: v) {
        cout << it << endl;
    }
    cout<<"--------"<<endl;
}
int main() {
    cout<<"SIPP-IP started... map:"<<map<<endl;
    ofstream out;
    fillActions();
    int num_of_free_cells = numOfFreeCells();
    for(int f = 0; f<factors.size(); ++f){
    	int factor = factors[f];
        num_of_obstacles = num_of_free_cells/factor;
        out.open("../results/results SIPP-IP/res-" + map + "-SIPP-IP-obs"+to_string(num_of_obstacles)+".txt");
        for (testNum = 0; testNum < NumOfTests; ++testNum) {
            clr();
            int stx, sty, sto;
            end_v = 0;
            readInputs(stx, sty, sto);
            auto start_time = high_resolution_clock::now();
            auto ans = search(stx, sty, sto, 0, 0);
            double diffTime = duration_cast<microseconds>(high_resolution_clock::now() - start_time).count();
            out << "Success:" << ans.second << ", Cost:"<< CLOSED_vec[ans.first].t_lower << ", Runtime:" << diffTime << "\n";
            if (ans.second) {
                // printSolutionStates(ans.first);
            }
        }
        out.close();
    }
    return 0;
}

