#include <bits/stdc++.h>
#include "../include/constants.h"

#define MX_HEIGHT 256
#define MX_WIDTH 256
#define MXT (INF/F+1)
using namespace std;
ofstream out;
ifstream in;
set<pair<int, int>> st[MX_HEIGHT][MX_WIDTH];
set<int> st1[MX_HEIGHT][MX_WIDTH];
bool mp[MX_HEIGHT][MX_WIDTH], rsrv[MX_HEIGHT][MX_WIDTH][MXT];
int H, W;


void clr();

bool cellInBigSpace[MX_HEIGHT][MX_WIDTH];
void dfs(int x, int y){
    if(cellInBigSpace[x][y])return;
    if(mp[x][y])return;
    cellInBigSpace[x][y] = 1;
    int dx[4] = {0, 0, 1, -1}, dy[4]={1, -1, 0, 0};
    for(int i=0; i<4; ++i){
        int xx = x+dx[i];
        int yy = y+dy[i];
        if(xx<H && xx>=0 && yy<W && yy>=0){
            dfs(xx, yy);
        }
    }
}
string MAP;

// we fix the start and end points for each map.
void generate_start_end_points() {
    int x, y, o = 0;
    if(MAP=="warehouse-10-20-10-2-2"){
        x = 4;
        y = 4;
        out << "start_cell: x= " << x << " y= " << y << " orientation= " << o << endl;
        mp[x][y] = true;
        x = 78;
        y = 163;
        out << "goal_cell: x= " << x << " y= " << y << endl;
        mp[x][y] = true;
    }
    else if(MAP=="random128"){
        x = 5;
        y = 6;
        out << "start_cell: x= " << x << " y= " << y << " orientation= " << o << endl;
        mp[x][y] = true;
        x = 125;
        y = 124;
        out << "goal_cell: x= " << x << " y= " << y << endl;
        mp[x][y] = true;
    }
    else if(MAP=="empty_64_64"){
        x = 5;
        y = 5;
        out << "start_cell: x= " << x << " y= " << y << " orientation= " << o << endl;
        mp[x][y] = true;
        x = 59;
        y = 59;
        out << "goal_cell: x= " << x << " y= " << y << endl;
        mp[x][y] = true;
    }
    else if(MAP=="room-64-64-16"){
        x = 6;
        y = 6;
        out << "start_cell: x= " << x << " y= " << y << " orientation= " << o << endl;
        mp[x][y] = true;
        x = 58;
        y = 58;
        out << "goal_cell: x= " << x << " y= " << y << endl;
        mp[x][y] = true;
    }
    else if(MAP=="Sydney_2_256"){
        x = 31;
        y = 23;
        out << "start_cell: x= " << x << " y= " << y << " orientation= " << o << endl;
        mp[x][y] = true;
        x = 239;
        y = 239;
        out << "goal_cell: x= " << x << " y= " << y << endl;
        mp[x][y] = true;
    }
    else{
        exit(0);
    }
}

struct pnt{
    pnt(int x, int y, int par_id, int cost):x(x), y(y), par_id(par_id), cost(cost){}
    int x, y, par_id, cost;
};
int NDX, NDY;
bool operator < (pnt a, pnt b){
    if(a.cost+abs(a.x-NDX)+abs(a.y-NDY) != b.cost+abs(b.x-NDX)+abs(b.y-NDY))
        return a.cost+abs(a.x-NDX)+abs(a.y-NDY) > b.cost+abs(b.x-NDX)+abs(b.y-NDY);
    if(abs(a.x-NDX)+abs(a.y-NDY) != abs(b.x-NDX)+abs(b.y-NDY))
        return abs(a.x-NDX)+abs(a.y-NDY) > abs(b.x-NDX)+abs(b.y-NDY);
    return make_pair(make_pair(a.x,a.y),make_pair(a.cost,a.par_id))<make_pair(make_pair(b.x,b.y),make_pair(b.cost,b.par_id));
}
bool operator == (pnt a, pnt b){
    return a.x == b.x && a.y==b.y && a.cost==b.cost;
}
bool mem[MX_HEIGHT][MX_WIDTH][MXT];
bool find_and_mark_timal_path(int stx, int sty, int st_t, int ndx, int ndy, int i_spd){
    queue<pnt> q;
    NDX = ndx; NDY = ndy;
    assert(mp[stx][sty]==0);
    assert(rsrv[stx][sty][st_t]==0);
    q.push(pnt(stx, sty, -1, st_t));
    int dx[9] = {0, -1, 0, 1, 1, 1, -1, -1, 0}, dy[9] = {1, 0, -1, 0, 1, -1, 1, -1, 0};
    vector<pnt> v;
    int ndT = MXT-1;
    while(ndT>0){
        if(rsrv[ndx][ndy][ndT-1]==1)break;
        --ndT;
    }
    while(!q.empty()){
        auto tmp = q.front();
        q.pop();
        if(tmp.x==NDX && tmp.y==NDY) {
            for (int i = tmp.cost; i < MXT; ++i) {
                rsrv[tmp.x][tmp.y][i] = 1;
            }
            int pr = tmp.par_id;
            rsrv[tmp.x][tmp.y][tmp.cost] = 1;
            while (pr != -1) {
                auto tmp1 = v[pr];
                for(int k=tmp.cost; k>=tmp1.cost; --k){
                    rsrv[tmp.x][tmp.y][k] = rsrv[tmp1.x][tmp1.y][k] =1;
                }
                tmp =tmp1;
                pr = tmp.par_id;
            }
            for(auto it:v){
                mem[it.x][it.y][it.cost]=0;
            }
            return 1;
        }
        if(tmp.cost>=MXT)continue;
        if(mem[tmp.x][tmp.y][tmp.cost] ){
            continue;
        }
        mem[tmp.x][tmp.y][tmp.cost] = 1;
        int sz = v.size();
        v.push_back(tmp);
        for(int i=0; i<9; ++i){
            int xx = tmp.x+dx[i];
            int yy = tmp.y+dy[i];
            if(xx>=0 && xx<H && yy>=0 && yy<W){
                if(!mp[xx][yy]){
                    bool l = true;
                    for(int k=tmp.cost; k<=tmp.cost+i_spd; ++k){
                        if(rsrv[tmp.x][tmp.y][k] || rsrv[xx][yy][k]){
                            l=false;
                            break;
                        }
                    }
                    if(l){
                        auto tmp1 = tmp;
                        tmp1.x = xx;
                        tmp1.y = yy;
                        tmp1.cost += i_spd;
                        tmp1.par_id = sz;
                        q.push(tmp1);
                    }
                }
            }
        }
    }
    for(auto it:v){
        mem[it.x][it.y][it.cost]=0;
    }
    return false;
}

bool mem1[MX_HEIGHT][MX_WIDTH];
bool find_and_mark_path(int stx, int sty, int st_t, int ndx, int ndy, int i_spd){
    queue<pnt> q;
    assert(mp[stx][sty]==0);
    NDX = ndx; NDY = ndy;
    q.push(pnt(stx, sty, -1, 0));
    int dx[9] = {0, -1, 0, 1, 1, 1, -1, -1, 0}, dy[9] = {1, 0, -1, 0, 1, -1, 1, -1, 0};
    vector<pnt> v;
    while(!q.empty()){
        auto tmp = q.front();
        q.pop();
        if(tmp.x==NDX && tmp.y==NDY) {
            st_t = min(st_t, MXT-tmp.cost-1);
            for (int i = tmp.cost; i < MXT-st_t; ++i) {
                rsrv[tmp.x][tmp.y][i+st_t] = 1;
                assert(i+st_t<=MXT);
            }
            int pr = tmp.par_id;
            rsrv[tmp.x][tmp.y][tmp.cost] = 1;
            while (pr != -1) {
                auto tmp1 = v[pr];
                for(int k=tmp.cost; k>=tmp1.cost; --k){
                    rsrv[tmp.x][tmp.y][k+st_t] = rsrv[tmp1.x][tmp1.y][k+st_t] = 1;
                    assert(k+st_t<=MXT);
                }
                tmp =tmp1;
                pr = tmp.par_id;
            }
            for(auto it:v){
                mem1[it.x][it.y]=0;
            }
            return 1;
        }
        if(tmp.cost>=MXT)continue;
        if(mem1[tmp.x][tmp.y]){
            continue;
        }
        mem1[tmp.x][tmp.y] = 1;
        int sz = v.size();
        v.push_back(tmp);
        for(int i=0; i<9; ++i){
            int xx = tmp.x+dx[i];
            int yy = tmp.y+dy[i];
            if(xx>=0 && xx<H && yy>=0 && yy<W){
                if(!mem1[xx][yy] && !mp[xx][yy]){
                    bool l = true;
                    if(l){
                        auto tmp1 = tmp;
                        tmp1.x = xx;
                        tmp1.y = yy;
                        tmp1.cost += i_spd;
                        tmp1.par_id = sz;
                        q.push(tmp1);
                    }
                }
            }
        }
    }
    for(auto it:v){
        mem1[it.x][it.y]=0;
    }
    return false;
}


vector<vector<int>> obs;
void generate_obstacles1(int numOfTests){
    random_device rd;
    mt19937 mt(rd());
    uniform_int_distribution<int> randHeight(0, H - 1);
    uniform_int_distribution<int> randWidth(0, W - 1);
    uniform_int_distribution<int> randSpeed(1, 10);
    uniform_int_distribution<int> randTime(0, 200);
    for(int i=0; i<numOfTests; ++i){
        if(MAP=="random-64-64-10" || MAP=="empty_64_64" ){// when the map is small we can consider the other agents' paths.
            int st_t = 0;
            while(true) {
                int stx = randHeight(mt), sty = randWidth(mt);
                while (mp[stx][sty] || rsrv[stx][sty][st_t]) {
                    stx = randHeight(mt);
                    sty = randWidth(mt);
                }
                mp[stx][sty] = true;
                int ndx = randHeight(mt), ndy = randWidth(mt);
                while (mp[ndx][ndy] || rsrv[ndx][ndy][MXT-1]) {
                    ndx = randHeight(mt);
                    ndy = randWidth(mt);
                }
                mp[stx][sty]=false;
                int i_spd = randSpeed(mt);
                if(find_and_mark_timal_path(stx, sty, st_t, ndx, ndy, i_spd)){
                    obs.push_back({stx, sty, st_t, ndx, ndy, i_spd});
                    break;
                }
            }
        }
        else{
            while(true) {
                int st_t = randTime(mt);
                int stx = randHeight(mt), sty = randWidth(mt);
                while (mp[stx][sty]) {
                    stx = randHeight(mt);
                    sty = randWidth(mt);
                }
                mp[stx][sty] = true;
                int ndx = randHeight(mt), ndy = randWidth(mt);
                while (mp[ndx][ndy]) {
                    ndx = randHeight(mt);
                    ndy = randWidth(mt);
                }
                mp[stx][sty]=false;
                int i_spd = randSpeed(mt);
                if(find_and_mark_path(stx, sty, st_t, ndx, ndy, i_spd)){
                    obs.push_back({stx, sty, st_t, ndx, ndy, i_spd});
                    break;
                }
            }
        }
    }
}
void generate_obstacles2(int id){
    auto it = obs[id];
    if(MAP=="random-64-64-10" || MAP=="empty_64_64") // when the map is small we can consider the other agents' paths.
        find_and_mark_timal_path(it[0], it[1], it[2], it[3], it[4], it[5]);
    else
        find_and_mark_path(it[0], it[1], it[2], it[3], it[4], it[5]);
}

int main() {
    ios::sync_with_stdio(0);
    vector<string> maps = {"room-64-64-16", "empty_64_64", "random128", "warehouse-10-20-10-2-2", "Sydney_2_256"};
    for (int k = 0; k < maps.size(); ++k) {
        MAP = maps[k];
        int sm = 0;
        in.open("../maps/" + maps[k] + ".txt");
        in >> H >> W;
        for (int i = 0; i < H; ++i) {
            for (int j = 0; j < W; ++j) {
                in >> mp[i][j];
                sm += mp[i][j]==0;
            }
        }
        in.close();
        cout<<"Number of free cells:"<<sm<<endl;
        int min_factor = 1e9;
        for(auto it:factors){
            min_factor = min(min_factor, it);
        }
        for (int i = 0; i < NumOfTests; ++i) {
            obs.clear();
            int num_of_obstacles = sm/min_factor; // we first generate the maximum number of obstacles.
            cout<<MAP<<"-test:"<<i<<", generating "<<num_of_obstacles<<" obstacles..."<<endl;
            in.open("../" + maps[k] + ".txt");
            in >> H >> W;
            for (int i = 0; i < H; ++i) {
                for (int j = 0; j < W; ++j) {
                    in >> mp[i][j];
                }
            }
            in.close();
            clr();
            out.open("../tests/" + maps[k] + "/" + maps[k] + "-test-" + to_string(i) + "-"+to_string(num_of_obstacles)+".txt");
            generate_start_end_points();
            out.close();
            generate_obstacles1(num_of_obstacles);
            for(int f = 0; f<factors.size(); ++f){
                int factor = factors[f];
                int num_of_obstacles = sm/factor;
                cout<<MAP<<"-test:"<<i<<", generating "<<num_of_obstacles<<" obstacles..."<<endl;
                in.open("../maps/" + maps[k] + ".txt");
                in >> H >> W;
                for (int i = 0; i < H; ++i) {
                    for (int j = 0; j < W; ++j) {
                        in >> mp[i][j];
                    }
                }
                in.close();
                clr();
                out.open("../tests/" + maps[k] + "/" + maps[k] + "-test-" + to_string(i) + "-"+to_string(num_of_obstacles)+".txt");
                generate_start_end_points();
                for(int cnt=0; cnt<num_of_obstacles; ++cnt){
                    generate_obstacles2(cnt);
                }
                for(int i=0; i<H; ++i){
                    for(int j=0; j<W; ++j){
                        for(int k=0; k<MXT; ++k){
                            if(rsrv[i][j][k]){
                                int tmp = k;
                                while(tmp<MXT&&rsrv[i][j][tmp]){++tmp;}
                                --tmp;
                                out<<"reserved_interval_at_cell: x= "<<i<<" y= "<<j<<" from_timestep: "<<k<<" to_timestep: "<<tmp<<endl;
                                k = tmp;
                            }
                        }
                    }
                }
                out.close();
            }
        }
    }
}


void clr() {
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            for (int k = 0; k < MXT; ++k) {
                rsrv[i][j][k] = 0;
            }
        }
    }
}