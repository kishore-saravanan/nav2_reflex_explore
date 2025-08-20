#include "nav2_reflex_explore/frontier_core.hpp"
#include <queue>
#include <cmath>

namespace nav2_reflex_explore {

static inline bool valid(int r,int c,int H,int W){ return r>=0 && c>=0 && r<H && c<W; }

// unknown cell that has at least one 4-neighbor free → frontier pixel
static inline bool is_frontier_px(const nav_msgs::msg::OccupancyGrid &m, int r, int c){
  const int H = (int)m.info.height, W = (int)m.info.width;
  const auto &d = m.data;
  if (!valid(r,c,H,W)) return false;
  if (d[r*W + c] != -1) return false;               // not unknown
  static const int d4[4][2]{{1,0},{-1,0},{0,1},{0,-1}};
  for (auto &o : d4){
    int rr=r+o[0], cc=c+o[1];
    if (!valid(rr,cc,H,W)) continue;
    if (d[rr*W + cc] == 0) return true;             // touches free
  }
  return false;
}

std::vector<Frontier> FrontierCore::extract(const nav_msgs::msg::OccupancyGrid &m){
  std::vector<Frontier> out;
  const int H = (int)m.info.height, W = (int)m.info.width;
  if (H<=0 || W<=0 || (int)m.data.size()!=H*W) return out;

  // mark frontier pixels
  std::vector<uint8_t> mask(H*W, 0);
  for (int r=0;r<H;++r)
    for (int c=0;c<W;++c)
      if (is_frontier_px(m,r,c)) mask[r*W+c]=1;

  // BFS cluster the mask
  std::vector<uint8_t> vis(H*W,0);
  const double res = m.info.resolution;
  const double ox  = m.info.origin.position.x;
  const double oy  = m.info.origin.position.y;

  static const int d8[8][2]{{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

  for (int r0=0;r0<H;++r0){
    for (int c0=0;c0<W;++c0){
      int id0=r0*W+c0;
      if (!mask[id0] || vis[id0]) continue;

      int count=0; double sx=0.0, sy=0.0;
      std::queue<std::pair<int,int>> q;
      q.emplace(r0,c0); vis[id0]=1;

      while(!q.empty()){
        auto [r,c]=q.front(); q.pop();
        ++count;
        // accumulate world-centroid of cells’ centers
        sx += ox + (c + 0.5) * res;
        sy += oy + (r + 0.5) * res;

        for (auto &o : d8){
          int rr=r+o[0], cc=c+o[1]; int id=rr*W+cc;
          if (!valid(rr,cc,H,W)) continue;
          if (!mask[id] || vis[id]) continue;
          vis[id]=1; q.emplace(rr,cc);
        }
      }

      if (count >= p_.min_frontier_cells){
        Frontier f;
        f.size = count;
        f.x = sx / (double)count;
        f.y = sy / (double)count;
        out.push_back(f);
      }
    }
  }
  return out;
}

}
