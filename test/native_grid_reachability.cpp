
#include "GridNavigator.h"
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>
using namespace qiming;
struct Stats { uint64_t maps=0,total=0,success=0,permanentEnd=0,dynamicEnd=0,disconnected=0,clearance=0; };
static bool topological(uint8_t s,uint8_t t,uint32_t blocked) {
 uint8_t q[25],head=0,tail=0;uint32_t seen=1UL<<s;q[tail++]=s;
 while(head<tail) {
  uint8_t v=q[head++];if(v==t)return true;
  int x=GridGeometry::column(v),y=GridGeometry::row(v);
  const int dx[]={1,0,-1,0},dy[]={0,1,0,-1};
  for(int j=0;j<4;++j) {int nx=x+dx[j],ny=y+dy[j];if(nx<0||nx>=5||ny<0||ny>=5)continue;
   uint8_t n=GridGeometry::id(nx,ny);if((seen|blocked)&(1UL<<n))continue;
   seen|=1UL<<n;q[tail++]=n;
  }
 }
 return false;
}
static void run(uint32_t extra,Stats &s) {
 GridGeometry g;g.blocked=GridGeometry::permanent|extra;++s.maps;
 for(uint8_t a=0;a<25;++a) for(uint8_t b=0;b<25;++b) {
  uint32_t ends=(1UL<<a)|(1UL<<b);
  bool pe=ends&GridGeometry::permanent,de=!pe&&(ends&extra);
  bool connected=!pe&&!de&&topological(a,b,g.blocked);
  for(float h:{0.f,90.f,180.f,270.f}) {
   GridPath p;++s.total;bool ok=g.plan(a,b,h,p);
   if(ok){assert(!pe&&!de&&connected);++s.success;}
   else if(pe)++s.permanentEnd;
   else if(de)++s.dynamicEnd;
   else if(!connected)++s.disconnected;
   else ++s.clearance;
  }
 }
}
int main() {
 std::vector<uint8_t> free;
 for(uint8_t v=0;v<25;++v)if(!(GridGeometry::permanent&(1UL<<v)))free.push_back(v);
 Stats s[4];run(0,s[0]);
 for(unsigned i=0;i<free.size();++i) {
  run(1UL<<free[i],s[1]);
  for(unsigned j=i+1;j<free.size();++j) {
   run((1UL<<free[i])|(1UL<<free[j]),s[2]);
   for(unsigned k=j+1;k<free.size();++k)
    run((1UL<<free[i])|(1UL<<free[j])|(1UL<<free[k]),s[3]);
  }
 }
 for(int k=0;k<4;++k) {
  auto &v=s[k];uint64_t valid=v.total-v.permanentEnd-v.dynamicEnd;
  printf("k=%d maps=%llu total=%llu success=%llu permanent_endpoint=%llu dynamic_endpoint=%llu disconnected=%llu clearance=%llu valid=%llu valid_success=%.6f%%\n",k,(unsigned long long)v.maps,(unsigned long long)v.total,(unsigned long long)v.success,(unsigned long long)v.permanentEnd,(unsigned long long)v.dynamicEnd,(unsigned long long)v.disconnected,(unsigned long long)v.clearance,(unsigned long long)valid,100.0*v.success/valid);
  assert(v.total==v.success+v.permanentEnd+v.dynamicEnd+v.disconnected+v.clearance);
 }
 GridGeometry g;GridPath p;g.blocked=GridGeometry::permanent|(1UL<<1)|(1UL<<9);
 assert(!g.plan(0,10,0,p));
 puts("example PASS: cells1+9 occupied => free start0 cannot reach free goal10");
}

