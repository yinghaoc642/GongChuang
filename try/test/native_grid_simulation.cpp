// Host discrete-event simulation of the real protocol, mission and mode FSM.
// Radar paths use an independent point-grid BFS, never GridGeometry::plan().
// Motors, QR and workstations are event models, not a physics/vision emulator.
#include "GridNavigator.h"
#include "NavigationMode.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <random>
#include <vector>
using namespace qiming;
namespace {
constexpr uint32_t islands=(1UL<<6)|(1UL<<8)|(1UL<<16)|(1UL<<18);
unsigned column(unsigned v) { return v/5%2 ? 4-v%5 : v%5; }
unsigned number(unsigned x,unsigned y) { return y*5+(y%2?4-x:x); }
void put16(std::vector<uint8_t>&p,size_t i,uint16_t v){p[i]=v;p[i+1]=v>>8;}
void put32(std::vector<uint8_t>&p,size_t i,uint32_t v){put16(p,i,v);put16(p,i+2,v>>16);}
bool independentPath(uint8_t start,uint8_t goal,uint32_t mask,std::vector<uint8_t>&path) {
 path.clear();mask|=islands;
 if(start>24||goal>24||(mask&((1UL<<start)|(1UL<<goal))))return false;
 int parent[25];std::fill(parent,parent+25,-1);std::deque<unsigned>q{start};parent[start]=start;
 while(!q.empty()&&parent[goal]<0){unsigned v=q.front();q.pop_front();
  int x=column(v),y=v/5;const int dx[]={0,-1,0,1},dy[]={-1,0,1,0};
  for(int i=0;i<4;++i){int nx=x+dx[i],ny=y+dy[i];if(nx<0||nx>4||ny<0||ny>4)continue;
   unsigned n=number(nx,ny);if(parent[n]>=0||(mask&(1UL<<n)))continue;parent[n]=v;q.push_back(n);}
 }
 if(parent[goal]<0)return false;
 for(unsigned v=goal;v!=start;v=parent[v])path.push_back(v);
 std::reverse(path.begin(),path.end());return true;
}
// Independent geometry oracle: continuous AABB sweep at the mission's cardinal
// headings, plus an exact disk/rectangle test for turns. Includes 5 mm margin.
bool footprintSafe(Point a,Point b,float heading,uint32_t mask,bool spin=false,
                   float halfForward=145,float halfRight=117,float margin=5) {
 const double angle=heading*std::acos(-1.0)/180;
 double hx=std::abs(std::cos(angle))*halfRight+std::abs(std::sin(angle))*halfForward+margin;
 double hy=std::abs(std::sin(angle))*halfRight+std::abs(std::cos(angle))*halfForward+margin;
 double radius=std::hypot(halfForward,halfRight)+margin;
 if(spin)hx=hy=radius;
 double l=std::min(a.x,b.x)-hx,r=std::max(a.x,b.x)+hx;
 double bottom=std::min(a.y,b.y)-hy,top=std::max(a.y,b.y)+hy;
 if(l<-.02||bottom<-.02||r>2400.02||top>2400.02)return false;
 auto hit=[&](double x,double y,double w,double h){
  if(spin){double dx=std::max({x-a.x,0.0,a.x-x-w}),dy=std::max({y-a.y,0.0,a.y-y-h});
   return dx*dx+dy*dy<radius*radius-.02;}
  return l<x+w-.02&&r>x+.02&&bottom<y+h-.02&&top>y+.02;
 };
 for(double x:{550.,1400.})for(double y:{550.,1400.})if(hit(x,y,450,450))return false;
 for(unsigned v=0;v<25;++v)if((mask|islands)&(1UL<<v))
  if(hit(column(v)*480,v/5*480,480,480))return false;
 return true;
}
// Independent feasibility oracle for the CURRENT fixed scan preparation and
// the full ordered mission. A more flexible scan position is deliberately not
// assumed: inability to select one is a limitation under test, not a dead road.
bool missionFeasible(unsigned zone,uint32_t mask) {
 Point home{150.f,zone==1?2250.f:150.f},inward{150.f,zone==1?2100.f:300.f};
 Point scan{350.f,inward.y},entry{240.f,zone==1?2160.f:240.f};
 if(!footprintSafe(home,inward,0,mask)||!footprintSafe(inward,scan,0,mask)||
    !footprintSafe(scan,scan,0,mask,true)||!footprintSafe(scan,entry,0,mask))return false;
 const unsigned cells[]={10,22,2,14,22,2,14,zone==1?20U:0U};
 const Point docks[]={{150,1200},{1200,2080},{1200,320},{2080,1200},
                       {1200,2080},{1200,320},{2080,1200},home};
 const float headings[]={0,90,-90,-180,90,-90,-180,0};
 unsigned start=zone==1?20:0;float heading=0;
 for(unsigned i=0;i<8;++i){std::vector<uint8_t>path;
  if(!independentPath(start,cells[i],mask,path))return false;
  Point at{column(start)*480.f+240,start/5*480.f+240};
  for(unsigned v:path){Point next{column(v)*480.f+240,v/5*480.f+240};
   if(!footprintSafe(at,next,heading,mask))return false;at=next;}
  if(headings[i]!=heading&&!footprintSafe(at,at,heading,mask,true))return false;
  heading=headings[i];if(!footprintSafe(at,docks[i],heading,mask))return false;
  if(i<7&&!footprintSafe(docks[i],at,heading,mask))return false;start=cells[i];
 }
 return true;
}
struct Options {
 unsigned latency=0,scanFrameMs=10,motionMs=0,qrMs=0,workMs=0,fragment=1;
 bool ackFirst=true,compress=false,noise=false,wrap=false;
 int homeDx=0,homeDy=0;
 bool customViews=false;uint32_t views[4]={0,0,0,0};unsigned faultPose=1,faultScan=0;
 enum Fault{NONE,NO_HELLO,NO_GRID,SCAN_OVERFLOW,BAD_CRC,STALE_SAVE,BAD_ACK,OLD_PATH,TX_FULL,NO_POSE_ACK,REJECT_POSE,WRONG_POSE,NO_SCAN_ACK}fault=NONE;
};
struct Device {
 GridClient client;Options opt;uint64_t time=100;uint32_t maps[4],map=0,record=7,sequence=0;
 uint8_t zone=1;int scans=0,stops=0,plans=0;unsigned poseChanges=0;bool moving=false;
 std::vector<RadarPose>scanPoses;uint8_t lastStart=255,lastGoal=255,lastStatus=255;
 RadarPose pose{234,2250,170*64,1};std::deque<link_message_t>tx;
 std::multimap<uint64_t,std::vector<uint8_t>>rx;
 Device(uint32_t m1,uint32_t m2,Options o):opt(o),maps{m1,m2,m2,m2}{if(o.customViews)std::copy(o.views,o.views+4,maps);if(o.wrap)time=UINT32_MAX-700;}
 uint32_t now()const{return uint32_t(time);}
 static bool writer(void*ctx,const uint8_t*b,size_t n){auto&d=*static_cast<Device*>(ctx);
  link_decoder_t dec{};link_decoder_init(&dec);link_message_t msg{};
  link_decoder_feed(&dec,b,n,[](const link_message_t*m,void*c){*static_cast<link_message_t*>(c)=*m;},&msg);
  assert(dec.good_frames==1);
  if(d.opt.fault==Options::TX_FULL&&msg.type!=LINK_CMD_STOP)return false;
  d.tx.push_back(msg);return true;
 }
 void emit(uint8_t type,std::vector<uint8_t>p,unsigned delay=0,uint8_t node=1){
  uint8_t b[LINK_MAX_ENCODED_FRAME];size_t n=link_encode_message(node,type,55,p.data(),p.size(),b,sizeof b);assert(n);
  std::vector<uint8_t>frame(b,b+n);
  if(opt.fault==Options::BAD_CRC&&type==LINK_EVT_OBSTACLE_GRID&&(!opt.faultScan||unsigned(scans)==opt.faultScan))frame[n-3]^=1;
  rx.emplace(time+opt.latency+delay,std::move(frame));
 }
 void ack(uint8_t cmd,unsigned delay=0){
  if(opt.fault==Options::NO_SCAN_ACK&&cmd==LINK_CMD_SINGLE_MEASURE&&(!opt.faultScan||unsigned(scans+1)==opt.faultScan))return;
  emit(LINK_EVT_ACK,{cmd,uint8_t(opt.fault==Options::BAD_ACK&&cmd==LINK_CMD_SINGLE_MEASURE&&(!opt.faultScan||unsigned(scans+1)==opt.faultScan))},delay);}
 RadarPose saved(unsigned z)const{return {int16_t(234+opt.homeDx),int16_t((z==1?2250:150)+opt.homeDy),170*64,uint8_t(z)};}
 void hello(unsigned delay=0){std::vector<uint8_t>p(13);p[0]=1;p[1]=35;p[2]=6;p[3]=zone;
  put16(p,4,pose.x);put16(p,6,pose.y);put16(p,8,pose.headingQ6);put16(p,11,720);emit(LINK_EVT_HELLO,p,delay);}
 void persisted(unsigned delay=0){std::vector<uint8_t>p(33);put16(p,0,11);put32(p,2,record);p[6]=zone;p[9]=scans?1:0;
  for(unsigned z=1;z<=2;++z){auto s=saved(z);unsigned i=z==1?11:18;put16(p,i,s.x);put16(p,i+2,s.y);put16(p,i+4,s.headingQ6);p[i+6]=1;}
  for(unsigned v=0;v<25;++v)if(map&(1UL<<v))p.push_back(v);p[10]=p.size()-33;emit(LINK_EVT_PERSISTED_STATE,p,delay);}
 void respond(const link_message_t&m){
  unsigned early=opt.ackFirst?0:2,late=opt.ackFirst?2:0;
  switch(m.type){
   case LINK_CMD_HELLO:if(opt.fault!=Options::NO_HELLO){hello(late);ack(m.type,early);}break;
   case LINK_CMD_SELECT_ZONE:zone=m.payload[0];pose=saved(zone);hello(late);ack(m.type,early);break;
   case LINK_CMD_GET_PERSISTED_STATE:persisted(late);ack(m.type,early);break;
   case LINK_CMD_HEARTBEAT:break; // Vendor heartbeats have no ACK.
   case LINK_CMD_STOP:++stops;break;
   case LINK_CMD_SET_POSE:{assert(!moving);++poseChanges;pose={int16_t(m.payload[1]|m.payload[2]<<8),int16_t(m.payload[3]|m.payload[4]<<8),uint16_t(m.payload[5]|m.payload[6]<<8),zone};
    if(poseChanges==opt.faultPose&&opt.fault==Options::NO_POSE_ACK)break;
    if(poseChanges==opt.faultPose&&opt.fault==Options::REJECT_POSE){emit(LINK_EVT_ACK,{m.type,1});break;}
    ack(m.type);
    if(poseChanges==opt.faultPose&&opt.fault==Options::WRONG_POSE){++pose.x;hello(10);}
    break;}

   case LINK_CMD_SINGLE_MEASURE:{
    assert(!moving&&scans<4);
    // Independent expected positions for right-mounted radar at the mirrored
    // scan stops. Check XY as well as angle: zone2 must move toward +Y.
    const int xs[]={234,434,392,308};
    const int y=scans==0?(zone==1?2250:150):(zone==1?2100:300)+(scans>=2?(zone==1?-73:73):0);
    const int heading=(170+(zone==1?-60:60)*(scans?scans-1:0)+360)%360;
    assert(pose.x==xs[scans]+opt.homeDx&&pose.y==y+opt.homeDy&&pose.headingQ6==heading*64);
    scanPoses.push_back(pose);bool activeFault=!opt.faultScan||unsigned(scans+1)==opt.faultScan;ack(m.type);emit(LINK_EVT_STATUS,{1,2,1,0,0},5);
    for(unsigned i=1;i<=14;++i)emit(LINK_EVT_STATUS,{2,2,1,uint8_t(i),uint8_t(activeFault&&opt.fault==Options::SCAN_OVERFLOW)},5+i*opt.scanFrameMs);
    map=maps[scans++];if(!activeFault||opt.fault!=Options::STALE_SAVE)++record;
    std::vector<uint8_t>p(5);put32(p,0,++sequence);
    for(unsigned v=0;v<25;++v)if(map&(1UL<<v))p.push_back(v);p[4]=p.size()-5;
    if(!activeFault||opt.fault!=Options::NO_GRID)emit(LINK_EVT_OBSTACLE_GRID,p,6+14*opt.scanFrameMs);
    emit(LINK_EVT_STATUS,{4,2,0,14,0},7+14*opt.scanFrameMs);break;
   }
   case LINK_CMD_PLAN_PATH:{
    ++plans;lastStart=m.payload[0];lastGoal=m.payload[1];std::vector<uint8_t>path;
    bool ok=independentPath(lastStart,lastGoal,map,path);lastStatus=ok?0:4;
    if((map|islands)&(1UL<<lastStart))lastStatus=2;
    else if((map|islands)&(1UL<<lastGoal))lastStatus=3;
    if(ok&&opt.compress&&path.size()>1){std::vector<uint8_t>shortPath;unsigned prev=lastStart;
     for(unsigned i=0;i<path.size();++i){if(i+1==path.size()||
       (int(column(path[i]))-int(column(prev)))!=(int(column(path[i+1]))-int(column(path[i])))||
       (int(path[i]/5)-int(prev/5))!=(int(path[i+1]/5)-int(path[i]/5))){shortPath.push_back(path[i]);}
      prev=path[i];}path=shortPath;
    }
    std::vector<uint8_t>p(6);put16(p,0,m.sequence+(opt.fault==Options::OLD_PATH?1:0));
    p[2]=lastStatus;p[3]=lastStart;p[4]=lastGoal;p[5]=path.size();p.insert(p.end(),path.begin(),path.end());
    emit(LINK_EVT_PATH_RESULT,p,late);ack(m.type,early);break;
   }
   default:assert(false);
  }
  if(opt.noise){emit(0xE0,{1,2,3});emit(LINK_EVT_ERROR,{1},0,2);}
 }
 void service(){
  unsigned guard=0;
  do{assert(++guard<100);
   while(!tx.empty()){auto m=tx.front();tx.pop_front();respond(m);}
   while(!rx.empty()&&rx.begin()->first<=time){auto p=rx.begin()->second;rx.erase(rx.begin());
    for(size_t offset=0;offset<p.size();offset+=opt.fragment)
     client.receive(p.data()+offset,std::min(size_t(opt.fragment),p.size()-offset),now());
   }
  }while(!tx.empty());
 }
};
struct Result {
 int outcome=0,error=0,clientError=0,phase=0,work=0,departures=0,scans=0,moves=0;
 uint32_t scanMaps[4]={};std::vector<RadarPose>scanPoses;unsigned poseChanges=0;
 int status=255,start=255,goal=255;float x=0,y=0;uint64_t elapsed=0;
};
Result simulate(unsigned zone,uint32_t m1,uint32_t m2,Options opt={},bool legacy=false) {
 Device d(m1,m2,opt);NavigationMode mode;assert(mode.selected()==NavigationMode::RADAR);
 if(legacy)assert(mode.select(NavigationMode::LEGACY));assert(mode.start(zone));
 assert(!mode.select(NavigationMode::LEGACY));
 if(legacy){assert(mode.state()==NavigationMode::FIXED_LEGACY);return {};}
 d.client.begin(Device::writer,&d,d.now(),zone);GridMission nav(d.client);nav.begin(zone,true);
 Result result;uint64_t began=d.time,motionEnd=0,qrAt=UINT64_MAX,workAt=UINT64_MAX;
 bool moving=false;unsigned resets=0,scanTurns=0;constexpr uint8_t steps[]={6,8,11,14,16,19};
 for(unsigned tick=0;tick<100000;++tick){
  d.time+=20;d.client.poll(d.now());d.service();
  bool finished=moving&&d.time>=motionEnd;if(finished)moving=false;
  auto before=nav.stage();auto a=nav.tick(d.now(),finished,d.time>=qrAt,d.time>=workAt);
  if(a.kind==NavAction::MOVE||a.kind==NavAction::TURN){
   if(mode.state()==NavigationMode::SCANNING&&nav.gridRouteSelected())assert(mode.mapAccepted(true));
   assert(!moving);++result.moves;Point from=nav.position(),to=from;
   double h=nav.heading()*std::acos(-1.0)/180;
   to.x+=-a.forward*std::sin(h)+a.right*std::cos(h);
   to.y+=-a.forward*std::cos(h)-a.right*std::sin(h);
   uint32_t mask=0;for(int i=0;i<d.scans;++i)mask|=d.maps[i];
   assert(footprintSafe(from,to,nav.heading(),mask,a.kind==NavAction::TURN&&std::abs(a.degrees)>.01));
   if(a.departure)++result.departures;
   if(a.kind==NavAction::TURN&&nav.stage()==GridMission::SCAN_POSE){
    assert(a.degrees==(zone==1?60:-60)&&scanTurns<2&&d.scans==int(2+scanTurns));++scanTurns;
   }
   if(a.kind==NavAction::TURN&&nav.stage()==GridMission::RESTORE_HEADING){
    assert(a.degrees==(zone==1?-120:120)&&scanTurns==2&&d.scans==4);++scanTurns;
   }

   moving=true;motionEnd=d.time+opt.motionMs;workAt=UINT64_MAX;
  }else if(a.kind==NavAction::QR_RESET){++resets;qrAt=d.time+opt.qrMs;
  }else if(a.kind==NavAction::WORK){assert(result.work<6&&a.workStep==steps[result.work]);++result.work;workAt=d.time+opt.workMs;
  }else if(a.kind==NavAction::CLEAR_ROUTE){
   uint32_t all=0;for(auto mask:d.maps)all|=mask;
   assert((all&~islands)==0&&result.moves==7&&d.scans==4&&scanTurns==3&&!moving);
   assert(std::abs(nav.position().x-(150+opt.homeDx))<.01&&std::abs(nav.position().y-((zone==1?2250:150)+opt.homeDy))<.01&&nav.heading()==0);
   assert(d.pose.x==d.saved(zone).x&&d.pose.y==d.saved(zone).y&&d.pose.headingQ6==d.saved(zone).headingQ6);
   assert(mode.mapAccepted(false));
   assert(mode.state()==NavigationMode::FIXED_CLEAR);result.outcome=1;break;
  }else if(a.kind==NavAction::FINISH){
   assert(result.work==6&&result.departures==6&&resets==1&&scanTurns==3&&d.scans==4);
   assert(std::abs(nav.position().x-(150+opt.homeDx))<.01&&std::abs(nav.position().y-((zone==1?2250:150)+opt.homeDy))<.01);
   mode.finish();result.outcome=2;break;
  }else if(a.kind==NavAction::FAULT){
   mode.fail();assert(mode.state()==NavigationMode::FAULT);result.outcome=3;
   result.phase=before;result.error=nav.error();
   for(int i=0;i<10;++i)assert(nav.tick(d.now()+i*20,true,true,true).kind==NavAction::NONE);
   break;
  }
  d.moving=moving;
 }
 assert(result.outcome);d.service();assert(d.stops>=1);
 std::copy(d.maps,d.maps+4,result.scanMaps);result.scanPoses=d.scanPoses;result.poseChanges=d.poseChanges;
 result.clientError=d.client.error();result.scans=d.scans;result.status=d.lastStatus;result.start=d.lastStart;result.goal=d.lastGoal;
 result.x=nav.position().x;result.y=nav.position().y;result.elapsed=d.time-began;return result;
}
void print(const char*group,unsigned zone,uint32_t m1,uint32_t m2,const Result&r){
 printf("{\"group\":\"%s\",\"zone\":%u,\"map1\":%u,\"map2\":%u,\"outcome\":%d,\"error\":%d,\"client_error\":%d,\"phase\":%d,\"work\":%d,\"departures\":%d,\"scans\":%d,\"moves\":%d,\"path_status\":%d,\"path_start\":%d,\"path_goal\":%d,\"x\":%.2f,\"y\":%.2f,\"virtual_ms\":%llu,\"expected_scan_maps\":[%u,%u,%u,%u],\"pose_updates\":%u,\"scan_poses\":[",
 group,zone,m1,m2,r.outcome,r.error,r.clientError,r.phase,r.work,r.departures,r.scans,r.moves,r.status,r.start,r.goal,r.x,r.y,(unsigned long long)r.elapsed,r.scanMaps[0],r.scanMaps[1],r.scanMaps[2],r.scanMaps[3],r.poseChanges);
 for(unsigned i=0;i<r.scanPoses.size();++i){auto p=r.scanPoses[i];
  printf("%s{\"x_mm\":%d,\"y_mm\":%d,\"heading_q6\":%u}",i?",":"",p.x,p.y,p.headingQ6);}
 puts("]}");
}
} // namespace
int main(){
 std::vector<unsigned>free;for(unsigned v=0;v<25;++v)if(!(islands&(1UL<<v)))free.push_back(v);
 std::vector<uint32_t>masks{0};for(unsigned i=0;i<free.size();++i){masks.push_back(1UL<<free[i]);
  for(unsigned j=i+1;j<free.size();++j){masks.push_back((1UL<<free[i])|(1UL<<free[j]));
   for(unsigned k=j+1;k<free.size();++k)masks.push_back((1UL<<free[i])|(1UL<<free[j])|(1UL<<free[k]));}}
 for(auto mask:masks)for(unsigned zone:{1U,2U}){
  auto r=simulate(zone,mask,mask);
  if(mask)assert((r.outcome==2)==missionFeasible(zone,mask));
  print("static_0_to_3",zone,mask,mask,r);
  if(mask){Options o;o.compress=true;o.ackFirst=false;o.fragment=7;
   r=simulate(zone,mask,0,o);assert((r.outcome==2)==missionFeasible(zone,mask));
   print("first_only_0_to_3",zone,mask,0,r);}
 }
 std::mt19937 random(20261119);
 for(unsigned i=0;i<512;++i){std::shuffle(free.begin(),free.end(),random);uint32_t m1=0,m2=0;unsigned count=4+i%7;
  for(unsigned k=0;k<count;++k)(k%2?m1:m2)|=1UL<<free[k];
  Options o;o.compress=i%2;o.ackFirst=i%3;o.fragment=1+i%17;o.noise=true;
  for(unsigned zone:{1U,2U})print("split_random_4_to_10",zone,m1,m2,simulate(zone,m1,m2,o));
 }
 for(unsigned zone:{1U,2U}){
  for(unsigned timing=0;timing<4;++timing){Options o;o.latency=timing*100;o.scanFrameMs=200;o.motionMs=1000;
   o.qrMs=timing?1200:0;o.workMs=2000;o.ackFirst=timing%2;o.compress=true;o.noise=true;o.wrap=timing==3;
   auto r=simulate(zone,1UL<<12,1UL<<12,o);assert(r.outcome==2);print("timing_wrap",zone,1UL<<12,1UL<<12,r);
  }
  for(unsigned i=0;i<50;++i){Options o;o.latency=random()%501;o.scanFrameMs=20+random()%231;
   o.motionMs=random()%1201;o.qrMs=random()%8001;o.workMs=random()%2501;
   o.ackFirst=i%2;o.fragment=1+random()%31;o.compress=true;o.noise=true;o.wrap=i%3==0;
   auto r=simulate(zone,1UL<<12,1UL<<12,o);assert(r.outcome==2);print("random_timing",zone,1UL<<12,1UL<<12,r);
  }
  for(int dx:{-33,0,33})for(int dy:{-5,0,5}){Options o;o.homeDx=dx;o.homeDy=dy;
   print("start_pose",zone,1UL<<12,1UL<<12,simulate(zone,1UL<<12,1UL<<12,o));}
  for(int fault=Options::NO_HELLO;fault<=Options::TX_FULL;++fault){Options o;o.fault=Options::Fault(fault);
   auto r=simulate(zone,1UL<<12,1UL<<12,o);assert(r.outcome==3&&r.clientError!=GridClient::OK);
   print("protocol_fault",zone,1UL<<12,1UL<<12,r);
  }
  Options timeout;timeout.qrMs=20000;auto r=simulate(zone,1UL<<12,1UL<<12,timeout);assert(r.outcome==3&&r.error==8);
  print("qr_timeout",zone,1UL<<12,1UL<<12,r);
  timeout={};timeout.scanFrameMs=600;r=simulate(zone,1UL<<12,1UL<<12,timeout);
  assert(r.outcome==3&&r.clientError==GridClient::TIMEOUT);print("scan_total_timeout",zone,1UL<<12,1UL<<12,r);
  auto clear=simulate(zone,0,1UL<<12);assert(clear.outcome==2&&clear.scans==4);print("empty_first_later_detection",zone,0,1UL<<12,clear);
  for(unsigned view=1;view<4;++view){Options o;o.customViews=true;o.views[view]=1UL<<12;o.motionMs=2000;o.latency=300;o.scanFrameMs=200;
   auto detected=simulate(zone,0,0,o);assert(detected.outcome==2&&detected.scans==4);
   print("obstacle_in_one_view",zone,0,1UL<<12,detected);
  }
  for(unsigned pose=1;pose<=5;++pose)for(auto fault:{Options::NO_POSE_ACK,Options::REJECT_POSE,Options::WRONG_POSE}){
   Options o;o.fault=fault;o.faultPose=pose;o.motionMs=1000;o.scanFrameMs=100;
   uint32_t mask=pose==5?0:1UL<<12;auto failed=simulate(zone,mask,mask,o);
   if(!(failed.outcome==3&&failed.clientError!=GridClient::OK&&failed.scans<=int(pose)))fprintf(stderr,"pose fault zone=%u pose=%u fault=%u outcome=%d client=%d scans=%d phase=%d error=%d\n",zone,pose,unsigned(fault),failed.outcome,failed.clientError,failed.scans,failed.phase,failed.error);
   assert(failed.outcome==3&&failed.clientError!=GridClient::OK&&failed.scans<=int(pose));
   print("pose_update_fault",zone,mask,mask,failed);
  }
  for(unsigned scan=2;scan<=4;++scan)for(auto fault:{Options::NO_GRID,Options::SCAN_OVERFLOW,Options::BAD_CRC,Options::STALE_SAVE,Options::BAD_ACK,Options::NO_SCAN_ACK}){
   Options o;o.fault=fault;o.faultScan=scan;auto failed=simulate(zone,1UL<<12,1UL<<12,o);
   assert(failed.outcome==3&&failed.clientError!=GridClient::OK&&failed.scans==int(scan));
   print("later_scan_fault",zone,1UL<<12,1UL<<12,failed);
  }
  simulate(zone,0,0,{},true); // Actual legacy controller is tested separately.
 }
}
