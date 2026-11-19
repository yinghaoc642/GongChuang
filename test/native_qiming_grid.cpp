#include "GridNavigator.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <vector>
using namespace qiming;
static void put16(std::vector<uint8_t>&p,size_t i,uint16_t v){p[i]=v;p[i+1]=v>>8;}
static void put32(std::vector<uint8_t>&p,size_t i,uint32_t v){put16(p,i,v);put16(p,i+2,v>>16);}
struct Device {
 GridClient client; std::deque<link_message_t> tx; uint32_t now=100,record=7,seq=0,map=0;
 uint32_t maps[4]={0,0,0,0}; std::vector<RadarPose> scanPoses,poseHistory; bool motionInFlight=false; int scans=0,stops=0; uint16_t savedHeading=170*64; uint8_t zone=1; RadarPose pose{234,2250,170*64,1};
 static bool writer(void*ctx,const uint8_t*b,size_t n){
  auto*d=static_cast<Device*>(ctx);link_decoder_t dec{};link_decoder_init(&dec);
  link_decoder_feed(&dec,b,n,[](const link_message_t*m,void*c){static_cast<Device*>(c)->tx.push_back(*m);},d);
  assert(dec.good_frames==1);return true;
 }
 void event(uint8_t type,std::vector<uint8_t> p,uint8_t node=1){
  uint8_t out[LINK_MAX_ENCODED_FRAME];size_t n=link_encode_message(node,type,123,p.data(),p.size(),out,sizeof out);assert(n);
  // Actual production framing, fragmented byte by byte across receive calls.
  for(size_t i=0;i<n;++i)client.receive(out+i,1,now);
 }
 void hello(){std::vector<uint8_t>p(13);p[0]=1;p[1]=35;p[2]=6;p[3]=zone;
  put16(p,4,pose.x);put16(p,6,pose.y);put16(p,8,pose.headingQ6);put16(p,11,720);event(LINK_EVT_HELLO,p);}
 void ack(uint8_t cmd){event(LINK_EVT_ACK,{cmd,0});}
 void grid(uint32_t mask,uint32_t sequence){std::vector<uint8_t>p(5);put32(p,0,sequence);
  for(uint8_t i=0;i<25;++i)if(mask&(1UL<<i))p.push_back(i);p[4]=p.size()-5;event(LINK_EVT_OBSTACLE_GRID,p);}
 void persisted(){std::vector<uint8_t>p(33);put16(p,0,11);put32(p,2,record);p[6]=2;p[9]=1;
  put16(p,11,234);put16(p,13,2250);put16(p,15,savedHeading);p[17]=1;
  put16(p,18,234);put16(p,20,150);put16(p,22,savedHeading);p[24]=1;
  for(uint8_t i=0;i<25;++i)if(map&(1UL<<i))p.push_back(i);p[10]=p.size()-33;event(LINK_EVT_PERSISTED_STATE,p);}
 void pump(){int guard=0;while(!tx.empty()){assert(++guard<30);auto m=tx.front();tx.pop_front();
  switch(m.type){
   case LINK_CMD_HELLO:hello();ack(m.type);break;
   case LINK_CMD_SELECT_ZONE:zone=m.payload[0];pose={234,int16_t(zone==1?2250:150),savedHeading,zone};hello();ack(m.type);break;
   case LINK_CMD_GET_PERSISTED_STATE:persisted();ack(m.type);break;
   case LINK_CMD_HEARTBEAT:ack(m.type);break;
   case LINK_CMD_STOP:++stops;break;
   case LINK_CMD_SET_POSE:assert(!motionInFlight);pose={int16_t(m.payload[1]|m.payload[2]<<8),int16_t(m.payload[3]|m.payload[4]<<8),uint16_t(m.payload[5]|m.payload[6]<<8),zone};poseHistory.push_back(pose);ack(m.type);break;
   case LINK_CMD_SINGLE_MEASURE:
    assert(!motionInFlight&&scans<4);scanPoses.push_back(pose);ack(m.type);event(LINK_EVT_STATUS,{1,2,1,0,0});
    for(uint8_t i=1;i<=14;++i)event(LINK_EVT_STATUS,{2,2,1,i,0});
    map=maps[scans++];grid(map,++seq);++record;
    // A delayed old scan must not replace the fresh map.
    if(seq>1)grid(1UL<<24,seq-1);
    event(LINK_EVT_STATUS,{4,2,0,14,0});break;
   case LINK_CMD_PLAN_PATH:{
    GridGeometry geometry;geometry.blocked=map|GridGeometry::permanent;GridPath path;
    bool ok=geometry.plan(m.payload[0],m.payload[1],0,path);
    std::vector<uint8_t>p(6);put16(p,0,m.sequence);p[2]=ok?0:4;p[3]=m.payload[0];p[4]=m.payload[1];p[5]=ok?path.count:0;
    if(ok)for(uint8_t i=0;i<path.count;++i)p.push_back(path.cells[i]);
    event(LINK_EVT_PATH_RESULT,p);ack(m.type);break;
   }
   default:assert(false);
  }
 }}
 void begin(uint8_t z=1){client.begin(writer,this,now,z);pump();assert(client.ready());}
};
static void geometryChecks(){
 GridGeometry g;assert(g.valid());for(int i=0;i<25;++i)assert(g.cell(g.center(i))==i);
 assert(g.rotationSafe({350,2100}));assert(g.rotationSafe({350,300}));assert(!g.rotationSafe({150,2250}));
 assert(!g.translationSafe({1200,1200},{720,720},0)); // fixed island even with arbitrary connector
 g.blocked|=1UL<<12;GridPath path;assert(g.plan(0,24,0,path));assert(g.pathValid(path,g.blocked));
 for(uint8_t i=0;i<path.count;++i)assert(path.cells[i]!=12);
 GridPath compressed{};compressed.start=10;compressed.goal=14;compressed.count=1;compressed.cells[0]=14;
 assert(!g.pathValid(compressed,g.blocked)); // occupied intermediate in compressed result
 g.blocked=GridGeometry::permanent;assert(g.pathValid(compressed,g.blocked));
 compressed.cells[0]=24;assert(!g.pathValid(compressed,0));
 g.blocked|=1UL<<0;assert(!g.plan(0,4,0,path));
 g.config.x[0]=1;assert(!g.valid());
 float dx,dy,f,r;bodyToWorld(123,45,90,dx,dy);worldToBody(dx,dy,90,f,r);assert(fabsf(f-123)<.001&&fabsf(r-45)<.001);
 // Independent physical direction checks: a mere transform/inverse roundtrip
 // cannot catch both functions mirroring CCW the same wrong way.
 bodyToWorld(0,-100,90,dx,dy);assert(fabsf(dx)<.001&&fabsf(dy-100)<.001); // legacy step6 LEFT -> RAW (+Y)
 bodyToWorld(0,100,90,dx,dy);assert(fabsf(dx)<.001&&fabsf(dy+100)<.001); // legacy step7 RIGHT -> PROCESS
 bodyToWorld(100,0,-90,dx,dy);assert(fabsf(dx-100)<.001&&fabsf(dy)<.001); // legacy step9 FORWARD -> STORAGE (+X)
 bodyToWorld(100,0,180,dx,dy);assert(fabsf(dx)<.001&&fabsf(dy-100)<.001);
}
static void mission(uint8_t zone,uint32_t map1,uint32_t map2,bool centeredDocks=false,bool originalWhenClear=false,int onlyView=-1,int headingSign=-1,int savedAngle=170){
 Device d;d.maps[0]=map1;for(int i=1;i<4;++i)d.maps[i]=map2;
 if(onlyView>=0)d.maps[onlyView]=1UL<<12;
 d.savedHeading=savedAngle*64;d.begin(zone);GridConfig config;config.radarHeadingPerCcw=headingSign;
 if(centeredDocks){config.raw={1200,2160};config.process={1200,240};config.storage={2160,1200};}
 GridMission nav(d.client,config);nav.begin(zone,originalWhenClear);
 bool completed=false,workDone=false,qr=false;unsigned work=0,departures=0,scanTurns=0,reset=0,moves=0,remaining=0;
 const uint8_t steps[6]={6,8,11,14,16,19};
 const int scanStep=zone==1?60:-60;
 for(unsigned tick=0;tick<4000&&nav.stage()!=GridMission::DONE;++tick){
  d.now+=20;if(remaining){completed=!--remaining;d.motionInFlight=remaining!=0;}d.client.poll(d.now);d.pump();
  NavAction a=nav.tick(d.now,completed,qr,workDone);completed=workDone=false;
  if(a.kind==NavAction::FAULT){printf("mission failure zone=%u stage=%u error=%u at %.1f %.1f heading %.1f\n",zone,nav.stage(),nav.error(),nav.position().x,nav.position().y,nav.heading());fflush(stdout);assert(false);}
  if(a.kind==NavAction::MOVE||a.kind==NavAction::TURN){
   if(a.departure){assert(fabsf(a.forward)>0.1f||fabsf(a.right)>0.1f||fabsf(a.degrees)>0.01f);++departures;}
   if(a.kind==NavAction::TURN&&nav.stage()==GridMission::SCAN_POSE){
    assert(scanTurns<2&&d.scans==int(2+scanTurns));
    assert(a.degrees==scanStep&&nav.heading()==scanStep*int(scanTurns));++scanTurns;
   }
   if(a.kind==NavAction::TURN&&nav.stage()==GridMission::RESTORE_HEADING){
    assert(scanTurns==2&&d.scans==4&&nav.heading()==2*scanStep);
    assert(a.degrees==-2*scanStep);++scanTurns;
   }
   if(a.kind==NavAction::MOVE)++moves;
   if(a.kind==NavAction::MOVE && nav.stage()==GridMission::OUT_FORWARD){
    assert(a.forward==(zone==1?150:-150)&&a.right==0); // inward exit from each zone
   }
   if(a.kind==NavAction::MOVE && nav.stage()==GridMission::ENTER_GRID){
    float dx,dy;bodyToWorld(a.forward,a.right,nav.heading(),dx,dy);
    Point end{nav.position().x+dx,nav.position().y+dy};
    assert(nav.geometry.cell(end)==(zone==1?20:0));
    assert(fabsf(end.x-240)<.01&&fabsf(end.y-(zone==1?2160:240))<.01);
   }
   if(a.kind==NavAction::MOVE && nav.stage()==GridMission::HOME){
    float dx,dy;bodyToWorld(a.forward,a.right,nav.heading(),dx,dy);
    assert(fabsf(nav.position().x+dx-150)<.01);
    assert(fabsf(nav.position().y+dy-(zone==1?2250:150))<.01);
   }
   remaining=5;d.motionInFlight=true;
  }
  if(a.kind==NavAction::QR_RESET){++reset;qr=true;}
  if(a.kind==NavAction::WORK){assert(work<6&&a.workStep==steps[work++]);
   assert(!nav.trackWorkstationMove(5000,0,0));
   assert(nav.trackWorkstationMove(2,1,0));assert(nav.trackWorkstationMove(-2,-1,0));
   assert(nav.alignWorkstationHeading(nav.heading()));workDone=true;
  }
 }
 assert(nav.stage()==GridMission::DONE&&work==6&&departures==6&&scanTurns==3&&reset==1&&moves>20);
 assert(d.scans==4);assert(nav.geometry.blocked==(map1|map2|(onlyView>=0?1UL<<12:0)|GridGeometry::permanent));
 assert(d.scanPoses.size()==4&&d.poseHistory.size()==4);
 const int16_t xs[]={234,434,392,308},ys[]={0,0,-73,-73};
 for(unsigned i=0;i<4;++i){const auto &p=d.scanPoses[i];
  assert(p.x==xs[i]&&p.y==(zone==1?(i?2100:2250):(i?300:150))+(zone==1?ys[i]:-ys[i]));
  int a=(savedAngle+headingSign*int(i?i-1:0)*scanStep+360)%360;assert(p.headingQ6==a*64);}
 assert(d.client.savedPose().x==234&&d.client.savedPose().headingQ6==savedAngle*64);
 assert(fabsf(nav.position().x-150)<.01&&fabsf(nav.position().y-(zone==1?2250:150))<.01);
 assert(d.pose.x==434&&d.pose.y==(zone==1?2100:300)&&d.pose.headingQ6==savedAngle*64);
 d.pump();assert(d.stops==1);
}
static void clearHandoff(uint8_t zone,uint32_t fixedOnly){
 Device d;for(auto &m:d.maps)m=fixedOnly;d.begin(zone);GridMission nav(d.client);nav.begin(zone,true);
 unsigned handoffs=0,moves=0,turns=0,remaining=0;bool done=false;
 for(unsigned i=0;i<200;++i){d.now+=20;if(remaining){done=!--remaining;d.motionInFlight=remaining!=0;}
  d.client.poll(d.now);d.pump();auto a=nav.tick(d.now,done,false,false);done=false;
  assert(a.kind!=NavAction::FAULT);
  if(a.kind==NavAction::MOVE||a.kind==NavAction::TURN){remaining=5;d.motionInFlight=true;
   if(a.kind==NavAction::MOVE)++moves;else {
    const int scanStep=zone==1?60:-60;
    assert(turns<3&&a.degrees==(turns<2?scanStep:-2*scanStep));++turns;
   }
  }
  if(a.kind==NavAction::CLEAR_ROUTE){assert(d.scans==4&&!d.motionInFlight);++handoffs;}
 }
 assert(handoffs==1&&d.scans==4&&moves==4&&turns==3&&nav.stage()==GridMission::CLEAR&&!nav.gridRouteSelected());
 assert(nav.position().x==150&&nav.position().y==(zone==1?2250:150)&&nav.heading()==0);
 assert(d.pose.x==234&&d.pose.y==(zone==1?2250:150)&&d.pose.headingQ6==170*64);
 assert(d.poseHistory.size()==5&&d.client.savedPose().x==234); // Session restored, calibration never saved.
}
static void protocolFailures(){
 {Device d;d.client.begin(Device::writer,&d,d.now,1);std::vector<uint8_t>p(13);p[0]=1;p[1]=34;p[2]=6;d.event(LINK_EVT_HELLO,p);assert(d.client.error()==GridClient::VERSION);}
 {Device d;d.begin();d.client.scan(d.now);d.ack(LINK_CMD_SINGLE_MEASURE);d.event(LINK_EVT_STATUS,{1,2,1,0,0});
  std::vector<uint8_t>p(7);put32(p,0,1);p[4]=2;p[5]=p[6]=12;d.event(LINK_EVT_OBSTACLE_GRID,p);assert(d.client.error()==GridClient::FRAME);}
 {Device d;d.begin();d.client.scan(d.now);d.ack(LINK_CMD_SINGLE_MEASURE);d.event(LINK_EVT_STATUS,{1,2,1,0,0});d.grid(0,1);d.event(LINK_EVT_STATUS,{4,2,0,14,0});
  // An unchanged Flash record cannot certify a new scan, even an empty one.
  d.persisted();assert(d.client.error()==GridClient::SAVED_GRID);}
 {Device d;d.begin();assert(d.client.plan(10,14,d.now));d.ack(LINK_CMD_PLAN_PATH);
  std::vector<uint8_t>p(7);p[2]=0;p[3]=10;p[4]=14;p[5]=1;p[6]=14;d.event(LINK_EVT_PATH_RESULT,p);
  GridPath path;assert(!d.client.takePath(path)); // wrong request sequence
  d.pump();assert(d.client.takePath(path)&&path.start==10&&path.goal==14);}
 {Device d;d.client.begin(Device::writer,&d,d.now,1);d.client.poll(d.now+1500);assert(d.client.error()==GridClient::TIMEOUT);}
 {Device d;d.begin();d.tx.clear();d.client.poll(d.now+5000);assert(d.client.error()==GridClient::LINK_LOST);}
 {Device d;d.begin();d.client.scan(d.now);d.ack(LINK_CMD_SINGLE_MEASURE);d.event(LINK_EVT_STATUS,{2,2,1,1,1});assert(d.client.error()==GridClient::SCAN_OVERFLOW);}
 {Device d;d.begin();d.client.scan(d.now);d.ack(LINK_CMD_SINGLE_MEASURE);uint32_t m;assert(!d.client.takeGrid(m));d.client.poll(d.now+3500);assert(d.client.error()==GridClient::TIMEOUT);}
 {Device d;d.begin();d.zone=2;d.hello();assert(d.client.error()==GridClient::STATE);}
 {Device d;d.begin();uint8_t bad[]={2,1,0};d.client.receive(bad,sizeof bad,d.now);assert(d.client.error()==GridClient::FRAME);}
 {Device d;d.begin();d.event(LINK_EVT_ERROR,{1},2);assert(d.client.error()==GridClient::OK);d.event(LINK_EVT_ERROR,{1});assert(d.client.error()==GridClient::DEVICE);}
 {Device d;d.begin();d.client.scan(d.now);d.ack(LINK_CMD_SINGLE_MEASURE);d.event(LINK_EVT_STATUS,{1,2,1,0,0});d.grid(0,1);d.event(LINK_EVT_STATUS,{4,2,0,13,0});uint32_t m;assert(!d.client.takeGrid(m));}
 {Device d;d.begin();GridMission nav(d.client);nav.begin(1);for(int i=0;i<200;++i){d.now+=200;d.client.poll(d.now);d.pump();auto a=nav.tick(d.now,true,false,false);if(a.kind==NavAction::FAULT)break;}assert(nav.error()==8);}
 {Device d;d.maps[0]=1UL<<20;d.begin();GridMission nav(d.client);nav.begin(1);for(int i=0;i<5;++i){d.pump();nav.tick(d.now,true,true,false);}assert(nav.stage()==GridMission::FAILED);}
}
int main(){geometryChecks();mission(1,0,0);mission(2,0,0);mission(1,1UL<<12,0);mission(2,0,1UL<<12);mission(1,0,0,true);
 mission(2,1UL<<12,0,false,true); // production mode: detected obstacle takes the full zone2 detour mission
 clearHandoff(1,0);clearHandoff(2,0);clearHandoff(2,GridGeometry::permanent);
 for(uint8_t zone:{1,2})for(int view=1;view<4;++view)for(int sign:{-1,1})mission(zone,0,0,false,true,view,sign);
 for(uint8_t zone:{1,2})for(int sign:{-1,1})for(int angle:{10,350})mission(zone,0,0,false,true,3,sign,angle);
 protocolFailures();puts("Qiming protocol, clear-route handoff, grid geometry and two-zone six-station missions PASS");}
