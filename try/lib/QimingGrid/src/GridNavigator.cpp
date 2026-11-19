#include "GridNavigator.h"
#include <math.h>
#include <string.h>
namespace qiming {
namespace {
constexpr float radians = 0.017453292519943295f;
bool finite(Point p) { return isfinite(p.x) && isfinite(p.y); }
bool overlap(float a, float b, float c, float d) { return a < d && b > c; }
// Two independent launch-zone plans in the SAME field coordinate system.
// Workstations are shared. Mirror the entry/return and scan turn direction.
struct LaunchPlan { float minY, inwardForward; uint8_t entryCell; int8_t scanStepDegrees; };
constexpr LaunchPlan launchPlans[2] = {
  {2100, 150, 20, 60}, // zone1: forward -> right -> CCW scans -> cell20
  {0, -150, 0, -60}    // zone2: backward -> right -> CW scans -> cell0
};
}
void bodyToWorld(float f, float r, float h, float &dx, float &dy) {
  const float s = sinf(h*radians), c = cosf(h*radians);
  // Map +X points left in the field drawing. CCW turns forward toward
  // body LEFT (-right), hence the minus-sine terms in this mirrored map.
  dx = -f*s + r*c; dy = -f*c - r*s;
}
void worldToBody(float dx, float dy, float h, float &f, float &r) {
  const float s = sinf(h*radians), c = cosf(h*radians);
  f = -dx*s - dy*c; r = dx*c - dy*s;
}
float shortTurn(float d) { while(d>180)d-=360; while(d<-180)d+=360; return d; }
uint8_t GridGeometry::id(uint8_t c, uint8_t r) { return r*5 + (r%2 ? 4-c : c); }
uint8_t GridGeometry::column(uint8_t v) { return row(v)%2 ? 4-v%5 : v%5; }
bool GridGeometry::valid() const {
  if (!isfinite(config.halfForward) || !isfinite(config.halfRight) || !isfinite(config.margin) ||
      config.halfForward<=0 || config.halfRight<=0 || config.margin<0 ||
      !isfinite(config.sensorForward) || !isfinite(config.sensorRight) ||
      (config.radarHeadingPerCcw!=-1 && config.radarHeadingPerCcw!=1)) return false;
  for (uint8_t i=0;i<6;++i) {
    if (!isfinite(config.x[i]) || !isfinite(config.y[i]) || config.x[i]<0 || config.x[i]>2400 ||
        config.y[i]<0 || config.y[i]>2400) return false;
    if (i && (config.x[i]-config.x[i-1]<300 || config.y[i]-config.y[i-1]<300)) return false;
  }
  if(config.x[0]!=0 || config.y[0]!=0 || config.x[5]!=2400 || config.y[5]!=2400)return false;
  return finite(config.qr) && finite(config.raw) && finite(config.process) && finite(config.storage);
}
uint8_t GridGeometry::cell(Point p) const {
  for (uint8_t r=0;r<5;++r) for(uint8_t c=0;c<5;++c)
    if(p.x>=config.x[c] && p.x<config.x[c+1] && p.y>=config.y[r] && p.y<config.y[r+1])return id(c,r);
  return 255;
}
Point GridGeometry::center(uint8_t v) const {
  if(v>24)return {NAN,NAN};
  const uint8_t c=column(v),r=row(v);
  return {(config.x[c]+config.x[c+1])*0.5f,(config.y[r]+config.y[r+1])*0.5f};
}
bool GridGeometry::rectangleFree(float l,float b,float r,float t) const {
  if(!isfinite(l)||!isfinite(b)||!isfinite(r)||!isfinite(t)||l < -0.01f||b < -0.01f||r>2400.01f||t>2400.01f)return false;
  // Actual field islands: 550..1000 / 1400..1850 in each axis (drawing 5-3).
  for(uint8_t y=0;y<2;++y)for(uint8_t x=0;x<2;++x){
    const float ix=x?1400:550,iy=y?1400:550;
    if(overlap(l,r,ix,ix+450)&&overlap(b,t,iy,iy+450))return false;
  }
  for(uint8_t v=0;v<25;++v)if(blocked&(1UL<<v)){
    const uint8_t x=column(v),y=row(v);
    if(overlap(l,r,config.x[x],config.x[x+1])&&overlap(b,t,config.y[y],config.y[y+1]))return false;
  }
  return true;
}
bool GridGeometry::translationSafe(Point a,Point b,float h) const {
  if(!finite(a)||!finite(b)||!isfinite(h))return false;
  const float c=fabsf(cosf(h*radians)),s=fabsf(sinf(h*radians));
  const float hx=c*config.halfRight+s*config.halfForward+config.margin;
  const float hy=s*config.halfRight+c*config.halfForward+config.margin;
  return rectangleFree(fminf(a.x,b.x)-hx,fminf(a.y,b.y)-hy,
                       fmaxf(a.x,b.x)+hx,fmaxf(a.y,b.y)+hy);
}
bool GridGeometry::rotationSafe(Point p) const {
  const float r=hypotf(config.halfForward,config.halfRight)+config.margin;
  if(!finite(p)||p.x<r||p.y<r||p.x+r>2400||p.y+r>2400)return false;
  // A full rotation sweeps a disk. Testing its bounding square would falsely
  // reject the intended startup spin next to a diagonally adjacent grid cell.
  auto hits=[&](float l,float b,float right,float top){
    const float dx=fmaxf(l-p.x,fmaxf(0,p.x-right));
    const float dy=fmaxf(b-p.y,fmaxf(0,p.y-top));
    return dx*dx+dy*dy<r*r;
  };
  for(uint8_t y=0;y<2;++y)for(uint8_t x=0;x<2;++x){
    const float ix=x?1400:550,iy=y?1400:550;
    if(hits(ix,iy,ix+450,iy+450))return false;
  }
  for(uint8_t v=0;v<25;++v)if(blocked&(1UL<<v)){
    const uint8_t x=column(v),y=row(v);
    if(hits(config.x[x],config.y[y],config.x[x+1],config.y[y+1]))return false;
  }
  return true;
}
bool GridGeometry::pathValid(const GridPath &p,uint32_t map) const {
  if(p.status||p.start>24||p.goal>24||p.count>25)return false;
  map |= permanent;
  if((map&(1UL<<p.start))||(map&(1UL<<p.goal)))return false;
  uint8_t previous=p.start;
  uint32_t seen=1UL<<previous;
  for(uint8_t i=0;i<p.count;++i){
    const uint8_t v=p.cells[i];
    if(v>24 || (seen&(1UL<<v)))return false;
    int x=column(previous),y=row(previous),ex=column(v),ey=row(v);
    if((x!=ex&&y!=ey)||(x==ex&&y==ey))return false;
    const int dx=(ex>x)-(ex<x),dy=(ey>y)-(ey<y);
    while(x!=ex||y!=ey){
      x+=dx;y+=dy;const uint8_t mid=id(x,y);
      if((map&(1UL<<mid))||(seen&(1UL<<mid)))return false;
      seen|=1UL<<mid;
    }
    previous=v;
  }
  return previous==p.goal && ((p.start==p.goal)==(p.count==0));
}
bool GridGeometry::plan(uint8_t start,uint8_t goal,float heading,GridPath &out) const {
  if(start>24||goal>24||((blocked|permanent)&((1UL<<start)|(1UL<<goal))))return false;
  uint8_t parent[25],queue[25],head=0,tail=0;
  memset(parent,255,sizeof(parent));parent[start]=start;queue[tail++]=start;
  while(head<tail&&parent[goal]==255){
    const uint8_t v=queue[head++];const int x=column(v),y=row(v);
    const int8_t dx[4]={1,0,-1,0},dy[4]={0,1,0,-1};
    for(uint8_t i=0;i<4;++i){
      const int nx=x+dx[i],ny=y+dy[i];if(nx<0||nx>4||ny<0||ny>4)continue;
      const uint8_t next=id(nx,ny);
      if(parent[next]!=255||((blocked|permanent)&(1UL<<next))||
         !translationSafe(center(v),center(next),heading))continue;
      parent[next]=v;queue[tail++]=next;
    }
  }
  if(parent[goal]==255)return false;
  out={};out.start=start;out.goal=goal;
  uint8_t reverse[25],n=0;
  for(uint8_t v=goal;v!=start;v=parent[v])reverse[n++]=v;
  out.count=n;
  for(uint8_t i=0;i<n;++i)out.cells[i]=reverse[n-1-i];
  return true;
}
void GridMission::begin(uint8_t zone, bool originalWhenClear) {
  stage_=CONNECT;zone_=zone;visit_=pathIndex_=qrLeg_=error_=0;
  originalWhenClear_=originalWhenClear;
  latestMap_=0;scanView_=0;gridSelected_=false;moving_=departure_=false;heading_=0;
  geometry.blocked=GridGeometry::permanent;
}
NavAction GridMission::fault(uint8_t code) {
  error_=code;stage_=FAILED;moving_=false;client_.stop();
  NavAction a;a.kind=NavAction::FAULT;return a;
}
NavAction GridMission::move(Point p,Stage next) {
  if(!geometry.translationSafe(position_,p,heading_))return fault(3);
  NavAction a;a.kind=NavAction::MOVE;
  worldToBody(p.x-position_.x,p.y-position_.y,heading_,a.forward,a.right);
  if(fabsf(a.forward)>0.1f||fabsf(a.right)>0.1f){
    a.departure=departure_;departure_=false;
  }
  endpoint_=p;endHeading_=heading_;moving_=true;stage_=next;return a;
}
NavAction GridMission::turn(float degrees,Stage next) {
  if(fabsf(degrees)>0.01f&&!geometry.rotationSafe(position_))return fault(4);
  NavAction a;a.kind=NavAction::TURN;a.degrees=degrees;
  if(fabsf(degrees)>0.01f){a.departure=departure_;departure_=false;}
  endpoint_=position_;endHeading_=heading_+degrees;moving_=true;stage_=next;return a;
}
NavAction GridMission::updatePose(uint32_t now,Stage next) {
  RadarPose p=client_.savedPose();
  float dx,dy;
  bodyToWorld(geometry.config.sensorForward,geometry.config.sensorRight,heading_,dx,dy);
  p.x=lroundf(position_.x+dx);p.y=lroundf(position_.y+dy);
  int32_t angle=int32_t(p.headingQ6)+lroundf(heading_*64)*geometry.config.radarHeadingPerCcw;
  angle%=23040;if(angle<0)angle+=23040;p.headingQ6=uint16_t(angle);
  if(!client_.setPose(p,now))return fault(5);stage_=next;return {};
}
Point GridMission::destination() const {
  switch(visit_){case 0:return geometry.config.qr;case 1:case 4:return geometry.config.raw;
    case 2:case 5:return geometry.config.process;case 3:case 6:return geometry.config.storage;default:return home_;}
}
float GridMission::destinationHeading() const {
  switch(visit_){case 1:case 4:return 90;case 2:case 5:return -90;
    case 3:case 6:return -180;default:return 0;}
}
NavAction GridMission::requestPath(uint32_t now) {
  if(!client_.plan(geometry.cell(position_),geometry.cell(destination()),now))return fault(5);
  stage_=WAIT_PATH;return {};
}
bool GridMission::trackWorkstationMove(float f,float r,float d) {
  if(stage_!=AT_WORK||!isfinite(f)||!isfinite(r)||!isfinite(d))return false;
  if(fabsf(d)>0.001f && (fabsf(f)>0.1f||fabsf(r)>0.1f||!geometry.rotationSafe(position_)))return false;
  float dx,dy;bodyToWorld(f,r,heading_,dx,dy);
  Point target{position_.x+dx,position_.y+dy};
  if(!geometry.translationSafe(position_,target,heading_))return false;
  position_=target;heading_+=d;return true;
}
bool GridMission::alignWorkstationHeading(float h) {
  if(stage_!=AT_WORK||!isfinite(h)||fabsf(shortTurn(h-heading_))>5||
     !geometry.translationSafe(position_,position_,h))return false;
  heading_=h;return true;
}
NavAction GridMission::tick(uint32_t now,bool motionFinished,bool qrReady,bool workFinished) {
  if(stage_==IDLE||stage_==DONE||stage_==FAILED||stage_==CLEAR)return {};
  if(client_.error()!=GridClient::OK)return fault(1);
  if(moving_){if(!motionFinished)return {};position_=endpoint_;heading_=endHeading_;moving_=false;}
  switch(stage_){
    case CONNECT: {
      if(zone_!=1&&zone_!=2)return fault(2);
      if(!client_.ready())return {};
      if(!geometry.valid())return fault(2);
      const RadarPose &p=client_.savedPose();
      home_={p.x-geometry.config.sensorRight,p.y+geometry.config.sensorForward};
      position_=home_;
      // Launch/finish footprint must fit completely inside the selected 300mm square.
      const float minY=launchPlans[zone_-1].minY;
      if(home_.x<geometry.config.halfRight || home_.x>300-geometry.config.halfRight ||
         home_.y<minY+geometry.config.halfForward || home_.y>minY+300-geometry.config.halfForward ||
         !geometry.translationSafe(home_,home_,0))return fault(2);
      if(!client_.scan(now))return fault(5);stage_=SCAN1;return {};
    }
    case SCAN1:
      if(!client_.takeGrid(latestMap_))return {};
      geometry.blocked|=latestMap_;
      // Safety pre-scan never bypasses the three views, even when empty.
      return move({position_.x,position_.y-launchPlans[zone_-1].inwardForward},OUT_FORWARD);
    case OUT_FORWARD:return move({position_.x+200,position_.y},OUT_RIGHT);
    case OUT_RIGHT:case SCAN_POSE:return updatePose(now,POSE_UPDATE);
    case POSE_UPDATE:
      if(!client_.takePoseDone())return {};
      if(!client_.scan(now))return fault(5);stage_=SCAN_VIEW;return {};
    case SCAN_VIEW:
      if(!client_.takeGrid(latestMap_))return {};
      // Each device scan replaces its grid; preserve ALL four scans in RAM.
      geometry.blocked|=latestMap_;
      if(scanView_<2){++scanView_;return turn(launchPlans[zone_-1].scanStepDegrees,SCAN_POSE);}
      return turn(-2.0f*launchPlans[zone_-1].scanStepDegrees,RESTORE_HEADING);
    case RESTORE_HEADING:return updatePose(now,RESTORE_POSE);
    case RESTORE_POSE:
      if(!client_.takePoseDone())return {};
      if(originalWhenClear_ && !(geometry.blocked & ~GridGeometry::permanent))
        return move({home_.x,position_.y},CLEAR_LATERAL);
      gridSelected_=true;
      if(geometry.cell(position_)!=launchPlans[zone_-1].entryCell)return fault(2);
      return move(geometry.center(geometry.cell(position_)),ENTER_GRID);
    case CLEAR_LATERAL:return move(home_,CLEAR_HOME);
    case CLEAR_HOME:return updatePose(now,CLEAR_POSE);
    case CLEAR_POSE:
      if(!client_.takePoseDone())return {};
      // Original controller may start only after returning to its launch pose.
      stage_=CLEAR;client_.stop();{NavAction a;a.kind=NavAction::CLEAR_ROUTE;return a;}
    case ENTER_GRID:case EGRESS:return requestPath(now);
    case WAIT_PATH: {
      GridPath candidate;if(!client_.takePath(candidate))return {};
      if(!geometry.pathValid(candidate,latestMap_))return fault(6);
      // Replan on the UNION, including footprint clearance. Only orthogonal grid edges.
      if(!geometry.plan(candidate.start,candidate.goal,heading_,path_))return fault(7);
      pathIndex_=0;stage_=FOLLOW;return {};
    }
    case FOLLOW:
      if(pathIndex_<path_.count)return move(geometry.center(path_.cells[pathIndex_++]),FOLLOW);
      return turn(shortTurn(destinationHeading()-heading_),DOCK_TURN);
    case DOCK_TURN:return move(destination(),visit_==7?HOME:DOCK);
    case DOCK:
      if(visit_==0){qrOrigin_=position_;qrStart_=now;qrLeg_=0;stage_=QR_WAIT;
        NavAction a;a.kind=NavAction::QR_RESET;return a;}
      stage_=AT_WORK;{static const uint8_t steps[6]={6,8,11,14,16,19};
        NavAction a;a.kind=NavAction::WORK;a.workStep=steps[visit_-1];return a;}
    case QR_WAIT:case QR_SEARCH: {
      if(qrReady){return move(qrOrigin_,QR_RETURN);}
      if(uint32_t(now-qrStart_)>=10000)return fault(8);
      if(uint32_t(now-qrStart_)<250)return {};
      // Bounded 20mm increments within the QR side lane; no cross-field sweep.
      const float targets[3]={100,-100,0};
      const float target=qrOrigin_.y+targets[qrLeg_];
      if(fabsf(position_.y-target)<0.1f)qrLeg_=(qrLeg_+1)%3;
      const float remaining=qrOrigin_.y+targets[qrLeg_]-position_.y;
      return move({position_.x,position_.y+fmaxf(-20,fminf(20,remaining))},QR_SEARCH);
    }
    case QR_RETURN:++visit_;return move(geometry.center(geometry.cell(position_)),EGRESS);
    case AT_WORK:
      if(!workFinished)return {};
      ++visit_;departure_=true;
      return move(geometry.center(geometry.cell(position_)),EGRESS);
    case HOME:stage_=DONE;client_.stop();{NavAction a;a.kind=NavAction::FINISH;return a;}
    default:return fault(9);
  }
}
} // namespace qiming
