// Chassis-only grid mission. No arm/servo/vision initialization or copied route.
#include <Arduino.h>
#include <AccelStepper.h>
#include <OneButton.h>
#include <Stm32SerialCompat.h>
#include <MecanumKinematics.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "radar_navigation_config.h"
namespace route_chassis {
#include "route_chassis_main.inc"
}
#include "radar_navigation.inc"
namespace rc = route_chassis;
namespace rn = radar_navigation;
void gridDebugFault() {
  rc::SerialDebug.print("[GRID DEBUG FAULT] nav/link=");
  rc::SerialDebug.print(rn::mission.error());rc::SerialDebug.print('/');
  rc::SerialDebug.println(rn::client.error());
  rn::stop();rc::disableDriveMotors();rc::programState=rc::PROGRAM_FAULT;
  rc::hmiSetRunStatus("FAULT");
}
void setup() {
  rc::setup();rn::setup();
  rc::SerialDebug.println("[GRID DEBUG] wheels only; six workstation actions skipped");
}
void loop() {
  if (!rn::serviceLink() && rc::programState==rc::PROGRAM_RUNNING) {gridDebugFault();return;}
  rc::runAllMotors();rc::startButton.tick();rc::receiveHmiStartCommand();rc::receiveImuData();
  if (rc::programState==rc::PROGRAM_RUNNING) rc::receiveQrData();
  rc::updateHmiYaw();rc::serviceBatteryVoltage();
  if(rc::abortRequested){rn::stop();rc::abortRoute();return;}
  if(rc::startRequested && rc::programState==rc::PROGRAM_WAITING &&
     uint32_t(millis()-rc::startRequestAcceptedMs)>=500){
    rc::beginRoute();
    if(rc::programState==rc::PROGRAM_RUNNING){
      navigation::gridMotionOwned=false;
      if(!navigation::mode.start(rc::selectedStartZoneNumber)){gridDebugFault();return;}
      if(navigation::mode.selected()==qiming::NavigationMode::RADAR)rn::begin(rc::selectedStartZoneNumber);
    }
  }
  if(rc::programState!=rc::PROGRAM_RUNNING)return;
  if(navigation::mode.state()==qiming::NavigationMode::FIXED_LEGACY ||
     navigation::mode.state()==qiming::NavigationMode::FIXED_CLEAR){rc::updateRoute();return;}
  const bool done=rn::serviceMotion();
  if(rc::programState==rc::PROGRAM_FAULT){gridDebugFault();return;}
  const qiming::NavAction a=rn::mission.tick(millis(),done,rc::scanFlag,true);
  switch(a.kind){
   case qiming::NavAction::MOVE:case qiming::NavAction::TURN:
    navigation::gridMotionOwned=true;
    if(navigation::mode.state()==qiming::NavigationMode::SCANNING&&rn::mission.gridRouteSelected())navigation::mode.mapAccepted(true);
    rn::startMotion(a);break;
   case qiming::NavAction::CLEAR_ROUTE:rn::stop();navigation::mode.mapAccepted(false);navigation::gridMotionOwned=false;break;
   case qiming::NavAction::QR_RESET:rc::resetQrReceiver();break;
   case qiming::NavAction::WORK:rc::SerialDebug.print("[GRID DEBUG] dock step=");rc::SerialDebug.println(a.workStep);break;
   case qiming::NavAction::FAULT:gridDebugFault();break;
   case qiming::NavAction::FINISH:rn::stop();rc::disableDriveMotors();rc::programState=rc::PROGRAM_FINISHED;rc::hmiSetRunStatus("FINISH");break;
   default:break;
  }
}
