//-------------------------------------------------------------------------------------------
/*! \file    arm7_door_push.cpp
    \brief   ODE 7-joint chain simulation with door and pushable object.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Oct.28, 2015
*/
//-------------------------------------------------------------------------------------------
#include "ode1/arm7_door_push.h"
#include <iostream>
#include <cassert>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;


//-------------------------------------------------------------------------------------------
namespace ode_x
{

int MaxContacts(4);  // maximum number of contact points per body
double VizJointLen(0.1);
double VizJointRad(0.02);
int    JointNum(7);
bool   FixedBase(true);
double TotalArmLen(1.0);
double LinkRad(0.03);
double FSThick(0.001);
double FSSize(0.004);
double BaseLenX(0.20);
double BaseLenY(0.30);
double BaseLenZ(0.15);
double GBaseLenX(0.08);
double GBaseLenY(0.16);
double GBaseLenZ(0.05);
double GripLenY(0.02);
double GripLenZ(0.12);
int ObjectMode(0);  // 0: None, 1: Box1, ...
double Box1PosX(0.7);
double Box1PosY(0.0);
double Box1SizeX(0.5);
double Box1SizeY(0.6);
double Box1SizeZ(0.4);
double Box1Density1(1.0);
double Box1Density2(50.0);
double TimeStep(0.04);
double Gravity(-1.0);
bool   EnableKeyEvent(true);  // If we use a default key events.

double HingeFMax(100.0);
double SliderFMax(1.0);
int ControlMode(0);  // 0: Position, 1: Velocity, 2: Torque/Force
std::vector<double> TargetAngles(JointNum,0.0);
std::vector<double> TargetGPos(2,0.0);
std::vector<double> TargetVel(JointNum,0.0);
std::vector<double> TargetGVel(2,0.0);
std::vector<double> TargetTorque(JointNum,0.0);
std::vector<double> TargetGForce(2,0.0);

bool Running(true);
void (*SensingCallback)(const TSensors2 &sensors)= NULL;
void (*DrawCallback)(void)= NULL;
void (*StepCallback)(const double &time, const double &time_step)= NULL;
void (*KeyEventCallback)(int command)= NULL;
//-------------------------------------------------------------------------------------------

static TEnvironment *Env(NULL);
static void NearCallback(void*,dGeomID,dGeomID);
//-------------------------------------------------------------------------------------------


dReal IndexedColors[][4]= {
    /*0*/ {1.0, 0.0, 0.0, 0.8},
          {0.0, 1.0, 0.0, 0.8},
          {0.0, 0.0, 1.0, 0.8},
    /*3*/ {1.0, 1.0, 0.0, 0.8},
          {1.0, 0.0, 1.0, 0.8},
          {0.0, 1.0, 1.0, 0.8},
    /*6*/ {1.0, 0.0, 0.0, 0.3},
          {0.0, 1.0, 0.0, 0.3},
          {0.0, 0.0, 1.0, 0.3},
    /*9*/ {1.0, 1.0, 0.0, 0.3},
          {1.0, 0.0, 1.0, 0.3},
          {0.0, 1.0, 1.0, 0.3},
    /*12*/{1.0, 1.0, 1.0, 0.8}};
inline void SetColor(int i)
{
  dReal *col= IndexedColors[i];
  dsSetColorAlpha(col[0],col[1],col[2],col[3]);
}
//-------------------------------------------------------------------------------------------

/* ODE::dBody pose to pose.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_array>
inline void ODEBodyToX(const dBody &body, t_array x)
{
  const dReal *p= body.getPosition();  // x,y,z
  const dReal *q= body.getQuaternion();  // qw,qx,qy,qz
  x[0]= p[0]; x[1]= p[1]; x[2]= p[2];
  x[3]= q[1]; x[4]= q[2]; x[5]= q[3]; x[6]= q[0];
}
//-------------------------------------------------------------------------------------------

/* ODE::dJointFeedback to force/torque.
    f: [0-2]: force, [3-5]: torque. */
template <typename t_array>
inline void ODEFeedbackToF(const dJointFeedback &feedback, t_array f)
{
  f[0]= feedback.f1[0]; f[1]= feedback.f1[1]; f[2]= feedback.f1[2];
  f[3]= feedback.t1[0]; f[4]= feedback.t1[1]; f[5]= feedback.t1[2];
}
//-------------------------------------------------------------------------------------------



//===========================================================================================
// class TDynRobot
//===========================================================================================

void TDynRobot::Draw()
{
  dsSetTexture (DS_WOOD);

  dReal rad, len;
  dReal sides[4];
  for (std::vector<TNCBox>::const_iterator itr(link_b_.begin()),last(link_b_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getLengths(sides);
    dsDrawBox (itr->getPosition(), itr->getRotation(), sides);
  }
  for (std::vector<TNCSphere>::const_iterator itr(link_sp_.begin()),last(link_sp_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    dsDrawSphere (itr->getPosition(), itr->getRotation(), itr->getRadius());
  }
  for (std::vector<TNCCapsule>::const_iterator itr(link_ca_.begin()),last(link_ca_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getParams(&rad, &len);
    dsDrawCapsule (itr->getPosition(), itr->getRotation(), len,rad);
  }
  for (std::vector<TNCCylinder>::const_iterator itr(link_cy_.begin()),last(link_cy_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getParams(&rad, &len);
    dsDrawCylinder (itr->getPosition(), itr->getRotation(), len,rad);
  }
  for (std::vector<TTriMeshGeom>::const_iterator itr(link_tm_.begin()),last(link_tm_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    // const dReal *pos = itr->getPosition();
    // const dReal *rot = itr->getRotation();
    const dVector3 pos={0,0,0,0};
    const dMatrix3 rot={1,0,0,0 ,0,1,0,0, 0,0,1,0};

    for (int i(dGeomTriMeshGetTriangleCount(*itr)); i>0; --i)
    {
      std::cerr<<"i:"<<i<<std::endl;
      dVector3 v[3];
      dGeomTriMeshGetTriangle(*itr, i-1, &v[0], &v[1], &v[2]);
      std::cerr<<v[0][0]<<" "<<v[0][1]<<" "<<v[0][2]<<std::endl;
      std::cerr<<v[1][0]<<" "<<v[1][1]<<" "<<v[1][2]<<std::endl;
      std::cerr<<v[2][0]<<" "<<v[2][1]<<" "<<v[2][2]<<std::endl;
      dsDrawTriangle(pos, rot, v[0], v[1], v[2], 1);
    }
  }

  dVector3 pos,axis;
  dMatrix3 rot;
  for (std::vector<TNCHingeJoint>::const_iterator itr(joint_h_.begin()),last(joint_h_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getAnchor(pos);
    itr->getAxis(axis);
    dRFromAxisAndAngle (rot,-axis[1],axis[0],0.0, 0.5*M_PI-std::asin(axis[2]));
    dsDrawCylinder (pos, rot, VizJointLen,VizJointRad);
  }

  // for (std::vector<TNCSliderJoint>::const_iterator itr(joint_s_.begin()),last(joint_s_.end()); itr!=last; ++itr)
  // {
    // SetColor(itr->ColorCode);
    // // itr->getAnchor(pos);
    // const dReal *p1(dBodyGetPosition(itr->getBody(0))), *p2(dBodyGetPosition(itr->getBody(1)));
    // for(int d(0); d<3; ++d)  pos[d]= 0.5*(p1[d]+p2[d]);
    // itr->getAxis(axis);
    // dRFromAxisAndAngle (rot,-axis[1],axis[0],0.0, 0.5*M_PI-std::asin(axis[2]));
    // dsDrawCylinder (pos, rot, VizJointLen, VizJointRad);
  // }
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class TJointChain1 : public TDynRobot
//===========================================================================================

// dJointFeedback joint_feedback1;  // TEST

/*override*/void TJointChain1::Create(dWorldID world, dSpaceID space)
{
  body_.clear();
  link_b_.clear();
  link_ca_.clear();
  link_cy_.clear();
  link_sp_.clear();
  link_tm_.clear();
  joint_b_.clear();
  joint_h_.clear();
  joint_h2_.clear();
  joint_s_.clear();
  joint_f_.clear();
  feedback_.clear();

  body_.resize(
      /*base,gripper:*/4
      + /*force sensors:*/8
      + /*chain:*/JointNum+1);
  link_b_.resize(/*base,gripper:*/4 + /*force sensors:*/8);
  link_cy_.resize(/*chain:*/JointNum+1);
  int ib(0);  // body counter

  // create boxes
  dReal h_grip_begin= 3.0*FSThick+BaseLenZ+TotalArmLen;
  dReal h_chain_begin= 2.0*FSThick+BaseLenZ;
  dReal box_size_pos[12][6]={
      // base:
      /* 0*/{BaseLenX,BaseLenY,BaseLenZ,  0.0,0.0,FSThick+0.5*BaseLenZ},
      // gripper base, gripper-1, gripper-2:
      /* 1*/{GBaseLenX,GBaseLenY,GBaseLenZ, 0.0,0.0,h_grip_begin+0.5*GBaseLenZ},
      /* 2*/{GBaseLenX,GripLenY,GripLenZ,   0.0,+0.5*(GBaseLenY-GripLenY),h_grip_begin+GBaseLenZ+0.5*GripLenZ},
      /* 3*/{GBaseLenX,GripLenY,GripLenZ,   0.0,-0.5*(GBaseLenY-GripLenY),h_grip_begin+GBaseLenZ+0.5*GripLenZ},
      // base force sensors:
      /* 4*/{FSSize,FSSize,FSThick,  +0.5*(BaseLenX-FSSize),+0.5*(BaseLenY-FSSize),0.5*FSThick},
      /* 5*/{FSSize,FSSize,FSThick,  +0.5*(BaseLenX-FSSize),-0.5*(BaseLenY-FSSize),0.5*FSThick},
      /* 6*/{FSSize,FSSize,FSThick,  -0.5*(BaseLenX-FSSize),-0.5*(BaseLenY-FSSize),0.5*FSThick},
      /* 7*/{FSSize,FSSize,FSThick,  -0.5*(BaseLenX-FSSize),+0.5*(BaseLenY-FSSize),0.5*FSThick},
      // force sensor between base and arm chain:
      /* 8*/{FSSize,FSSize,FSThick,  0.0,0.0,FSThick+BaseLenZ+0.5*FSThick},
      // force sensor between arm chain and gripper base:
      /* 9*/{FSSize,FSSize,FSThick,  0.0,0.0,h_chain_begin+TotalArmLen+0.5*FSThick},
      // force sensors on grippers:
      /*10*/{GBaseLenX,FSThick,GripLenZ,   0.0,+(0.5*GBaseLenY-GripLenY-0.5*FSThick),h_grip_begin+GBaseLenZ+0.5*GripLenZ},
      /*11*/{GBaseLenX,FSThick,GripLenZ,   0.0,-(0.5*GBaseLenY-GripLenY-0.5*FSThick),h_grip_begin+GBaseLenZ+0.5*GripLenZ}
    };
  for(int i(0); i<12; ++i,++ib)
  {
    dReal *size(box_size_pos[i]), *pos(box_size_pos[i]+3);
    body_[ib].create(world);
    body_[ib].setPosition(pos[0],pos[1],pos[2]);
    dMass m;
    m.setBox(1.0,size[0],size[1],size[2]);
    // m.adjust(mass);
    body_[ib].setMass(&m);
    link_b_[i].create(space,size[0],size[1],size[2]);
    link_b_[i].setBody(body_[ib]);
  }

  // create arm chain; ib=12,...,12+JointNum
  double clen= TotalArmLen/(double)(JointNum+1);
  for(int i(0); i<JointNum+1; ++i,++ib)
  {
    link_cy_[i].create(space,/*radius*/LinkRad,/*length*/clen);
    body_[ib].create(world);
    body_[ib].setPosition(0.0, 0.0, h_chain_begin+(0.5+(double)i)*clen);
    dMass m;
    // m.setCylinder(1.0,/*z*/3,/*radius*/LinkRad,/*length*/clen);
    dMassSetCylinder(&m,1.0,/*z*/3,/*radius*/LinkRad,/*length*/clen);
    body_[ib].setMass(&m);
    link_cy_[i].setBody(body_[ib]);
    link_cy_[i].ColorCode= i%6;
  }

  joint_h_.resize(/*chain:*/JointNum);
  if(FixedBase)  joint_f_.resize(/*fixing force sensors:*/10 + /*fixing base:*/1);
  else           joint_f_.resize(/*fixing force sensors:*/10);
  joint_s_.resize(/*grippers:*/2);
  feedback_.resize(/*force sensors:*/8 + /*joint force sensors:*/JointNum);
  int ifb(0);  // feedback counter

  dBodyID fixed_idxs[11][2]={
      // base force sensors:
      /* 0*/{body_[4],body_[0]},
      /* 1*/{body_[5],body_[0]},
      /* 2*/{body_[6],body_[0]},
      /* 3*/{body_[7],body_[0]},
      // force sensor between base and arm chain:
      /* 4*/{body_[8],body_[0]},
      /* 5*/{body_[8],body_[12]},
      // force sensor between arm chain and gripper base:
      /* 6*/{body_[9],body_[12+JointNum]},
      /* 7*/{body_[9],body_[1]},
      // force sensors on grippers:
      /* 8*/{body_[10],body_[2]},
      /* 9*/{body_[11],body_[3]},
      // fixing base:
      /*10*/{0,body_[0]}
    };
  for(int j(0),j_end(joint_f_.size()); j<j_end; ++j)
  {
    joint_f_[j].create(world);
    joint_f_[j].attach(fixed_idxs[j][0],fixed_idxs[j][1]);
    joint_f_[j].set();
  }
  int fs_idxs[8]= {0,1,2,3, 5, 7, 8,9};
  for(int i(0); i<8; ++i,++ifb)
  {
    dJointSetFeedback(joint_f_[fs_idxs[i]], &feedback_[ifb]);
  }
  // dJointSetFeedback(joint_f_[0], &joint_feedback1);  //TEST

  dReal fmax(HingeFMax);  // NOTE: set zero to control by joint torque
  if(ControlMode==2)  fmax= 0.0;  // Torque/Force control
  for(int j(0); j<JointNum; ++j,++ifb)
  {
    joint_h_[j].create(world);
    joint_h_[j].attach(body_[12+j+1],body_[12+j]);
    joint_h_[j].setAnchor(0.0, 0.0, h_chain_begin+(double)(j+1)*clen);
    switch(j%3)
    {
    case 0: joint_h_[j].setAxis(0.0,0.0,1.0); break;
    case 1: joint_h_[j].setAxis(1.0,0.0,0.0); break;
    case 2: joint_h_[j].setAxis(0.0,1.0,0.0); break;
    }
    joint_h_[j].setParam(dParamFMax,fmax);
    dJointSetFeedback(joint_h_[j], &feedback_[ifb]);
  }
  // dJointSetFeedback(joint_h_[1], &joint_feedback1);  //TEST

  dReal fmaxs(SliderFMax);  // NOTE: set zero to control by joint torque
  if(ControlMode==2)  fmaxs= 0.0;  // Torque/Force control
  joint_s_[0].create(world);
  joint_s_[0].attach(body_[1],body_[2]);
  joint_s_[0].setAxis(0.0,+1.0,0.0);
  joint_s_[0].setParam(dParamFMax,fmaxs);

  joint_s_[1].create(world);
  joint_s_[1].attach(body_[1],body_[3]);
  joint_s_[1].setAxis(0.0,-1.0,0.0);
  joint_s_[1].setParam(dParamFMax,fmaxs);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TObjBox1 : public TDynRobot
//===========================================================================================

// dJointFeedback joint_feedback1;  // TEST

/*override*/void TObjBox1::Create(dWorldID world, dSpaceID space)
{
  body_.clear();
  link_b_.clear();
  link_ca_.clear();
  link_cy_.clear();
  link_sp_.clear();
  link_tm_.clear();
  joint_b_.clear();
  joint_h_.clear();
  joint_h2_.clear();
  joint_s_.clear();
  joint_f_.clear();
  feedback_.clear();

  body_.resize(2);
  link_b_.resize(2);
  int ib(0);  // body counter

  {
    body_[ib].create(world);
    body_[ib].setPosition(Box1PosX,Box1PosY+0.25*Box1SizeY,0.5*Box1SizeZ);
    dMass m;
    m.setBox(Box1Density1,Box1SizeX,0.5*Box1SizeY,Box1SizeZ);
    // m.adjust(mass);
    body_[ib].setMass(&m);
    link_b_[0].create(space,Box1SizeX,0.5*Box1SizeY,Box1SizeZ);
    link_b_[0].setBody(body_[ib]);
    link_b_[0].ColorCode= 1;
    ++ib;
  }
  {
    body_[ib].create(world);
    body_[ib].setPosition(Box1PosX,Box1PosY-0.25*Box1SizeY,0.5*Box1SizeZ);
    dMass m;
    m.setBox(Box1Density2,Box1SizeX,0.5*Box1SizeY,Box1SizeZ);
    // m.adjust(mass);
    body_[ib].setMass(&m);
    link_b_[1].create(space,Box1SizeX,0.5*Box1SizeY,Box1SizeZ);
    link_b_[1].setBody(body_[ib]);
    link_b_[1].ColorCode= 2;
    ++ib;
  }

  joint_f_.resize(1);

  joint_f_[0].create(world);
  joint_f_[0].attach(body_[0],body_[1]);
  joint_f_[0].set();
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TEnvironment
//===========================================================================================

void TEnvironment::Create()
{
  contactgroup_.create();
  world_.setGravity(0,0,Gravity);
  dWorldSetCFM(world_.id(),1e-5);
  plane_.create(space_,0,0,1,0);

  chain_.Create(world_,space_);
  // ObjectMode: 0: None, 1: Box1, ...
  if(ObjectMode==1)  box1_.Create(world_,space_);
  // geom_.Create(world_,space_);

  time_= 0.0;
  sensors_.Clear();
  sensors_.SetZeros(JointNum);
}
//-------------------------------------------------------------------------------------------

void TEnvironment::StepSim(const double &time_step)
{
  ControlCallback(time_step);

  sensors_.ResetForStep();

  space_.collide(0,&NearCallback);
  world_.step(time_step);
  time_+= time_step;

  contactgroup_.empty();
}
//-------------------------------------------------------------------------------------------

void TEnvironment::Draw()
{
  if(Running)  EDrawCallback();

  chain_.Draw();
  // ObjectMode: 0: None, 1: Box1, ...
  if(ObjectMode==1)  box1_.Draw();
  // geom_.Draw();
}
//-------------------------------------------------------------------------------------------

void TEnvironment::ControlCallback(const double &time_step)
{
  // 0: Position, 1: Velocity, 2: Torque/Force
  if(ControlMode==0)  // Position control
  {
    dReal Kp(10.0);
    for(int j(0); j<JointNum; ++j)
      chain_.SetVelH(j, Kp*(TargetAngles[j]-chain_.GetAngleH(j)));
    dReal Kps(10.0);
    for(int j(0); j<2; ++j)
      chain_.SetVelS(j, Kps*(TargetGPos[j]-chain_.GetPosS(j)));
  }
  else if(ControlMode==1)  // Velocity control
  {
    for(int j(0); j<JointNum; ++j)
      chain_.SetVelH(j, TargetVel[j]);
    for(int j(0); j<2; ++j)
      chain_.SetVelS(j, TargetGVel[j]);
  }
  else if(ControlMode==2)  // Torque/Force control
  {
    for(int j(0); j<JointNum; ++j)
      chain_.AddTorqueH(j, TargetTorque[j]);
    for(int j(0); j<2; ++j)
      chain_.AddForceS(j, TargetGForce[j]);
  }
  /*TEST of getForce: it gives the force added by user (not actually simulated force); i.e. useless in general.
  dVector3 f;
  int i=1;
  std::cerr<<"F= "<<chain_.Body(i).getForce()[0]<<", "<<chain_.Body(i).getForce()[1]<<", "<<chain_.Body(i).getForce()[2]<<std::endl;
  //*/
  /*TEST of dJointSetFeedback
  std::cerr<<"F= "<<joint_feedback1.f1[0]<<", "<<joint_feedback1.f1[1]<<", "<<joint_feedback1.f1[2]<<std::endl;
  std::cerr<<"T= "<<joint_feedback1.t1[0]<<", "<<joint_feedback1.t1[1]<<", "<<joint_feedback1.t1[2]<<std::endl;
  //*/
}
//-------------------------------------------------------------------------------------------

void TEnvironment::EDrawCallback()
{
  for(int j(0); j<JointNum; ++j)
    sensors_.JointAngles[j]= chain_.GetAngleH(j);

  for(int i(0); i<12+JointNum+1; ++i)
    ODEBodyToX(chain_.Body(i), sensors_.LinkX.begin()+7*i);

  for(int i(0); i<8+JointNum; ++i)
    ODEFeedbackToF(chain_.GetFeedback(i), sensors_.Forces.begin()+6*i);

  for(int i(0); i<12+JointNum+1; ++i)
    sensors_.Masses[i]= chain_.Body(i).getMass().mass;

  // ObjectMode: 0: None, 1: Box1, ...
  if(ObjectMode==1)
  {
    ODEBodyToX(box1_.Body(0), sensors_.Box1X.begin());
    sensors_.Box1X[0]= 0.5*(sensors_.Box1X[0]+box1_.Body(1).getPosition()[0]);
    sensors_.Box1X[1]= 0.5*(sensors_.Box1X[1]+box1_.Body(1).getPosition()[1]);
    sensors_.Box1X[2]= 0.5*(sensors_.Box1X[2]+box1_.Body(1).getPosition()[2]);
  }

  sensors_.Time= time_;

  if(SensingCallback!=NULL)  SensingCallback(sensors_);
  if(DrawCallback!=NULL)  DrawCallback();
}
//-------------------------------------------------------------------------------------------

/* Called when b1 and b2 are colliding.
    Return whether we ignore this collision (true: ignore collision). */
bool TEnvironment::CollisionCallback(dBodyID &b1, dBodyID &b2, std::valarray<dContact> &contact)
{
  return false;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================


static void NearCallback(void *data, dGeomID o1, dGeomID o2)
{
  assert(Env!=NULL);

  // do nothing if the two bodies are connected by a joint
  dBodyID b1= dGeomGetBody(o1);
  dBodyID b2= dGeomGetBody(o2);
  if(b1 && b2 && dAreConnected(b1,b2)) return;

  std::valarray<dContact> contact(MaxContacts);   // up to MaxContacts contacts per link
  for(int i(0); i<MaxContacts; ++i)
  {
    contact[i].surface.mode= dContactBounce | dContactSoftCFM;
    contact[i].surface.mu= 0.001; // dInfinity;
    contact[i].surface.mu2= 0.1;
    contact[i].surface.bounce= 0.1;
    contact[i].surface.bounce_vel= 0.01;
    contact[i].surface.soft_cfm= 0.1;
  }
  if(int numc=dCollide(o1,o2,MaxContacts,&contact[0].geom,sizeof(dContact)))
  {
    if(Env->CollisionCallback(b1,b2,contact))  return;  // ignore if the callback returns true
    for(int i(0); i<numc; ++i)
    {
      dJointID c= dJointCreateContact(Env->WorldID(),Env->ContactGroupID(),&contact[i]);
      dJointAttach (c,b1,b2);
    }
  }
}
//-------------------------------------------------------------------------------------------

static void SimStart()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0.0,1.2447,0.7};
  static float hpr[3] = {-90.0000,-7.0000,0.0000};
  dsSetViewpoint (xyz,hpr);
}
//-------------------------------------------------------------------------------------------

static void SimLoop(int pause)
{
  assert(Env!=NULL);

  if (!pause && Running)
  {
    Env->StepSim(TimeStep);
  }

  Env->Draw();
  if(StepCallback!=NULL)  StepCallback(Env->Time(), TimeStep);
}
//-------------------------------------------------------------------------------------------

static void SimKeyevent(int command)
{
  assert(Env!=NULL);

  if(KeyEventCallback!=NULL)  KeyEventCallback(command);
  if(EnableKeyEvent)
  {
    switch(command)
    {
    case 'r':
    case 'R': Create(); break;
    case ' ': Running= !Running; break;
    case 'z':
      for(int j(0); j<JointNum; ++j)  TargetAngles[j]+= 0.01;
      break;
    case 'x':
      for(int j(0); j<JointNum; ++j)  TargetAngles[j]-= 0.01;
      break;
    case 'a':  TargetAngles[0]+= 0.01;  break;
    case 's':  TargetAngles[0]-= 0.01;  break;
    case 'A':  TargetAngles[1]+= 0.01;  break;
    case 'S':  TargetAngles[1]-= 0.01;  break;
    case 'd':  TargetAngles[2]+= 0.01;  break;
    case 'f':  TargetAngles[2]-= 0.01;  break;
    case 'D':  TargetAngles[3]+= 0.01;  break;
    case 'F':  TargetAngles[3]-= 0.01;  break;
    case 'g':  TargetAngles[4]+= 0.01;  break;
    case 'h':  TargetAngles[4]-= 0.01;  break;
    case 'G':  TargetAngles[5]+= 0.01;  break;
    case 'H':  TargetAngles[5]-= 0.01;  break;
    case ']':  TargetGPos[0]+= 0.002; TargetGPos[1]+= 0.002;  break;
    case '[':  TargetGPos[0]-= 0.002; TargetGPos[1]-= 0.002;  break;
    case 'n':
      std::cerr<<"Input number of joints > ";
      std::cin>>JointNum;
      std::cerr<<"New number ("<<JointNum<<") is effective after reset"<<std::endl;
      break;
    }
  }
}
//-------------------------------------------------------------------------------------------

void Create()
{
  assert(Env!=NULL);
  TargetAngles.resize(JointNum);
  TargetVel.resize(JointNum);
  TargetTorque.resize(JointNum);
  for(int j(0); j<JointNum; ++j)
  {
    TargetAngles[j]= 0.0;
    TargetVel[j]= 0.0;
    TargetTorque[j]= 0.0;
  }
  Env->Create();
}
//-------------------------------------------------------------------------------------------

void Reset()
{
  Create();
}
//-------------------------------------------------------------------------------------------

void Run(int argc, char **argv, const char *texture_path, int winx, int winy)
{
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &SimStart;
  fn.step = &SimLoop;
  fn.command = &SimKeyevent;
  fn.stop = 0;
  fn.path_to_textures = texture_path;

  dInitODE2(0);

  TEnvironment env;
  Env= &env;
  // env.Create();
  Create();

  dsSimulationLoop(argc,argv,winx,winy,&fn);

  dCloseODE();
}
//-------------------------------------------------------------------------------------------

void Stop()
{
  dsStop();
}
//-------------------------------------------------------------------------------------------


}  // end of ode_x
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
