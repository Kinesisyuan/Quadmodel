#include "rigid_body.h"
using namespace std;
using namespace Eigen;

RigidBody::RigidBody()
{
  mass = 0;
  inertia << 0,0,0,0,0,0,0,0,0;
  position << 0,0,0;
  velocity << 0,0,0;
  angular_velocity << 0,0,0;
  pose = Quaterniond(1,0,0,0);

  state = vector<double>(13);
}
//set parameters when create a RigidBody
void RigidBody::set_mass(double m)
{
  mass = m;
  return;
}
void RigidBody::set_inertia(Matrix3d i)
{
  inertia = i;
  return;
}
//exert force and torque
void RigidBody::set_applied_force(Vector3d f)
{
  applied_force = f;
  return;
}
void RigidBody::applied_torque(Vector3d tau)
{
  applied_torque = tau;
  return;
}
//initialize position, velocity, pose and angular_velocity
void RigidBody::set_position(Vector3d p)
{
  position = p;
  return;
}
void RigidBody::set_velocity(Vector3d v)
{
  velocity = v;
  return;
}
void RigidBody::set_angular_velocity(Vector3d a)
{
  angular_velocity = a;
  return;
}
void RigidBody::set_pose(Quaterniond q)
{
  post = q;
  return;
}
//get private information
Matrix3d RigidBody::get_inertia()
{
  return inertia;
}
Vector3d RigidBody::get_position()
{
  return position;
}
Vector3d RigidBody::get_velocity()
{
  return velocity;
}
Vector3d RigidBody::get_angular_velocity()
{
  return velocity;
}
Quaterniond RigidBody::get_pose()
{
  return pose;
}
//updating state
void RigidBody::update_state()
{
  //state.block<3,1>(0,0) = position;
  //state.block<3,1>(3,0) = velocity;
  //state.block<4,1>(6,0) << pose.w(), pose.x(), pose.y(), post.z();
  //state.block<3,1>(10,0) = angular_velocity;

  state[0] = position[0];
  state[1] = position[1];
  state[2] = position[2];

  state[3] = velocity[0];
  state[4] = velocity[1];
  state[5] = velocity[2];

  state[6] = pose.w();
  state[7] = pose.x();
  state[8] = pose.y();
  state[9] = pose.z();

  state[10] = angular_velocity[0];
  state[11] = angular_velocity[1];
  state[12] = angular_velocity[2];
}
//get the derivative of quaternion pose
Quaterniond RigidBody::dpose_dt(Quaterniond p, Vector3d omega)
{
  Wx = omega(0);
  Wy = omega(1);
  Wz = omega(2);
  Vector4d pose_vec(p.w(), p.x(), p.y(), p.z());
  MatrixXd omega_mat;
  omega_mat << 0, -Wx, -Wy, -Wz,
               Wx, 0, Wz, -Wy,
               Wy, -Wz, 0, Wx,
               Wz, Wy, -Wx, 0;
  Vector4d dpdt = 0.5 * omega_mat * pose_vec;
  Quaterniond dpose(dpdt(0), dpdt(1), dpdt(2), dpdt(3));
  return dpose;
}

typedef vector<double> state_type
void operator()(const state_type &x, state_type &dxdt, const double /* t */)
{
  Vector3d dpdt = velocity;
  Vector3d dvdt = applied_force/mass;
  Quaterniond dqdt = dpose_dt(pose, angular_velocity);
  //Newton-Euler equation
  Vector3d angular_acc = inertia.inverse() *
                         (applied_torque - angular_velocity.cross(inertia * angular_velocity));
  dxdt[0] = dpdt[0];
  dxdt[1] = dpdt[1];
  dxdt[2] = dpdt[2];

  dxdt[3] = dvdt[0];
  dxdt[4] = dvdt[1];
  dxdt[5] = dvdt[2];

  dxdt[6] = dqdt.w();
  dxdt[7] = dqdt.x();
  dxdt[8] = dqdt.y();
  dxdt[9] = dqdt.z();

  dxdt[10] = angular_acc[0];
  dxdt[11] = angular_acc[1];
  dxdt[12] = angular_acc[2];
}
