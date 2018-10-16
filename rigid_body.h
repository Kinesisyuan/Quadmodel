#ifndef RigidBody
#define RigidBody

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
using namespace std;
using namespace Eigen;

class RigidBody
{
public:
  RigidBody();
  //set property
  void set_mass(double m);
  void set_inertia(Matrix3d i);
  void set_position(Vector3d p);
  void set_velocity(Vector3d v);
  void set_angular_velocity(Vector3d a);
  void set_pose(Quaterniond q);
  //get property
  double get_mass();
  Matrix3d get_inertia();
  Vector3d get_position();
  Vector3d get_velocity();
  Vector3d get_angular_velocity();
  Quaterniond get_pose();
  //external force and torque
  void set_applied_force(Vector3d f);
  void applied_torque(Vector3d tau);
  //update state every dt
  void update_state(double dt);
  Quaterniond dpose_dt(Quaterniond p, Vector3d omega);
private:
  double mass;
  //MatrixXd state; //13 states, 13x1 vector
  Matrix3d inertia;
  Vector3d position, velocity, angular_velocity;
  Quaterniond pose; //quaeternion
  Vector3d applied_force;
  Vector3d applied_torque;
  vector<double> state;
};
