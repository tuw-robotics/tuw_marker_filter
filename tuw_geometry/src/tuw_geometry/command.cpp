#include "tuw_geometry/command.h"
using namespace tuw;

Command::Command () : cv::Vec<double,2>( 0, 0 ) {}

Command::Command ( double v, double w ) : cv::Vec<double,2>( v, w ) {}

Command::Command ( const Command &o ) : cv::Vec<double,2>( o) {}

double &Command::v(){
  return this->val[0];
}

const double &Command::v() const{
  return this->val[0];
}

double &Command::w(){
  return this->val[1];
}

const double &Command::w() const{
  return this->val[1];
}

void Command::set ( double v, double w ) {
  this->val[0] = v, this->val[1] = w;
};