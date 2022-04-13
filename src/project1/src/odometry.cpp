#include "ros/ros.h"

class Odometry{

public:
  Odometry() { //variabili in cui mi salvo initialpose

    sub = n.subscribe("/cmd_vel", 1000, &Odometry::odometryCallback, this);

  }

  void odometryCallback(/* arguments */) { //get dei parametri della pose attuale, che ho settato con l'odometryCallback di prima.
    /* code */
  }



private:


}

int main(int argc, char**argv){

}
