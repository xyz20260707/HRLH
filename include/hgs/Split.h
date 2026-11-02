#ifndef SPLIT_H
#define SPLIT_H

#include "Params.h"
#include "Individual.h"

class Split
{

 private:

 Params * params ;
 int maxVehicles ;

 std::vector < std::vector < double > > potential;
 std::vector < std::vector < int > > pred;

public:

  void generalSplit(Individual * indiv, int nbMaxVehicles);

  Split(Params * params);

};
#endif
