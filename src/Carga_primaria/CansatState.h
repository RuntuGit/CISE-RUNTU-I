#pragma once
#include "Cansat.h"

// Forward declaration to resolve circular dependency/include
class Cansat;

class CansatState{
public:
  virtual void name() = 0;
	virtual void enter(Cansat* cansat) = 0;
	virtual void toggle(Cansat* cansat) = 0;
	virtual void exit(Cansat* cansat) = 0;
	virtual ~CansatState() {}
};
