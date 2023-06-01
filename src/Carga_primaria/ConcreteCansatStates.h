#pragma once
#include "Cansat.h"
#include "CansatState.h"
#include "Arduino.h"

class Start : public CansatState{
public:
  void name();
	void enter(Cansat* cansat);
	void toggle(Cansat* cansat);
	void exit(Cansat* cansat) {}
	static CansatState& getInstance();

private:
	Start() {}
	Start(const Start& other);
	Start& operator=(const Start& other);
};

class Ascending : public CansatState
{
public:
  void name();
	void enter(Cansat* cansat);
	void toggle(Cansat* cansat);
	void exit(Cansat* cansat) {}
	static CansatState& getInstance();

private:
	Ascending() {}
	Ascending(const Ascending& other);
	Ascending& operator=(const Ascending& other);
};

class FreeFall : public CansatState
{
public:
  void name();
	void enter(Cansat* cansat);
	void toggle(Cansat* cansat);
	void exit(Cansat* cansat) {}
	static CansatState& getInstance();

private:
	FreeFall() {}
	FreeFall(const FreeFall& other);
	FreeFall& operator=(const FreeFall& other);
};

class GyroFall : public CansatState
{
public:
  void name();
	void enter(Cansat* cansat);
	void toggle(Cansat* cansat);
	void exit(Cansat* cansat) {}
	static CansatState& getInstance();

private:
	GyroFall() {}
	GyroFall(const GyroFall& other);
	GyroFall& operator=(const GyroFall& other);
};

class Landing : public CansatState
{
public:
  void name();
	void enter(Cansat* cansat);
	void toggle(Cansat* cansat);
	void exit(Cansat* cansat);
	static CansatState& getInstance();

private:
	Landing() {}
	Landing(const Landing& other);
	Landing& operator=(const Landing& other);
};

class Idle : public CansatState
{
public:
  void name();
	void enter(Cansat* cansat);
	void toggle(Cansat* cansat);
	void exit(Cansat* cansat) {}
	static CansatState& getInstance();

private:
	Idle() {}
	Idle(const Idle& other);
	Idle& operator=(const Idle& other);
};
  