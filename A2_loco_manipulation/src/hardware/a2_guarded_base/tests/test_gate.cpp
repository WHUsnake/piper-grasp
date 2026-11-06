#include "lease_gate.hpp"
#include <cassert>
#include <limits>
#include <iostream>
using V=std::array<double,3>;
const V zero{{0,0,0}}, move{{.05,0,0}};
LeaseGate armed(){LeaseGate g;g.enabled=true;assert(g.claim("token",1));return g;}
int main(){
  {LeaseGate g;assert(!g.claim("token",1));assert(!g.tick(10));}
  {auto g=armed();assert(!g.claim("second",1.1));assert(g.accept("token","task",1,"stationary",zero,1.1,0,true));assert(g.accept("token","task",2,"navigation",move,1.2,0,true));assert(g.tick(1.3));assert(!g.tick(1.7));assert(g.phase==LeaseGate::Phase::RECOVERY);assert(!g.accept("token","task",3,"navigation",move,1.71,0,true));}
  {auto g=armed();assert(g.accept("token","task",1,"grasp",zero,1.1,0,true));assert(!g.accept("token","task",2,"navigation",move,1.2,0,true));}
  {auto g=armed();assert(!g.accept("token","task",1,"grasp",move,1.1,0,true));}
  {auto g=armed();assert(!g.accept("wrong","task",1,"navigation",move,1.1,0,true));}
  {auto g=armed();assert(g.accept("token","task",1,"stationary",zero,1.1,0,true));assert(!g.accept("token","task",1,"navigation",move,1.2,0,true));}
  {auto g=armed();assert(!g.accept("token","task",1,"navigation",move,1.1,.3,true));}
  {auto g=armed();assert(!g.accept("token","task",1,"navigation",move,1.1,-.1,true));}
  {auto g=armed();assert(!g.accept("token","task",1,"navigation",move,1.1,0,false));}
  {auto g=armed();assert(!g.accept("token","task",1,"navigation",V{{.11,0,0}},1.1,0,true));}
  {auto g=armed();assert(!g.accept("token","task",1,"navigation",V{{std::numeric_limits<double>::quiet_NaN(),0,0}},1.1,0,true));}
  {auto g=armed();assert(g.accept("token","task",1,"navigation",move,1.1,0,true));assert(g.accept("token","task",2,"stationary",zero,1.2,0,true));assert(!g.accept("token","task",3,"navigation",move,1.3,0,true));}
  {auto g=armed();assert(!g.accept("token","task",1,"invalid",zero,1.1,0,true));}
  {auto g=armed();assert(g.accept("token","task",1,"stationary",zero,1.1,0,true));assert(!g.accept("token","other",2,"navigation",move,1.2,0,true));}
  std::cout<<"14 lease scenarios passed; no hardware used\n";
}
