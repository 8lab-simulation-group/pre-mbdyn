# ! /bin/bash

rm -f *.o

g++ -c Vec3d.cc
g++ -c Frame.cc
g++ -c ReferenceFrame.cc
g++ -c Node.cc
g++ -c DummyNode.cc
g++ -c Element.cc
g++ -c RigidBody.cc
g++ -c Joint.cc
g++ -c Platform.cc
g++ -c Tower.cc
g++ -c Nacelle.cc
g++ -c Blade.cc
g++ -c test.cc
g++ -c InputData.cc

g++   *.o -o out

rm -f *.o