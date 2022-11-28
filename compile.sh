# ! /bin/bash

rm -f *.o

g++ -c Vec3d.cc
g++ -c Frame.cc
g++ -c ReferenceFrame.cc
g++ -c Node.cc
g++ -c Element.cc
g++ -c RigidBody.cc
g++ -c Joint.cc
g++ -c Platform.cc
g++ -c main.cc
g++ -c InputData.cc

g++   *.o -o pre-mbdyn

rm -f *.o