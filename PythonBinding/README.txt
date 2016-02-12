This folder contains all of what we need to create bindings for C++ and Python. 

The only files that should be changed are DroneUtils.h and DroneUtils.cpp. The Makefile works,
and is just just with make. If OpenCV is installed correctly, the whole thing should be compiled.
Do not delete or alter any of the other files.

The next step is to link the .so file to where Python looks for it. I'm not sure how to do it
for Raspberry Pi yet, but worse case scenario, if you place the .so file in the same directory as the 
Python file, it should work fine. 

To add more functions in,

Write the function in DroneUtils.cpp and the function declaration in DroneUtils.h. 
At the bottom of DroneUtils.cpp, there is a weird section of code about BOOST::PYTHON. This is
how Python will be able to see the methods. To add your new method in, add a line like

def("methodName", methodName);

where the others are, then remake it. 

Let me know if there are any questions.