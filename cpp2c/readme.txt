1. get the cpp code of Copter.h
2. run c preprocessor (cpp) using the command in cmd.txt to convert
Copter.h to copter.i
3. copy the copter struct to a new empty file: yu.i
4. in yu.i, go through the code, comment all complicated classes. for
the rest of data structures, look for the dependant typedefs in
copter.i, and copy them back to yu.i
5. run addserialize on yu.i
