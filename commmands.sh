cd build
cmake ..
make
./executable 

# for debug
make && valgrind --leak-check=full ./executable