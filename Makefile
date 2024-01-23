CXX_FLAGS=-std=c++11 -O2 -Wall -Wextra

INCLUDE_FLAGS=`pkg-config --cflags ompl eigen3`
# Linker options
LD_FLAGS=`pkg-config --libs ompl`

# The c++ compiler to invoke
CXX=c++
all: scenario1 scenario2 scenario3 scenario4 scenario5

clean:
	rm -f *.o
	rm -f scenario1 scenario2 scenario3 scenario4 scenario5 scenario6

%.o: src/%.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@

scenario1: scenario1.o DRRT.o CollisionChecking.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)

scenario2: scenario2.o DRRT.o CollisionChecking.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)

scenario3: scenario3.o DRRT.o CollisionChecking.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)

scenario4: scenario4.o DRRT.o CollisionChecking.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)

scenario5: scenario5.o DRRT.o CollisionChecking.o
	$(CXX) $(CXX_FLAGS) $(INCLUDE_FLAGS) -Lsrc/. -o $@ $^ $(LD_FLAGS)
