CXX=g++
IDIR=./include
INC=-I./include -I./include/msr_hhl
# Sit back and remove all warnings :)
WARN=-Wall -Wextra -pedantic -fsanitize=address,undefined -fno-sanitize-recover -static-libasan
# CXXFLAGS=-std=c++11 -O3 $(INC) $(WARN)
CXXFLAGS=-std=c++11 -O3 $(INC)

DEPS_UTIL = food_data_util.cpp graph_file_util.cpp graph_util.cpp global.cpp simulation_util.cpp constants.cpp hhl_query.cpp
DEPS_CLASS = config.cpp event.cpp order.cpp vehicle.cpp dsu.cpp
DEPS_ALGO = routeplan.cpp route_recommendation.cpp hungarian-lsap.cpp hungarian.cpp vehicle_assignment.cpp vehicle_assignment_baseline.cpp mdrp_baseline.cpp

_DEPS = $(DEPS_UTIL) $(DEPS_CLASS) $(DEPS_ALGO)
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))
DEPS_OBJ = $(patsubst $(IDIR)/%.cpp,$(IDIR)/%.o,$(DEPS)) $(IDIR)/hungarian-lsap.o

$(IDIR)/%.o: $(IDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

main.o: main.cpp $(DEPS_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f *.o
	rm -f $(IDIR)/*.o