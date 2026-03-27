SRCS = corner.cpp follow_the_gap.cpp lidar_data.cpp obstacle.cpp gap.cpp

CXXFLAGS = -Wall -O2 -std=c++11

libftg.a: $(SRCS:%.cpp=%.o)
	ar rvs $@ $(SRCS:%.cpp=%.o)

clean:
	-rm -f $(SRCS:%.cpp=%.o) libftg.a
