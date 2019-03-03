CXX = g++-8
CXX_FLAGS = -std=c++17 # important flags.
CXX_FLAGS += -Wuninitialized -Wall -Wextra -Werror -pedantic -Wfatal-errors # source code quality.
CXX_FLAGS += -O3 -ffast-math # "production" version.
#CXX_FLAGS += -O0 -g # debug version.
CXX_FLAGS += -I./ # include paths.

SRCS = v-opt.cpp fileio/PointSet.cpp TourModifier.cpp point_quadtree/Node.cpp

%.o: %.cpp; $(CXX) $(CXX_FLAGS) -o $@ -c $<

OBJS = $(SRCS:.cpp=.o)

all: $(OBJS); $(CXX) $^ -o v-opt.out

clean: ; rm -rf v-opt.out $(OBJS) *.dSYM
