
# ——— CPLEX ———
LIB_FORMAT  = static_pic
SYSTEM = x86-64_sles10_4.1
CPLEX_DIR      = /opt/cplex/cplex125/cplex
CONCERT_DIR    = /opt/cplex/cplex125/concert

CPLEX_INCLUDE = -I$(CPLEX_DIR)/include -I$(CONCERT_DIR)/include
CPLEX_LIB = -L$(CPLEX_DIR)/lib/$(SYSTEM)/$(LIB_FORMAT)  -L$(CONCERT_DIR)/lib/$(SYSTEM)/$(LIB_FORMAT)
CPLEX_FLAGS = -lconcert -lilocplex -lcplex -pthread -m64 -O3 -fPIC -fexceptions -DNGraph -DIL_STD 
# —————————————

INCLUDE = $(CPLEX_INCLUDE) 
LIB = $(CPLEX_LIB) 
FLAGS = $(CPLEX_FLAGS) -lm 

all: spanner.cpp spanner.h
	g++ $(INCLUDE) spanner.cpp $(LIB) $(FLAGS) -o spanner



