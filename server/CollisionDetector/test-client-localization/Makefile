
CXX_FLAGS=-I./omniorb-CORBA/
LD_FLAGS=-lomniORB4 -lomnithread
all:
	g++ -o OpenHRPCommonSK.o -c ./omniorb-CORBA/OpenHRPCommonSK.cc ${CXX_FLAGS}
	g++ -o LocalizationSK.o -c ./omniorb-CORBA/LocalizationSK.cc ${CXX_FLAGS}
	g++ -o test-client.o -c test-client.cpp ${CXX_FLAGS}
	g++ -o test-client test-client.o OpenHRPCommonSK.o LocalizationSK.o ${LD_FLAGS}
