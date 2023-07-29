# Environment
MKDIR=mkdir
RM=rm -f
CP=cp
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++

#exemplos
EXAMPLESDIR=.

# Object Directory
OBJECTDIR=objects

#fonte
SOURCEDIR=..

#executavel
EXECDIR=bin

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/verifier.o \
	${OBJECTDIR}/mediator.o \
	${OBJECTDIR}/flow.o \
	${OBJECTDIR}/channel.o \
	${OBJECTDIR}/qospa.o \
	${OBJECTDIR}/router.o \
	${OBJECTDIR}/qospm_init.o \
	${OBJECTDIR}/qosp.o \
	${OBJECTDIR}/snmp_cisco_toolkit.o \
	${OBJECTDIR}/local_host.o \
	${OBJECTDIR}/qos_monitor.o \
	${OBJECTDIR}/qospentity.o \
	${OBJECTDIR}/qospm_interface.o \
	${OBJECTDIR}/monitor.o \
	${OBJECTDIR}/qospA_tasks.o \
	${OBJECTDIR}/communicationentity.o \
	${OBJECTDIR}/failuredetector.o \
	${OBJECTDIR}/traffic_listener.o \
	${OBJECTDIR}/channel_verifier.o \
	${OBJECTDIR}/monitoringapplication.o \
	${OBJECTDIR}/channelmanager.o \
	${OBJECTDIR}/qosp_tasks.o \
	${OBJECTDIR}/mt_support.o \
	${OBJECTDIR}/network.o

# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS= $(shell /usr/xenomai/bin/xeno-config --xeno-cflags) $(net-snmp-config --cflags) -pthread
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS= $(/usr/xenomai/bin/xeno-config --xeno-ldflags) -lrtdm -lm -lnative $(net-snmp-config --libs)

clean:
	${RM} -r ${EXECDIR}/*
	${RM} -r ${OBJECTDIR}/*
	
${OBJECTDIR}/verifier.o: ${SOURCEDIR}/verifier.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/verifier.o ${SOURCEDIR}/verifier.cpp

${OBJECTDIR}/mediator.o: ${SOURCEDIR}/mediator.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/mediator.o ${SOURCEDIR}/mediator.cpp

${OBJECTDIR}/flow.o: ${SOURCEDIR}/flow.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/flow.o ${SOURCEDIR}/flow.cpp

${OBJECTDIR}/channel.o: ${SOURCEDIR}/channel.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/channel.o ${SOURCEDIR}/channel.cpp

${OBJECTDIR}/qospa.o: ${SOURCEDIR}/qospa.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qospa.o ${SOURCEDIR}/qospa.cpp

${OBJECTDIR}/router.o: ${SOURCEDIR}/router.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/router.o ${SOURCEDIR}/router.cpp

${OBJECTDIR}/qosp_tasks.o: ${SOURCEDIR}/qosp_tasks.cpp 
	${RM} $@
	${LINK.cc} -O2 -MMD -MP -MF $@ ${CCFLAGS} -l/usr/xenomai/include -L/usr/xenomai/lib -lpthread -lrtdm -lm -l/usr/xenomai/lib/native -Wl,-Bsymbolic-functions -L/usr/lib -lnetsnmp -o ${OBJECTDIR}/qosp_tasks.o ${SOURCEDIR}/qosp_tasks.cpp

${OBJECTDIR}/qospm_init.o: ${SOURCEDIR}/qospm_init.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qospm_init.o ${SOURCEDIR}/qospm_init.cpp

${OBJECTDIR}/qosp.o: ${SOURCEDIR}/qosp.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qosp.o ${SOURCEDIR}/qosp.cpp

${OBJECTDIR}/snmp_cisco_toolkit.o: ${SOURCEDIR}/snmp_cisco_toolkit.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/snmp_cisco_toolkit.o ${SOURCEDIR}/snmp_cisco_toolkit.cpp

${OBJECTDIR}/local_host.o: ${SOURCEDIR}/local_host.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/local_host.o ${SOURCEDIR}/local_host.cpp

${OBJECTDIR}/qos_monitor.o: ${SOURCEDIR}/qos_monitor.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qos_monitor.o ${SOURCEDIR}/qos_monitor.cpp

${OBJECTDIR}/mt_support.o: ${SOURCEDIR}/mt_support.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/mt_support.o ${SOURCEDIR}/mt_support.cpp

${OBJECTDIR}/qospentity.o: ${SOURCEDIR}/qospentity.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qospentity.o ${SOURCEDIR}/qospentity.cpp

${OBJECTDIR}/qospm_interface.o: ${SOURCEDIR}/qospm_interface.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qospm_interface.o ${SOURCEDIR}/qospm_interface.cpp

${OBJECTDIR}/monitor.o: ${SOURCEDIR}/monitor.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/monitor.o ${SOURCEDIR}/monitor.cpp

${OBJECTDIR}/qospA_tasks.o: ${SOURCEDIR}/qospA_tasks.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qospA_tasks.o ${SOURCEDIR}/qospA_tasks.cpp

${OBJECTDIR}/main.o: ${SOURCEDIR}/main.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/main.o ${SOURCEDIR}/main.cpp

${OBJECTDIR}/communicationentity.o: ${SOURCEDIR}/communicationentity.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/communicationentity.o ${SOURCEDIR}/communicationentity.cpp

${OBJECTDIR}/failuredetector.o: ${SOURCEDIR}/failuredetector.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/failuredetector.o ${SOURCEDIR}/failuredetector.cpp

${OBJECTDIR}/traffic_listener.o: ${SOURCEDIR}/traffic_listener.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/traffic_listener.o ${SOURCEDIR}/traffic_listener.cpp

${OBJECTDIR}/channel_verifier.o: ${SOURCEDIR}/channel_verifier.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/channel_verifier.o ${SOURCEDIR}/channel_verifier.cpp

${OBJECTDIR}/monitoringapplication.o: ${SOURCEDIR}/monitoringapplication.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/monitoringapplication.o ${SOURCEDIR}/monitoringapplication.cpp

${OBJECTDIR}/channelmanager.o: ${SOURCEDIR}/channelmanager.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/channelmanager.o ${SOURCEDIR}/channelmanager.cpp

${OBJECTDIR}/network.o: ${SOURCEDIR}/network.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/network.o ${SOURCEDIR}/network.cpp
	
#examples
${OBJECTDIR}/qosp_main.o: qosp_main.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	#$(COMPILE.cc) -O2 -MMD -MP -MF $@.d ${CCFLAGS} -o ${OBJECTDIR}/qosp_main.o -g qosp_main.cpp
	$(COMPILE.cc) ${CCFLAGS} -o ${OBJECTDIR}/qosp_main.o -g qosp_main.cpp
	
${OBJECTDIR}/qospa_main.o: qospa_main.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	#${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qospa_main.o qospa_main.cpp	
	
${OBJECTDIR}/qosp_test_qos.o: qosp_test_qos..cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@	${CCFLAGS} -o ${OBJECTDIR}/qosp_test_qos.o qosp_test_qos.cpp
	
${OBJECTDIR}/verify_channel/qosp_client_process.o: verify_channel/qosp_client_process.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qospa.o verify_channel/qosp_client_process.cpp	
	
${OBJECTDIR}/verify_channel/qosp_server_process.o: verify_channel/qosp_server_process.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qosp_server_process.o verify_channel/qosp_server_process.cpp

${OBJECTDIR}/verify_channel/qosp_test_verify.o: verify_channel/qosp_test_verify.cpp 
	#${MKDIR} -p ${OBJECTDIR}
	${RM} $@
	$(COMPILE.cc) -O2 -MMD -MP -MF $@ ${CCFLAGS} -o ${OBJECTDIR}/qosp_test_verify.o verify_channel/qosp_test_verify.cpp


#binarios
qosp_main: ${OBJECTFILES} ${OBJECTDIR}/qosp_main.o
	${LINK.cc} -L/usr/xenomai/lib -lpthread -lrtdm -lm -lnative -Wl,-Bsymbolic-functions -L/usr/lib -lnetsnmp -o ${EXECDIR}/qosp ${OBJECTFILES} ${OBJECTDIR}/qosp_main.o
	
${EXECDIR}/qospa_main: ${OBJECTFILES} ${OBJECTDIR}/qospa_main.o
	${LINK.cc} -o ${EXECDIR}/qospa ${OBJECTFILES} ${OBJECTDIR}/qospa_main.o ${LDLIBSOPTIONS} 	
	
${EXECDIR}/qosp_test_qos: ${OBJECTFILES} ${OBJECTDIR}/qosp_test_qos.o
	${LINK.cc} -o ${EXECDIR}/qosp_test_qos ${OBJECTFILES} ${OBJECTDIR}/qosp_test_qos.o ${LDLIBSOPTIONS} 

${EXECDIR}/qosp_client_process: ${OBJECTFILES} ${OBJECTDIR}/qosp_client_process.o
	${LINK.cc} -o ${EXECDIR}/qosp_client_process ${OBJECTFILES} ${OBJECTDIR}/qosp_client_process.o ${LDLIBSOPTIONS} 
	
${EXECDIR}/qosp_server_process: ${OBJECTFILES} ${OBJECTDIR}/qosp_client_process.o
	${LINK.cc} -o ${EXECDIR}/qosp_server_process ${OBJECTFILES} ${OBJECTDIR}/qosp_server_process.o ${LDLIBSOPTIONS} 	
	
${EXECDIR}/qosp_test_verify: ${OBJECTFILES} ${OBJECTDIR}/qosp_test_verify.o
	${LINK.cc} -o ${EXECDIR}/qosp_test_verify ${OBJECTFILES} ${OBJECTDIR}/qosp_test_verify.o ${LDLIBSOPTIONS} 		
