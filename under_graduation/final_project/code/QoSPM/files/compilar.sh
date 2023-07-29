#!/bin/sh
CFLAGS=`/usr/xenomai/bin/xeno-config --xeno-cflags`
CFLAGS="${CFLAGS} -pthread"
LDFLAGS=`/usr/xenomai/bin/xeno-config --xeno-ldflags`  
LDFLAGS="${LDFLAGS} -lrtdm -lm -lnative `net-snmp-config --libs`"
PRINCIPAL=qosp_main #pode ser um dos arquivos cpp contidos na pasta examples
g++  ${CFLAGS} ${LDFLAGS} ${PRINCIPAL}.cpp verifier.cpp mediator.cpp flow.cpp channel.cpp qospa.cpp router.cpp qospm_init.cpp qosp.cpp snmp_cisco_toolkit.cpp local_host.cpp qos_monitor.cpp qospentity.cpp qospm_interface.cpp monitor.cpp qospA_tasks.cpp communicationentity.cpp failuredetector.cpp traffic_listener.cpp channel_verifier.cpp monitoringapplication.cpp channelmanager.cpp qosp_tasks.cpp mt_support.cpp network.cpp -o ${PRINCIPAL}
