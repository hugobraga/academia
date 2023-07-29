#!/bin/sh
PROCESS_NAME="./qospm"
process=`ps aux | grep -i "${PROCESS_NAME}$"`
echo $process > proc.tmp
result=`awk '{if (($11 == "./qospm") && ($1 == "root")) print $2}' proc.tmp`
#result=`awk '($11 == "./main") && ($1 == "root") {print $2}' proc.tmp`
if [ ${#result} -gt 2 ]
then
(kill -9 $result)
fi
#(kill -9 $result)
#awk '$11 == "./main" {print $2}'
