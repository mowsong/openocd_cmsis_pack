#!/bin/sh

for tgt in TV2C2D4MA0_S1_DQ_S25HL01GTPB01;
do
  tgt=`basename ${tgt} .mk`
  echo "----- ${tgt} -----"
  make -rR TARGET=${tgt} clean
  make -rR TARGET=${tgt}
done
