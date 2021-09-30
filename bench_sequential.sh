#!/bin/bash

PENALTY=(2 5 20 30)
DDL=(1.2 1.3 1.8 2.0)
CAPACITY=(2 3 5 6)
WORKERS=(1000 2000 3000 4000)
REQUESTS=(10000 50000 100000 150000 200000)

dataset=$1
algo=$2
declare -A exec
exec['gdp']='pruneGDP'
exec['rand']='pruneGDPRand'

echo ${exec[${algo}]}
nohup cmake-build-debug/${exec[${algo}]} data/${dataset}/${dataset}.node data/${dataset}/${dataset}.edge data/${dataset}/${dataset}.label data/${dataset}/${dataset}.order data/${dataset}/${dataset}_taxi.txt data/${dataset}/${dataset}_order.txt 4 1.5 5000 250000 10 ./${dataset}_${algo}_4_1.5x_10_5000_250000_5.txt &
echo "Varying PENALTY"
for penalty in ${PENALTY[@]}
    do
      echo $penalty
      nohup cmake-build-debug/${exec[${algo}]} data/${dataset}/${dataset}.node data/${dataset}/${dataset}.edge data/${dataset}/${dataset}.label data/${dataset}/${dataset}.order data/${dataset}/${dataset}_taxi.txt data/${dataset}/${dataset}_order.txt 4 1.5 5000 250000 $penalty ./${dataset}_${algo}_4_1.5x_${penalty}_5000_250000_5.txt &
    done

echo "Varying DDL"
for ddl in ${DDL[@]}
    do
      echo $ddl
      nohup cmake-build-debug/${exec[${algo}]} data/${dataset}/${dataset}.node data/${dataset}/${dataset}.edge data/${dataset}/${dataset}.label data/${dataset}/${dataset}.order data/${dataset}/${dataset}_taxi.txt data/${dataset}/${dataset}_order.txt 4 $ddl 5000 250000 10 ./${dataset}_${algo}_4_${ddl}x_10_5000_250000_5.txt &
    done

echo "Varying CAPACITY"
for cap in ${CAPACITY[@]}
    do
      echo $cap
      nohup cmake-build-debug/${exec[${algo}]} data/${dataset}/${dataset}.node data/${dataset}/${dataset}.edge data/${dataset}/${dataset}.label data/${dataset}/${dataset}.order data/${dataset}/${dataset}_taxi.txt data/${dataset}/${dataset}_order.txt $cap 1.5 5000 250000 10 ./${dataset}_${algo}_${cap}_1.5x_10_5000_250000_5.txt &
    done

echo "Varying WORKERS"
for m in ${WORKERS[@]}
    do
      echo $m
      nohup cmake-build-debug/${exec[${algo}]} data/${dataset}/${dataset}.node data/${dataset}/${dataset}.edge data/${dataset}/${dataset}.label data/${dataset}/${dataset}.order data/${dataset}/${dataset}_taxi.txt data/${dataset}/${dataset}_order.txt 4 1.5 $m 250000 10 ./${dataset}_${algo}_4_1.5x_10_${m}_250000_5.txt &
    done

echo "Varying REQUESTS"
for n in ${REQUESTS[@]}
    do
      echo $n
      nohup cmake-build-debug/${exec[${algo}]} data/${dataset}/${dataset}.node data/${dataset}/${dataset}.edge data/${dataset}/${dataset}.label data/${dataset}/${dataset}.order data/${dataset}/${dataset}_taxi.txt data/${dataset}/${dataset}_order.txt 4 1.5 5000 $n 10 ./${dataset}_${algo}_4_1.5x_10_5000_${n}_5.txt &
    done
