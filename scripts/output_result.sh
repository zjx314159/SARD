#!/bin/bash

PENALTY=(2 5 10 20 30)
BATCH_TIME=(1 3 5 7 9)
DDL=(1.2 1.3 1.5 1.8 2.0)
CAPACITY=(2 3 4 5 6)
WORKERS=(1000 2000 3000 4000 5000)
REQUESTS=(10000 50000 100000 150000 200000 250000)

declare -A label
label['gdp']="marker='^',markersize=18,markerfacecolor='none',linewidth=2,label='pruneGDP',color='DodgerBlue'"
label['rand']="marker='+',markersize=15,markerfacecolor='none',linewidth=2,label='random',color='orange'"
label['rtv']="marker='D',markersize=15,markerfacecolor='none',linewidth=2,label='RTV',color='r'"
label['gas']="marker='s',markersize=15,markerfacecolor='none',linewidth=2,label='GAS',color='green'"
label['sard-o0']="marker='p',markersize=18,markerfacecolor='none',linewidth=2,label='SARD',color='MediumPurple'"
label['sard-o1']="marker='o',markersize=18,markerfacecolor='none',linewidth=2,label='SARD-O1',color='cyan'"
label['sard-o2']="marker='*',markersize=18,markerfacecolor='none',linewidth=2,label='SARD-O2',color='peru'"

datasets=('chengdu' 'nyc')
declare -A dist
declare -A rate
declare -A time

clearCache(){
  for algo in ${!label[@]}
    do
      unset dist[${algo}]
      unset rate[${algo}]
      unset time[${algo}]
    done
}

outputRes(){
  for algo in ${!label[@]}
    do
      if [ $1 == 'BATCH_TIME' ] && ([ ${algo} == 'gdp' ] || [ ${algo} == 'rand' ]); then
        continue
      fi
      echo "plt.plot(x,[${dist[${algo}]%?}],${label[${algo}]})"
    done
  for algo in ${!label[@]}
    do
      if [ $1 == 'BATCH_TIME' ] && ([ ${algo} == 'gdp' ] || [ ${algo} == 'rand' ]); then
        continue
      fi
      echo "plt.plot(x,[${rate[${algo}]%?}],${label[${algo}]})"
    done
  for algo in ${!label[@]}
    do
      if [ $1 == 'BATCH_TIME' ] && ([ ${algo} == 'gdp' ] || [ ${algo} == 'rand' ]); then
        continue
      fi
      echo "plt.plot(x,[${time[${algo}]%?}],${label[${algo}]})"
    done
}

# Name arr
extractData(){
  for dataset in ${datasets[@]}
  do
    echo ""
    clearCache
    echo "Varying $1 on ${dataset}"
    for algo in ${!label[@]}
    do
      arr=$2
      if [ $1 == 'BATCH_TIME' ] && ([ ${algo} == 'gdp' ] || [ ${algo} == 'rand' ]); then
        continue
      fi
      for args in ${arr[@]}
        do
          dist[${algo}]=${dist[${algo}]}`grep -oP '(?<== )[0-9]+.[0-9]+' ${dataset}_${algo}_${args}.txt | sed -n 3p`","
          rate[${algo}]=${rate[${algo}]}`grep -oP '(?<=Rate: )[0-9]+.[0-9]+' ${dataset}_${algo}_${args}.txt | head -n 1`","
          tmp_time=`grep -oP '(?<=Running Time: )[0-9]+.[0-9]+' ${dataset}_${algo}_${args}.txt`
          time[${algo}]=${time[${algo}]}`echo "scale=3; ${tmp_time}/1000" | bc`","
        done
    done
    outputRes $1
  done
}


i=0
files=()
for m in ${WORKERS[@]}
  do
    files[${i}]=4_1.5x_10_${m}_250000_5
    ((i++))
  done
extractData 'WORKERS' "${files[*]}"

i=0
for m in ${DDL[@]}
  do
    files[${i}]=4_${m}x_10_5000_250000_5
    ((i++))
  done
extractData 'DDL' "${files[*]}"

i=0
for m in ${CAPACITY[@]}
  do
    files[${i}]=${m}_1.5x_10_5000_250000_5
    ((i++))
  done
extractData 'CAPACITY' "${files[*]}"

i=0
for m in ${PENALTY[@]}
  do
    files[${i}]=4_1.5x_${m}_5000_250000_5
    ((i++))
  done
extractData 'PENALTY' "${files[*]}"

i=0
for m in ${REQUESTS[@]}
  do
    files[${i}]=4_1.5x_10_5000_${m}_5
    ((i++))
  done
extractData 'REQUESTS' "${files[*]}"

i=0
for m in ${BATCH_TIME[@]}
  do
    files[${i}]=4_1.5x_10_5000_250000_${m}
    ((i++))
  done
extractData 'BATCH_TIME' "${files[*]}"


#for m in ${WORKERS[@]}
#    do
#      dist=$dist`grep -oP '(?<== )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_10_${m}_250000_5.txt | sed -n 3p`","
#      rate=$rate`grep -oP '(?<=Rate: )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_10_${m}_250000_5.txt | head -n 1`","
#      tmp_time=`grep -oP '(?<=Running Time: )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_10_${m}_250000_5.txt`
#      time=$time`echo "scale=3; ${tmp_time}/1000" | bc`","
#      ((i++))
#    done
#echo "plt.plot(x,[${dist%?}],${label[${algo}]})"
#echo "plt.plot(x,[${rate%?}],${label[${algo}]})"
#echo "plt.plot(x,[${time%?}],${label[${algo}]})"

#
#dist=''
#rate=''
#time=''
#i=0
#echo "Varying DDL"
#for m in ${DDL[@]}
#    do
#      dist=$dist`grep -oP '(?<== )[0-9]+.[0-9]+' ${dataset}_${algo}_4_${m}x_10_5000_250000_5.txt | sed -n 3p`","
#      rate=$rate`grep -oP '(?<=Rate: )[0-9]+.[0-9]+' ${dataset}_${algo}_4_${m}x_10_5000_250000_5.txt | head -n 1`","
#      tmp_time=`grep -oP '(?<=Running Time: )[0-9]+.[0-9]+' ${dataset}_${algo}_4_${m}x_10_5000_250000_5.txt`
#      time=$time`echo "scale=3; ${tmp_time}/1000" | bc`","
#      ((i++))
#    done
#echo "plt.plot(x2,[${dist%?}],${label[${algo}]})"
#echo "plt.plot(x2,[${rate%?}],${label[${algo}]})"
#echo "plt.plot(x2,[${time%?}],${label[${algo}]})"
#
#
#dist=''
#rate=''
#time=''
#i=0
#for m in ${CAPACITY[@]}
#    do
#      dist=$dist`grep -oP '(?<== )[0-9]+.[0-9]+' ${dataset}_${algo}_${m}_1.5x_10_5000_250000_5.txt | sed -n 3p`","
#      rate=$rate`grep -oP '(?<=Rate: )[0-9]+.[0-9]+' ${dataset}_${algo}_${m}_1.5x_10_5000_250000_5.txt | head -n 1`","
#      tmp_time=`grep -oP '(?<=Running Time: )[0-9]+.[0-9]+' ${dataset}_${algo}_${m}_1.5x_10_5000_250000_5.txt`
#      time=$time`echo "scale=3; ${tmp_time}/1000" | bc`","
#      ((i++))
#    done
#echo "Varying CAPACITY"
#echo "plt.plot(x3,[${dist%?}],${label[${algo}]})"
#echo "plt.plot(x3,[${rate%?}],${label[${algo}]})"
#echo "plt.plot(x3,[${time%?}],${label[${algo}]})"
#
#
#
#dist=''
#rate=''
#time=''
#i=0
#for m in ${PENALTY[@]}
#    do
#      dist=$dist`grep -oP '(?<== )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_${m}_5000_250000_5.txt | sed -n 3p`","
#      rate=$rate`grep -oP '(?<=Rate: )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_${m}_5000_250000_5.txt | head -n 1`","
#      tmp_time=`grep -oP '(?<=Running Time: )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_${m}_5000_250000_5.txt`
#      time=$time`echo "scale=3; ${tmp_time}/1000" | bc`","
#      ((i++))
#    done
#echo "Varying PENALTY"
#echo "plt.plot(x4,[${dist%?}],${label[${algo}]})"
#echo "plt.plot(x4,[${rate%?}],${label[${algo}]})"
#echo "plt.plot(x4,[${time%?}],${label[${algo}]})"
#
#
#
#dist=''
#rate=''
#time=''
#i=0
#for m in ${REQUESTS[@]}
#    do
#      dist=$dist`grep -oP '(?<== )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_10_5000_${m}_5.txt | sed -n 3p`","
#      rate=$rate`grep -oP '(?<=Rate: )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_10_5000_${m}_5.txt | head -n 1`","
#      tmp_time=`grep -oP '(?<=Running Time: )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_10_5000_${m}_5.txt`
#      time=$time`echo "scale=3; ${tmp_time}/1000" | bc`","
#      ((i++))
#    done
#echo "Varying REQUESTS"
#echo "plt.plot(x5,[${dist%?}],${label[${algo}]})"
#echo "plt.plot(x5,[${rate%?}],${label[${algo}]})"
#echo "plt.plot(x5,[${time%?}],${label[${algo}]})"
#
#
#if [ ${algo} == 'sard' ] || [ ${algo} == 'vldb' ] || [ ${algo} == 'rtv' ]; then
#  dist=''
#  rate=''
#  time=''
#  i=0
#  for m in ${BATCH_TIME[@]}
#      do
#        dist=$dist`grep -oP '(?<== )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_10_5000_250000_${m}.txt | sed -n 3p`","
#        rate=$rate`grep -oP '(?<=Rate: )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_10_5000_250000_${m}.txt | head -n 1`","
#        tmp_time=`grep -oP '(?<=Running Time: )[0-9]+.[0-9]+' ${dataset}_${algo}_4_1.5x_10_5000_250000_${m}.txt`
#        time=$time`echo "scale=3; ${tmp_time}/1000" | bc`","
#        ((i++))
#      done
#  echo "Varying BATCH_TIME"
#  echo "plt.plot(x6,[${dist%?}],${label[${algo}]})"
#  echo "plt.plot(x6,[${rate%?}],${label[${algo}]})"
#echo "plt.plot(x6,[${time%?}],${label[${algo}]})"
#fi