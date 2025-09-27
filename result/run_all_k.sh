#!/bin/bash

for n in $(seq 1 550)
do
	    # 生成一个临时配置文件
	        cp ../instances/mapf/sample.txt temp.txt
		    sed -i "s/^agents=.*/agents=$n/" temp.txt

		        # 输出结果文件按agent数量区分
			    ./mapf -i temp.txt -s PIBTM -o result__window.txt -v
		    done

