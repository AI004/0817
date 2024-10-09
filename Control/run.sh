cd ./bin/Sources/python_scripts
python3 read_abs.py
result=`python3 check_abs.py`
if [ "$result" = "True" ]
	then
	cd ../../
	echo 'pndxyz' | sudo -S sh PndControl.sh
else	
	echo "abs missed, retry"	
fi
