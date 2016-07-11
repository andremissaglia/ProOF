#!/bin/bash
for fenix in $(cat fenix.txt); do
	cmd="ssh $fenix 'bash -s' < ./cancelThis.sh";					echo	$cmd;
	#cmd="ssh $fenix";					echo	$cmd;
	#	cmd="killall shell_line.sh";	echo $cmd;
	#	cmd="killall java";				echo $cmd;
	#cmd="exit";							echo $cmd;
	#cmd="cd $1"
	#echo $cmd
done
