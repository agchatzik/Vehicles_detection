#!/usr/bin/env sh
function guess {
	num=$(ls | wc -l)
	re='^[0-9]+$'
	echo "how many files are in the current directory?"
	while [[ true ]]
	read answer
	do 
		if ! [[ $answer =~ $re ]]
		then
			echo "error: Not a number. Try again!" 
			continue
  		elif [[ $answer -eq $num ]]
  		then
     		echo "Congratulations!"
     		break
  		elif [[ $answer -gt $num ]]
  		then
     		echo "Too high! Try Again!"
  		else
     		echo "Too low! Try Again!"
  		fi
	done
}

guess