README.md:
	touch README.md
	echo "TITLE: Guessinggame\n" >> README.md
	echo "Make file run on 2021-01-28\n" >> README.md
	echo "Number of lines in guessing.sh\n" >> README.md
	wc -l guessinggame.sh | egrep -o "[0-9]+" >> README.md
	echo "INSTRUCTIONS\n" >> README.md
	echo "source guessinggame.sh\n" >> README.md
	echo "guess" >> README.md


	




