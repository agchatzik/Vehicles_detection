README.md:
	touch README.md
	echo "TITLE: Guessinggame" >> README.md
	echo "Make file run on 2021-01-28" >> README.md
	echo "Number of lines in guessing.sh" >> README.md
	wc -l guessinggame.sh | egrep -o "[0-9]+" >> README.md
	echo "INSTRUCTIONS" >> README.md
	echo "source guessinggame.sh" >> README.md
	echo "guess" >> README.md


	




