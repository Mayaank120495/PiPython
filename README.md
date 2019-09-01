How to install python 3 in ubuntu and make it as default

sudo apt-get update
sudo apt-get install python3

Now if you have to make python3 as default python version

edit the .bashrc file with sudo permissions 
 sudo nano ~/.bashrc
 
add the following line at the end of the file
alias python='/usr/bin/python3.5'

update-alternatives --install /usr/bin/python python /usr/bin/python3.5 10

This gives a priority of 10 for the path of python3. The disadvantage of editing .bashrc file is that it will not work while using the commands with sudo.
