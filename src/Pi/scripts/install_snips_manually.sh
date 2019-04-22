sudo apt-get update
sudo apt-get install -y dirmngr
sudo bash -c  'echo "deb https://raspbian.snips.ai/stretch stable main" > /etc/apt/sources.list.d/snips.list'   # depends on paltform and processor => this is for ubuntu arm processor
sudo apt-key adv --keyserver pgp.mit.edu --recv-keys F727C778CCB0A455
sudo apt-get update
sudo apt-get install -y snips-platform-voice
sudo apt-get install -y snips-platform-demo
sudo apt-get install -y snips-tts snips-template snips-skill-server
sudo apt-get install -y snips-watch
