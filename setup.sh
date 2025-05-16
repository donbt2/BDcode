#Not meant to run directly as a shell script. Copy each command for error detection.
#This is pulled directly from Boston Dynamics' docs.  See https://dev.bostondynamics.com/docs/python/quickstart for more.


#Clone the repo:
git clone https://github.com/boston-dynamics/spot-sdk.git

#Install and setup virtualenv:
python3 -m pip install virtualenv
python3 -m virtualenv --python=/usr/bin/python3 helloboston
source helloboston/bin/activate

#Install packages and test:
python3 -m pip uninstall bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
python3 -m pip install bosdyn-client bosdyn-mission
python3
# >>> import bosdyn.client
# >>> help(bosdyn.client)
# >>> exit()

#Ping your Spot:
ping [spot-ip]
python3 -m bosdyn.client [spot-ip] id

#Test the robot:
cd ~/spot-sdk/python/examples/hello_spot # or wherever you installed Spot SDK
python3 -m pip install -r requirements.txt # will install dependent packages
export BOSDYN_CLIENT_USERNAME=user
export BOSDYN_CLIENT_PASSWORD=password
python3 hello_spot.py [spot-ip]

# This will error because there isn't an e-stop set up.  Set up the e-stop:
cd ~/spot-sdk/python/examples/estop # or wherever you installed Spot SDK
python3 -m pip install -r requirements.txt # will install dependent packages
python3 estop_nogui.py [spot-ip]

#Now try this again:

cd ~/spot-sdk/python/examples/hello_spot # or wherever you installed Spot SDK
python3 hello_spot.py [spot-ip]

#This should start the  program.  Spot will make a few poses, take a picture, and sit down.  Press the estop gui button to kill the process.
