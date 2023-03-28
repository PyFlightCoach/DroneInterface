This project will contain an opinionated but tidy way to talk to a Drone through PyMavLink 
using the PyFlightCoach Libraries


No documentation yet, see examples

for testing run sitl in a docker container:

cd sitl

docker build -t sitl .

docker run --rm -it -p 5760:5760 -p 5761:5761 --name=sitl sitl


