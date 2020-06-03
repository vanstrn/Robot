### Installation
1. Install [Docker](https://docs.docker.com/get-docker/) - Used to as environment emulator and build platform for the robot.
2. Install [docker-compose](https://docs.docker.com/compose/install/) - Used to execute multiple coordinated docker containers at once.
3. Build (`make images`) or pull docker images (`TBD`)

### Current capabilities 
- GPS Node 
- Controller Node - Read data from controller connected to computer
- Robot Control Node - Reads data and sends control signals to robot.
- Motor Node - Takes control siignals and converts them to motor movements.
- IMU Node
- LIDAR Node

### Creating new Python based Node
1. Create new python file in `./...`
2. Add new launch interface to `setup.py`
3. Build the package with `colcon build`

### Creating new C++ based Node
1. TBD
