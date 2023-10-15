# Assignment 1 - Tarek Z

##  Problem Description

we are to make two nodes namely a driver and a simulator. The driver can be thought of a command center which is transfering differential wheel velcotiy instruction to a simulator, can be think of as a robot in the field, which will adjust is velcity accordingly. 

The driver node will have one topic - cmd_vel, which will contain linear velocity information of both the wheels, published at a certain frequency. The simulator node will contain two nodes, one will grab the cmd_vel, extract differential velocity information and perform calcualtions based on it. It will also have another node through which it will publish its final position after the velcotiy calculation. 

## Code Description 

### The Driver node


