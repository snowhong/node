#Basic Command
rosinit
rosnode list
rostopic list
rosnode info /node_1
rosservice list
rostopic type /pose
rosmsg show geometry_msgs/Twist
rosmsg list
rosshutdown

##Subscribe Topics
```
laser = rossubscriber('/scan')
```

Use receive to wait for a new message (the second argument is a time-out in seconds). The output scandata contains the received message data.
```
scandata = receive(laser,10)
```

plot(scandata,'MaximumRange',7)

###Using Callback in Subscription
```
robotpose = rossubscriber('/pose',@exampleHelperROSPoseCallback)
```

* Stop the pose subscriber by clearing the subscriber variable
```
clear robotpose
```
##Publish Topics

```
chatterpub = rospublisher('/chatter',rostype.std_msgs_String)
pause(2) % Wait to ensure publisher is setup
```
