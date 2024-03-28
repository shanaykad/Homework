Nodes:
    /turtle-spawn
        spawns x turtles initially, then have a timer that spawns more periodically
        target turtle
        publishes topic turtle_array with names and poses (x and y)
        service kill-target
    /controller-node
        subscribes to /turtle1/pose
        publishes to /turtle1/cmd_vel
            turtle cannot stop and turn (cheating) so it must travel along a circle until the forward vector of the active turtle matches the vector given by (delta x, delta y)
            uses trig to figure out what cmd_vel must be published to reach target turtle
            can delete target turtles (and remove from array) once the active turtle is within some radius of the target turtle
        subscribed to turtle_array
        calls kill-target when within radius