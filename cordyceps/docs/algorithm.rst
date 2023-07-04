Algorithm 
--------------

The Cordyceps controller implements a variant of the Pure Pursuit algorithm.
The Pure Pursuit algorithm is a path tracking algorithm used for robotics and autonomous navigation.
The algorithm determines the steering angle of the robot based on the current position relative to the desired path.
A lookahead distance is used to determine the point on the path that the robot should aim for.

More information about the Pure Pursuit algorithm can be found here: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf

The Pure Pursuit algorithm is chosen for the following reasons:
    * It is deterministic
    * Low computational cost
    * It is easy to implement
    * It is easy to tune

More information about how the Pure Pursuit algorithm is implemented in the Cordyceps controller can be found here: https://github.com/TeamSolenopsis/Cordyceps/blob/gh-pages/Cordyceps.pdf