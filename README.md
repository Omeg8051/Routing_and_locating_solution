# Routing_and_locating_solution
<p>This piece of code is a solution for indoor locationing without the need for lidar. It is written for a 2 wheel differential drive robot to route through a finite indoor space.
</p>
<br>
<p>
The firmware is writtened for the Elisa3 proto in Webots simulation patform.
</p>

<h1>About the positioning library</h1>
<p>
The positioning library treats the motion of the machine as orbiting about a center a radius away at with a tangent velocity.
It takes the tangential velocity at the edge of both wheels and process it into an orbitinbg radius and a tangential velocity.
It integrates the change made by orbiting about a center at the tangential velocity over a series of small time frame.
</p>

<h1>Used applications</h1>
<p>
Microsoft Visual studio
<br>
Webits
</p>
