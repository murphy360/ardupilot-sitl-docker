"""A torpedo's physics, control and guidance: the core of a simulator front seat.

A torpedo cannot hover or move sideways. It holds speed, heading and depth on its fins
and a stern thruster, follows lines and flies circles. A hull comes as a model file
(schema 1, ``torpedo_sitl.model``) read at start-up.
"""
