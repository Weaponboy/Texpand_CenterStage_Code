package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Obstacles;


import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Vector.Vector2D;

@SuppressWarnings("unused")
public interface Obstacle {
	Vector2D distance(Vector2D position);
}