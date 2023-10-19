package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance;

import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Angle.Units;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Obstacles.Obstacle;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Obstacles.ObstacleMap;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Obstacles.RectangularObstacle;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;


import java.util.ArrayList;

public class EmptyObstacleMap implements ObstacleMap{
	private final ArrayList<Obstacle> additionalObstacles;
	private final double robotSize;

	/**
	 * @param units               the units of your robot size
	 * @param additionalObstacles a list of additional obstacles that you can manipulate in order to add your own realtime obstacle avoidance and detection
	 * @param robotSize           the radius of the robot, you should test with setting this to either 1/2 the width of the robot (representative of the robot driving forwards and backwards) or the longest radius you can measure (to fully prevent running into an obstacle). picking the latter option may affect the willingness of your robot to go through tight spaces during auto
	 */
	public EmptyObstacleMap(@NotNull Units units, @Nullable ArrayList<Obstacle> additionalObstacles, double robotSize) {
		this.additionalObstacles = additionalObstacles;
		this.robotSize = units.toMillimeters(robotSize);

		new RectangularObstacle(Units.MILLIMETER, -10, 0, 0, 3100);

	}

	/**
	 * creates a totally empty obstacle map, with the inability for you to add your own obstacles later,
	 * always returns that the nearest obstacle is infinitely far away
	 */
	public EmptyObstacleMap() {
		this(Units.MILLIMETER, new ArrayList<>(0), 0);
	}

	@Override
	public double getRobotSize() {
		return robotSize;
	}

	@Override
	public ArrayList<Obstacle> getAdditionalObstacles() {
		return additionalObstacles;
	}

	@Override
	public Obstacle[] getObstacles() {
		return new Obstacle[0];
	}
}
