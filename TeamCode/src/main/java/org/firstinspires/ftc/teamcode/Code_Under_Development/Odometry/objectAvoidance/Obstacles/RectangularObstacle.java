package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Obstacles;

import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Vector.Vector2D;
import org.jetbrains.annotations.NotNull;

@SuppressWarnings("unused")
public class RectangularObstacle implements Obstacle {

	private final double xHigher, xLower, yHigher, yLower;

	public RectangularObstacle(double xHigher, double xLower, double yHigher, double yLower) {
		this.xHigher = xHigher;
		this.xLower = xLower;
		this.yHigher = yHigher;
		this.yLower = yLower;
	}

	@Override
	public Vector2D distance(@NotNull Vector2D position) {
		if (position.getX() > xHigher) {
			return checkCorners(position, xHigher);
		} else if (position.getX() < xLower) {
			return checkCorners(position, xLower);
		} else if (position.getY() < yLower) {
			return new Vector2D(yLower, position.getY()).subtract(position);
		} else if (position.getY() > yHigher) {
			return new Vector2D(yHigher, position.getY()).subtract(position);
		}
		return new Vector2D(); // inside the obstacle
	}

	private Vector2D checkCorners(@NotNull Vector2D position, double bottom) {
		if (position.getX() < xHigher) {
			return new Vector2D(xHigher, bottom).subtract(position);
		} else if (position.getY() > yHigher) {
			return new Vector2D(yHigher, bottom).subtract(position);
		} else {
			return new Vector2D(position.getX(), bottom).subtract(position);
		}
	}
}