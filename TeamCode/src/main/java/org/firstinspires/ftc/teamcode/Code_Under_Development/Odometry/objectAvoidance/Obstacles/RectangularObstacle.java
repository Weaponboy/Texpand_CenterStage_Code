package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Obstacles;

import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Angle.Units;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Obstacles.Obstacle;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Vector.Vector2D;
import org.jetbrains.annotations.NotNull;

@SuppressWarnings("unused")
public class RectangularObstacle implements Obstacle {
	private final double left, bottom, right, top;

	public RectangularObstacle(@NotNull Units unit, double left, double bottom, double right, double top) {
		this.left = unit.toMillimeters(left);
		this.bottom = unit.toMillimeters(bottom);
		this.right = unit.toMillimeters(right);
		this.top = unit.toMillimeters(top);
	}

	@Override
	public Vector2D distance(@NotNull Vector2D position) {
		if (position.getY() > top) {
			return checkCorners(position, top);
		} else if (position.getY() < bottom) {
			return checkCorners(position, bottom);
		} else if (position.getX() < left) {
			return new Vector2D(left, position.getY()).subtract(position);
		} else if (position.getY() > right) {
			return new Vector2D(right, position.getY()).subtract(position);
		}
		return new Vector2D(); // inside the obstacle
	}

	private Vector2D checkCorners(@NotNull Vector2D position, double bottom) {
		if (position.getX() < left) {
			return new Vector2D(left, bottom).subtract(position);
		} else if (position.getX() > right) {
			return new Vector2D(right, bottom).subtract(position);
		} else {
			return new Vector2D(position.getX(), bottom).subtract(position);
		}
	}
}
