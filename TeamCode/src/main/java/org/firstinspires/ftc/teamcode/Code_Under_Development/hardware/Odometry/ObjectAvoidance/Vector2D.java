package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.jetbrains.annotations.NotNull;

import java.util.Locale;

@SuppressWarnings("unused")
public class Vector2D {
	private double x, y;

	public Vector2D(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * constructs a new default vector with values of 0 for both x and y
	 */
	public Vector2D() {
		this(0, 0);
	}


	public double getX() {
		return x;
	}

	/**
	 * mutates state
	 *
	 * @param x
	 */
	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	/**
	 * mutates state
	 *
	 * @param y
	 */
	public void setY(double y) {
		this.y = y;
	}


	public double getMagnitude() {
		return Math.hypot(x, y);
	}

	/**
	 * mutates state
	 *
	 * @param x
	 * @param y
	 * @return self
	 */
	public Vector2D set(double x, double y) {
		this.x = x;
		this.y = y;
		return this;
	}

	/**
	 * non-mutating
	 *
	 * @param x
	 * @param y
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D add(double x, double y) {
		return new Vector2D(this.x + x, this.y + y);
	}

	/**
	 * non-mutating
	 *
	 * @param x
	 * @param y
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D subtract(double x, double y) {
		return new Vector2D(this.x - x, this.y - y);
	}

	/**
	 * non-mutating
	 *
	 * @param other
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D add(@NotNull Vector2D other) {
		return this.add(other.x, other.y);
	}

	/**
	 * non-mutating
	 *
	 * @param other
	 * @return a new vector with the desired operation applied
	 */
	public Vector2D subtract(@NotNull Vector2D other) {
		return this.subtract(other.x, other.y);
	}

	/**
	 * dot product
	 *
	 * @param other
	 * @return
	 */
	public double dot(@NotNull Vector2D other) {
		return this.getX() * other.getX() + this.getY() * other.getY();
	}

	@Override
	public boolean equals(@Nullable @org.jetbrains.annotations.Nullable Object obj) {
		if (!(obj instanceof Vector2D)) return false;
		Vector2D other = (Vector2D) obj;
		return this.getX() == other.getX() && this.getY() == other.getY();
	}

	@NonNull
	@NotNull
	@Override
	public String toString() {
		return String.format(Locale.ENGLISH, "x: %f, y: %f", getX(), getY());
	}
}
