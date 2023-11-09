/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;

public class NavX {
	private AHRS ahrs;
	private double gyroZero;

	public NavX(edu.wpi.first.wpilibj.I2C.Port kmxp) {
		ahrs = new AHRS(kmxp);
		gyroZero = 0;
	}

	public NavX(Port kmxp) {
		ahrs = new AHRS(kmxp);
		gyroZero = 0;
	}

	public AHRS getAHRS() {
		return ahrs;
	}

	public Rotation2d getWrappedRotation2d() {
		return Rotation2d.fromRadians(
			MathUtil.angleModulus(
				ahrs.getRotation2d().getRadians()
			)
		);
	}

	public Rotation2d getRotation2d() {
		return ahrs.getRotation2d();
	}

	/** Interval: [-pi, pi] @return Radians */
	public double getAngle() {
		return getYaw();
	}

	/** Interval: [-pi, pi] @return Radians */
	public double getOffsetedAngle() {
		return getAngle() - getGyroZero();
	}

	/** Interval: [-pi, pi] @return Radians */
	public double getYaw() {
		return -Math.toRadians(ahrs.getYaw());
	}

	/** Interval: [-pi, pi] @return Radians */
	public double getPitch() {
		return -Math.toRadians(ahrs.getPitch());
	}

	/** Interval: [-pi, pi] @return Radians */
	public double getRoll() {
		return -Math.toRadians(ahrs.getRoll());
	}

	public double getGyroZero() {
		return gyroZero;
	}

	public void zeroGyro() {
		gyroZero = getAngle();
	}

	public void setGyroZero(Rotation2d newZero) {
		gyroZero = MathUtil.angleModulus(newZero.getRadians());
	}

	public void offsetGyroZero(double offsetRadians) {
		gyroZero -= MathUtil.angleModulus(offsetRadians);
	}
}
