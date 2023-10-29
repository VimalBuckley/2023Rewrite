package frc.robot.hardware.interfaces;

import com.pathplanner.lib.auto.PIDConstants;

public interface EncodedMotorController {
	/** Units are radians/sec */
	public void setAngularVelocity(double targetAngularVelocity);

	/** Units are radians/sec */
	public double getAngularVelocity();

	/** Units are radians */
	public void setAngle(double targetAngle);

	/** Units are radians */
	public double getAngle();

	/** Units are percent */
	public void setOutput(double targetOutput);

	/** Units are percent */
	public double getOutput();

	public void setPID(PIDConstants pid);

	/** Units are radians */
	public void setMinAngle(double minAngle);

	/** Units are radians */
	public void setMaxAngle(double maxAngle);

	/** Units are percent */
	public void setMinOutput(double minOutput);

	/** Units are percent */
	public void setMaxOutput(double maxOutput);

	public void setInverted(boolean shouldInvert);

	public void setBrakeOnIdle(boolean shouldBreak);

	/** Units are radians */
	public void setAngleTolerance(double tolerance);
}
