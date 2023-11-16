package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.EncodedMotorController;

public class SwerveModule {
	private EncodedMotorController driveMotor;
	private EncodedMotorController angleMotor;
	private Translation2d translationFromCenter;

	public SwerveModule(
		SwerveMotorConfig driveConfig,
		SwerveMotorConfig angleConfig,
		Translation2d translationToCenter
	) {
		driveMotor = driveConfig.motor;
		angleMotor = angleConfig.motor;

		driveMotor
			.setInversion(driveConfig.invert)
			.configureCurrentLimit(driveConfig.currentLimit)
			.setPID(driveConfig.pid);
		angleMotor
			.setInversion(angleConfig.invert)
			.configureCurrentLimit(angleConfig.currentLimit)
			.setPID(angleConfig.pid);
		
		this.translationFromCenter = translationToCenter;
	}

	public void drive(SwerveModuleState initialTargetState) {
		SwerveModuleState targetState = optimizeTalon(
			initialTargetState,
			getModuleState().angle
		);
		setModuleVelocity(
				targetState.speedMetersPerSecond * // This is scales the velocity by how off the wheel is from the
													// target angle.
			Math.abs(targetState.angle.minus(getModuleState().angle).getCos())
		);
		setModuleAngle(targetState.angle.getRadians());
	}

	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(
			driveMotor.getAngularVelocity() *
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER_METERS /
			2,
			new Rotation2d(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO)
		);
	}

	public double getAngularVelocity() {
		return angleMotor.getAngularVelocity() * SwerveConstants.ANGLE_RATIO;
	}

	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(
			driveMotor.getAngle() /
			(2 * Math.PI) * 
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER_METERS *
			Math.PI,
			getModuleState().angle
		);
	}

	public Translation2d getTranslationFromCenter() {
		return translationFromCenter;
	}

	public void setModuleAngle(double targetAngleRadians) {
		angleMotor.setAngle(targetAngleRadians / SwerveConstants.ANGLE_RATIO);
	}

	public void setModuleVelocity(double targetVelocityMetersPerSecond) {
		driveMotor.setAngularVelocity(
			targetVelocityMetersPerSecond *
			2 /
			(SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER_METERS)
		);
	}

	/**
	 * Adjusts the target angle and speed based on the current angle of the swerve
	 * module.
	 * If the difference between the target angle and current angle is greater than
	 * 90 degrees,
	 * the target speed is negated and the target angle is adjusted by 180 degrees.
	 *
	 * @param targetAngle  the desired angle for the swerve module to reach
	 * @param targetSpeed  the desired speed for the swerve module to reach
	 * @param currentAngle the current angle of the swerve module
	 * @return a Pair object containing the adjusted target speed and angle
	 */
	private Pair<Double, Double> adjustTargetAngleAndSpeed(
			double targetAngle,
			double targetSpeed,
			double currentAngle) {
		double delta = targetAngle - currentAngle;
		if (Math.abs(delta) > 90) {
			targetSpeed = -targetSpeed;
			targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
		}
		return new Pair<>(targetSpeed, targetAngle);
	}

	/**
	 * Minimize the change in heading the desired swerve module state would require
	 * by potentially reversing the direction the wheel spins. Customized from
	 * WPILib's version to include placing in appropriate scope for CTRE onboard
	 * control.
	 * 
	 * @see <a
	 *      href=https://www.chiefdelphi.com/t/swerve-modules-flip-180-degrees-periodically-conditionally/393059/3
	 *      >Chief Delphi Post Concerning The Issue</a>
	 * 
	 * @param desiredState The desired state.
	 * @param currentAngle The current module angle.
	 */
	private SwerveModuleState optimizeTalon(SwerveModuleState desiredState, Rotation2d currentAngle) {
		double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
		double targetSpeed = desiredState.speedMetersPerSecond;
		Pair<Double, Double> adjustedValues = adjustTargetAngleAndSpeed(targetAngle, targetSpeed,
				currentAngle.getDegrees());
		return new SwerveModuleState(
				adjustedValues.getFirst(),
				Rotation2d.fromDegrees(adjustedValues.getSecond()));
	}

	/**
	 * Places the given angle in the appropriate 0 to 360 degree scope based on the
	 * reference angle.
	 * 
	 * TODO: Test if this works :D
	 * 
	 * @param scopeReference the reference angle to place the new angle in scope of
	 * @param newAngle       the angle to place in the scope
	 * @return the new angle within the appropriate 0 to 360 degree scope
	 */
	private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
		double delta = newAngle - scopeReference;
		delta += 180; // shift range to [0, 360]
		delta %= 360; // normalize to [0, 360]
		if (delta < 0)
			delta += 360; // correct negative values
		delta -= 180; // shift range back to [-180, 180]
		return scopeReference + delta;
	}

	public static class SwerveMotorConfig {
		public EncodedMotorController motor;
		public boolean invert;
		public int currentLimit;
		public PIDConstants pid;

		private SwerveMotorConfig(Builder builder) {
			this.motor = builder.motor;
			this.invert = builder.invert;
			this.currentLimit = builder.currentLimit;
			this.pid = builder.pid;
		}

		public static class Builder {
			private EncodedMotorController motor;
			private boolean invert = false;
			private int currentLimit = 25;
			private PIDConstants pid = new PIDConstants(0, 0, 0);

			public Builder motor(EncodedMotorController motor) {
				this.motor = motor;
				return this;
			}

			public Builder invert(boolean invert) {
				this.invert = invert;
				return this;
			}

			public Builder currentLimit(int currentLimit) {
				this.currentLimit = currentLimit;
				return this;
			}

			public Builder pid(PIDConstants pid) {
				this.pid = pid;
				return this;
			}

			public SwerveMotorConfig build() {
				return new SwerveMotorConfig(this);
			}
		}
	}
}
