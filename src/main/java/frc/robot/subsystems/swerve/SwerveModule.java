package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.EnumConstants.TalonModel;
import frc.robot.hardware.EncodedMotorController;
import frc.robot.hardware.SparkMaxMotorController;
import frc.robot.hardware.TalonMotorController;

public class SwerveModule {
	private EncodedMotorController driveMotor;
	private EncodedMotorController angleMotor;
	private Translation2d translationFromCenter;

	public SwerveModule(
		SwerveMotorConfig driveConfig,
		SwerveMotorConfig angleConfig,
		Translation2d translationToCenter
	) {
		driveMotor = driveConfig.isNeo ? 
			new SparkMaxMotorController(driveConfig.canId, MotorType.kBrushless) :
			new TalonMotorController(driveConfig.canId, TalonModel.TalonFX);
		angleMotor = angleConfig.isNeo ? 
			new SparkMaxMotorController(angleConfig.canId, MotorType.kBrushless) :
			new TalonMotorController(angleConfig.canId, TalonModel.TalonFX);

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
			targetState.speedMetersPerSecond  *// This is scales the velocity by how off the wheel is from the target angle.
			Math.abs(targetState.angle.minus(getModuleState().angle).getCos())
		);
		setModuleAngle(targetState.angle.getRadians());
	}

	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(
			driveMotor.getAngularVelocity() *
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER /
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
			SwerveConstants.WHEEL_DIAMETER *
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
			(SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER)
		);
	}

	/**
	 * Minimize the change in heading the desired swerve module state would require
	 * by potentially
	 * reversing the direction the wheel spins. Customized from WPILib's version to
	 * include placing
	 * in appropriate scope for CTRE onboard control.
	 * <p>
	 * The WPI optimize function assumes that the pid controller is continuous,
	 * which
	 * means that if you are at 359 degrees and set a target of 1 degree, it assumes
	 * the controller will be able to tell the module to move 2 degrees forwards and
	 * not 358 degrees backwards (which is how the wpilib pid controller functions).
	 * <p>
	 * However CTRE onboard pid controllers are not continuous, if it’s at a
	 * setpoint at 359 and you tell it to go 1, it will rotate 358 degrees backwards
	 * instead of 2 degrees forwards. If you want it to go to 1 you actually need to
	 * tell it to go to 361 degrees. To do this you need the custom optimize
	 * function defined here.
	 * <p>
	 * 
	 * @see <a
	 *      href=https://www.chiefdelphi.com/t/swerve-modules-flip-180-degrees-periodically-conditionally/393059/3
	 *      >Chief Delphi Post Concerning The Issue</a>
	 * 
	 * @param desiredState The desired state.
	 * @param currentAngle The current module angle.
	 */
	public static SwerveModuleState optimizeTalon(SwerveModuleState desiredState, Rotation2d currentAngle) {
		double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
		double targetSpeed = desiredState.speedMetersPerSecond;
		double delta = targetAngle - currentAngle.getDegrees();
		if (Math.abs(delta) > 90) {
			targetSpeed = -targetSpeed;
			targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
		}
		return new SwerveModuleState(
				targetSpeed,
				Rotation2d.fromDegrees(targetAngle));
	}

	/**
	 * Places the given angle in the appropriate 0 to 360 degree scope based on the
	 * reference angle.
	 * 
	 * @param scopeReference the reference angle to base the scope on
	 * @param newAngle       the angle to place in the scope
	 * @return the new angle within the appropriate 0 to 360 degree scope
	 */
	private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
		double lowerBound;
		double upperBound;
		double lowerOffset = scopeReference % 360;
		if (lowerOffset >= 0) {
			lowerBound = scopeReference - lowerOffset;
			upperBound = scopeReference + (360 - lowerOffset);
		} else {
			upperBound = scopeReference - lowerOffset;
			lowerBound = scopeReference - (360 + lowerOffset);
		}
		while (newAngle < lowerBound) {
			newAngle += 360;
		}
		while (newAngle > upperBound) {
			newAngle -= 360;
		}
		if (newAngle - scopeReference > 180) {
			newAngle -= 360;
		} else if (newAngle - scopeReference < -180) {
			newAngle += 360;
		}
		return newAngle;
	}

	public static class SwerveMotorConfig {
		public boolean isDriveMotor;
		public boolean isNeo;
		public boolean invert;
		public int canId;
		public int currentLimit;
		public PIDConstants pid;
		public SwerveMotorConfig(
			int canId, boolean isDriveMotor, boolean invert, 
			boolean isNeo, int currentLimit, PIDConstants pid) {
			this.canId = canId;
			this.isDriveMotor = isDriveMotor;
			this.invert = invert;
			this.isNeo = isNeo;
			this.currentLimit = currentLimit;
			this.pid = pid;
		}
	}
}
