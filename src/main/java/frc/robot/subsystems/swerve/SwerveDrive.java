package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.EnumConstants.DriveMode;
import frc.robot.Constants.EnumConstants.DriveSens;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.NavX;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase implements Loggable {

	private static SwerveDrive instance;
	private NavX gyro;
	private Vision vision;
	private SwerveModule[] modules;
	private SwerveDriveKinematics kinematics;
	private SwerveDriveOdometry odometry;
	private SwerveDrivePoseEstimator poseEstimator;
	private DriveMode mode;
	private PIDController anglePID;
	private DriveSens sens;

	private SwerveDrive() {
		anglePID = new PIDController(4, 0, 0);
		anglePID.enableContinuousInput(-Math.PI, Math.PI);
		anglePID.setTolerance(Math.PI / 32, Math.PI / 32);
		anglePID.setSetpoint(0);
		mode = DriveMode.AngleCentric;
		sens = DriveSens.Fast;
		modules =
			new SwerveModule[] {
				new SwerveModule(
					SwerveConstants.FRONT_LEFT_DRIVE_CONFIG,
					SwerveConstants.FRONT_LEFT_ANGLE_CONFIG,
					SwerveConstants.FRONT_LEFT_MODULE_TRANSLATION
				),
				new SwerveModule(
					SwerveConstants.FRONT_RIGHT_DRIVE_CONFIG,
					SwerveConstants.FRONT_RIGHT_ANGLE_CONFIG,
					SwerveConstants.FRONT_RIGHT_MODULE_TRANSLATION
				),
				new SwerveModule(
					SwerveConstants.BACK_LEFT_DRIVE_CONFIG,
					SwerveConstants.BACK_LEFT_ANGLE_CONFIG,
					SwerveConstants.BACK_LEFT_MODULE_TRANSLATION
				),
				new SwerveModule(
					SwerveConstants.BACK_RIGHT_DRIVE_CONFIG,
					SwerveConstants.BACK_RIGHT_ANGLE_CONFIG,
					SwerveConstants.BACK_RIGHT_MODULE_TRANSLATION
				),
			};
		gyro = new NavX(I2C.Port.kMXP);
		vision = Vision.getInstance();
		kinematics = new SwerveDriveKinematics(getModuleTranslations());
		odometry =
			new SwerveDriveOdometry(
				kinematics,
				gyro.getRotation2d(),
				getModulePositions(),
				vision.getRobotPose()
			);
		poseEstimator =
			new SwerveDrivePoseEstimator(
				kinematics,
				gyro.getRotation2d(),
				getModulePositions(),
				vision.getRobotPose()
			);
	}

	public static synchronized SwerveDrive getInstance() {
		return instance == null ? new SwerveDrive() : instance;
	}

	@Override
	public void periodic() {
		odometry.update(gyro.getRotation2d(), getModulePositions());
		poseEstimator.update(gyro.getRotation2d(), getModulePositions());
		if (vision.getTagId() != -1) {
			poseEstimator.addVisionMeasurement(
				vision.getRobotPose(),
				Timer.getFPGATimestamp()
			);
		}
	}

	public void driveAngleCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		CommandXboxController controller
	) {
		double rotationalVelocity = -controller.getRightX() * sens.rotationalSens;
		if (Math.abs(controller.getRightY()) > 0.5) {
			setTargetAngle(90 - Math.signum(controller.getRightY() * 90));
		} else if (Math.abs(controller.getRightX()) > 0.1) {
			setTargetAngle(getTargetAngle() + rotationalVelocity);
		}
		driveAngleCentric(forwardVelocity, sidewaysVelocity);
	}

	public void driveAngleCentric(
		double forwardVelocity,
		double sidewaysVelocity
	) {
		driveFieldCentric(
			forwardVelocity,
			sidewaysVelocity,
			anglePID.atSetpoint() ?
				0 :
				anglePID.calculate(Math.toRadians(getRobotAngleDegrees()))
		);
	}

	public void driveFieldCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		double rotationalVelocity
	) {
		driveModules(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				forwardVelocity,
				sidewaysVelocity,
				rotationalVelocity,
				Rotation2d.fromDegrees(getRobotAngleDegrees())
			)
		);
	}

	public void driveRobotCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		double rotationalVelocity
	) {
		driveModules(
			new ChassisSpeeds(
				forwardVelocity,
				sidewaysVelocity,
				rotationalVelocity
			)
		);
	}

	public void driveModules(ChassisSpeeds targetChassisSpeeds) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(
			discretize(targetChassisSpeeds)
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			states,
			SwerveConstants.MAX_LINEAR_SPEED
		);
		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(states[i]);
		}
	}

	/**
	 * Fixes situation where robot drifts in the direction it's rotating in if turning and translating at the same time
	 * @see https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
	 */
	private static ChassisSpeeds discretize(
		ChassisSpeeds originalChassisSpeeds
	) {
		double vx = originalChassisSpeeds.vxMetersPerSecond;
		double vy = originalChassisSpeeds.vyMetersPerSecond;
		double omega = originalChassisSpeeds.omegaRadiansPerSecond;
		double dt = 0.02; // This should be the time these values will be used, so normally just the loop time
		Pose2d desiredDeltaPose = new Pose2d(
			vx * dt,
			vy * dt,
			new Rotation2d(omega * dt)
		);
		Twist2d twist = new Pose2d().log(desiredDeltaPose);
		return new ChassisSpeeds(
			twist.dx / dt,
			twist.dy / dt,
			twist.dtheta / dt
		);
	}

	public NavX getGyro() {
		return gyro;
	}

	public void zeroModules() {
		for (SwerveModule module : modules) {
			module.setModuleVelocity(0);
			module.setModuleAngle(0);
		}
	}

	public Translation2d[] getModuleTranslations() {
		Translation2d[] translations = new Translation2d[modules.length];
		for (int i = 0; i < modules.length; i++) {
			translations[i] = modules[i].getTranslationFromCenter();
		}
		return translations;
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getModuleState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			positions[i] = modules[i].getModulePosition();
		}
		return positions;
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	public Pose2d getEstimatorPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Pose2d getOdometryPose() {
		return odometry.getPoseMeters();
	}

	public void resetPose(Pose2d newPose) {
		odometry.resetPosition(
			gyro.getRotation2d(),
			getModulePositions(),
			newPose
		);
		poseEstimator.resetPosition(
			gyro.getRotation2d(),
			getModulePositions(),
			newPose
		);
	}

	public double getRobotAngleDegrees() {
		return Math.toDegrees(gyro.getOffsetedAngle());
	}

	/**
	 * Sets the field-centric zero to some angle relative to the robot
	 * <p>CCW is positive
	 * @param offsetRadians the angle relative to the robot, in radians
	 */
	public void resetRobotAngle(double offsetRadians) {
		gyro.offsetGyroZero(offsetRadians);
		setTargetAngle(Math.toDegrees(offsetRadians));
		MessagingSystem.getInstance().addMessage("Swerve -> Reset Gyro");
	}

	public void resetRobotAngle() {
		resetRobotAngle(0);
	}

	public double getCurrentZero() {
		return gyro.getGyroZero();
	}

	public void setDriveSens(DriveSens newSens) {
		sens = newSens;
	}

	public void setTargetAngle(double angleDegrees) {
		anglePID.setSetpoint(Math.toRadians(angleDegrees));
	}

	public double getTargetAngle() {
		return Math.toDegrees(anglePID.getSetpoint());
	}

	public DriveMode getDriveMode() {
		return mode;
	}

	public void setDriveMode(DriveMode newDriveMode) {
		mode = newDriveMode;
	}

	public Command goRobotCentricCommand() {
		return Commands.startEnd(
			() -> setDriveMode(DriveMode.RobotCentric), 
			() -> setDriveMode(DriveMode.AngleCentric)
		);
	}

	public Command teleopDriveCommand(CommandXboxController controller) {
		return Commands.run(
			() -> {
				double forwardVelocity = -controller.getLeftY() * sens.forwardSens;
				double sidewaysVelocity = -controller.getLeftX() * sens.sidewaysSens;
				double rotationalVelocity = -controller.getRightX() * sens.rotationalSens;
				switch (mode) {
					default:
					case AngleCentric:
						driveAngleCentric(
							forwardVelocity,
							sidewaysVelocity,
							controller
						);
						break;
					case RobotCentric:
						driveRobotCentric(
							forwardVelocity,
							sidewaysVelocity,
							rotationalVelocity
						);
				}
			},
			this
		);
	}

	@Override
	public void logData(LogTable table) {
		table.put(
			"Front Left Module Velocity (M/S)",
			modules[0].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Front Left Module Angle (Radians)",
			modules[0].getModuleState().angle.getRadians()
		);
		table.put(
			"Front Right Module Velocity (M/S)",
			modules[1].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Front Right Module Angle (Radians)",
			modules[1].getModuleState().angle.getRadians()
		);
		table.put(
			"Back Left Module Velocity (M/S)",
			modules[2].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Back Left Module Angle (Radians)",
			modules[2].getModuleState().angle.getRadians()
		);
		table.put(
			"Back Right Module Velocity (M/S)",
			modules[3].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Back Right Module Angle (Radians)",
			modules[3].getModuleState().angle.getRadians()
		);
		Logger.getInstance().recordOutput("Swerve Odometry", getOdometryPose());
		Logger.getInstance().recordOutput("Swerve + Vision Odometry", getEstimatorPose());
		Logger.getInstance().recordOutput("Module States", getModuleStates());
	}

	@Override
	public String getTableName() {
		return "Swerve";
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty(
			"Gyro Angle: ",
			() -> Math.toDegrees(gyro.getAngle()),
			null
		);
		builder.addDoubleProperty(
			"Gyro Offset From Zero: ",
			() -> getRobotAngleDegrees() % 360,
			null
		);
		builder.addDoubleProperty(
			"Current Forward Speed: ",
			() -> getChassisSpeeds().vxMetersPerSecond,
			null
		);
		builder.addDoubleProperty(
			"Current Sideways Speed: ",
			() -> getChassisSpeeds().vyMetersPerSecond,
			null
		);
		builder.addDoubleProperty(
			"Current Rotational Speed: ",
			() -> getChassisSpeeds().omegaRadiansPerSecond,
			null
		);
		builder.addDoubleProperty(
			"Estimated X: ",
			() -> getEstimatorPose().getX(),
			null
		);
		builder.addDoubleProperty(
			"Estimated Y: ",
			() -> getEstimatorPose().getY(),
			null
		);
		builder.addDoubleProperty(
			"Estimated Rotation: ",
			() -> Math.toDegrees(MathUtil.angleModulus(getEstimatorPose().getRotation().getRadians())),
			null
		);
	}
}
