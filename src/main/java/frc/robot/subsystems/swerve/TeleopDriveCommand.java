package frc.robot.subsystems.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A swerve command with support for two swerve control modes:
 * <p>
 * Field-Centric:
 * Robot moves relative to the field's axes.
 * When pushing the joystick forward, the robot moves down the field, no matter which way it is facing
 * (Actually, it moves in whatever direction is zeroed to, this just assumes that the gyro is zeroed down the field)
 * <p>
 * Robot-Centric:
 * The robot moves relative to itself.
 * When pushing the joystick foward, the robot moves in whatever direction it is facing.
 * For our purposes, the front of the robot is the intake side.
 */
public class TeleopDriveCommand extends CommandBase {
	private SwerveDrive swerve;
	private CommandXboxController controller;
	private DriveMode driveMode;
	private Rotation2d targetAngle;
	private Rotation2d alignmentAngle;
	private DriveSens driveSens;
	private SlewRateLimiter xLimiter = new SlewRateLimiter(1.75);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(1.75);
	
	private double sidewaysVelocity;
	private double forwardVelocity;
	private double rotationalVelocity;

	public TeleopDriveCommand(CommandXboxController xboxController) {
		swerve = SwerveDrive.getInstance();
		controller = xboxController;
		addRequirements(swerve);
	}

    @Override
    public void initialize() {
        driveMode = DriveMode.AngleCentric;
		targetAngle = swerve.getRobotAngle();
		alignmentAngle = swerve.getRobotAngle();
		driveSens = DriveSens.Fast;
		Shuffleboard.getTab("Display").addString("Drive Mode", () -> driveMode.name());
		Shuffleboard.getTab("Display").addDouble("Target Angle Degrees ", () -> targetAngle.getDegrees());
    }

	@Override
	public void execute() {
		double rightX = -controller.getRightX();
		double rightY = -controller.getRightY();
		double leftX = -controller.getLeftX();
		double leftY = -controller.getLeftY();

		if (Math.abs(rightY) > 0.5) {
			targetAngle = Rotation2d.fromDegrees(90 - 90 * Math.signum(rightY));
		}
		if (Math.abs(rightX) > 0.1) {
			targetAngle = Rotation2d.fromDegrees(targetAngle.getDegrees() + rightX * driveSens.rotational);
		}

		if(driveSens.slew) {
			sidewaysVelocity = xLimiter.calculate(leftX) * driveSens.forward;
			forwardVelocity = yLimiter.calculate(leftY) * driveSens.sideways;
		} else {
			sidewaysVelocity = leftX * driveSens.forward;
			forwardVelocity = leftY * driveSens.sideways;
		}
		rotationalVelocity = rightX * driveSens.rotational;

		switch (driveMode) {
			case RobotCentric:
				swerve.driveRobotCentric(
					new ChassisSpeeds(
						forwardVelocity, 
						sidewaysVelocity,
						rotationalVelocity
					)
                );
				break;
			case AngleCentric:
				swerve.driveAngleCentric(
                    forwardVelocity, 
                    sidewaysVelocity, 
                    targetAngle
                );
                break;
			case AlignToTarget:
				swerve.driveAlignToTarget(
					forwardVelocity, 
					sidewaysVelocity,
					alignmentAngle
				);
				break;
        }
	}

    public Command toggleSlowModeCommand() {
        return Commands.startEnd(
            () -> {
                driveSens = DriveSens.Slow;
            },
            () -> {
                driveSens = DriveSens.Fast;
            }
        );
    }

    public Command toggleRobotCentricCommand() {
        return Commands.startEnd(
            () -> driveMode = DriveMode.RobotCentric,
            () -> {
				driveMode = DriveMode.AngleCentric;
				targetAngle = swerve.getRobotAngle();
			}
        );
    }

	public Command toggleAlignToAngleCommand() {
		return Commands.startEnd(
			() -> {
				driveMode = DriveMode.AlignToTarget;
				alignmentAngle = swerve.getRobotAngle();
			}, 
			() -> {
				driveMode = DriveMode.AngleCentric;
				targetAngle = swerve.getRobotAngle();
			}
		);
	}

	public Command setTargetAngleCommand(Rotation2d newTarget) {
		return Commands.runOnce(
			() -> targetAngle = newTarget
		);
	}

    public Command resetGyroCommand() {
        return Commands.runOnce(
            () -> {
                targetAngle = new Rotation2d();
                swerve.resetRobotAngle(new Rotation2d());
            }
        );    
    }

	public static enum DriveMode {
		AngleCentric,
		RobotCentric,
		AlignToTarget
	}

	public static enum DriveSens {
		Fast(4, 4, 3.5, true),
		Slow(0.8, 0.8, 0.5, false);

		public double forward;
		public double sideways;
		public double rotational;
		public boolean slew;

		private DriveSens(double forward, double sideways, double rotational, boolean doSlew) {
			this.forward = forward;
			this.sideways = sideways;
			this.rotational = rotational;
			this.slew = doSlew;
		}
	}

	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("Drive Mode", () -> driveMode.name(), null);
		builder.addDoubleProperty("Target Angle: ", () -> targetAngle.getDegrees(), null);
	}
}