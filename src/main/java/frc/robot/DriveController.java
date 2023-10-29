package frc.robot;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDriveCommand;
import frc.robot.subsystems.vision.Vision;

public class DriveController extends CommandXboxController {
	private static DriveController instance;
	private SwerveDriveCommand swerveCommand;
	private final Trigger switchDriveModeButton = this.x();
	private final Trigger resetGyroButton = this.a();
	private final Trigger slowModeButton = this.leftBumper();
	private final Trigger cancelButton = this.start();

	private DriveController() {
		super(JoystickConstants.DRIVER_PORT);
		setButtons();
		if (RobotState.isTest()) {
			addToShuffleBoard();
		}
	}

	public static synchronized DriveController getInstance() {
		return instance == null ? new DriveController() : instance;
	}

	private void setButtons() {
		swerveCommand = new SwerveDriveCommand(this);
		SwerveDrive.getInstance().setDefaultCommand(swerveCommand);

		switchDriveModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.switchDriveMode()));

		resetGyroButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.resetGyro(0)));

		slowModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.slowSpeed()));
		slowModeButton.toggleOnFalse(new InstantCommand(() -> swerveCommand.fastSpeed()));

		cancelButton.toggleOnTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
	}

	private void addToShuffleBoard() {
		Shuffleboard.getTab("Messaging").add("Messaging System", MessagingSystem.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve", SwerveDrive.getInstance());
		Shuffleboard.getTab("Vision").add("Vision", Vision.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
	}

	public Command rumbleCommand(double timeSeconds) {
		return Commands.startEnd(
			() -> getHID().setRumble(RumbleType.kBothRumble, 0.5),
			() -> getHID().setRumble(RumbleType.kBothRumble, 0)
		).withTimeout(timeSeconds);
	}
}
