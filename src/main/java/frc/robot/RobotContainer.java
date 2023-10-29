package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDriveCommand;

public class RobotContainer {
	private CommandXboxController driveController;
	private Autonomous autonomous;
	private MessagingSystem messaging;
	private Command autoCommand;

	public RobotContainer() {
		setupDriveController();
		autonomous = Autonomous.getInstance();
		messaging = MessagingSystem.getInstance();
	}

	public void setupDriveController() {
		driveController = new CommandXboxController(JoystickConstants.DRIVER_PORT);
		SwerveDriveCommand swerveCommand = new SwerveDriveCommand(driveController);
		SwerveDrive.getInstance().setDefaultCommand(swerveCommand);

		driveController.x().onTrue(Commands.runOnce(() -> swerveCommand.switchDriveMode()));
		driveController.a().onTrue(Commands.runOnce(() -> swerveCommand.resetGyro(0)));
		driveController.leftBumper().onTrue(Commands.runOnce(() -> swerveCommand.slowSpeed()));
		driveController.leftBumper().onFalse(Commands.runOnce(() -> swerveCommand.fastSpeed()));
		driveController.start().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
	}

	public Command rumbleCommand(double timeSeconds) {
		return Commands.startEnd(
			() -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0.5),
			() -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0)
		).withTimeout(timeSeconds);
	}

	public void autonomousInit() {
		messaging.enableMessaging();
		messaging.addMessage("Auto Started");
		autoCommand = autonomous.getAutonCommand();
		if (autoCommand != null) {
			autoCommand.schedule();
		} else {
			messaging.addMessage("No Auto Command Selected");
		}
	}

	public void teleopInit() {
		messaging.enableMessaging();
		messaging.addMessage("Teleop Started");
		if (autoCommand != null) {
			autoCommand.cancel();
		}
	}
}
