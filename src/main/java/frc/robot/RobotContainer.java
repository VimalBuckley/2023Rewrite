package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.TeleopDriveCommand;

public class RobotContainer {
	private CommandXboxController xbox;
	private SwerveDrive swerve;
	private Autonomous autonomous;
	private MessagingSystem messaging;
	private Command autoCommand;

	private final int DRIVER_PORT = -2;

	public RobotContainer() {
		swerve = SwerveDrive.getInstance();
		autonomous = Autonomous.getInstance();
		messaging = MessagingSystem.getInstance();
		setupDriveController();
	}

	public void setupDriveController() {
		xbox = new CommandXboxController(DRIVER_PORT);
		TeleopDriveCommand swerveCommand = new TeleopDriveCommand(xbox);
		swerve.setDefaultCommand(swerveCommand);

		Trigger switchDriveModeButton = xbox.x();
		Trigger resetGyroButton = xbox.a();
		Trigger slowModeButton = xbox.leftBumper();
		Trigger cancelationButton = xbox.start();

		switchDriveModeButton.toggleOnTrue(swerveCommand.toggleRobotCentricCommand());
		resetGyroButton.onTrue(swerveCommand.resetGyroCommand());
		slowModeButton.whileTrue(swerveCommand.toggleSlowModeCommand());
		cancelationButton.onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
	}


	public Command rumbleCommand(double timeSeconds) {
		return Commands.startEnd(
			() -> xbox.getHID().setRumble(RumbleType.kBothRumble, 0.5),
			() -> xbox.getHID().setRumble(RumbleType.kBothRumble, 0)
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
