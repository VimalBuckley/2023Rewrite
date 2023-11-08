package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.EnumConstants.IntakeState;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.placer.Placer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.TeleopDriveCommand;

public class RobotContainer {
	private CommandXboxController xbox;
	private CommandJoystick flightSim;
	private SwerveDrive swerve;
	private Placer placer;
	private Autonomous autonomous;
	private MessagingSystem messaging;
	private Command autoCommand;

	public RobotContainer() {
		// placer = Placer.getInstance();
		swerve = SwerveDrive.getInstance();
		autonomous = Autonomous.getInstance();
		messaging = MessagingSystem.getInstance();
		setupDriveController();
		// setupOperatorController();
	}

	public void setupDriveController() {
		xbox = new CommandXboxController(JoystickConstants.DRIVER_PORT);
		TeleopDriveCommand swerveCommand = new TeleopDriveCommand(xbox);
		swerve.setDefaultCommand(swerveCommand);

		Trigger switchDriveModeButton = xbox.x();
		Trigger resetGyroButton = xbox.a();
		Trigger setTargetAngleButton = xbox.b();
		Trigger slowModeButton = xbox.leftBumper();
		Trigger cancelationButton = xbox.start();

		switchDriveModeButton.toggleOnTrue(swerveCommand.toggleRobotCentricCommand());
		resetGyroButton.onTrue(swerveCommand.resetGyroCommand(new Rotation2d()));
		setTargetAngleButton.onTrue(swerveCommand.setTargetAngleCommand(Rotation2d.fromDegrees(90)));
		slowModeButton.whileTrue(swerveCommand.toggleSlowModeCommand());
		cancelationButton.onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
	}

	public void setupOperatorController() {
		flightSim = new CommandJoystick(JoystickConstants.OPERATOR_PORT);
		Trigger placeButton = flightSim.button(1);
		Trigger cubeButton = flightSim.button(12);
		Trigger coneButton = flightSim.button(6);
		Trigger substationButton = flightSim.button(10);
		Trigger topButton = flightSim.button(8);
		Trigger middleButton = flightSim.button(7);
		Trigger groundButton = flightSim.button(9);

		placeButton
			.onTrue(placer.runIntakeCommand(IntakeState.Place))
			.onFalse(placer.setPlacerCommand(PlacerState.Travel, IntakeState.Off));

		cubeButton
			.onTrue(placer.runIntakeCommand(IntakeState.PickupCube))
			.onFalse(placer.setPlacerCommand(PlacerState.Travel, IntakeState.Off));

		coneButton
			.onTrue(placer.runIntakeCommand(IntakeState.PickupCone))
			.onFalse(placer.setPlacerCommand(PlacerState.Travel, IntakeState.Off));

		substationButton.onTrue(placer.setPlacerCommand(PlacerState.Substation));
		topButton.onTrue(placer.setPlacerCommand(PlacerState.Top));
		middleButton.onTrue(placer.setPlacerCommand(PlacerState.Middle));
		groundButton.onTrue(placer.setPlacerCommand(PlacerState.Ground));
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
