package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.DriveController;

public class RumbleCommand extends CommandBase {
	private CommandGenericHID controller;
	private double time;
	private double localTime;

	public RumbleCommand(double timeSeconds) {
		this.controller = DriveController.getInstance();
		this.time = 50 * timeSeconds; // 50 is how many times execute is called per second.
	}

	public void initialize() {
		controller.getHID().setRumble(RumbleType.kBothRumble, 0.5);
		localTime = 0;
	}

	public void execute() {
		localTime++;
	}

	public boolean isFinished() {
		return localTime >= time;
	}

	public void end(boolean interrupted) {
		controller.getHID().setRumble(RumbleType.kBothRumble, 0);
	}
}