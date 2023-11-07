package frc.robot.subsystems.placer;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeState;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.Constants.EnumConstants.TalonModel;
import frc.robot.Constants.PlacerConstants;
import frc.robot.hardware.EncodedMotorController;
import frc.robot.hardware.SparkMaxMotorController;
import frc.robot.hardware.TalonMotorController;
import frc.robot.utilities.Loggable;
import org.littletonrobotics.junction.LogTable;

public class Placer extends SubsystemBase implements Loggable {
	private static Placer instance;
	private EncodedMotorController outputMotor;
	private EncodedMotorController intakeAngleMotor;
	private EncodedMotorController armAngleMotor;
	private EncodedMotorController extensionMotor;
	private GamePiece currentGamePiece;
	private PlacerState targetPlacerState;

	private Placer() {
		extensionMotor = new TalonMotorController(
			PlacerConstants.ARM_EXTENSION_MOTOR_ID,
			TalonModel.TalonSRX
		);
		extensionMotor.setPID(new PIDConstants(0.4, 0, 0))
		.setMinOutput(-0.3)
		.setMaxOutput(0.6)
		.setAngleTolerance(400)
		.setMaxAngle(15.3398);

		armAngleMotor = new SparkMaxMotorController(
			PlacerConstants.ARM_ANGLE_MOTOR_ID,
			MotorType.kBrushless
		);
		armAngleMotor.setInversion(true)
		.setPID(new PIDConstants(0.04, 0, 0))
		.setMinOutput(-0.5)
		.setMaxOutput(0.75);

		intakeAngleMotor = new SparkMaxMotorController(
			PlacerConstants.INTAKE_ANGLE_MOTOR_ID,
			MotorType.kBrushless
		);
		intakeAngleMotor.setBrakeOnIdle(true)
		.setMinAngle(Units.rotationsToRadians(-40))
		.setPID(new PIDConstants(1, 0, 0))
		.setMinOutput(-0.3)
		.setMaxOutput(0.3);

		outputMotor = new SparkMaxMotorController(
			PlacerConstants.INTAKE_RUN_MOTOR_ID,
			MotorType.kBrushless
		);
		outputMotor.setBrakeOnIdle(false);

		currentGamePiece = GamePiece.Cone;
	}

	public static synchronized Placer getInstance() {
		if (instance == null) instance = new Placer();
        return instance;
	}

	@Override
	public String getTableName() {
		return "Placer";
	}

	@Override
	public void logData(LogTable table) {
		table.put("Output (Percent)", getOutput());
		table.put("Intake Angle (Radians", getIntakeAngle());
		table.put("Arm Angle (Radians", getArmAngle());
		table.put("Extension (Radians)", getArmExtension());
		table.put("Game Piece", getGamePiece().name());
		table.put("Placer State", getPlacerState().name());
	}

	public void setOutput(double output) {
		outputMotor.setOutput(output);
	}

	public double getOutput() {
		return outputMotor.getOutput();
	}

	public void setIntakeAngle(double angleRadians) {
		intakeAngleMotor.setAngle(angleRadians);
	}

	public double getIntakeAngle() {
		return intakeAngleMotor.getAngle();
	}

	public void setArmAngle(double angleRadians) {
		armAngleMotor.setAngle(angleRadians);
	}

	public double getArmAngle() {
		return armAngleMotor.getAngle();
	}

	public void setArmExtension(double extensionAsRadians) {
		extensionMotor.setAngle(extensionAsRadians);
	}

	public double getArmExtension() {
		return extensionMotor.getAngle();
	}

	public void setGamePiece(GamePiece newGamePiece) {
		currentGamePiece = newGamePiece;
	}

	public GamePiece getGamePiece() {
		return currentGamePiece;
	}

	public void setPlacerState(PlacerState newPlacerState) {
		targetPlacerState = newPlacerState;
	}

	public PlacerState getPlacerState() {
		return targetPlacerState;
	}

	public Command setPlacerCommand(
		PlacerState placerState,
		IntakeState intakeState
	) {
		return setPlacerCommand(placerState)
			.alongWith(runIntakeCommand(intakeState));
	}

	public Command setPlacerCommand(PlacerState placerState) {
		return Commands.runOnce(() -> {
			setPlacerState(placerState);
			setArmExtension(placerState.extension);
			setArmAngle(placerState.armAngle);
			setIntakeAngle(placerState.intakeAngle);
		});
	}

	public Command runIntakeCommand(IntakeState intakeState) {
		return Commands.runOnce(() -> {
			if (intakeState == IntakeState.Place) {
				setOutput(getGamePiece().outputToPlace);
			} else {
				setOutput(intakeState.output);
			}
			if (
				intakeState == IntakeState.PickupCube &&
				getPlacerState() == PlacerState.Substation
			) {
				setIntakeAngle(getIntakeAngle() - 10);
			}
			if (intakeState.newGamePiece != null) {
				setGamePiece(intakeState.newGamePiece);
			}
		});
	}

	public Command zeroPlacerCommand() {
		return setPlacerCommand(
			RobotState.isAutonomous() ? PlacerState.Zero : PlacerState.Travel,
			IntakeState.Off
		);
	}

	public Command placeCommand() {
		return runIntakeCommand(IntakeState.Place)
			.andThen(Commands.waitSeconds(1))
			.andThen(setPlacerCommand(PlacerState.Travel, IntakeState.Off));
	}

	public Command placeCommand(PlacerState placerState) {
		return setPlacerCommand(placerState)
			.andThen(Commands.waitSeconds(1.5))
			.andThen(placeCommand());
	}
}
