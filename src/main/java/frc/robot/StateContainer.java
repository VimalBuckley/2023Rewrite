package frc.robot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeState;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.utilities.Loggable;

public class StateContainer implements Loggable {
    private static StateContainer instance;
    public GamePiece currentGamePiece = GamePiece.Cone;
    public PlacerState targetPlacerState = PlacerState.Start;
    public IntakeState targetIntakeState = IntakeState.Off;
    public Pose2d currentRobotPosition = new Pose2d();

    private StateContainer() {}

    public static synchronized StateContainer getInstance() {
        return instance == null ? new StateContainer() : instance;
    }

    @Override
    public void logData(Logger logger, LogTable table) {
        table.put("Current Game Piece", currentGamePiece.name());
        table.put("Target Placer State", targetPlacerState.name());
        table.put("Target Intake State", targetIntakeState.name());
        logger.recordOutput("Current Robot Pose", currentRobotPosition);
    }

    @Override
    public String getTableName() {
        return "Robot State";
    }
}
