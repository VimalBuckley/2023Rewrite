package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;
import frc.robot.utilities.Loggable;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;


public class Vision extends SubsystemBase implements Loggable {
	private static Vision instance;
	private Limelight aprilTagLimelight;
	private Limelight gamePieceLimelight;

	private Vision() {
		aprilTagLimelight = new Limelight("limelight-hehehe");
		gamePieceLimelight = new Limelight("limelight-haha");
		Shuffleboard.getTab("Display").addDouble(
			"Horizontal Offset", 
			() -> getGamePieceHorizontalOffset().orElse(new Rotation2d()).getDegrees()
		);
	}

	public static synchronized Vision getInstance() {
		if (instance == null) instance = new Vision();
		return instance;
	}

	@Override
	public void logData(LogTable table) {
		table.put("Tag ID", getTagId().orElse(0));
		Logger.getInstance().recordOutput("Vision Odometry", getRobotPose().orElse(new Pose2d()));
	}

	@Override
	public String getTableName() {
		return "Vision";
	}

	public Limelight getAprilTageLimelight() {
		return aprilTagLimelight;
	}

	public Limelight getGamePieceLimelight() {
		return gamePieceLimelight;
	}

	public boolean seesTag() {
		return aprilTagLimelight.hasValidTargets();
	}

	public Optional<Integer> getTagId() {
		return seesTag()
			? Optional.of(aprilTagLimelight.getTargetTagId())
			: Optional.empty();
	}

	public Optional<Pose2d> getRobotPose() {
		return getRobotPose(DriverStation.getAlliance());
	}

	public Optional<Pose2d> getRobotPose(Alliance poseOrigin) {
		return seesTag() 
			? Optional.of(aprilTagLimelight.getRobotPoseToAlliance(poseOrigin))
			: Optional.empty();
	}

	public Optional<Pose2d> getRelativeTargetPose() {
		return seesTag()
			? Optional.of(aprilTagLimelight.getTargetPoseToRobot())
			: Optional.empty();
	}

	public Optional<Rotation2d> getGamePieceHorizontalOffset() {
		return seesGamePiece()
			? Optional.of(gamePieceLimelight.getHorizontalOffsetFromCrosshair())
			: Optional.empty();
	}

	public Optional<Rotation2d> getGamePieceVerticalOffset() {
		return seesGamePiece()
			? Optional.of(gamePieceLimelight.getVerticalOffsetFromCrosshair())
			: Optional.empty();
	}

	public Optional<Double> getGamePieceTakenArea() {
		return seesGamePiece()
			? Optional.of(gamePieceLimelight.getTargetArea())
			: Optional.empty();
	}

	public Optional<Rotation2d> getGamePieceSkew() {
		return seesGamePiece()
			? Optional.of(gamePieceLimelight.getSkew())
			: Optional.empty();
	}

	public boolean seesGamePiece() {
		return gamePieceLimelight.hasValidTargets();
	}
}
