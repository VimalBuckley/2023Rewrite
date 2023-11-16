package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CANConstants;
import frc.robot.hardware.TalonMotorController;
import frc.robot.hardware.TalonMotorController.TalonModel;
import frc.robot.subsystems.swerve.SwerveModule.SwerveMotorConfig;

public class SwerveConstants {
    public static final double MAX_LINEAR_SPEED =
			((1276 * 9.42) / 60) / 12 * 0.3048; // 1276 is rpm, 9.42 is wheel circumference (in.), final units are m/s
    public static final double MAX_LINEAR_ACCELERATION = 4; // Random number
    public static final double MAX_ROTATIONAL_SPEED =
        MAX_LINEAR_SPEED / (4 / 3); // 4/3 is (about) the radius from the center of the robot to the swerve drive wheels.
    public static final double MAX_ROTATIONAL_ACCELERATION = 4; // Linear Acceleration/radius
    /** Drive rotations per motor rotation */
    public static final double DRIVE_RATIO = 1 / 5.;
    /** Angle rotations per motor rotation */
    public static final double ANGLE_RATIO = 1 / 6.75;

    public static final SwerveMotorConfig FRONT_LEFT_DRIVE_CONFIG = new SwerveMotorConfig.Builder()
            .motor(new TalonMotorController(CANConstants.SWERVE_FRONT_LEFT_DRIVE, TalonModel.TalonFX))
            .invert(true)
            .currentLimit(35)
            .pid(new PIDConstants(0.075, 0, 0))
            .build();

    public static final SwerveMotorConfig FRONT_LEFT_ANGLE_CONFIG = new SwerveMotorConfig.Builder()
            .motor(new TalonMotorController(CANConstants.SWERVE_FRONT_LEFT_ANGLE, TalonModel.TalonFX))
            .invert(false)
            .currentLimit(25)
            .pid(new PIDConstants(0.3, 0, 0))
            .build();

    public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(
        0.3175,
        0.2413
    );

    public static final SwerveMotorConfig FRONT_RIGHT_DRIVE_CONFIG = new SwerveMotorConfig.Builder()
            .motor(new TalonMotorController(CANConstants.SWERVE_FRONT_RIGHT_DRIVE, TalonModel.TalonFX))
            .invert(false)
            .currentLimit(35)
            .pid(new PIDConstants(0.05, 0, 0))
            .build();

    public static final SwerveMotorConfig FRONT_RIGHT_ANGLE_CONFIG = new SwerveMotorConfig.Builder()
            .motor(new TalonMotorController(CANConstants.SWERVE_FRONT_RIGHT_ANGLE, TalonModel.TalonFX))
            .invert(false)
            .currentLimit(25)
            .pid(new PIDConstants(0.3, 0, 0))
            .build();

    public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(
        0.3175,
        -0.2413
    );

    public static final SwerveMotorConfig BACK_LEFT_DRIVE_CONFIG = new SwerveMotorConfig.Builder()
            .motor(new TalonMotorController(CANConstants.SWERVE_BACK_LEFT_DRIVE, TalonModel.TalonFX))
            .invert(true)
            .currentLimit(35)
            .pid(new PIDConstants(0.075, 0, 0))
            .build();

    public static final SwerveMotorConfig BACK_LEFT_ANGLE_CONFIG = new SwerveMotorConfig.Builder()
            .motor(new TalonMotorController(CANConstants.SWERVE_BACK_LEFT_ANGLE, TalonModel.TalonFX))
            .invert(false)
            .currentLimit(25)
            .pid(new PIDConstants(0.25, 0, 0))
            .build();

    public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(
        -0.3175,
        0.2413
    );

    public static final SwerveMotorConfig BACK_RIGHT_DRIVE_CONFIG = new SwerveMotorConfig.Builder()
            .motor(new TalonMotorController(CANConstants.SWERVE_BACK_RIGHT_DRIVE, TalonModel.TalonFX))
            .invert(false)
            .currentLimit(35)
            .pid(new PIDConstants(0.05, 0, 0))
            .build();

    public static final SwerveMotorConfig BACK_RIGHT_ANGLE_CONFIG = new SwerveMotorConfig.Builder()
            .motor(new TalonMotorController(CANConstants.SWERVE_BACK_RIGHT_ANGLE, TalonModel.TalonFX))
            .invert(false)
            .currentLimit(25)
            .pid(new PIDConstants(0.3, 0, 0))
            .build();

    public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(
        -0.3175,
        -0.2413
    );
    
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
}
