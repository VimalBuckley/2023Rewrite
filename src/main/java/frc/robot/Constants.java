package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveModule.SwerveMotorConfig;
import static frc.robot.Constants.PlacerConstants.*;

public class Constants {
	public static class JoystickConstants {
		public static final int OPERATOR_PORT = 1;
		public static final int DRIVER_PORT = 2;
	}

	public static class SwerveConstants {
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

		public static final SwerveMotorConfig FRONT_LEFT_DRIVE_CONFIG = new SwerveMotorConfig(
			3, 
			true, 
			true, 
			false, 
			35, 
			new PIDConstants(0.075, 0, 0)
		);
		public static final SwerveMotorConfig FRONT_LEFT_ANGLE_CONFIG = new SwerveMotorConfig(
			7, 
			false, 
			false, 
			false, 25, 
			new PIDConstants(0.3, 0, 0)
		);
		public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(
			0.3175,
			0.2413
		);

		public static final SwerveMotorConfig FRONT_RIGHT_DRIVE_CONFIG = new SwerveMotorConfig(
			4, 
			true, 
			false, 
			false, 
			35, 
			new PIDConstants(0.05, 0, 0)
		);
		public static final SwerveMotorConfig FRONT_RIGHT_ANGLE_CONFIG = new SwerveMotorConfig(
			8, 
			false, 
			false, 
			false, 
			25, 
			new PIDConstants(0.3, 0, 0)
		);
		public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(
			0.3175,
			-0.2413
		);

		public static final SwerveMotorConfig BACK_LEFT_DRIVE_CONFIG = new SwerveMotorConfig(
			2, 
			true, 
			true, 
			false, 
			35, 
			new PIDConstants(0.075, 0, 0)
		);
		public static final SwerveMotorConfig BACK_LEFT_ANGLE_CONFIG = new SwerveMotorConfig(
			6, 
			false, 
			false, 
			false, 25, 
			new PIDConstants(0.25, 0, 0)
		);
		public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(
			-0.3175,
			0.2413
		);

		public static final SwerveMotorConfig BACK_RIGHT_DRIVE_CONFIG = new SwerveMotorConfig(
			9, 
			true, 
			false, 
			false, 
			35, 
			new PIDConstants(0.05, 0, 0)
		);
		public static final SwerveMotorConfig BACK_RIGHT_ANGLE_CONFIG = new SwerveMotorConfig(
			5, 
			false, 
			false, 
			false, 
			25, 
			new PIDConstants(0.3, 0, 0)
		);
		public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(
			-0.3175,
			-0.2413
		);
		
		public static final double WHEEL_DIAMETER = 0.0762; // in meters
	}

	public static class PlacerConstants {
		public static final int INTAKE_RUN_MOTOR_ID = 13;
		public static final int INTAKE_ANGLE_MOTOR_ID = 12;
		public static final int ARM_ANGLE_MOTOR_ID = 10;
		public static final int ARM_EXTENSION_MOTOR_ID = 15;

		public static final double ZERO_EXTENSION = 0.0;
		public static final double GROUND_EXTENSION = 12.22;
		public static final double MIDDLE_EXTENSION = 6.0;
		public static final double TOP_EXTENSION = 17.0;
		public static final double SUBSTATION_EXTENSION = 8.62;

		public static final double ARM_START_ANGLE = 0.0;
		public static final double ARM_ZERO_ANGLE = -10;
		public static final double ARM_TRAVEL_ANGLE = -110;
		public static final double ARM_MIDDLE_ANGLE = -8;
		public static final double ARM_TOP_ANGLE = -10;
		public static final double ARM_SUBSTATION_ANGLE = -13;
		public static final double ARM_GROUND_ANGLE = -243;

		public static final double INTAKE_ZERO_ANGLE = -27;
		public static final double INTAKE_GROUND_ANGLE = -55.38;
		public static final double INTAKE_MIDDLE_ANGLE = -140;
		public static final double INTAKE_TOP_ANGLE = -70;
		public static final double INTAKE_SUBSTATION_ANGLE = -130;

		public static final double PICKUP_CONE_OUTPUT = 0.8;
		public static final double PICKUP_CUBE_OUTPUT = -0.6;
		public static final double PLACE_CONE_OUTPUT = -0.9;
		public static final double PLACE_CUBE_OUTPUT = 0.8;
	}

	public static class VisionConstants {
		public static int APRIL_TAG_PIPLINE = 0;
		public static int REFLECTIVE_TAPE_PIPELINE = 0;
	}

	public static class EnumConstants {
		public static enum GamePiece {
			Cube(PlacerConstants.PLACE_CUBE_OUTPUT),
			Cone(PlacerConstants.PLACE_CONE_OUTPUT),
			None(0.0);

			public double outputToPlace;

			private GamePiece(double output) {
				outputToPlace = output;
			}
		}

		public static enum PlacerState {
			Start(
				ZERO_EXTENSION,
				ARM_START_ANGLE,
				INTAKE_ZERO_ANGLE
			),
			Zero(
				ZERO_EXTENSION,
				ARM_ZERO_ANGLE,
				INTAKE_ZERO_ANGLE
			),
			Travel(
				ZERO_EXTENSION,
				ARM_TRAVEL_ANGLE,
				INTAKE_ZERO_ANGLE
			),
			Ground(
				GROUND_EXTENSION,
				ARM_GROUND_ANGLE,
				INTAKE_GROUND_ANGLE
			),
			Middle(
				MIDDLE_EXTENSION,
				ARM_MIDDLE_ANGLE,
				INTAKE_MIDDLE_ANGLE
			),
			Top(
				TOP_EXTENSION,
				ARM_TOP_ANGLE,
				INTAKE_TOP_ANGLE
			),
			Substation(
				SUBSTATION_EXTENSION,
				ARM_SUBSTATION_ANGLE,
				INTAKE_SUBSTATION_ANGLE
			);

			public double extension;
			public double armAngle;
			public double intakeAngle;

			private PlacerState(double extension, double armAngle, double intakeAngle) {
				this.extension = extension;
				this.armAngle = armAngle;
				this.intakeAngle = intakeAngle;
			}
		}

		public static enum IntakeState {
			PickupCube(PICKUP_CUBE_OUTPUT, GamePiece.Cube),
			PickupCone(PICKUP_CONE_OUTPUT, GamePiece.Cone),
			Place(0, GamePiece.None),
			Off(0, null);

			public double output;
			public GamePiece newGamePiece;

			private IntakeState(double output, GamePiece newGamePiece) {
				this.output = output;
				this.newGamePiece = newGamePiece;
			}
		}

		public static enum DriveMode {
			AngleCentric,
			RobotCentric
		}

		public static enum DriveSens {
			Fast(4, 4, 3.5),
			Slow(0.8, 0.8, 0.5);

			public double forwardSens;
			public double sidewaysSens;
			public double rotationalSens;

			private DriveSens(double forward, double sideways, double rotational) {
				forwardSens = forward;
				sidewaysSens = sideways;
				rotationalSens = rotational;
			}
		}

		public static enum CameraMode {
			VisionProcessor,
			DriverCamera,
		}

		public static enum TalonModel {
			TalonFX("Talon FX", 2048 / Math.PI / 2),
			TalonSRX("Talon SRX", 4096 / Math.PI / 2);
			
			public String name;
			public double ticksPerRadian;
			private TalonModel(String name, double ticksPerRadian) {
				this.name = name;
				this.ticksPerRadian = ticksPerRadian;
			}
		}
	}
}
