package frc.robot.hardware;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.util.Units;
import frc.robot.hardware.interfaces.SwerveMotorController;
import frc.robot.subsystems.swerve.SwerveModule.SwerveMotorConfig;

public class SparkMaxMotorController extends CANSparkMax implements SwerveMotorController {
	public SparkMaxMotorController(int deviceID, MotorType type) {
		super(deviceID, type);
	}

    @Override
	public double getAngle() {
		return Units.radiansToRotations(getEncoder().getPosition());
	}

    @Override
	public void setAngle(double position) {
		getPIDController()
			.setReference(Units.radiansToRotations(position), ControlType.kPosition);
	}

    @Override
	public void setOutput(double output) {
		set(output);
	}

    @Override
	public double getOutput() {
		return get();
	}

    @Override
	public double getAngularVelocity() {
		return Units.rotationsPerMinuteToRadiansPerSecond(getEncoder().getVelocity());
	}
	
    @Override
	public void setAngularVelocity(double velocity) {
		getPIDController()
			.setReference(
				Units.radiansPerSecondToRotationsPerMinute(velocity),
				ControlType.kVelocity
			);
	}

    @Override
	public void configureForSwerve(SwerveMotorConfig config) {
		setInverted(config.invert);
        getPIDController().setP(config.pid.kP);
        getPIDController().setI(config.pid.kI);
        getPIDController().setD(config.pid.kD);
        getPIDController().setFF(0);
        setSmartCurrentLimit(config.currentLimit);
	}

	@Override
	public void setPID(PIDConstants pid) {
		SparkMaxPIDController controller = getPIDController();
		controller.setP(pid.kP);
		controller.setI(pid.kI);
		controller.setD(pid.kD);
	}

	@Override
	public void setMinAngle(double minPosition) {
		setSoftLimit(SoftLimitDirection.kReverse, (float) Units.radiansToRotations(minPosition));
	}

	@Override
	public void setMaxAngle(double maxPosition) {
		setSoftLimit(SoftLimitDirection.kForward, (float) Units.radiansToRotations(maxPosition));
	}

	@Override
	public void setMinOutput(double minOutput) {
		SparkMaxPIDController controller = getPIDController();
		controller.setOutputRange(minOutput, controller.getOutputMax());
	}

	@Override
	public void setMaxOutput(double maxOutput) {
		SparkMaxPIDController controller = getPIDController();
		controller.setOutputRange(controller.getOutputMin(), maxOutput);
	}

	@Override
	public void setInverted(boolean shouldInvert) {
		super.setInverted(shouldInvert);
	}

	@Override
	public void setBrakeOnIdle(boolean shouldBreak) {
		if (shouldBreak) {
			setIdleMode(IdleMode.kBrake);
		} else {
			setIdleMode(IdleMode.kCoast);
		}
	}

	@Override
	public void setAngleTolerance(double tolerance) {
		// Not possible on a spark max
	}
}
