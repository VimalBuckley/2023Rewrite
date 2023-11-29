package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.auto.PIDConstants;

/**
 * Talented Singer
 */
public class SingingMotorController extends TalonFX implements EncodedMotorController {

    public static final double ENCODER_TICKS_PER_RADIAN = 2048 / Math.PI / 2;

    public SingingMotorController(int deviceID) {
        super(deviceID);
        config_IntegralZone(0, 0);
        configMotionCruiseVelocity(10000);
        configMotionAcceleration(10000);
        configAllowableClosedloopError(0, 0);
        configClearPositionOnQuadIdx(true, 10);
    }

    @Override
    public void setOutput(double targetPercentOutput) {
        set(ControlMode.PercentOutput, targetPercentOutput);
    }

    @Override
    public double getOutput() {
        return getMotorOutputPercent();
    }

    @Override
    public void setAngularVelocity(double targetAngularVelocity) {
        set(ControlMode.Velocity, targetAngularVelocity * ENCODER_TICKS_PER_RADIAN / 10.0);
    }

    @Override
    public double getAngularVelocity() {
        return getSelectedSensorVelocity() / ENCODER_TICKS_PER_RADIAN * 10;
    }

    @Override
    public void setAngle(double targetAngle) {
        set(ControlMode.MotionMagic, targetAngle * ENCODER_TICKS_PER_RADIAN);
    }

    @Override
    public double getAngle() {
        return getSelectedSensorPosition() / ENCODER_TICKS_PER_RADIAN;
    }

    @Override
    public SingingMotorController setCurrentLimit(int currentLimit) {
        configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        true,
                        currentLimit,
                        currentLimit + 1,
                        0.1),
                50);
        return this;
    }

    @Override
    public SingingMotorController setPID(PIDConstants pid) {
        config_kP(0, pid.kP);
        config_kI(0, pid.kI);
        config_kD(0, pid.kD);
        return this;
    }

    @Override
    public SingingMotorController setMinAngle(double minPosition) {
        configReverseSoftLimitEnable(true);
        configReverseSoftLimitThreshold(minPosition * ENCODER_TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public SingingMotorController setMaxAngle(double maxPosition) {
        configForwardSoftLimitEnable(true);
        configForwardSoftLimitThreshold(maxPosition * ENCODER_TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public SingingMotorController setMinOutput(double minOutput) {
        configPeakOutputReverse(minOutput);
        return this;
    }

    @Override
    public SingingMotorController setMaxOutput(double maxOutput) {
        configPeakOutputForward(maxOutput);
        return this;
    }

    @Override
    public SingingMotorController setInversion(boolean shouldInvert) {
        setInverted(shouldInvert);
        return this;
    }

    @Override
    public SingingMotorController setBrakeOnIdle(boolean shouldBreak) {
        setNeutralMode(
                shouldBreak
                        ? NeutralMode.Brake
                        : NeutralMode.Coast);
        return this;
    }

    @Override
    public SingingMotorController setAngleTolerance(double tolerance) {
        configAllowableClosedloopError(0, tolerance * ENCODER_TICKS_PER_RADIAN);
        return this;
    }
}
