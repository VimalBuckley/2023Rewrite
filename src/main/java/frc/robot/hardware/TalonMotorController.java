package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.pathplanner.lib.auto.PIDConstants;

import frc.robot.Constants.EnumConstants.TalonModel;

public class TalonMotorController extends BaseTalon implements EncodedMotorController{
    private TalonModel model;

    public TalonMotorController(int deviceID, TalonModel model) {
        super(deviceID, model.name);
        this.model = model;
        if (model == TalonModel.TalonFX) {
            configureForSwerve();
        }
    }

    private void configureForSwerve() {
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
        set(ControlMode.Velocity, targetAngularVelocity * model.ticksPerRadian / 10.0);
    }

    @Override
    public double getAngularVelocity() {
        return getSelectedSensorVelocity() / model.ticksPerRadian * 10;
    }

    @Override
    public void setAngle(double targetAngle) {
        if (model == TalonModel.TalonSRX) {
            set(ControlMode.Position, targetAngle * model.ticksPerRadian);
        } else {
            set(ControlMode.MotionMagic, targetAngle * model.ticksPerRadian);
        }
    }

    @Override
    public double getAngle() {
        return getSelectedSensorPosition() / model.ticksPerRadian;
    }

    @Override
    public EncodedMotorController configureCurrentLimit(int currentLimit) {
        configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                    true, 
                    currentLimit, 
                    currentLimit + 1, 
                    0.1
                ), 
            50
        );
        return this;
    }

    @Override
    public EncodedMotorController setPID(PIDConstants pid) {
        config_kP(0, pid.kP);
        config_kI(0, pid.kI);
        config_kD(0, pid.kD);
        return this;
    }

    @Override
    public EncodedMotorController setMinAngle(double minPosition) {
        configReverseSoftLimitEnable(true);
        configReverseSoftLimitThreshold(minPosition * model.ticksPerRadian);
        return this;
    }

    @Override
    public EncodedMotorController setMaxAngle(double maxPosition) {
        configForwardSoftLimitEnable(true);
        configForwardSoftLimitThreshold(maxPosition * model.ticksPerRadian);
        return this;
    }

    @Override
    public EncodedMotorController setMinOutput(double minOutput) {
       configPeakOutputReverse(minOutput);
       return this;
    }

    @Override
    public EncodedMotorController setMaxOutput(double maxOutput) {
        configPeakOutputForward(maxOutput);
        return this;
    }
    
    @Override
    public EncodedMotorController setInversion(boolean shouldInvert) {
        super.setInverted(shouldInvert);
        return this;
    }

    @Override
    public EncodedMotorController setBrakeOnIdle(boolean shouldBreak) {
        if (shouldBreak) {
            setNeutralMode(NeutralMode.Brake);
        } else {
            setNeutralMode(NeutralMode.Coast);
        }
        return this;
    }

    @Override
    public EncodedMotorController setAngleTolerance(double tolerance) {
        configAllowableClosedloopError(0, tolerance * model.ticksPerRadian);
        return this;
    }
}
