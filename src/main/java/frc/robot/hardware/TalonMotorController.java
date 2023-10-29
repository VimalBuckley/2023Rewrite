package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.pathplanner.lib.auto.PIDConstants;

import frc.robot.Constants.EnumConstants.TalonModel;
import frc.robot.hardware.interfaces.SwerveMotorController;
import frc.robot.subsystems.swerve.SwerveModule.SwerveMotorConfig;

public class TalonMotorController extends BaseTalon implements SwerveMotorController{
    private TalonModel model;

    public TalonMotorController(int deviceID, TalonModel model) {
        super(deviceID, model.name);
        this.model = model;
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
    public void configureForSwerve(SwerveMotorConfig config){
        configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, config.currentLimit, config.currentLimit + 1, 0.1), 50);
        setInverted(config.invert);
        config_kP(0, config.pid.kP);
        config_kI(0, config.pid.kI);
        config_kD(0, config.pid.kD);
        config_IntegralZone(0, 0);
        configMotionCruiseVelocity(10000);
        configMotionAcceleration(10000);
        configAllowableClosedloopError(0, 0);
        configClearPositionOnQuadIdx(true, 10);
    }

    @Override
    public void setPID(PIDConstants pid) {
        config_kP(0, pid.kP);
        config_kI(0, pid.kI);
        config_kD(0, pid.kD);
    }

    @Override
    public void setMinAngle(double minPosition) {
        configReverseSoftLimitEnable(true);
        configReverseSoftLimitThreshold(minPosition * model.ticksPerRadian);
    }

    @Override
    public void setMaxAngle(double maxPosition) {
        configForwardSoftLimitEnable(true);
        configForwardSoftLimitThreshold(maxPosition * model.ticksPerRadian);
    }

    @Override
    public void setMinOutput(double minOutput) {
       configPeakOutputReverse(minOutput);
    }

    @Override
    public void setMaxOutput(double maxOutput) {
        configPeakOutputForward(maxOutput);
    }
    
    @Override
    public void setInverted(boolean shouldInvert) {
        super.setInverted(shouldInvert);
    }

    @Override
    public void setBrakeOnIdle(boolean shouldBreak) {
        if (shouldBreak) {
            setNeutralMode(NeutralMode.Brake);
        } else {
            setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public void setAngleTolerance(double tolerance) {
        configAllowableClosedloopError(0, tolerance * model.ticksPerRadian);
    }
}
