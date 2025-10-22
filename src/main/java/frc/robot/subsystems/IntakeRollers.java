package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeRollers extends SubsystemBase {
    
    TalonFX rollerMotor = new TalonFX(IntakeConstants.kRollerMotorID);
    TalonFXConfiguration effectorConfig = new TalonFXConfiguration();

    double targetReference;
    ControlType currentControlType;

    public IntakeRollers() {
        effectorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        effectorConfig.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(0).withTorqueClosedLoopRampPeriod(0).withVoltageClosedLoopRampPeriod(0);
        effectorConfig.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(0).withTorqueOpenLoopRampPeriod(0).withVoltageOpenLoopRampPeriod(0);
        effectorConfig.Slot0.withKP(0.5);
                    
        rollerMotor.getConfigurator().apply(effectorConfig);


        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    @Override
    public void periodic() {}

    public void set(double speed) {
        rollerMotor.set(speed);
        currentControlType = ControlType.kDutyCycle;
    }

    public void setVelocity(double velocity) {
        rollerMotor.setControl(new VelocityDutyCycle(velocity));
        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    public void setPosition(double position){
        rollerMotor.setControl(new PositionDutyCycle(position));
        
        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    public void setVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
        currentControlType = ControlType.kVoltage;
    }

    public void setEncoderPosition(double position) {
        rollerMotor.setPosition(position);
    }

    public double getVelocity() {
        return rollerMotor.getVelocity().getValueAsDouble();
    }

    public double getPosition() {
        return rollerMotor.getPosition().getValueAsDouble();
    }

    public ControlType getControlType() {
        return currentControlType;
    }

    public boolean atTarget(double threshold) {
        if (currentControlType == ControlType.kVelocity) {
            return Math.abs(getVelocity() - targetReference) < threshold;
        } else if (currentControlType == ControlType.kPosition) {
            return Math.abs(getPosition() - targetReference) < threshold;
        } else {
            return false;
        }
    }
}