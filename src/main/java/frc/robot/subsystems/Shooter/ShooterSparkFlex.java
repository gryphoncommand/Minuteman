package frc.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSparkFlex extends SubsystemBase implements ShooterIO {
    private final SparkFlex shooterMotor = new SparkFlex(ShooterConstants.kShooterMotorID, MotorType.kBrushless);
    private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private final SparkClosedLoopController pid;

    private double targetReference = 0;
    private ControlType currentControlType = ControlType.kDutyCycle;

    public ShooterSparkFlex() {
        shooterMotor.configure(Configs.Shooter.ShooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        pid = shooterMotor.getClosedLoopController();

        SmartDashboard.putNumber("Shooter Target RPM", 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity (RPM)", getVelocity());
        SmartDashboard.putString("Shooter Control Mode", currentControlType.toString());
    }

    @Override
    public void set(double speed) {
        shooterMotor.set(speed);
        currentControlType = ControlType.kDutyCycle;
    }

    @Override
    public void setVelocity(double rpm) {
        pid.setReference(rpm, ControlType.kVelocity);
        targetReference = rpm;
        currentControlType = ControlType.kVelocity;
    }
    
    @Override
    public void setVoltage(double volts) {
        shooterMotor.setVoltage(volts);
        currentControlType = ControlType.kVoltage;
    }

    @Override
    public void setEncoderPosition(double position) {
        shooterEncoder.setPosition(position);
    }

    @Override
    public double getVelocity() {
        // Encoder velocity is in RPM by default for SparkFlex
        return shooterEncoder.getVelocity();
    }

    @Override
    public boolean atTarget(double threshold) {
        if (currentControlType == ControlType.kVelocity) {
            return Math.abs(getVelocity() - targetReference) < threshold;
        } else if (currentControlType == ControlType.kPosition) {
            return Math.abs(shooterEncoder.getPosition() - targetReference) < threshold;
        } else {
            return false;
        }
    }

    @Override
    public SubsystemBase returnSubsystem() {
        return this;
    }
}
