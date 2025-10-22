package frc.robot.subsystems.Spindexer;

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
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    private final SparkFlex spindexerMotor = new SparkFlex(SpindexerConstants.kSpindexerMotorID, MotorType.kBrushless);
    private final RelativeEncoder encoder = spindexerMotor.getEncoder();
    private final SparkClosedLoopController pid;

    @SuppressWarnings("unused")
    private double targetVelocityRPM = 0.0;

    public Spindexer() {
        spindexerMotor.configure(
            Configs.Spindexer.SpindexerConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        encoder.setPosition(0);
        pid = spindexerMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spindexer Pos (rot)", getSpindexerPosition());
        SmartDashboard.putNumber("Spindexer Vel (RPM)", getSpindexerVelocity());
        SmartDashboard.putNumber("Spindexer Current (A)", spindexerMotor.getOutputCurrent());
    }

    /** Set open-loop motor output (-1 to 1) */
    public void setSpeed(double percentOutput) {
        spindexerMotor.set(percentOutput);
    }

    /** Set velocity in RPM (closed-loop mode) */
    public void setVelocity(double rpm) {
        targetVelocityRPM = rpm;
        pid.setReference(rpm, ControlType.kVelocity);
    }

    /** Stop the motor */
    public void stop() {
        spindexerMotor.stopMotor();
    }

    /** @return Spindexer position in rotations */
    public double getSpindexerPosition() {
        return encoder.getPosition();
    }

    /** @return Spindexer velocity in RPM */
    public double getSpindexerVelocity() {
        return encoder.getVelocity(); // SparkFlex returns RPM by default
    }
}
