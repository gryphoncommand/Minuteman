package frc.robot.subsystems.Spindexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.SpindexerConstants;

public class Passthrough extends SubsystemBase {
    private final SparkFlex passthroughMotor = new SparkFlex(SpindexerConstants.kPassthroughMotorID, MotorType.kBrushless);

    public Passthrough() {
        passthroughMotor.configure(Configs.SubsystemBaseConfig.subsystemConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        passthroughMotor.getClosedLoopController().setReference(0, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Passthrough Pos (rot)", passthroughMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Passthrough Current (A)", passthroughMotor.getOutputCurrent());
    }

    /** Sets the passthrough motor speed (-1.0 to 1.0). */
    public void setSpeed(double percentOutput) {
        passthroughMotor.set(percentOutput);
    }

    /** Stops the passthrough motor. */
    public void stop() {
        passthroughMotor.set(0);
    }
}
