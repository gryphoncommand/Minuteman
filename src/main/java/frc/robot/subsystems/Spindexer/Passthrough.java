package frc.robot.subsystems.Spindexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;


// TODO: make this a spark flex because carson hates me :(
public class Passthrough extends SubsystemBase {
    private final TalonFX passthroughMotor = new TalonFX(SpindexerConstants.kPassthroughMotorID);
    private final DutyCycleOut openLoopRequest = new DutyCycleOut(0);

    public Passthrough() {
        passthroughMotor.setNeutralMode(NeutralModeValue.Brake);
        passthroughMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Passthrough Pos (rot)", passthroughMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Passthrough Current (A)", passthroughMotor.getSupplyCurrent().getValueAsDouble());
    }

    /** Sets the passthrough motor speed (-1.0 to 1.0). */
    public void setSpeed(double percentOutput) {
        passthroughMotor.setControl(openLoopRequest.withOutput(percentOutput));
    }

    /** Stops the passthrough motor. */
    public void stop() {
        passthroughMotor.setControl(openLoopRequest.withOutput(0));
    }
}
