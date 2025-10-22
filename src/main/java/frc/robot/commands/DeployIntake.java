package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeExtender.IntakeExtenderIO;

public class DeployIntake extends Command {
    private IntakeExtenderIO intakeExtender;    

    public DeployIntake(IntakeExtenderIO intakeExtender) {
        this.intakeExtender = intakeExtender;
        addRequirements(intakeExtender.returnSubsystem());
    }

    @Override
    public void initialize() {
        intakeExtender.setPosition(IntakeConstants.kIntakeAngle);
    }

    @Override
    public boolean isFinished() {
        return intakeExtender.atTarget(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        intakeExtender.set(0);
    }
        
}
