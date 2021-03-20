package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Intake;
import team3647.frc2020.subsystems.Intake.IntakeState;

public class StowIntake extends CommandBase {
    private final Intake m_intake;

    public StowIntake(Intake intake) {
        this.m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_intake.setIntakeMode(IntakeState.STOWED);
    }

    @Override
    public void execute() {
        m_intake.moveInner();
        m_intake.moveOuter();
        m_intake.spinIntakeMotor();

    }

    @Override
    public void end(boolean interrupted) {

    }
    
    @Override
    public boolean isFinished() {
        //make it run forever so that withTimeOut will stop it exactly at parameter passed in
        return false;
    }

}