package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Intake;
import team3647.frc2020.subsystems.Intake.IntakeState;

public class LoadingStationIntake extends CommandBase {
    private final Intake intake;
    
    public LoadingStationIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override 
    public void initialize() {
        intake.setIntakeMode(IntakeState.LOADING_STATION);
    }

    @Override
    public void execute() {
        intake.moveInner();
        intake.moveOuter();
        intake.spinIntakeMotor();

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // run forever so that withtimeout will stop it exactly at the parameter
        return false;
    }
}