package team3647.frc2020.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.KickerWheel;

public class AccelerateKickerWheel extends CommandBase {
    private final KickerWheel kicker;
    private double demandRPM;
    


    public AccelerateKickerWheel(KickerWheel kicker, double demand) {
        this.kicker = kicker;
        this.demandRPM = demand;
        addRequirements(kicker);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();

    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        kicker.setRPM(demandRPM);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        kicker.end();
    }
}