package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.BallStopper;
import team3647.frc2020.subsystems.HotDogIndexer;
import team3647.lib.IndexerSignal;

public class LoadBalls extends CommandBase {
    private final HotDogIndexer indexer;
    private final BallStopper stopper;

    public LoadBalls(HotDogIndexer indexer, BallStopper stopper) {
        this.indexer = indexer;
        this.stopper = stopper;
        addRequirements(indexer, stopper);
    }

    @Override
    public void initialize() {
        stopper.extend();
    }

    @Override
    public void execute() {
        indexer.setSignal(IndexerSignal.GO);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.end();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}