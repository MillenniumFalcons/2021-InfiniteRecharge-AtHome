package team3647.frc2020.subsystems;

import edu.wpi.first.wpilibj.Solenoid;

public class BallStopper implements PeriodicSubsystem {
    private final Solenoid stopper;
    
    public BallStopper(int pin) {
        this.stopper = new Solenoid(pin);
    }

    public void extend() {
        stopper.set(true);
    }

    public void retract() {
        stopper.set(false);
    }

    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.readPeriodicInputs();
    }

    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.writePeriodicOutputs();
    }
    
    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }

    
}