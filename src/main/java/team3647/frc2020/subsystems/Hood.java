package team3647.frc2020.subsystems;
 
import edu.wpi.first.wpilibj.Servo;
 
public class Hood implements PeriodicSubsystem {
    private final Servo hood;
    private final PeriodicIO pIO;
 
    public Hood(int PWMPort) {
        this.hood = new Servo(PWMPort);
        pIO = new PeriodicIO();
    }

    public class PeriodicIO {
        public double demand;
    }

 
    public void setPosition(double demand) {
        // code has min demand and max demand, why? Why do that if I know that I'm not going to set the demand over the servo's capablities?
        pIO.demand = demand;
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
        hood.set(pIO.demand);
    }
 
    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
