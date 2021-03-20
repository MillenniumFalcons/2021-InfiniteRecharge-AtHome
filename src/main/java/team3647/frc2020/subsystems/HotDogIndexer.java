package team3647.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import team3647.lib.IndexerSignal;
import team3647.lib.drivers.TalonSRXFactory;
import team3647.lib.drivers.VictorSPXFactory;

public class HotDogIndexer implements PeriodicSubsystem {
    private final TalonSRX rightVerticalRoller;
    private final VictorSPX leftVerticalRoller;
    private final VictorSPX horizontalRoller;
    private final VictorSPX tunnelBeltRoller;
    private final DigitalInput tunnelBallDetection;
    private final PeriodicIO pIO;

    private boolean bannerSensorValue;

    class PeriodicIO {
        public double leftOut = 0;
        public double rightOut = 0;
        public double horizOut = 0;
        public double tunnelOut = 0;
    }

    public HotDogIndexer(TalonSRXFactory.Configuration RVConfig, VictorSPXFactory.Configuration LVConfig, VictorSPXFactory.Configuration hRollerConfig, VictorSPXFactory.Configuration TBConfig, int ballDetectPin) {
        rightVerticalRoller = TalonSRXFactory.createTalon(RVConfig);
        leftVerticalRoller = VictorSPXFactory.createVictor(LVConfig);
        horizontalRoller = VictorSPXFactory.createVictor(hRollerConfig);
        tunnelBeltRoller = VictorSPXFactory.createVictor(TBConfig);
        tunnelBallDetection = new DigitalInput(ballDetectPin);
        pIO = new PeriodicIO();
        
    }

    public void setSignal(IndexerSignal demand) {
        pIO.leftOut = demand.getLeftVerticalOutput();
        pIO.rightOut = demand.getRightVerticalOutput();
        pIO.horizOut = demand.getHorizontalRollersOutput();
        pIO.tunnelOut = demand.getTunnelOutput();

        
    }

    public void setManual(double demand) {
        pIO.leftOut = Math.signum(demand);
        pIO.rightOut = Math.signum(demand) * 0.8;
        pIO.horizOut = demand;
        pIO.tunnelOut = demand;
    }

    public boolean getTunnelBallDetection() {
        return bannerSensorValue;
    }

    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.readPeriodicInputs();
        this.bannerSensorValue = !tunnelBallDetection.get();
    }

    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.writePeriodicOutputs();

        leftVerticalRoller.set(ControlMode.PercentOutput, pIO.leftOut);
        rightVerticalRoller.set(ControlMode.PercentOutput, pIO.rightOut);
        horizontalRoller.set(ControlMode.PercentOutput, pIO.horizOut);
        tunnelBeltRoller.set(ControlMode.PercentOutput, pIO.tunnelOut);
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.end();
        setSignal(IndexerSignal.STOP);
    }



    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }

}