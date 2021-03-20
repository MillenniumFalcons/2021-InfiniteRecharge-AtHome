package team3647.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import team3647.lib.drivers.TalonSRXFactory;
import team3647.lib.wpi.Solenoid;

public class Intake implements PeriodicSubsystem {
    private final Solenoid inner;
    private final Solenoid outer;
    private final TalonSRX intakeMotor;
    private IntakeState state;

    public Intake(int innerPairPin, int outerPairPin, TalonSRXFactory.Configuration intakeMotorConfig) {
        this.inner = new Solenoid(innerPairPin);
        this.outer = new Solenoid(outerPairPin);
        this.intakeMotor = TalonSRXFactory.createTalon(intakeMotorConfig);
    }

    public enum IntakeState {
        GROUND(true, true, -0.7), LOADING_STATION(true, false, 0), TACOBELL(false, true, -0.5), STOWED(false, false, 0);

        public final boolean innerCond;
        public final boolean outerCond;
        public final double motorDemand;

        IntakeState(boolean innerExtend, boolean outerExtend, double motorDemand) {
            this.innerCond = innerExtend;
            this.outerCond = outerExtend;
            this.motorDemand = motorDemand;
        }

    }

    public void setIntakeMode(IntakeState state) {
        this.state = state;
    }

    public void moveInner() {
        inner.set(state.innerCond);
    }

    public void moveOuter() {
        outer.set(state.outerCond);
    }

    public void retractInner() {
        inner.set(false);
    }
    public void extendInner() {
        inner.set(true);
    }

    public void retractOuter() {
        inner.set(true);
    }

    public void spinIntakeMotor() {
        setOpenLoop(state.motorDemand);
    }

    public void setOpenLoop(double demand) {
        this.intakeMotor.set(ControlMode.PercentOutput, demand);
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
    public void end() {
        // TODO Auto-generated method stub
        PeriodicSubsystem.super.end();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Intake";
    }
    
}