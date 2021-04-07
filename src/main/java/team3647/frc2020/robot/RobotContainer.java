package team3647.frc2020.robot;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3647.frc2020.autonomous.Trajectories;
import team3647.frc2020.commands.AccelerateKickerFlywheel;
import team3647.frc2020.commands.ArcadeDrive;
import team3647.frc2020.commands.GoStraightDistancePID;
import team3647.frc2020.commands.GroundIntake;
import team3647.frc2020.commands.IndexerManualMode;
import team3647.frc2020.commands.LoadBalls;
import team3647.frc2020.commands.LoadingStationIntake;
import team3647.frc2020.commands.StowIntake;
import team3647.frc2020.commands.TacoBell;
import team3647.frc2020.commands.TunnelIn;
import team3647.frc2020.commands.TunnelOut;
import team3647.frc2020.inputs.Joysticks;
import team3647.frc2020.subsystems.BallStopper;
import team3647.frc2020.subsystems.Drivetrain;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.Hood;
import team3647.frc2020.subsystems.HotDogIndexer;
import team3647.frc2020.subsystems.Intake;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.GroupPrinter;
import team3647.lib.wpi.Compressor;



public class RobotContainer {
  private final Joysticks controller = new Joysticks(0);
  private final Joysticks coController = new Joysticks(1);

  public final Drivetrain dt = new Drivetrain(Constants.cDrivetrain.leftMasterConfig, Constants.cDrivetrain.rightMasterConfig, Constants.cDrivetrain.leftSlaveConfig, Constants.cDrivetrain.rightSlaveConfig, Constants.cDrivetrain.leftMasterPIDConfig, 
  Constants.cDrivetrain.rightMasterPIDConfig, Constants.cDrivetrain.kWheelDiameter, 16, Constants.cDrivetrain.kS,
  Constants.cDrivetrain.kV, Constants.cDrivetrain.kA);

  private final CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
  private final GroupPrinter printer = new GroupPrinter();
  private final Compressor airCompressor = new Compressor(0);
  private final Hood hood = new Hood(Constants.cHood.pwmPort);
  private final Intake intake = new Intake(Constants.cIntake.innerPistonsPin, Constants.cIntake.outerPistonsPin, Constants.cIntake.intakeMotorConfig);
  private final BallStopper stopper = new BallStopper(Constants.cBallStopper.solenoidPin);
  private final HotDogIndexer m_Indexer = new HotDogIndexer(Constants.cIndexer.rightRollersConfig, Constants.cIndexer.leftRollersConfig, Constants.cIndexer.horizontalRollersConfig, Constants.cIndexer.tunnelConfig, Constants.cIndexer.bannerSensorPin);
  private final KickerWheel kicker = new KickerWheel(Constants.cKickerWheel.masterConfig, Constants.cKickerWheel.pidConfig);
  private final Flywheel flywheel = new Flywheel(Constants.cFlywheel.masterConfig, Constants.cFlywheel.slaveConfig, Constants.cFlywheel.pidConfig);
  //public final Command autonomousCommand = new GoStraightDistance(dt, 10);
  //public final Command autonomousCommand = new GoStraightDistancePID(dt, 10, Constants.cDrivetrain.kP, Constants.cDrivetrain.kI, Constants.cDrivetrain.kD); 
  
  //auto Trajectory following

  //GalaticSearch
  private final RamseteCommand GalaticSearch_PathA_Red_Movement = new RamseteCommand(Trajectories.GalaticSearch_A_RedTraject, dt::getRobotPose, new RamseteController(), 
  Constants.cDrivetrain.kDriveKinematics, dt::setClosedLoopVelocity, dt);
  private final RamseteCommand GalaticSearch_PathB_Red_Movement = new RamseteCommand(Trajectories.GalaticSearch_B_RedTraject, dt::getRobotPose, new RamseteController(),
  Constants.cDrivetrain.kDriveKinematics, dt::setClosedLoopVelocity, dt);

  //AutoNav
  //BarrelRace
  private final RamseteCommand AutoNav_BarrelRace_Movement = new RamseteCommand(Trajectories.AutoNav_BarrelRace, dt::getRobotPose, new RamseteController(),
  Constants.cDrivetrain.kDriveKinematics, dt::setClosedLoopVelocity, dt);
  //Slalom
  private final RamseteCommand AutoNav_Slalom_Movement = new RamseteCommand(Trajectories.AutoNav_Slalom, dt::getRobotPose, new RamseteController(),
  Constants.cDrivetrain.kDriveKinematics, dt::setClosedLoopVelocity, dt);
  //Bounce
  private final RamseteCommand AutoNav_Bounce_forwardA_Movement = new RamseteCommand(Trajectories.AutoNav_Bounce_forwardA, dt::getRobotPose, new RamseteController(),
  Constants.cDrivetrain.kDriveKinematics, dt::setClosedLoopVelocity, dt);
  private final RamseteCommand AutoNav_Bounce_backwardA_Movement = new RamseteCommand(Trajectories.AutoNav_Bounce_backwardA, dt::getRobotPose, new RamseteController(),
  Constants.cDrivetrain.kDriveKinematics, dt::setClosedLoopVelocity, dt);
  private final RamseteCommand AutoNav_Bounce_forwardsB_Movement = new RamseteCommand(Trajectories.AutoNav_Bounce_forwardsB, dt::getRobotPose, new RamseteController(),
  Constants.cDrivetrain.kDriveKinematics, dt::setClosedLoopVelocity, dt);
  private final RamseteCommand AutoNav_Bounce_backwardB_Movement = new RamseteCommand(Trajectories.AutoNav_Bounce_backwardB, dt::getRobotPose, new RamseteController(),
  Constants.cDrivetrain.kDriveKinematics, dt::setClosedLoopVelocity, dt);

  //auto movement + other movements/commands

  //once deadline ends, the other command ends
  //Galatic search sequence
  private final SequentialCommandGroup GalaticSearch_PathA_Red = new SequentialCommandGroup(new ParallelDeadlineGroup(GalaticSearch_PathA_Red_Movement, new ParallelCommandGroup(new LoadBalls(m_Indexer, stopper), new GroundIntake(intake))), new InstantCommand(dt::end).withTimeout(0.1));
  private final SequentialCommandGroup GalaticSearch_PathB_Red = new SequentialCommandGroup(new ParallelDeadlineGroup(GalaticSearch_PathB_Red_Movement, new ParallelCommandGroup(new LoadBalls(m_Indexer, stopper), new GroundIntake(intake))), new InstantCommand(dt::end).withTimeout(0.1));

  //bounce sequence
  private final SequentialCommandGroup AutoNav_Bounce = new SequentialCommandGroup(AutoNav_Bounce_forwardA_Movement, AutoNav_Bounce_backwardA_Movement, AutoNav_Bounce_forwardsB_Movement, AutoNav_Bounce_backwardB_Movement, new InstantCommand(dt::end).withTimeout(0.1));
  // private final SequentialCommandGroup testPathMove = new SequentialCommandGroup(AutoNav_Bounce_forwardA_Movement, AutoNav_Bounce_backwardA_Movement, new InstantCommand(dt::end).withTimeout(0.1));



  public RobotContainer() {
    
    intake.extendInner();
    airCompressor.start();
    stopper.extend();
    configButtonBindings();
    //m_commandScheduler.registerSubsystem(dt, m_Indexer, intake, hood, kicker, flywheel);
    m_commandScheduler.registerSubsystem(dt, printer);
    m_commandScheduler.setDefaultCommand(dt,
        new ArcadeDrive(dt, controller::getLeftStickY, controller::getRightStickX));
    m_commandScheduler.setDefaultCommand(m_Indexer, new IndexerManualMode(m_Indexer, controller::getRightStickY));
    printer.addDouble("drivetain x", dt::getx);
    printer.addDouble("drivetain y", dt::gety);
    printer.addDouble("drivetain heading", dt::getHeading);
  }

  public void init(){
    dt.init();
    //GalaticSearchRedA dt init
    dt.setOdometry(Trajectories.AutoNav_Bounce_forwardA.getInitialPose(), new Rotation2d());
    //GalaticSearchRedB
    //dt.setOdometry(Trajectories.GalaticSearch_B_RedTraject.getInitialPose(), new Rotation2d());
    //AutoNav Slalom
    //dt.setOdometry(Trajectories.AutoNav_Slalom.getInitialPose(), new Rotation2d());
    //AutoNav Bounce
    //dt.setOdometry(Trajectories.AutoNav_Bounce_forwardA.getInitialPose(), new Rotation2d());

  //  m_Indexer.init();
  //   intake.init();
  //   hood.init();
  //   kicker.init();
  //   flywheel.init();
  }

  public Command getAutonomousCommand() {
    return AutoNav_Bounce;
  }

  public boolean getDrivetrainSlowed() {
    return dt.getSlow();
  }

  private void configButtonBindings(){
    controller.buttonX.whenActive(new InstantCommand(() -> dt.setSlow(!dt.getSlow()), dt));

    //what should the presets be?
    coController.dPadDown.whenActive(new InstantCommand(() -> hood.setPosition(0)));
    coController.dPadUp.whenActive(new InstantCommand(() -> hood.setPosition(0.3)));
    coController.dPadLeft.whenActive(new InstantCommand(() -> hood.setPosition(0.6)));
    coController.dPadRight.whenActive(new InstantCommand(() -> hood.setPosition(1)));

    //shooting
    coController.buttonA.whenActive(new SequentialCommandGroup(new InstantCommand(() -> hood.setPosition(Constants.cShooting.hoodFlywheelKicker[0][0])),
                                    new AccelerateKickerFlywheel(flywheel, kicker, Constants.cShooting.hoodFlywheelKicker[0][1], Constants.cShooting.hoodFlywheelKicker[0][2])));

    //automatic organize feeder
    coController.buttonA.whenHeld(new SequentialCommandGroup(new TunnelOut(m_Indexer), new TunnelIn(m_Indexer).withTimeout(0.5)));
    //indexer manual mode
    coController.buttonB.whenHeld(new IndexerManualMode(m_Indexer, controller::getRightStickY));

    //intake positions
    //GroundIntake
    coController.leftTrigger.whenActive(new SequentialCommandGroup(new RunCommand(intake::retractInner, intake).withTimeout(0.5), new ParallelCommandGroup(new GroundIntake(intake), new LoadBalls(m_Indexer, stopper))));
    //loading station
    coController.rightBumper.whenActive(new ParallelCommandGroup(new LoadingStationIntake(intake), new LoadBalls(m_Indexer, stopper)));
    // stowed
    coController.leftTrigger.whenReleased(new StowIntake(intake).withTimeout(0.5));
    // tacobell
    coController.rightTrigger.whenActive(new ParallelCommandGroup(new TacoBell(intake), new TunnelOut(m_Indexer)));
  }

  public void stopDrivetrain() {
    dt.end();
    dt.setBrake();
  }

  public void setCoast() {
    dt.setCoast();
  }

  
}