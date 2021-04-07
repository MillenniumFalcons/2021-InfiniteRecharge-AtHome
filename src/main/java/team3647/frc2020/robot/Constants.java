package team3647.frc2020.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpiutil.math.MathUtil;
import team3647.lib.util.Units;
//import team3647.frc2020.subsystems.VisionController;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import team3647.lib.drivers.SparkMaxFactory;
import team3647.lib.drivers.TalonSRXFactory;
import team3647.lib.drivers.VictorSPXFactory;
import team3647.lib.util.InterpolatingDouble;
import team3647.lib.util.InterpolatingTreeMap;

public class Constants {

    public static class cDrivetrain {
        // CAN id
        public static final int leftMasterPin = 1;
        public static final int leftSlavePin = 2;
        public static final int rightMasterPin = 3;
        public static final int rightSlavePin = 4;
        public static final int stallCurrent = 35;
        public static final int maxCurrent = 60;

        public static final int climbLimitPin = 0;

        // pcm pin
        public static final int shifterPin = 2;

        // meters (6inches)
        public static final double kWheelDiameter = .1524;

        // volts
        public static final double kS = 0.214;
        public static final double kV = 2.62;
        public static final double kA = 0.42;
        // public static final double kA = 0.15;

        public static final double kP = 0.0001;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double onBoardkP = 14.8;

        public static final double kTrackwidthMeters = 0.6455447508046545;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 2;  
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;

        public static final double maxVoltage = 11.0;
        public static final double gearboxReduction = 9.0 / 42.0 * 24.0 / 50.0;

        public static final double neoRotationsToMeters =
                gearboxReduction * kWheelDiameter * Math.PI;

        public static final SparkMaxFactory.Configuration leftMasterConfig =
                new SparkMaxFactory.Configuration(leftMasterPin, false)
                        .currentLimiting(true, maxCurrent, stallCurrent).idleMode(IdleMode.kBrake)
                        .voltageCompensation(true, 12.0);

        public static final SparkMaxFactory.Configuration rightMasterConfig =
                new SparkMaxFactory.Configuration(rightMasterPin, true)
                        .currentLimiting(true, maxCurrent, stallCurrent).idleMode(IdleMode.kBrake)
                        .voltageCompensation(true, 12.0);

        public static final SparkMaxFactory.Configuration leftSlaveConfig =
                SparkMaxFactory.Configuration.mirrorWithCANID(leftMasterConfig, leftSlavePin);

        public static final SparkMaxFactory.Configuration rightSlaveConfig =
                SparkMaxFactory.Configuration.mirrorWithCANID(rightMasterConfig, rightSlavePin);

        public static final ClosedLoopConfig leftMasterPIDConfig = new ClosedLoopConfig()
                .encoderVelocityToRPM(gearboxReduction).encoderTicksToUnits(neoRotationsToMeters)
                .maxVelocity(kMaxSpeedMetersPerSecond).configPID(kP, kI, kD);
        public static final ClosedLoopConfig rightMasterPIDConfig = new ClosedLoopConfig()
                .encoderVelocityToRPM(gearboxReduction).encoderTicksToUnits(neoRotationsToMeters)
                .maxVelocity(kMaxSpeedMetersPerSecond).configPID(kP, kI, kD);

    }

    public static class cIntake {
        public static final int outerPistonsPin = 4;
        public static final int innerPistonsPin = 3;
        public static final int intakeMotorPin = 8;

        public static final boolean inverted = true;
        public static TalonSRXFactory.Configuration intakeMotorConfig =
                new TalonSRXFactory.Configuration(intakeMotorPin, inverted)
                        .configOpenLoopRampRate(.3);
    }


    public static class cVisionController {
        // 8ft in meters
        public static final double kGoalHeight = Units.inches_to_meters(98.25);

        // 30inches in meters
        public static final double kCameraHeight = Units.feet_to_meters(3);

        // can be either 75 or 56 degrees depending on the lens setting used
        public static final double kFOV = 75;

        /** in micro meters */
        public static final double kSensorHeight = 2952;
        /** in micro meters */
        public static final double kSensorWidth = 3984;

        /** micrometers per pixel */
        public static final double kPixelSize = 6;

        /** in degrees */
        public static final double camAngle = 23.3;

        public static final double kImageCaptureLatency = 11.0 / 1000.0; // miliseconds

        public static final String camIP = "10.36.47.15";

        /*public static final VisionController.CamConstants camConstants =
                new VisionController.CamConstants(kGoalHeight, kCameraHeight, camAngle,
                        kImageCaptureLatency);*/
    }

    public static class cTurret {
        public static final int masterPin = 17;
        public static final boolean inverted = true;
        public static final double nominalVoltage = 10.0;
        public static final int limitSwitchPin = 6;
        public static final boolean sensorInverted = false;

        public static final double kMinRotationDeg = -20;
        public static final double kMaxRotationDeg = 200;

        public static final double kCruiseVelocityRPM = 150;
        public static final double kAccelerationRPMs = 300;
        // amps
        public static final int peakCurrent = 40;
        // milis
        public static final int peakCurrentDuration = 1000;
        // amps
        public static final int continuousCurrent = 20;

        public static final double reductionFromEncoder = 16.0 / 130.0;

        public static final double encoderTicksToUnits =
                (reductionFromEncoder / magEncoderTicksPerRev) * 360.0;
        public static final double encoderVelocityToRPM = encoderTicksToUnits / 360.0 * 10 * 60;

        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 30;

        public static final double kS = 1.2;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double forwardDeg = 0;
        public static final double backwardDeg = 180;
        public static final double leftDeg = 90;
        public static final double rightDeg = -90;

        public static final TalonSRXFactory.Configuration masterConfig =
                new TalonSRXFactory.Configuration(masterPin, inverted)
                        .neutralMode(NeutralMode.Brake).voltageCompensation(true, nominalVoltage)
                        .currentLimiting(true, peakCurrent, peakCurrentDuration, continuousCurrent);

        public static final ClosedLoopConfig pidConfig = new ClosedLoopConfig()
                .encoderTicksToUnits(encoderTicksToUnits).encoderVelocityToRPM(encoderVelocityToRPM)
                .encoderAccelerationToUnits(encoderVelocityToRPM).positionThreshold(.5)
                .maxVelocity(50).maxAcceleration(kAccelerationRPMs).sensorInverted(sensorInverted)
                .configPID(kP, kI, kD).configFeedForward(kS, kV, kA);
    }

    public static class cIndexer {
        public static final int leftVerticalRollersPin = 22;
        public static final int rightVerticalRollersPin = 21;
        public static final int tunnelPin = 23;
        public static final int horizontalRollersPin = 24;
        public static final int bannerSensorPin = 1;

        public static final int PP_VerticalPDPSlot = 10;
        public static final int tunnelPDPSlot = 8;

        public static boolean rightVerticalRollersInverted = false;
        public static boolean leftVerticalRollersInverted = true;
        public static boolean tunnelInverted = false;
        public static boolean horizontalRollersInverted = false;

        public static VictorSPXFactory.Configuration leftRollersConfig =
                new VictorSPXFactory.Configuration(leftVerticalRollersPin)
                        .setInverted(leftVerticalRollersInverted).configOpenLoopRampRate(.3)
                        .setPDPSlot(10);
        public static TalonSRXFactory.Configuration rightRollersConfig =
                new TalonSRXFactory.Configuration(rightVerticalRollersPin,
                        rightVerticalRollersInverted);
        public static VictorSPXFactory.Configuration tunnelConfig =
                new VictorSPXFactory.Configuration(tunnelPin).setInverted(tunnelInverted)
                        .setPDPSlot(8);
        public static VictorSPXFactory.Configuration horizontalRollersConfig =
                new VictorSPXFactory.Configuration(horizontalRollersPin)
                        .setInverted(horizontalRollersInverted).configOpenLoopRampRate(.3);

    }

    public static class cKickerWheel {
        public static final int masterPin = 15;
        public static final boolean inverted = false;
        public static final int peakCurrent = 60;
        public static final int peakCurrentDuration = 1000;
        public static final int continuousCurrent = 20;
        public static final double nominalVoltage = 12.0;
        public static final NeutralMode neutralMode = NeutralMode.Brake;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double encoderTicksToUnits = 1 / magEncoderTicksPerRev;
        public static final double encoderVelocityToRPM = encoderTicksToUnits * 10 * 60;

        public static final double visionDistanceToRPM = 1;

        public static TalonSRXFactory.Configuration masterConfig =
                new TalonSRXFactory.Configuration(masterPin, inverted)
                        .currentLimiting(true, 20, 1, 10).voltageCompensation(true, 12);
        // .neutralMode(NeutralMode.Brake).voltageCompensation(true, nominalVoltage)
        // .currentLimiting(true, peakCurrent, peakCurrentDuration, continuousCurrent);
        public static ClosedLoopConfig pidConfig = new ClosedLoopConfig().configPID(kP, kI, kD);
        // .configFeedForward(kS, kV, kA).encoderTicksToUnits(encoderTicksToUnits)
        // .encoderVelocityToRPM(encoderVelocityToRPM)
        // .encoderAccelerationToUnits(encoderVelocityToRPM);

        public static double getFlywheelOutputFromFlywheelRPM(double rpm) {
            // return MathUtil.clamp(rpm / 2 * 0.000111111, .5, 1);
            return .9;
        }
    }

    public static class cShooting {
        //contains, hood position, flywheel velociy, kicker velocity in that order
        //hood position in
        //kicker velocity in
        //flyhwheel velocity in 
        public static final double[][] hoodFlywheelKicker = {{0,0,0},
                                                             {0,0,0},
                                                             {0,0,0},
                                                             {0,0,0}};
    }

    public static class cFlywheel {
        public static final int masterPin = 11;
        public static final int slavePin = 12;
        public static final boolean masterInverted = false;
        public static final boolean slaveInverted = false;

        public static final int maxFreeSpeedCurrent = 60;
        public static final int maxStallCurrent = 40;

        public static final double nominalVoltage = 12.0;
        public static final NeutralMode neutralMode = NeutralMode.Brake;

        public static final double kP = 0.0005;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.623;
        public static final double kV = 0.0631;
        // public static final double kV = 0.03;
        public static final double kA = 0.0241;

        public static final double compkS = 0.393;
        public static final double compkV = 0.0635;
        public static final double compkA = 0.0215;

        public static final double kGearboxReduction = 2;
        public static final double encoderTicksToUnits = 42;
        public static final double encoderVelocityToRPM = 2;

        public static final double visionDistanceToRPM = 1;

        public static SparkMaxFactory.Configuration masterConfig =
                new SparkMaxFactory.Configuration(masterPin, masterInverted)
                        .idleMode(IdleMode.kBrake).voltageCompensation(true, nominalVoltage)
                        .currentLimiting(true, maxFreeSpeedCurrent, maxStallCurrent);
        public static SparkMaxFactory.Configuration slaveConfig =
                SparkMaxFactory.Configuration.mirrorWithCANID(masterConfig, slavePin);

        public static ClosedLoopConfig pidConfig = new ClosedLoopConfig().configPID(kP, kI, kD)
                .configFeedForward(compkS, compkV, compkA).encoderTicksToUnits(encoderTicksToUnits)
                .velocityThrehsold(125).encoderVelocityToRPM(encoderVelocityToRPM)
                .encoderAccelerationToUnits(encoderVelocityToRPM);

        public static double calculateRPM(double distance) {
            return distance;
        }

        public static double[][] kFlywheelDistanceRPM =
                {{Units.feet_to_meters(4), 2500}, {Units.feet_to_meters(6), 3000},
                        {Units.feet_to_meters(8), 3500}, {Units.feet_to_meters(10), 4000},
                        {Units.feet_to_meters(12), 4500}, {Units.feet_to_meters(14), 4800},
                        {Units.feet_to_meters(16), 4900}, {Units.feet_to_meters(18), 5300},
                        {Units.feet_to_meters(20), 5500}, {Units.meters_to_feet(22), 5700}};

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap =
                new InterpolatingTreeMap<>();

        static {
            for (double[] pair : kFlywheelDistanceRPM) {
                kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]),
                        new InterpolatingDouble(pair[1]));
            }
        }

        public static double getFlywheelRPM(double range) {
            InterpolatingDouble d =
                    kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range));

            return d == null ? 6000 : MathUtil.clamp(d.value, 2000, 8000);
        }
    }

    public static class cHood {
        public static int pwmPort = 2;
        public static double minPosition = .3;
        public static double maxPosition = .9;
        public static double trenchShotPosition = .645;
        public static double cpShotPosition = .66;
        public static double rightUpToTowerShotPosition = .25;

        public static double[][] kHoodDistancePosition =
                {{Units.feet_to_meters(4), .31}, {Units.feet_to_meters(6), .43},
                        {Units.feet_to_meters(8), .52}, {Units.feet_to_meters(10), .55},
                        {Units.feet_to_meters(12), .575}, {Units.feet_to_meters(14), .60},
                        {Units.feet_to_meters(16), .605}, {Units.feet_to_meters(18), .62},
                        {Units.feet_to_meters(20), .63}, {Units.meters_to_feet(22), .635}};

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap =
                new InterpolatingTreeMap<>();
        static {
            for (double[] pair : kHoodDistancePosition) {
                kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]),
                        new InterpolatingDouble(pair[1]));
            }
        }

        public static double getHoodPosition(double range) {
            InterpolatingDouble d = kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range));
            return d == null ? .61 : MathUtil.clamp(d.value, .2, .7);
        }
    }

    public static class cLED {
        public static final int canifierPin = 0;
    }

    public static class cBallStopper {
        public static final int solenoidPin = 6;
    }

    public static class cField {
        //Galatic Search Red A
        public static final Pose2d GalaticSearch_A_Red_startingPoint = new Pose2d(new Translation2d(Units.inches_to_meters(30), Units.inches_to_meters(150)), new Rotation2d(Units.degrees_to_radians(180)));
        public static final Translation2d GalaticSearch_A_Red_firstBall = new Translation2d(Units.inches_to_meters(90), Units.inches_to_meters(90));
        public static final Translation2d GalaticSearch_A_Red_secondBall = new Translation2d(Units.inches_to_meters(153), Units.inches_to_meters(60));
        public static final Translation2d GalaticSearch_A_Red_thirdBall = new Translation2d(Units.inches_to_meters(180), Units.inches_to_meters(140));
        public static final Pose2d GalaticSearch_A_Red_endingPoint = new Pose2d(new Translation2d(Units.inches_to_meters(330), Units.inches_to_meters(150)), new Rotation2d(Units.degrees_to_radians(180)));
        //Galatic Search Red B
        public static final Pose2d GalaticSearch_B_Red_startingPoint = new Pose2d(new Translation2d(Units.inches_to_meters(30), Units.inches_to_meters(150)), new Rotation2d(Units.degrees_to_radians(180)));
        public static final Translation2d GalaticSearch_B_Red_firstBall = new Translation2d(Units.inches_to_meters(90), Units.inches_to_meters(120));
        public static final Translation2d GalaticSearch_B_Red_secondBall = new Translation2d(Units.inches_to_meters(150), Units.inches_to_meters(60));
        public static final Translation2d GalaticSearch_B_Red_thirdBall = new Translation2d(Units.inches_to_meters(210), Units.inches_to_meters(110));
        public static final Pose2d GalaticSearch_B_Red_endingPoint = new Pose2d(new Translation2d(Units.inches_to_meters(330), Units.inches_to_meters(120)), new Rotation2d(Units.degrees_to_radians(180)));

        //AUTONAV Barrel Race
        public static final Pose2d AutoNav_Barrel_Race_Start = new Pose2d(new Translation2d(Units.inches_to_meters(30), Units.inches_to_meters(90)), new Rotation2d(0));
        public static final Translation2d AUTONAV_Barrel_Race_1 = new Translation2d(Units.inches_to_meters(163.78), Units.inches_to_meters(92.13));
        public static final Translation2d AUTONAV_Barrel_Race_2 = new Translation2d(Units.inches_to_meters(199.61), Units.inches_to_meters(60.63));
        public static final Translation2d AUTONAV_Barrel_Race_3 = new Translation2d(Units.inches_to_meters(187.01), Units.inches_to_meters(29.13));
        public static final Translation2d AUTONAV_Barrel_Race_4 = new Translation2d(Units.inches_to_meters(158.66), Units.inches_to_meters(20.87));
        public static final Translation2d AUTONAV_Barrel_Race_5 = new Translation2d(Units.inches_to_meters(129.92), Units.inches_to_meters(40.55));
        public static final Translation2d AUTONAV_Barrel_Race_6 = new Translation2d(Units.inches_to_meters(127.56), Units.inches_to_meters(71.26));
        public static final Translation2d AUTONAV_Barrel_Race_7 = new Translation2d(Units.inches_to_meters(145.67), Units.inches_to_meters(89.76));
        public static final Translation2d AUTONAV_Barrel_Race_8 = new Translation2d(Units.inches_to_meters(176.38), Units.inches_to_meters(92.13));
        public static final Translation2d AUTONAV_Barrel_Race_9 = new Translation2d(Units.inches_to_meters(210), Units.inches_to_meters(90));
        public static final Translation2d AUTONAV_Barrel_Race_10 = new Translation2d(Units.inches_to_meters(240), Units.inches_to_meters(95));
        public static final Translation2d AUTONAV_Barrel_Race_11 = new Translation2d(Units.inches_to_meters(270), Units.inches_to_meters(125));
        public static final Translation2d AUTONAV_Barrel_Race_12 = new Translation2d(Units.inches_to_meters(240), Units.inches_to_meters(160));
        public static final Translation2d AUTONAV_Barrel_Race_13 = new Translation2d(Units.inches_to_meters(210), Units.inches_to_meters(135));
        public static final Translation2d AUTONAV_Barrel_Race_14 = new Translation2d(Units.inches_to_meters(200), Units.inches_to_meters(100));
        public static final Translation2d AUTONAV_Barrel_Race_15 = new Translation2d(Units.inches_to_meters(205), Units.inches_to_meters(75));
        public static final Translation2d AUTONAV_Barrel_Race_16 = new Translation2d(Units.inches_to_meters(220), Units.inches_to_meters(60));
        public static final Translation2d AUTONAV_Barrel_Race_17 = new Translation2d(Units.inches_to_meters(250), Units.inches_to_meters(40));
        public static final Translation2d AUTONAV_Barrel_Race_18 = new Translation2d(Units.inches_to_meters(280), Units.inches_to_meters(30));
        public static final Translation2d AUTONAV_Barrel_Race_19 = new Translation2d(Units.inches_to_meters(320), Units.inches_to_meters(35));
        public static final Translation2d AUTONAV_Barrel_Race_20 = new Translation2d(Units.inches_to_meters(335), Units.inches_to_meters(60));
        public static final Translation2d AUTONAV_Barrel_Race_21 = new Translation2d(Units.inches_to_meters(300), Units.inches_to_meters(90));
        public static final Translation2d AUTONAV_Barrel_Race_22 = new Translation2d(Units.inches_to_meters(180), Units.inches_to_meters(90));
        public static final Translation2d AUTONAV_Barrel_Race_23 = new Translation2d(Units.inches_to_meters(90), Units.inches_to_meters(90));


        public static final Pose2d AutoNav_Barrel_Race_END = new Pose2d(new Translation2d(Units.inches_to_meters(30), Units.inches_to_meters(90)), new Rotation2d(180));

        //AUTONAV SLALOM
        public static final Pose2d AutoNav_Slalom_START = new Pose2d(new Translation2d(Units.inches_to_meters(30), Units.inches_to_meters(30)), new Rotation2d(0));
        public static final Translation2d AutoNav_Slalom_A= new Translation2d(2.194, 1.354);
        public static final Translation2d AutoNav_Slalom_B = new Translation2d(3.161, 2.363);
        public static final Translation2d AutoNav_Slalom_C = new Translation2d(4.593, 2.788);
        public static final Translation2d AutoNav_Slalom_D = new Translation2d(6.42, 1.98);
        public static final Translation2d AutoNav_Slalom_E = new Translation2d(7.192, 0.692);
        public static final Translation2d AutoNav_Slalom_F = new Translation2d(8.402, 0.8);
        public static final Translation2d AutoNav_Slalom_G = new Translation2d(8.702, 1.506);
        public static final Translation2d AutoNav_Slalom_H = new Translation2d(7.934, 2.409);
        public static final Translation2d AutoNav_Slalom_I = new Translation2d(7.163, 2.338);
        public static final Translation2d AutoNav_Slalom_J = new Translation2d(6.92, 1.922);
        public static final Translation2d AutoNav_Slalom_K = new Translation2d(6.666, 1.171);
        public static final Translation2d AutoNav_Slalom_L = new Translation2d(5.967, 0.593);
        public static final Translation2d AutoNav_Slalom_M = new Translation2d(4.695, 0.434);
        public static final Translation2d AutoNav_Slalom_N = new Translation2d(2.988, 0.889);
        public static final Translation2d AutoNav_Slalom_O = new Translation2d(2.083, 1.884);
        public static final Translation2d AutoNav_Slalom_P = new Translation2d(1.556, 2.333);
        public static final Pose2d AutoNav_Slalom_END = new Pose2d(new Translation2d(Units.inches_to_meters(30), Units.inches_to_meters(90)), new Rotation2d(180));

        //AUTONAV BOUNCE
        public static final Pose2d BounceForwardA_START = new Pose2d(new Translation2d(Units.inches_to_meters(30), Units.inches_to_meters(90)), new Rotation2d(0));
        public static final Translation2d BounceForwardA_firstPoint = new Translation2d(Units.inches_to_meters(80), Units.inches_to_meters(95));
        public static final Translation2d BounceForwardA_secondPoint = new Translation2d(Units.inches_to_meters(90), Units.inches_to_meters(120));
        public static final Pose2d BounceForwardA_END = new Pose2d(new Translation2d(Units.inches_to_meters(90), Units.inches_to_meters(150)), new Rotation2d(Units.degrees_to_radians(90)));
        //forwadA to backwardsA
        public static final Pose2d BounceBackwardsA_START = new Pose2d(new Translation2d(Units.inches_to_meters(90), Units.inches_to_meters(150)), new Rotation2d(Units.degrees_to_radians(90)));
        public static final Translation2d BounceBackwardsA_Setup = new Translation2d(Units.inches_to_meters(120), Units.inches_to_meters(60));
        public static final Translation2d BounceBackwardsA_firstPoint = new Translation2d(Units.inches_to_meters(150), Units.inches_to_meters(30));
        public static final Translation2d BounceBackwardsA_secondPoint = new Translation2d(Units.inches_to_meters(185), Units.inches_to_meters(60));
        //public static final Translation2d BounceBackwardsA_thirdPoint = new Translation2d(Units.inches_to_meters(), Units.inches_to_meters());
        //public static final Translation2d BounceBackwardsA_fourthPoint = new Translation2d(Units.inches_to_meters(180), Units.inches_to_meters(90));
        public static final Pose2d BounceBackwardsA_END = new Pose2d(new Translation2d(Units.inches_to_meters(180), Units.inches_to_meters(140)), new Rotation2d(Units.degrees_to_radians(-90)));
        //backwardsA to forwardB
        public static final Pose2d BounceForwardB_START = new Pose2d(new Translation2d(Units.inches_to_meters(180), Units.inches_to_meters(150)), new Rotation2d(Units.degrees_to_radians(-90)));
        public static final Translation2d BounceForwardsB_firstPoint = new Translation2d(Units.inches_to_meters(180), Units.inches_to_meters(90));
        public static final Translation2d BounceForwardsB_secondPoint = new Translation2d(Units.inches_to_meters(190), Units.inches_to_meters(50));
        public static final Translation2d BounceForwardsB_thirdPoint = new Translation2d(Units.inches_to_meters(220), Units.inches_to_meters(25));
        public static final Translation2d BounceForwardsB_fourthPoint = new Translation2d(Units.inches_to_meters(245), Units.inches_to_meters(30));
        public static final Translation2d BounceForwardsB_fifthPoint = new Translation2d(Units.inches_to_meters(267), Units.inches_to_meters(60));
        public static final Pose2d BounceForwardsB_END = new Pose2d(new Translation2d(Units.inches_to_meters(270), Units.inches_to_meters(150)), new Rotation2d(90));
        //forwardB to backwardA
        public static final Pose2d BounceBackwardsB_START = new Pose2d(new Translation2d(Units.inches_to_meters(270), Units.inches_to_meters(150)), new Rotation2d(Units.degrees_to_radians(90)));
        public static final Translation2d BounceBackwardsB_firstPoint = new Translation2d(Units.inches_to_meters(270), Units.inches_to_meters(110));
        public static final Translation2d BounceBackwardsB_secondPoint = new Translation2d(Units.inches_to_meters(300), Units.inches_to_meters(93));
        public static final Pose2d BounceBackwardsB_END = new Pose2d(new Translation2d(Units.inches_to_meters(330), Units.inches_to_meters(93)), new Rotation2d(Units.degrees_to_radians(180)));
    }

    public static class cClimber {
        public static int solenoidPin = 7;
    }

    /**
     * In order to make the class not be able to be an object
     */
    private Constants() {
    }

    public static final double magEncoderTicksPerRev = 4096.0;
}