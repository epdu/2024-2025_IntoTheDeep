//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
///**
// * This is NOT an opmode.
// *
// * This class can be used to define all the specific hardware for a single robot.
// * In this case that robot is powerpuffs`s robot from Pushbot.
// * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
// *
// * This hardware class assumes the following device names have been configured on the robot:
// * Note:  All names are lower case and some have single spaces between words.
// *    DcMotor RFMotor;
// *    DcMotor LFMotor;
// *    DcMotor RBMotor;
// *    DcMotor LBMotor;
// * RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
// * LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
// * RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
// * LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
// * Motor channel:  Left  drive motor:        "left_drive"
// * Motor channel:  Right drive motor:        "right_drive"
// * Motor channel:  Manipulator drive motor:  "left_arm"
// * Servo channel:  Servo to open left claw:  "left_hand"
// * Servo channel:  Servo to open right claw: "right_hand"
// *
// *
// */
////Good version 02082024
//public class HardwareTeletubbies12132024
//{
//    /* local OpMode members. */
//    HardwareMap hwMap =  null;
//    /* Public OpMode members. */
//    public DcMotor RFMotor;
//    public DcMotor LFMotor;
//    public DcMotor RBMotor;
//    public DcMotor LBMotor;
//    public Servo IClaw;
//    public Servo OClaw;
//    public Servo Wrist;
//    public Servo Wristzyaw;
//    public Servo Wristxpitch;
//    /*
//           x right side of the robot
//           y forward
//           z celling
//           z--yaw
//           x--pitch
//           y-- roll
//    */
//    public Servo IArmL;
//    public Servo IArmR;
//    public ServoImplEx OArmL;
//    public ServoImplEx OArmR;
//    public DcMotor HSMotor; //horizontal Slides motor  extruder
//    public DcMotor VSMotorL; //vertical Slides motor left
//    public DcMotor VSMotorR; //vertical Slides motor right
//    public Servo TServo; // For testing
////   public ServoImplEx myServo;
//    public ServoImplEx myServo;
////           public DcMotor TMotor; // For testing
//
////           public Servo ArmL;
////           public Servo ArmR;
////           public Servo V4BL;
////           public Servo V4BR;
////           public DcMotor liftMotorL;
////           public DcMotor liftMotorR;
////           public Servo ArmL;
////           public Servo ClawR;
////           public Servo ClawL;
////           public Servo ArmR;
////           public Servo ArmL;
////           public Servo Drone;
//    IMU imu;
//    public static final double DriveTrains_POWER       =  0.5 ;
//    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;
//    private ElapsedTime period  = new ElapsedTime();
//    /* Constructor */
//    /* Initialize standard Hardware interfaces */
//    public void init(HardwareMap ahwMap) {
//        // Save reference to Hardware map
//        hwMap = ahwMap;
//
////Begin Definition and Initialization of Drivetrain Motors
//        LFMotor   = hwMap.get(DcMotor.class, "LFMotor");//02022024 control hub port 0
//        RFMotor  = hwMap.get(DcMotor.class, "RFMotor"); //02022024 control hub port 1
//        LBMotor   = hwMap.get(DcMotor.class, "LBMotor");//02022024 control hub port 2
//        RBMotor  = hwMap.get(DcMotor.class, "RBMotor");//02022024 control hub port 3
//
//        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Set all motors to run without encoders.
//        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // End Definition and Initialization of Drivetrain Motors
///*
// Expansion port 1 output is reversed
// Expansion port 1 output is reversed
// Expansion port 1 output is reversed
// */
//
////Begin Definition and Initialization of Horizontal Slides  Motor
//
//        HSMotor = hwMap.get(DcMotor.class, "HSMotor");// expansion  hub  port 3
//        int positionH = HSMotor.getCurrentPosition();
//        HSMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//
////End Definition and Initialization of Horizontal Slides  Motor
//
//
////Begin Definition and Initialization of Vertical Slides Motors
//
//        VSMotorL = hwMap.get(DcMotor.class, "VSMotorL");// expansion  hub port 0
//        int positionVL = VSMotorL.getCurrentPosition();
//        VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//        VSMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
//        VSMotorR = hwMap.get(DcMotor.class, "VSMotorR");// expansion  hub port 2
//        int positionVR = VSMotorR.getCurrentPosition();
//        VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//
////End Definition and Initialization of Vertical Slides Motors
//
//        // Set all motors to zero power
//        setAllPower(0);
//
////Begin Definition and Initialization of Testing Motors and Servos
//
////               TServo= hwMap.get(Servo.class, "TS");//only for servo program testing
////               TServo.setPosition(0.5);// for safe
//
//// Assuming "myServo" is your Axon Max+ servo object
//
////              myServo = hwMap.get(ServoImplEx.class, "myServo");
////              myServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
////              myServo.setPosition(0.5);// for safe
///*
// This sets the PWM range for the servo.
// 500: Represents the minimum pulse width in microseconds.
// 2500: Represents the maximum pulse width in microseconds
// */
//
////               TMotor = hwMap.get(DcMotor.class, "TM");//02022024 control hub port 0 //only for motor program testing
////               TMotor = hwMap.get(DcMotor.class, "TM");//02022024 control hub port 0 //only for motor program testing
////               TMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////               TMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
////End Definition and Initialization of Testing Motors and Servos
//
////Begin Definition and Initialization of intake Claw Servo
//
////               IClaw = hwMap.get(Servo.class, "IClaw");//control hub port ?
////               IClaw.setPosition(0.543);// 12122024
//
////End Definition and Initialization of intake Claw Servo
//
////Begin Definition and Initialization of Wristzyaw Servo
//
////               Wristzyaw = hwMap.get(Servo.class, "Wristzyaw");//control hub port x
////               Wristzyaw.setPosition(0.5);// 12122024
//
////End Definition and Initialization of Wristzyaw Servo
//
//
////Begin Definition and Initialization of Wristxpitch Servo
//
////               Wristxpitch = hwMap.get(Servo.class, "Wristxpitch");//control hub port x
////               Wristxpitch.setDirection(Servo.Direction.REVERSE);
////               Wristxpitch.setPosition(0.05);// wrist good
//
////End Definition and Initialization of Wristxpitch Servo
//
//
////Begin Definition and Initialization of intake ArmL and ArmR Servos
//
////               IArmL = hwMap.get(Servo.class, "IArmL");//control hub port
////               IArmR = hwMap.get(Servo.class, "IArmR");//control hub port
////               IArmR.setDirection(Servo.Direction.REVERSE);
////               IArmL.setPosition(0.8);//  12132024
////               IArmR.setPosition(0.8);//
//
////End Definition and Initialization of intake ArmL and ArmR Servos
//
//
//
////Begin Definition and Initialization of outtake Claw Servo
//
//               OClaw = hwMap.get(Servo.class, "OClaw");//control hub port  good
//               OClaw.setPosition(0.543);//  12122024
////End Definition and Initialization of outtake Claw Servo
//
////Begin Definition and Initialization of outtake ArmL and ArmR Servos
//
////               OArmL = hwMap.get(ServoImplEx.class, "OArmL");//control hub port
////               OArmR = hwMap.get(ServoImplEx.class, "OArmR");;//control hub port
////               OArmL.setPwmRange(new PwmControl.PwmRange(500, 2500));
////               OArmR.setPwmRange(new PwmControl.PwmRange(500, 2500));
////               OArmR.setDirection(Servo.Direction.REVERSE);
////               OArmL.setPosition(0.01);//  good
////               OArmR.setPosition(0.01);//
//
////End Definition and Initialization of outtake ArmL and ArmR Servos
//
//
////Old hardwire
////Begin Definition and Initialization of V4B Servos
//
////               V4BR = hwMap.get(Servo.class, "V4BR");//control hub port
////               V4BL = hwMap.get(Servo.class, "V4BL");//control hub port
////               V4BL.setDirection(Servo.Direction.REVERSE);
////               V4BL.setPosition(0.32);//  good
////               V4BR.setPosition(0.32);//
//
////End Definition and Initialization of V4B Servos
//
////               Drone = hwMap.get(Servo.class, "Drone");//expan  hub port 5
////               Drone.setPosition(0);
////               ClawR = hwMap.get(Servo.class, "ClawR");//control hub port 2
////               ClawL = hwMap.get(Servo.class, "ClawL");//control hub port 3
////               ClawR.setPosition(0.71);
//////        ClawL.setPosition(0.4);
////               ClawL.setPosition(0.08);
//////        ClawL.setDirection(Servo.Direction.REVERSE);
////               Wrist = hwMap.get(Servo.class, "wrist");//control hub port 5
////               Wrist.setPosition(0.8);
//////        Wrist.setPosition(0.34);
////               ArmL = hwMap.get(Servo.class, "ArmL");
////               ArmR = hwMap.get(Servo.class, "ArmR");
////               ArmL.setDirection(Servo.Direction.REVERSE);0000
//
//
////inorder to reduce the ESD problems, we updated to be REV 9 axis imu with i2c port 1, imuinternal for the
//// REV control hub build in imu
//
//
//        imu = hwMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//
//        imu.initialize(parameters);
////        parameters.angleUnit           = IMU.AngleUnit.DEGREES;
////        parameters.accelUnit           = IMU.AccelUnit.METERS_PERSEC_PERSEC;
////        parameters.calibrationDataFile = "IMUCalibration.json"; // see the calibration sample opmode
////        parameters.loggingEnabled      = true;
////        parameters.loggingTag          = "IMU";
////        parameters.imuOrientationOnRobot = new JustLoggingAccelerationIntegrator();
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//
//    }
//    //Set power to all motors
//
//    public void setMotorPower(double lF, double rF, double lB, double rB){
//        LFMotor.setPower(lF*DriveTrains_POWER);
//        LBMotor.setPower(lB*DriveTrains_POWER);
//        RBMotor.setPower(rB*DriveTrains_POWER);
//        RFMotor.setPower(rF*DriveTrains_POWER);
//    }
//    public void setAllPower(double p){
//        setMotorPower(p,p,p,p);
//    }
//}
//
