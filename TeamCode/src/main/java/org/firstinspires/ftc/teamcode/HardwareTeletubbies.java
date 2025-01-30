
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//impor
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import static org.firstinspires.ftc.teamcode.Constants_CS.*;
/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is powerpuffs`s robot from Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *    DcMotor RFMotor;
 *    DcMotor LFMotor;
 *    DcMotor RBMotor;
 *    DcMotor LBMotor;
 * RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
 * LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
 * RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
 * LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 *
 *
 */
//Good version 02082024
public class HardwareTeletubbies
{
    /* local OpMode members. */
    HardwareMap hwMap =  null;
    /* Public OpMode members. */
    public DcMotor RFMotor;
    public DcMotor LFMotor;
    public DcMotor RBMotor;
    public DcMotor LBMotor;
    public Servo IClaw;
    public Servo OClaw;
    public Servo Wrist;
    public Servo Wristzyaw;
    public Servo Wristxpitch;
    /*
           x right side of the robot
           y forward
           z celling
           z--yaw
           x--pitch
           y-- roll
    */
    public Servo IArmL;
    public Servo IArmR;
    public ServoImplEx OArmL;
    public ServoImplEx OArmR;
    public DcMotor HSMotor; //horizontal Slides motor  extruder
    public DcMotor VSMotorL; //vertical Slides motor left
    public DcMotor VSMotorR; //vertical Slides motor right
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    public double GlobalX = 0;
    public double GlobalY = 0;
    public double GlobalH = 0;
    public Servo TServo; // For testing
    //   public ServoImplEx myServo;
    public ServoImplEx myServo;
//           public DcMotor TMotor; // For testing

    IMU imu;
    public static final double DriveTrains_POWER       =  0.5 ;
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    private ElapsedTime period  = new ElapsedTime();
    /* Constructor */
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

//Begin Definition and Initialization of Drivetrain Motors
        LFMotor   = hwMap.get(DcMotor.class, "LFMotor");//02022024 control hub port 0
        RFMotor  = hwMap.get(DcMotor.class, "RFMotor"); //02022024 control hub port 1
        LBMotor   = hwMap.get(DcMotor.class, "LBMotor");//02022024 control hub port 2
        RBMotor  = hwMap.get(DcMotor.class, "RBMotor");//02022024 control hub port 3

        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // End Definition and Initialization of Drivetrain Motors
/*
 Expansion port 1 output is reversed
 Expansion port 1 output is reversed
 Expansion port 1 output is reversed
 */

//Begin Definition and Initialization of Horizontal Slides  Motor

        HSMotor = hwMap.get(DcMotor.class, "HSMotor");// expansion  hub  port 3
        int positionH = HSMotor.getCurrentPosition();
        HSMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        HSMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//End Definition and Initialization of Horizontal Slides  Motor


//Begin Definition and Initialization of Vertical Slides Motors

        VSMotorL = hwMap.get(DcMotor.class, "VSMotorL");// expansion  hub port 0
        int positionVL = VSMotorL.getCurrentPosition();
        VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        VSMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        VSMotorR = hwMap.get(DcMotor.class, "VSMotorR");// expansion  hub port 2
        int positionVR = VSMotorR.getCurrentPosition();
        VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

//End Definition and Initialization of Vertical Slides Motors

        // Set all motors to zero power
        setAllPower(0);
//////////////////////////////////////////////////////////////////////////////////////////////////
//Begin Definition and Initialization of intake Claw Servo

        IClaw = hwMap.get(Servo.class, "IClaw");//control hub port 5
//               IClaw.setPosition(0.543);// 12122024
        IClaw.setPosition(IClawCloseInitialization);

//End Definition and Initialization of intake Claw Servo

//Begin Definition and Initialization of Wristzyaw Servo

        Wristzyaw = hwMap.get(Servo.class, "Wristzyaw");//control hub port 1
        Wristzyaw.setPosition(0.5);// 12122024

//End Definition and Initialization of Wristzyaw Servo

//Begin Definition and Initialization of Wristxpitch Servo

        Wristxpitch = hwMap.get(Servo.class, "Wristxpitch");//control hub port 4
//        Wristxpitch.setDirection(Servo.Direction.REVERSE); //moved servo from one side to the other side
        Wristxpitch.setPosition(0.05);//

//End Definition and Initialization of Wristxpitch Servo

//Begin Definition and Initialization of intake ArmL and ArmR Servos

        IArmL = hwMap.get(Servo.class, "IArmL");//control hub port 2
        IArmR = hwMap.get(Servo.class, "IArmR");//control hub port 3
        IArmR.setDirection(Servo.Direction.REVERSE);
        IArmL.setPosition(0.6);//  12132024
        IArmR.setPosition(0.6);//

//End Definition and Initialization of intake ArmL and ArmR Servos

//Begin Definition and Initialization of outtake Claw Servo
        OClaw = hwMap.get(Servo.class, "OClaw");//expansion hub port  0==>5
//               OClaw.setPosition(0.548);//  12122024
        OClaw.setPosition(OClawCloseInitialization);
//End Definition and Initialization of outtake Claw Servo

//Begin Definition and Initialization of outtake ArmL and ArmR Servos

        OArmL = hwMap.get(ServoImplEx.class, "OArmL");//expansion hub port 5==>2
        OArmR = hwMap.get(ServoImplEx.class, "OArmR");;//expansion hub port 2==>0
        OArmL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        OArmR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        OArmR.setDirection(Servo.Direction.REVERSE);
        OArmL.setPosition(0.97);//  good
        OArmR.setPosition(0.97);//

////End Definition and Initialization of outtake ArmL and ArmR Servos
//////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////GoBildaPinpointDriver//////////////////////////////
//        odo = hwMap.get(GoBildaPinpointDriver.class,"odo"); //expansion hub i2c port 1
//         /*
//        Set the odometry pod positions relative to the point that the odometry computer tracks around.
//        The X pod offset refers to how far sideways from the tracking point the
//        X (forward) odometry pod is. Left of the center is a positive number,
//        right of center is a negative number. the Y pod offset refers to how far forwards from
//        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
//        backwards is a negative number.
//         */
//        //  odo.setOffsets(-84.0, -224.0); //these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setOffsets(-210, -150);
//        /*
//        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
//        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
//        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
//        number of ticks per mm of your odometry pod.
//         */
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        //odo.setEncoderResolution(13.26291192);
//        /*
//        Set the direction that each of the two odometry pods count. The X (forward) pod should
//        increase when you move the robot forward. And the Y (strafe) pod should increase when
//        you move the robot to the left.
//         */
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        /*
//        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
//        The IMU will automatically calibrate when first powered on, but recalibrating before running
//        the robot is a good idea to ensure that the calibration is "good".
//        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
//        This is recommended before you run your autonomous, as a bad initial calibration can cause
//        an incorrect starting value for x, y, and heading.
//         */
//        //odo.recalibrateIMU();
//        odo.resetPosAndIMU();
/////////////////////////////////////////GoBildaPinpointDriver/////////////////////////////


//Begin Definition and Initialization of Testing Motors and Servos

//               TServo= hwMap.get(Servo.class, "TS");//only for servo program testing
//               TServo.setPosition(0.5);// for safe

// Assuming "myServo" is your Axon Max+ servo object

//              myServo = hwMap.get(ServoImplEx.class, "myServo");
//              myServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
//              myServo.setPosition(0.5);// for safe
/*
 This sets the PWM range for the servo.
 500: Represents the minimum pulse width in microseconds.
 2500: Represents the maximum pulse width in microseconds
 */

//               TMotor = hwMap.get(DcMotor.class, "TM");//02022024 control hub port 0 //only for motor program testing
//               TMotor = hwMap.get(DcMotor.class, "TM");//02022024 control hub port 0 //only for motor program testing
//               TMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//               TMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//End Definition and Initialization of Testing Motors and Servos

//Old hardwire

//inorder to reduce the ESD problems, we updated to be REV 9 axis imu with i2c port 1, imuinternal for the
// REV control hub build in imu


        imu = hwMap.get(IMU.class, "imu");  //control I2C port 1
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        imu.initialize(parameters);
// Gobilda IMU
        // expansion hub I2C 1


//        parameters.angleUnit           = IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.imuOrientationOnRobot = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

    }
    //Set power to all motors

    public void setMotorPower(double lF, double rF, double lB, double rB){
        LFMotor.setPower(lF*DriveTrains_POWER);
        LBMotor.setPower(lB*DriveTrains_POWER);
        RBMotor.setPower(rB*DriveTrains_POWER);
        RFMotor.setPower(rF*DriveTrains_POWER);
    }
    public void setAllPower(double p){
        setMotorPower(p,p,p,p);
    }
}

