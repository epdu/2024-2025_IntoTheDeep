////Alan taught me a nice way to debug code, each time he will make a new file with mame-AutonomousCopyALan from Autonomous
////Blue left setup  use hardware init Red Alliance works, add blue still need to adjust hsv range, contours either too big or too small
////set the distance from front of robot to the block of game element
///*  Using the specs from the motor, you would need to find the encoder counts per revolution (of the output shaft).
//     Then, you know that corresponds to 360 degrees of wheel rotation, which means the distance travelled is the circumference
//      of the wheel (2 * pi * r_wheel). To figure out how many encoder ticks correspond to the distance you wanna go,
//      just multiply the distance by the counts / distance you calculated above. Hope that helps!
//// 11.87374348
////537 per revolution 11.87374348 inch
//*/
//package org.firstinspires.ftc.teamcode;
//
//import static java.lang.Math.abs;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.openftc.easyopencv.OpenCvCamera;
//
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
////HSV
////OpenCvVisionProcessor
////HardwarePowerpuffs
////works well except controlHubCam.stopStreaming(); check this 0225morning
//@Autonomous(name = "Auto VisionPortal V5 recover HSV")
//public class AutonomoTele extends LinearOpMode {
//    HardwareTeletubbies robot = new HardwareTeletubbies();// Use a Powerpuffs's hardware
////    public String allianceColor="red";// "null" for init set to be "red" or "blue" for each match
//        public String allianceColor="blue";
//    public String parkingSide="right";// "null" for init  set to be "left" or "right" for each match
//    //    public String parkingSide="left";
//    public double sleepingTime=0.0;// set to be any number if need to avoid collision with alliance
//    public boolean autoParkingDone=false;
//    //    private static final double[] redBlobColorThresholds = {20.0, 120.0, 180.0, 240.0, 90.0, 120.0};
////    private static final double[] blueBlobColorThresholds = {20.0, 250.0, 40.0, 250.0, 160.0, 240.0};
//    public float speedMultiplier=0.5f;
//    public float speedLimiter =0.5f;
//    //    public boolean targetFound = false;
//    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
//    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
//    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
//    double  turn            = 0;        // Desired turning power/speed (-1 to +1)
//    //    public boolean targetFound = false;
////    public double drive = 0;
////    public double turn = 0;
////    public double strafe = 0;
//    DistanceSensor LeftSensor;
//    DistanceSensor RightSensor;
//    IMU imu;
//    double ticksPerRotation;
//    double initialFR;
//    double initialFL;
//    double initialBR;
//    double initialBL;
//    double positionFR;
//    double positionFL;
//    double positionBR;
//    double positionBL;
//    public String updates;
//    public int i = 0;
//    double targetheading;
//    double heading;
//    double previousHeading;
//    double processedHeading;
//    double  distanceInInch;
//    double  distanceInInchDouble;
//    double  distanceRFMotor;
//    double  distanceRBMotor;
//    double  distanceLFMotor;
//    double  distanceLBMotor;
//    private double wheelDiameterInInches = 3.77953;  // Adjust this based on your mecanum wheel diameter
//    public String teamPropLocations;  //= "Left"
//    public String PurplePixel;
//    //    boolean
//    public boolean found;
//    public boolean  dropPurplePixelDone=false;
//    public boolean  dropYellowPixelDone=false;
//    double redVal;
//    double blueVal;
//    double greenVal;
//    double liftInitial;
//    double liftIdealPos;
//    double liftIdealPower;
//    int result;
//    double cX = 0;
//    double cY = 0;
//    double width = 0;
//
//    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)move
//    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        /*
//         * Initialize the drive system variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.init(hardwareMap);
//
//*/
//        initVisionPortal() ;
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // TODO: Need to do red or blue according to alliance color.
//            while (found==false) {
//
//            }
//            moveForward(0.3, 5);
//            strafeRight(0.3, 40);
//            moveBackward(0.3, 10);
//
//            }
//        }
//
//
//    }
//
//
//      public void stopMotors(double p){
//        robot.setAllPower(p);
//    }
//    public void moveForward(double power, double distanceInInch) {
//        movement(power, -distanceInInch,-distanceInInch,-distanceInInch,-distanceInInch) ;
//    }
//    public void moveBackward(double power, double distanceInInch) {
//        movement(power, +distanceInInch,+distanceInInch,+distanceInInch,+distanceInInch) ;
//    }
//    public void turnRight(double power, double distanceInInch) {
//        movement(power, +distanceInInch,+distanceInInch,-distanceInInch,-distanceInInch);
//    }
//    public void turnLeft(double power, double distanceInInch) {
//        movement(power, -distanceInInch,-distanceInInch,+distanceInInch,+distanceInInch);
//    }
//    public void strafeRight(double power, double distanceInInch) {
//        movement(power, +distanceInInch,-distanceInInch,-distanceInInch,+distanceInInch);
//    }
//    public void strafeLeft(double power, double distanceInInch) {
//        movement(power, -distanceInInch,+distanceInInch,+distanceInInch,-distanceInInch);
//    }
//    public void movement(double power, double distanceRF,double distanceRB,double distanceLF,double distanceLB) {
////input distance in inches, robot will finish movement "moveForward moveBackward ,turnRight turnLeft  strafeRight and strafeLeft"
//        distanceRFMotor=(double)(distanceRF*537/(Math.PI * wheelDiameterInInches));
//        distanceRBMotor=(double)(distanceRB*537/(Math.PI * wheelDiameterInInches));
//        distanceLFMotor=(double)(distanceLF*537/(Math.PI * wheelDiameterInInches));
//        distanceLBMotor=(double)(distanceLB*537/(Math.PI * wheelDiameterInInches));
//        robot.RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.RFMotor.setTargetPosition((int) distanceRFMotor);
//        robot.RBMotor.setTargetPosition((int) distanceRBMotor);
//        robot.LFMotor.setTargetPosition((int) distanceLFMotor);
//        robot.LBMotor.setTargetPosition((int) distanceLBMotor);
//        robot.RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.RFMotor.setPower(+power);
//        robot.RBMotor.setPower(+power);
//        robot.LFMotor.setPower(+power);
//        robot.LBMotor.setPower(+power);
//        telemetry.addData("distanceRFMotor", distanceRFMotor);
//        telemetry.addData("distanceRBMotor", distanceRBMotor);
//        telemetry.addData("distanceLFMotor", distanceLFMotor);
//        telemetry.addData("distanceLBMotor", distanceLBMotor);
//        telemetry.update();
//        while (robot.RFMotor.isBusy() || robot.RBMotor.isBusy() || robot.LFMotor.isBusy() || robot.LBMotor.isBusy() ||false) {}
//        robot.RFMotor.setPower(0);
//        robot.RBMotor.setPower(0);
//        robot.LFMotor.setPower(0);
//        robot.LBMotor.setPower(0);
//    }
//
//    public void moveRobot(double x, double y, double yaw) {
//        telemetry.addData("x", x);
//        telemetry.addData("y", y);
//        telemetry.addData("yaw", yaw);
//        telemetry.update();
//        double leftFrontPower    =  x -y +yaw;
//        double rightFrontPower   =  x +y -yaw;
//        double leftBackPower     =  x +y +yaw;
//        double rightBackPower    =  x -y -yaw;
//
//        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        robot.LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        telemetry.addData("leftFrontPower", leftFrontPower);
//        telemetry.addData("rightFrontPower", rightFrontPower);
//        telemetry.addData("leftBackPower", leftBackPower);
//        telemetry.addData("rightBackPower", rightBackPower);
//        telemetry.update();
//        robot.LFMotor.setPower(leftFrontPower);
//        robot.RFMotor.setPower(rightFrontPower);
//        robot.LBMotor.setPower(leftBackPower);
//        robot.RBMotor.setPower(rightBackPower);
//
//    }
//}