package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Constants_CS.IClawCloseLose;
import static org.firstinspires.ftc.teamcode.Constants_CS.IClawOpen;
import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_A_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_Y_HIGH;
import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_Y_HIGHH;
import static org.firstinspires.ftc.teamcode.Constants_CS.SLIDE_POWER_H;
import static org.firstinspires.ftc.teamcode.Constants_CS.SLIDE_POWER_V;
import static org.firstinspires.ftc.teamcode.Constants_CS.speedMultiplier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_Y_LOW;


@TeleOp(name = "AAA Community Fair")
//V1 with pid for both slides but not odo
public class CommunityFair extends LinearOpMode {
    public float DriveTrains_ReducePOWER=0.6f;
    //   DriveTrains_ReducePOWER = 0.75f;
//    DriveTrains_ReducePOWER = speedLimiterSlower;//************************
    HardwareFair robot = new HardwareFair();
    public String fieldOrRobotCentric = "robot";
    boolean move = false;
    // 在类顶部声明PID控制器
    // 状态变量
    // pid for VSlides
    private boolean pidActiveVS = false; // PID 控制是否激活
    private int pidTargetPositionVS = 0; // PID 控制目标位置
    private PIDController pidControllerVS = new PIDController(0.005, 0.0000005, 0.0002);// (0.005, 0.0000005, 0.0002) good for target 300 (1.9, 0.014, 4.9)
    // Tune these values  POSITION_B_EXTRUDETransfer = 600;//horizontal slides  out //600 is too much

    // pid for HSlides
    int controlMode = 1;

    ButtonHandler dpadDownHandler = new ButtonHandler();
    ButtonHandler dpadUpHandler = new ButtonHandler();
    ButtonHandler dpadLeftHandler = new ButtonHandler();
    ButtonHandler dpadRightHandler = new ButtonHandler();
    ButtonHandler leftBumperHandler = new ButtonHandler();
    ButtonHandler rightBumperHandler = new ButtonHandler();
    ButtonHandler gamepad1XHandler = new ButtonHandler();
    ButtonHandler gamepad1BHandler = new ButtonHandler();
    ButtonHandler gamepad1YHandler = new ButtonHandler();
    ButtonHandler gamepad1AHandler = new ButtonHandler();
    ButtonHandler gamepad1BackHandler = new ButtonHandler();

    ButtonHandler gamepad2XHandler = new ButtonHandler();
    ButtonHandler gamepad2BHandler = new ButtonHandler();
    ButtonHandler gamepad2YHandler = new ButtonHandler();
    ButtonHandler gamepad2AHandler = new ButtonHandler();
    ButtonHandler gamepad2BackHandler = new ButtonHandler();
    Gyro gyro = new Gyro(); // 创建 Gyro 类的对象
    private volatile boolean isRunning = true;
    ElapsedTime delayTimer = new ElapsedTime();
    /*
    package mypackage; // 与 Gyro 类的包名一致
            Gyro gyro = new Gyro(); // 创建 Gyro 类的对象
            gyro.turn();            // 调用 turn() 方法
     */
    public String armPositionCuzBorS ="NOLL"; //new variable for it and arm will go back of robo

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
//        gyro.robot.init(hardwareMap);
        Thread driveTrainThread = new Thread(this::runDriveTrain);
        Thread updateVSlidePIDControl = new Thread(this::runupdateVSlidePIDControl);
        Thread intakeThread = new Thread(this::runIntake);
        Thread outtakeThread = new Thread(this::runOuttake);

        driveTrainThread.start();
        updateVSlidePIDControl();
        intakeThread.start();
        outtakeThread.start();

        waitForStart();

        while (opModeIsActive()) {
//

            dpadDownHandler.update(gamepad1.dpad_down);
            dpadUpHandler.update(gamepad1.dpad_up);
            dpadLeftHandler.update(gamepad1.dpad_left);
            dpadRightHandler.update(gamepad1.dpad_right);
            leftBumperHandler.update(gamepad1.left_bumper);
            rightBumperHandler.update(gamepad1.right_bumper);
            gamepad1XHandler.update(gamepad1.x);
            gamepad1BHandler.update(gamepad1.b);
            gamepad1YHandler.update(gamepad1.y);
            gamepad1AHandler.update(gamepad1.a);
            gamepad1BackHandler.update(gamepad1.back);


            if (gamepad1.left_trigger > 0.3 && gamepad1.left_trigger <= 0.7) { // 轻按
                robot.IClaw.setPosition(0.75);
            }
            if (gamepad1.right_trigger > 0.3 && gamepad1.right_trigger <= 0.7) { // 轻按
                robot.IClaw.setPosition(0.5);
            }

//******************end  IArm L and R*****************

            // out take
            dpadDownHandler.update(gamepad2.dpad_down);
            dpadUpHandler.update(gamepad2.dpad_up);
            dpadLeftHandler.update(gamepad2.dpad_left);
            dpadRightHandler.update(gamepad2.dpad_right);
            leftBumperHandler.update(gamepad2.left_bumper);
            rightBumperHandler.update(gamepad2.right_bumper);
            gamepad2XHandler.update(gamepad2.x);
            gamepad2BHandler.update(gamepad2.b);
            gamepad2YHandler.update(gamepad2.y);
            gamepad2AHandler.update(gamepad2.a);



            if (gamepad2.y) { //top
                startVSlidePIDControl(2850);
            }
            if (gamepad2.a) { //bottom
                startVSlidePIDControl(0);
            }
            if (gamepad2.b) { //low
                startVSlidePIDControl(1200);
            }
            if (gamepad2.x) { //med
                startVSlidePIDControl(1850);
            }



            moveDriveTrain_FieldCentric() ;
            updateVSlidePIDControl();
            intake();
            outtake();

////////////////////////////////////


        } //end of while loop
        // Stop all threads when op mode stops
        isRunning = false;
        try {
            driveTrainThread.join();
            intakeThread.join();
            outtakeThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    } //end of run mode

    // Thread for drive train
    private void runDriveTrain() {
        while (isRunning) {
            moveDriveTrain_FieldCentric();
//            sleep(50); // Add a short delay to prevent CPU overutilization
            while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
                // Other tasks can be processed here
            }
        }
    }

    // Thread for drive train
    private void runupdateVSlidePIDControl() {
        while (isRunning) {
            updateVSlidePIDControl();
//            sleep(50); // Add a short delay to prevent CPU overutilization
            while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
                // Other tasks can be processed here
            }
        }
    }

    // Thread for intake
    private void runIntake() {
        while (isRunning) {
            intake();
//            sleep(50); // Add a short delay to prevent CPU overutilization
            while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
                // Other tasks can be processed here
            }
        }
    }

    // Thread for outtake
    private void runOuttake() {
        while (isRunning) {
            outtake();
//            sleep(50); // Add a short delay to prevent CPU overutilization
            while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
                // Other tasks can be processed here
            }
        }
    }


    //Begin Definition and Initialization of intake()
    public void intake() {
    }
//End Definition and Initialization of intake()


    //Begin Definition and Initialization of outtake()
    public void outtake() {
    }
//End Definition and Initialization of outtake()

    //Begin Definition and Initialization of steptestservo()    //Begin debugging with a step increment of 0.05  SGC - servoGamepadControl
    public void servoGamepadControl() {
    }
///////////////////End Definition and Initialization of steptestservo()

//End Definition and Initialization of gamepad


    /// 初始化 PID 控制器
    private void startVSlidePIDControl(int targetPosition) {
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidControllerVS.reset();
        pidControllerVS.enable();
        pidControllerVS.setSetpoint(targetPosition);
        pidControllerVS.setTolerance(10); // 允许误差范围
        pidTargetPositionVS = targetPosition;
        pidActiveVS = true; // 激活 PID 控制
    }
    // 在主循环中调用的非阻塞 PID 控制逻辑
    private void updateVSlidePIDControl() {
        if (!pidActiveVS) return; // 如果 PID 未激活，直接返回

        int currentPositionL = robot.VSMotorR.getCurrentPosition();

        // 计算 PID 输出
        double powerL = pidControllerVS.performPID(currentPositionL);
        robot.VSMotorR.setPower(powerL*0.6); // change it to make it move faster

        // 输出 Telemetry 信息
        telemetry.addData("PID Target", pidTargetPositionVS);
        telemetry.addData("Current Position L", currentPositionL);
        telemetry.addData("Power L", powerL);
        telemetry.update();

//        // 如果达到目标位置，停止滑轨运动，但保持抗重力功率
//        if (pidControllerVS.onTarget()) {
//            robot.VSMotorL.setPower(0.1); // 保持位置的最小功率
//            robot.VSMotorR.setPower(0.1);
//            pidActiveVS = false; // 停止 PID 控制
//        }
        // 在 updateVSlidePIDControl 中加入抗重力逻辑
        if (!pidActiveVS && Math.abs(robot.VSMotorR.getCurrentPosition() - pidTargetPositionVS) > 10) {
            double holdPowerVS = pidControllerVS.performPID(robot.VSMotorR.getCurrentPosition());
            robot.VSMotorR.setPower(holdPowerVS);
            pidActiveVS = false; // 停止 PID 控制
        }

    }

//////////////startVSlidePIDControl/////////////


///////////startHSlidePIDControl///////////////

    /// 初始化 PID 控制器

    // 在主循环中调用的非阻塞 PID 控制逻辑

//////////////startHSlidePIDControl/////////////



    public void moveDriveTrain_FieldCentric() {
        double y = gamepad1.left_stick_y * (0.45); // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * (0.45);
        double rx = -gamepad1.right_stick_x * (0.45); //*(0.5) is fine

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        //******************************************temp
//        if (gamepad1.back) {
//            robot.imu.resetYaw();
//        }
//******************************************temp

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        robot.LFMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
        robot.LBMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
        robot.RFMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
        robot.RBMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
    }

    public void moveDriveTrain_RobotCentric() {
        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double robot_x = gamepad1.left_stick_x;
        double robot_rx = gamepad1.right_stick_x*0.5; // If a smooth turn is required 0.5

        double fl = robot_y - robot_x - robot_rx;
        double bl = robot_y + robot_x - robot_rx;
        double fr = robot_y + robot_x + robot_rx;
        double br = robot_y - robot_x + robot_rx;

        robot.LFMotor.setPower(fl * speedMultiplier);
        robot.LBMotor.setPower(bl * speedMultiplier);
        robot.RFMotor.setPower(fr * speedMultiplier);
        robot.RBMotor.setPower(br * speedMultiplier);

    }


    public void moveDriveTrain() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = (gamepad1.right_stick_x*0.5);
        double fl = y - x - rx;
        double bl = y + x - rx;
        double fr = y + x + rx;
        double br = y - x + rx;
        robot.LFMotor.setPower(fl*DriveTrains_ReducePOWER);
        robot.LBMotor.setPower(bl*DriveTrains_ReducePOWER);
        robot.RFMotor.setPower(fr*DriveTrains_ReducePOWER);
        robot.RBMotor.setPower(br*DriveTrains_ReducePOWER);
    }



    public void RobotCentricDriveTrain () {
        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double robot_x = gamepad1.left_stick_x;
        double robot_rx = gamepad1.right_stick_x*0.5; // If a smooth turn is required 0.5

        double fl = robot_y - robot_x - robot_rx;
        double bl = robot_y + robot_x - robot_rx;
        double fr = robot_y + robot_x + robot_rx;
        double br = robot_y - robot_x + robot_rx;

        robot.LFMotor.setPower(fl * speedMultiplier);
        robot.LBMotor.setPower(bl * speedMultiplier);
        robot.RFMotor.setPower(fr * speedMultiplier);
        robot.RBMotor.setPower(br * speedMultiplier);

    }

    //Begin Definition and Initialization of Horizontal Slides

//End Definition and Initialization of Horizontal Slides

//Begin Definition and Initialization of Vertical Slides by gamepad2.left_stick_y

    public void liftVertSlidesHigh () {
        double liftVertSlides_y = -gamepad2.left_stick_y;
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.VSMotorR.setPower(liftVertSlides_y*0.45);
        robot.VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        //up joystick makes the slides rotate clockwise on the out right side
        //when looking at the robots right side from the outside wall the slide pulley spins clockwise/to the right when the joystick is pushed up
    }

//End Definition and Initialization of Horizontal Slides by gamepad2.left_stick_x

    //Begin Definition and Initialization of Vertical Slides
    private void moveVSlideToPosition ( int targetPosition){
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("liftMotorR.getCurrentPosition()",robot.VSMotorR.getCurrentPosition());
        telemetry.update();
        robot.VSMotorR.setTargetPosition(-targetPosition);
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.VSMotorR.setPower(+SLIDE_POWER_V);
        move = true;
        while (robot.VSMotorR.isBusy() && move) {
            // Wait until the motor reaches the target position
        }
//        while (robot.VSMotorR.isBusy() && move) {
        //           // Wait until the motor reaches the target position
        //       }
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("after while liftMotorR.getCurrentPosition()",robot.VSMotorR.getCurrentPosition());
        telemetry.update();

//        robot.VSMotorL.setPower(0.2);
//        robot.VSMotorR.setPower(0.2);
//        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//        robot.VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
// Fine-tune the position using a PID-like approach
//        holdSlidePosition(targetPosition);
        move = false;
    }
    //////////////////////////




    ///////////////////////////
    private void holdSlidePosition(int targetPosition) {
        final double HOLD_POWER = 0.1; // Minimal power to hold the position
        final int POSITION_TOLERANCE = 10; // Allowable deviation from the target

        while (true) {
            int currentPositionR = robot.VSMotorR.getCurrentPosition();

            // Check if the slide is within the tolerance
            if (Math.abs(currentPositionR + targetPosition) <= POSITION_TOLERANCE &&
                    Math.abs(currentPositionR + targetPosition) <= POSITION_TOLERANCE) {
                robot.VSMotorR.setPower(0);
            } else {
                // Apply minimal power to correct the position
                robot.VSMotorR.setPower(HOLD_POWER);
            }

            // Optionally break the loop based on a condition or timer
            // Example: break if a stop flag is set
            if (!move) {
                break;
            }

            // Add telemetry to monitor holding behavior
            telemetry.addData("Holding Position R", currentPositionR);
            telemetry.update();
        }
    }
}








