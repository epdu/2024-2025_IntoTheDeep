package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants_CS.SLIDE_POWER_V;
import static org.firstinspires.ftc.teamcode.Constants_CS.speedMultiplier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "A Longhorn Scrim")
public class LonghornScrim extends LinearOpMode {
    public float DriveTrains_ReducePOWER=0.6f;
    HardwareScrim robot = new HardwareScrim();

    public String fieldOrRobotCentric = "robot";
    boolean move = false;

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


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
//        gyro.robot.init(hardwareMap);
        Thread driveTrainThread = new Thread(this::runDriveTrain);
        Thread intakeThread = new Thread(this::runIntake);
        Thread outtakeThread = new Thread(this::runOuttake);

        driveTrainThread.start();
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
            }
            if (gamepad1.right_trigger > 0.3 && gamepad1.right_trigger <= 0.7) { // 轻按
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
            }
            if (gamepad2.a) { //bottom
            }
            if (gamepad2.b) { //low
            }
            if (gamepad2.x) { //med
            }


            moveDriveTrain_FieldCentric() ;
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


    public void intake() {
    }

    public void outtake() {
    }

    public void servoGamepadControl() {
    }

    public void moveDriveTrain_FieldCentric() {
        double y = gamepad1.left_stick_y * (0.45); // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * (0.45);
        double rx = -gamepad1.right_stick_x * (0.45); //*(0.5) is fine

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
}
