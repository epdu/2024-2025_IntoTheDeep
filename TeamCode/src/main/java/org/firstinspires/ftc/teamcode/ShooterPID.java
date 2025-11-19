package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants_CS.speedMultiplier;
import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "A TeleOp ShooterPID")
public class ShooterPID extends LinearOpMode {
    public float DriveTrains_ReducePOWER = 0.8f;
    HardwareScrim robot = new HardwareScrim();

    public String fieldOrRobotCentric = "robot";
    boolean move = false;
    private boolean pidActiveShooter = false; // PID 控制是否激活
    private int pidTargetSpeedShooter = 0; // PID 控制目标位置
    private PIDController pidControllerShooter = new PIDController(0.005, 0.0000005, 0.0002);// (0.005, 0.0000005, 0.0002) good for target 300 (1.9, 0.014, 4.9)
    // Tune these values  POSITION_B_EXTRUDETransfer = 600;//horizontal slides  out //600 is too much
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

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
//        gyro.robot.init(hardwareMap);
        Thread driveTrainThread = new Thread(this::runDriveTrain);
        Thread intakeThread = new Thread(this::runIntake);
        Thread outtakeThread = new Thread(this::runOuttake);
        Thread updateShooterPIDControl = new Thread(this::runupdateShooterPIDControl);

        driveTrainThread.start();
        intakeThread.start();
        outtakeThread.start();
        updateShooterPIDControl();

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Insert whatever other initialization stuff you do here
        waitForStart();

        while(opModeIsActive()) {
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
                double shooterpower = 0.5;
                robot.ShooterMotor.setPower(shooterpower);
            }
            if (gamepad1.left_trigger > 0.7 && gamepad1.left_trigger <= 1.0) { // 轻按
                double shooterpower = 0.5;
                robot.ShooterMotor.setPower(shooterpower);
                double intakepower1 = 0.4;
                robot.IntakeMotor.setPower(intakepower1);
            }

            if (gamepad1.left_bumper) { //low
                robot.ShooterMotor.setPower(0);
            }
            if (gamepad1.right_trigger > 0.3 && gamepad1.right_trigger <= 0.7) { // 轻按
                double intakepower = 0.77;
                    robot.IntakeMotor.setPower(intakepower);
                    robot.ShooterMotor.setPower(-0.2);
            }
            if (gamepad1.right_bumper) { //top stop
                robot.IntakeMotor.setPower(0);
                robot.ShooterMotor.setPower(0);
            }
            if (gamepad1.y) { //bottom
                double intakepower1 = 0.4;
                robot.IntakeMotor.setPower(intakepower1);
                robot.ShooterMotor.setPower(0);

            }
            if (gamepad1.a) { //bottom
                double intakepower2 = -0.4;
                robot.IntakeMotor.setPower(intakepower2);
            }
//            if (gamepad1.b) { //med
//                robot.blocker.setPosition(0);
//            }
//            if (gamepad1.x) { //med
//                robot.blocker.setPosition(0.5);
//            }

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


            //put driver2 commands in
                moveDriveTrain_FieldCentric() ;
//                moveDriveTrain_RobotCentric();
                intake();
                outtake();
                updateShooterPIDControl();

/////////////////////////////////////////////////


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
        private void runDriveTrain () {
            while (isRunning) {
                moveDriveTrain_RobotCentric();
//            sleep(50); // Add a short delay to prevent CPU overutilization
                while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
                    // Other tasks can be processed here
                }
            }
        }

        // Thread for drive train

        // Thread for intake
        private void runIntake () {
            while (isRunning) {
                intake();
//            sleep(50); // Add a short delay to prevent CPU overutilization
                while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
                    // Other tasks can be processed here
                }
            }
        }

        // Thread for outtake
        private void runOuttake () {
            while (isRunning) {
                outtake();
//            sleep(50); // Add a short delay to prevent CPU overutilization
                while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
                    // Other tasks can be processed here
                }
            }
        }
    private void runupdateShooterPIDControl() {
        while (isRunning) {
            updateShooterPIDControl();
//            sleep(50); // Add a short delay to prevent CPU overutilization
            while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
                // Other tasks can be processed here
            }
        }
    }


        public void intake () {
        }

        public void outtake () {
        }

        public void servoGamepadControl () {
        }
////////////////////////////////////////////////////////
///////////////////////////////////////////////////////

//    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
//    setPower()
//    RUN_USING_ENCODER
//    setVelocity(ticks per second)

///////////startShooterPIDControl///////////////

    /// 初始化 PID 控制器
    private void startShooterPIDControl(int targetPosition) {
        robot.ShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ShooterMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidControllerShooter.reset();
        pidControllerShooter.enable();
        pidControllerShooter.setSetpoint(targetPosition);
        pidControllerShooter.setTolerance(10); // 允许误差范围
        pidTargetSpeedShooter = targetPosition;
        pidActiveShooter = true; // 激活 PID 控制
    }
    // 在主循环中调用的非阻塞 PID 控制逻辑
    private void updateShooterPIDControl() {
        if (!pidActiveShooter) return; // 如果 PID 未激活，直接返回

        int currentPositionL = robot.ShooterMotorL.getCurrentPosition();

        // 计算 PID 输出
        double powerL = pidControllerShooter.performPID(currentPositionL);
        robot.ShooterMotorL.setPower(powerL*0.8); // change it to make it move faster both at the same time
        robot.ShooterMotorR.setPower(powerL*0.8); // change it to make it move faster

        // 输出 Telemetry 信息
        telemetry.addData("PID Target", pidTargetSpeedShooter);
        telemetry.addData("Current Position L", currentPositionL);
        telemetry.addData("Power L", powerL);
        telemetry.update();

//        // 如果达到目标位置，停止滑轨运动，但保持抗重力功率
//        if (pidControllerShooter.onTarget()) {
//            robot.ShooterMotorL.setPower(0.1); // 保持位置的最小功率
//            robot.ShooterMotorR.setPower(0.1);
//            pidActiveShooter = false; // 停止 PID 控制
//        }
        // 在 updateShooterPIDControl 中加入抗重力逻辑
        if (!pidActiveShooter && Math.abs(robot.ShooterMotorL.getCurrentPosition() - pidTargetSpeedShooter) > 10) {
            double holdPowerVS = pidControllerShooter.performPID(robot.ShooterMotorL.getCurrentPosition());
            robot.ShooterMotorL.setPower(holdPowerVS);
            robot.ShooterMotorR.setPower(holdPowerVS);
            pidActiveShooter = false; // 停止 PID 控制
        }

    }

    //////////////startShooterPIDControl///////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////

        public void moveDriveTrain_FieldCentric () {
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

        public void moveDriveTrain_RobotCentric () {
            double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double robot_x = gamepad1.left_stick_x;
            double robot_rx = gamepad1.right_stick_x * 0.5; // If a smooth turn is required 0.5

            double fl = robot_y - robot_x - robot_rx;
            double bl = robot_y + robot_x - robot_rx;
            double fr = robot_y + robot_x + robot_rx;
            double br = robot_y - robot_x + robot_rx;

            robot.LFMotor.setPower(fl * speedMultiplier);
            robot.LBMotor.setPower(bl * speedMultiplier);
            robot.RFMotor.setPower(fr * speedMultiplier);
            robot.RBMotor.setPower(br * speedMultiplier);

        }


        public void moveDriveTrain () {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = (gamepad1.right_stick_x * 0.5);
            double fl = y - x - rx;
            double bl = y + x - rx;
            double fr = y + x + rx;
            double br = y - x + rx;
            robot.LFMotor.setPower(fl * DriveTrains_ReducePOWER);
            robot.LBMotor.setPower(bl * DriveTrains_ReducePOWER);
            robot.RFMotor.setPower(fr * DriveTrains_ReducePOWER);
            robot.RBMotor.setPower(br * DriveTrains_ReducePOWER);
        }


        public void RobotCentricDriveTrain () {
            double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double robot_x = gamepad1.left_stick_x;
            double robot_rx = gamepad1.right_stick_x * 0.5; // If a smooth turn is required 0.5

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

