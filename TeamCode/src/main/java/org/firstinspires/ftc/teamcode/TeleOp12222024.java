//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.teamcode.Constants_CS.IArmLDown;
//import static org.firstinspires.ftc.teamcode.Constants_CS.IArmLDownForPick;
//import static org.firstinspires.ftc.teamcode.Constants_CS.IArmLUp;
//import static org.firstinspires.ftc.teamcode.Constants_CS.IArmRDown;
//import static org.firstinspires.ftc.teamcode.Constants_CS.IArmRDownForPick;
//import static org.firstinspires.ftc.teamcode.Constants_CS.IArmRUp;
//import static org.firstinspires.ftc.teamcode.Constants_CS.IClawCloseLose;
//import static org.firstinspires.ftc.teamcode.Constants_CS.IClawCloseTight;
//import static org.firstinspires.ftc.teamcode.Constants_CS.IClawOpen;
//import static org.firstinspires.ftc.teamcode.Constants_CS.OArmRearSpecimenPick;
//import static org.firstinspires.ftc.teamcode.Constants_CS.OArmTransferPosition;
//import static org.firstinspires.ftc.teamcode.Constants_CS.OClawCloseSuperTight;
//import static org.firstinspires.ftc.teamcode.Constants_CS.OClawCloseTight;
//import static org.firstinspires.ftc.teamcode.Constants_CS.OClawOpen;
//import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_A_BOTTOM;
//import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_B_EXTRUDE;
//import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_B_EXTRUDETransfer;
//import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_B_EXTRUDETransferC;
//import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_B_EXTRUDE_MORE;
//import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_X_IN;
//import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_Y_HIGH;
//import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_Y_LOW;
//import static org.firstinspires.ftc.teamcode.Constants_CS.SLIDE_POWER_H;
//import static org.firstinspires.ftc.teamcode.Constants_CS.SLIDE_POWER_V;
//import static org.firstinspires.ftc.teamcode.Constants_CS.WristxpitchDown;
//import static org.firstinspires.ftc.teamcode.Constants_CS.WristxpitchIntermedia4PositionAdjust;
//import static org.firstinspires.ftc.teamcode.Constants_CS.WristxpitchUp;
//import static org.firstinspires.ftc.teamcode.Constants_CS.WristzyawLeft;
//import static org.firstinspires.ftc.teamcode.Constants_CS.WristzyawRight;
//import static org.firstinspires.ftc.teamcode.Constants_CS.speedMultiplier;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//@TeleOp(name = "A QualTeleOp 12202024 V0")
//public class TeleOp12222024 extends LinearOpMode {
//    public float DriveTrains_ReducePOWER=0.75f;
// //   DriveTrains_ReducePOWER = 0.75f;
////    DriveTrains_ReducePOWER = speedLimiterSlower;//************************
//    HardwareTeletubbies robot = new HardwareTeletubbies();
//    public String fieldOrRobotCentric = "robot";
//    boolean move = false;
//
//    private PIDController pidControllerL = new PIDController(1.9, 0.014, 4.9); // Tune these values  POSITION_B_EXTRUDETransfer = 600;//horizontal slides  out //600 is too much
//    private PIDController pidControllerR = new PIDController(1.9, 0.014, 4.9); // Tune these values
//    int controlMode = 1;
//    ButtonHandler dpadDownHandler = new ButtonHandler();
//    ButtonHandler dpadUpHandler = new ButtonHandler();
//    ButtonHandler leftBumperHandler = new ButtonHandler();
//    ButtonHandler rightBumperHandler = new ButtonHandler();
//    ButtonHandler gamepad1XHandler = new ButtonHandler();
//    ButtonHandler gamepad1BHandler = new ButtonHandler();
//    ButtonHandler gamepad1YHandler = new ButtonHandler();
//    ButtonHandler gamepad1AHandler = new ButtonHandler();
//    ButtonHandler gamepad1BackHandler = new ButtonHandler();
//    Gyro gyro = new Gyro(); // 创建 Gyro 类的对象
//    private volatile boolean isRunning = true;
//    ElapsedTime delayTimer = new ElapsedTime();
///*
//package mypackage; // 与 Gyro 类的包名一致
//        Gyro gyro = new Gyro(); // 创建 Gyro 类的对象
//        gyro.turn();            // 调用 turn() 方法
// */
//    @Override
//    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        robot.init(hardwareMap);
//        gyro.robot.init(hardwareMap);
//        Thread driveTrainThread = new Thread(this::runDriveTrain);
//        Thread intakeThread = new Thread(this::runIntake);
//        Thread outtakeThread = new Thread(this::runOuttake);
//
//        driveTrainThread.start();
//        intakeThread.start();
//        outtakeThread.start();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
////            telemetry.addData("Status", "All systems running...");
////            telemetry.update();
////            moveDriveTrain_RobotCentric(); // Select either RobotCentricDriveTrain() or FieldCentricDriveTrain() based on your requirements.
//            if (gamepad1BackHandler.isShortPress()) { //fix it later;
//                DriveTrains_ReducePOWER = 0.25f;
//                telemetry.addData("DriveTrains_ReducePOWER", DriveTrains_ReducePOWER);
//                telemetry.update();
//                // Non-blocking delay to prevent rapid mode switching
//                delayTimer.reset();
//                while (delayTimer.milliseconds() < 300 && opModeIsActive()) {
//                    // Other tasks can be processed here
//                } // 防止快速连击导致模式快速切换
//            }
//            if (gamepad1BackHandler.isLongPress()) { //fix it later;
//                DriveTrains_ReducePOWER = 0.75f;
//                telemetry.addData("DriveTrains_ReducePOWER", DriveTrains_ReducePOWER);
//                telemetry.update();
//                // Non-blocking delay to prevent rapid mode switching
//                delayTimer.reset();
//                while (delayTimer.milliseconds() < 300 && opModeIsActive()) {
//                    // Other tasks can be processed here
//                } // 防止快速连击导致模式快速切换
//            }
//
//            if (gamepad1.start) { // 切换控制模式
//                controlMode = (controlMode + 1) % 2; // 假设两种模式 0 和 1
//                telemetry.addData("Control Mode", controlMode == 0 ? "Mode 0: Standard" : "Mode 1: Advanced");
//                telemetry.update();
//                // Non-blocking delay to prevent rapid mode switching
//                delayTimer.reset();
//                while (delayTimer.milliseconds() < 300 && opModeIsActive()) {
//                    // Other tasks can be processed here
//                } // 防止快速连击导致模式快速切换
//            }
//
//            moveDriveTrain_FieldCentric() ;
//            intake();
//            outtake();
//
////            moveDriveTrain(); //robot centric
////            liftVertSlidesHigh();
////            extrHoriSlidesLong();
////            servoGamepadControl();
//////////////////////////////////////
//
//
//        } //end of while loop
//        // Stop all threads when op mode stops
//        isRunning = false;
//        try {
//            driveTrainThread.join();
//            intakeThread.join();
//            outtakeThread.join();
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt();
//        }
//
//    } //end of run mode
//
//    // Thread for drive train
//    private void runDriveTrain() {
//        while (isRunning) {
//            moveDriveTrain_FieldCentric();
////            sleep(50); // Add a short delay to prevent CPU overutilization
//            while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
//                // Other tasks can be processed here
//            }
//        }
//    }
//
//    // Thread for intake
//    private void runIntake() {
//        while (isRunning) {
//            intake();
////            sleep(50); // Add a short delay to prevent CPU overutilization
//            while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
//                // Other tasks can be processed here
//            }
//        }
//    }
//
//    // Thread for outtake
//    private void runOuttake() {
//        while (isRunning) {
//            outtake();
////            sleep(50); // Add a short delay to prevent CPU overutilization
//            while (delayTimer.milliseconds() < 50 && opModeIsActive()) {
//                // Other tasks can be processed here
//            }
//        }
//    }
//
//
////Begin Definition and Initialization of intake()
//    public void intake() {
//
////Begin Definition and Initialization of gamepad
////        if (gamepad1.start) { // 切换控制模式
////            controlMode = (controlMode + 1) % 2; // 假设两种模式 0 和 1
////            telemetry.addData("Control Mode", controlMode == 0 ? "Mode 0: Standard" : "Mode 1: Advanced");
////            telemetry.update();
////            sleep(300); // 防止快速连击导致模式快速切换
////        }
//// 根据不同模式定义按键功能
//        switch (controlMode) {
//            case 0:
//                // intake
//                dpadDownHandler.update(gamepad1.dpad_down);
//                dpadUpHandler.update(gamepad1.dpad_up);
//                leftBumperHandler.update(gamepad1.left_bumper);
//                rightBumperHandler.update(gamepad1.right_bumper);
//                gamepad1XHandler.update(gamepad1.x);
//                gamepad1BHandler.update(gamepad1.b);
//                gamepad1YHandler.update(gamepad1.y);
//                gamepad1AHandler.update(gamepad1.a);
//                gamepad1BackHandler.update(gamepad1.back);
//                //Begin  moveHSlideToPosition
//                if (gamepad1BHandler.isShortPress()) { //IN
//                    moveHSlideToPosition(POSITION_X_IN);
//                    telemetry.addData("Status", "POSITION_X_IN");
//                    telemetry.update();
//                    gamepad1BHandler.reset();
//                }
//                if (gamepad1XHandler.isShortPress()) { //EXTRUDE
//                    moveHSlideToPosition(POSITION_B_EXTRUDE);
//                    telemetry.addData("Status", "POSITION_B_EXTRUDE");
//                    telemetry.update();
////                    sleep(600);
////                    gyro.turn(90);// 调用 turn() 方法turn(90);
////                    telemetry.addData("Status", "gyro.turn");
////                    telemetry.update();
////                    sleep(1000);
//                    gamepad1XHandler.reset();
//                }
//                if (gamepad1XHandler.isLongPress()) { //EXTRUDE_MORE
//                    moveHSlideToPosition(POSITION_B_EXTRUDE_MORE);
//                    gamepad1XHandler.reset();
//                }
//                //End  moveHSlideToPosition
//
////Begin  open and close of intakeclaw 12122024 finetuned
//
//                if (gamepad1.left_trigger > 0.3 && gamepad1.left_trigger <= 0.7) { // 轻按
//                    robot.IClaw.setPosition(IClawOpen); //12122024
//                }
//                if (gamepad1.right_trigger > 0.3 && gamepad1.right_trigger <= 0.7) { // 轻按
//                    robot.IClaw.setPosition(IClawCloseLose); //0.54 moveable 0.542 barely movable 0.543 hold
//                }
//                if (gamepad1.right_trigger > 0.7) { // 深按
//                    robot.IClaw.setPosition(IClawCloseTight); //0.54 moveable 0.542 barely movable 0.543 hold
//                }
//
////End open and close of intakeclaw
//
////Begin  Wristzyaw
//                if (gamepad2.b) { //right
//                    robot.Wristzyaw.setPosition(WristzyawRight); //Wristzyaw right 45 degree 12122024
//                }
//                if (gamepad2.x) { //left
//                    robot.Wristzyaw.setPosition(WristzyawLeft); // Wristzyaw left 45 degree 12122024 // robot.Wristzyaw.setPosition(0.65); for left
//                }
//
////one key ready for pick
//                if (gamepad1.left_bumper) { //up if arm is Horizontal, the the wrist is vertical up and down
//                    robot.OArmL.setPosition(OArmRearSpecimenPick);
//                    robot.OArmR.setPosition(OArmRearSpecimenPick);
//                    robot.OClaw.setPosition(OClawOpen); //
//                    delayTimer.reset();
//                    while (delayTimer.milliseconds() < 200 && opModeIsActive()) {
//                        // Other tasks can be processed here
//                    }
//                    robot.Wristxpitch.setPosition(WristxpitchDown);
//                    robot.IClaw.setPosition(IClawOpen);
//                    robot.IArmL.setPosition(IArmLDown);
//                    robot.IArmR.setPosition(IArmRDown);
//                    //                    moveHSlideToPosition(POSITION_B_EXTRUDETransferC);
////                    sleep(500);
//                    //                    robot.OClaw.setPosition(OClawOpen); //open
//                }
//
////one key ready for pick up
//
////one key ready for transfer
//                if (gamepad1.right_bumper) { //
//                    robot.OArmL.setPosition(OArmTransferPosition);//transfer position
//                    robot.OArmR.setPosition(OArmTransferPosition);
//                    robot.Wristxpitch.setPosition(WristxpitchIntermedia4PositionAdjust); // Wristxpitch
//                    sleep(600);
//                    robot.IClaw.setPosition(IClawCloseTight); //  0.54
//                    moveHSlideToPosition(POSITION_B_EXTRUDETransfer);
//                    sleep(600);
//                    robot.Wristxpitch.setPosition(WristxpitchUp); // Wristxpitch
//                    robot.IArmL.setPosition(IArmLUp);
//                    robot.IArmR.setPosition(IArmRUp);
//                    sleep(600);
//                    robot.OClaw.setPosition(OClawCloseTight); // close 0.543 hold
//                    sleep(600);
//                    robot.IClaw.setPosition(IClawOpen); //open
//                    sleep(300);
//                    moveHSlideToPosition(POSITION_B_EXTRUDETransferC);
//                    sleep(300);
//                    robot.OArmL.setPosition(OArmRearSpecimenPick);
//                    robot.OArmR.setPosition(OArmRearSpecimenPick);
////                    sleep(600);
////                    moveVSlideToPosition(-POSITION_Y_HIGH);// high
////                    moveVSlideToPositionPID(-POSITION_Y_HIGH);// high
//
//                }
//
////one key ready for transfer
//
////******************Begin  IArm L and R****************
//
//                if (gamepad1.y) { //up
//                    robot.IArmL.setPosition(IArmLUp);  // always same as hardware IArmL.setPosition(0.6);
//                    robot.IArmR.setPosition(IArmRUp);
//                }
//                if (gamepad1.a ) { //down
//                    robot.IArmL.setPosition(IArmLDownForPick);
//                    robot.IArmR.setPosition(IArmRDownForPick); //
//                }
//
////******************end  IArm L and R*****************
//
//
//
//                break;
//
//            case 1:
//                // out take
//                dpadDownHandler.update(gamepad1.dpad_down);
//                dpadUpHandler.update(gamepad1.dpad_up);
//                leftBumperHandler.update(gamepad1.left_bumper);
//                rightBumperHandler.update(gamepad1.right_bumper);
//                gamepad1XHandler.update(gamepad1.x);
//                gamepad1BHandler.update(gamepad1.b);
//                gamepad1YHandler.update(gamepad1.y);
//                gamepad1AHandler.update(gamepad1.a);
//                //Begin  moveVSlideToPosition
//
//                // 左触发器双功能：轻按和深按
//                if (gamepad1BHandler.isShortPress()) { //IN
////                    moveVSlideToPositionPID(POSITION_A_BOTTOM);// slides down
//                    moveVSlideToPosition(POSITION_A_BOTTOM);// slides down
//                    gamepad1BHandler.reset();
//                }
//                if (gamepad1XHandler.isShortPress()) { //EXTRUDE
////                    moveVSlideToPositionPID(-POSITION_Y_LOW);
//                moveVSlideToPosition(-POSITION_Y_LOW);// slides move to middle
//                    gamepad1XHandler.reset();
//                }
//                if (gamepad1XHandler.isLongPress()) { //EXTRUDE_MORE
////                    moveVSlideToPositionPID(-POSITION_Y_HIGH);
//                moveVSlideToPosition(-POSITION_Y_HIGH);// high
//                    gamepad1XHandler.reset();
//                }
////                    if (gamepad1XHandler.isLongPress()) { //EXTRUDE_MORE
////                        moveVSlideToPosition(-POSITION_Y_HIGHH);// very high
////                        gamepad1XHandler.reset();
////                    }
//
//                //************End  moveVSlideToPosition***************
//
////one key ready for pick
//                if (gamepad1.left_bumper) { //up if arm is Horizontal, the the wrist is vertical up and down
//                    robot.OArmL.setPosition(OArmRearSpecimenPick);
//                    robot.OArmR.setPosition(OArmRearSpecimenPick);
//                    sleep(200);
//                    robot.IClaw.setPosition(IClawOpen);
//                    sleep(200);
//                }
//
////one key ready for pick up
//
//
//
////Begin  OArm L and R
//
//                if (gamepad1.y) { //rear specimen    OArmTransferPosition   OArmRearSpecimenPick
//                    robot.OArmL.setPosition(OArmRearSpecimenPick);
//                    robot.OArmR.setPosition(OArmRearSpecimenPick);
//                }
//                if (gamepad1.a) { //front transfer
//                    robot.OArmL.setPosition(OArmTransferPosition);
//                    robot.OArmR.setPosition(OArmTransferPosition);
//                }
//
////end  OArm L and R
//
////Begin  open and close of outtakeclaw 12122024 finetuned
//
//                if (gamepad1.left_trigger > 0.3 && gamepad1.left_trigger <= 0.7) { // 轻按
//                    robot.OClaw.setPosition(OClawOpen); //12122024
//                }
//                if (gamepad1.right_trigger > 0.3 && gamepad1.right_trigger <= 0.7) { // 轻按
//                    robot.OClaw.setPosition(OClawCloseTight);
//                }
//                if (gamepad1.right_trigger > 0.7) { // 深按
//                    robot.OClaw.setPosition(OClawCloseSuperTight); //
//                }
//
////End open and close of outtakeclaw
//
////End Definition and Initialization of gamepad
//
//                break;
//
//            // 如果需要更多模式，可以继续添加 case。
//        }
//
//
//
//    }
////End Definition and Initialization of intake()
//
//
////Begin Definition and Initialization of outtake()
//    public void outtake() {
//    }
////End Definition and Initialization of outtake()
//
////Begin Definition and Initialization of steptestservo()
//    public void servoGamepadControl() {
//        //Begin debugging with a step increment of 0.05  SGC - servoGamepadControl
//
///**
// * This code snippet controls the position of a servo motor using the gamepad triggers.
// *
// * **Purpose**:
// * - The left trigger (`gamepad1.left_trigger`) increases the servo's position by a fixed step (`SERVO_STEP`).
// * - The right trigger (`gamepad1.right_trigger`) decreases the servo's position by a fixed step (`SERVO_STEP`).
// * - The servo position is constrained between 0.01 (minimum) and 0.99 (maximum) to prevent invalid values.
// * - The current servo position is displayed on the telemetry for real-time monitoring.
// *
// * **Usage Instructions**:
// * 1. Press the **left trigger** (`gamepad1.left_trigger`) to move the servo incrementally towards its maximum position.
// * 2. Press the **right trigger** (`gamepad1.right_trigger`) to move the servo incrementally towards its minimum position.
// * 3. The servo's position is updated with a small delay (`sleep(200)` milliseconds) to prevent rapid changes from multiple trigger presses.
// * 4. Adjust `SERVO_STEP` as needed to control the increment size for finer or coarser adjustments.
// *
// * **Setup**:
// * - Ensure the servo is connected to the correct port and initialized in the `robot.TServo` variable.
// * - Configure the `SERVO_STEP` variable to determine how much the position changes with each trigger press.
// * - Calibrate the servo movement range (e.g., 0.01 to 0.99) based on your servo's physical limits to avoid damage.
// */
//
//
////            if (gamepad1.left_trigger > 0.3) {
////                servoPosition = servoPosition + SERVO_STEP;
////                if (servoPosition >= 1.0) {
////                    servoPosition = 0.99; // 限制最大值
////                }
////                robot.TServo.setPosition(servoPosition);
////                telemetry.addData("Servo Position", servoPosition);
////                telemetry.update();
////                sleep(200);
////            }
////            if (gamepad1.right_trigger > 0.3) {
////                servoPosition = servoPosition - SERVO_STEP;
////                if (servoPosition <= 0.0) {
////                    servoPosition = 0.01; // 限制最小值
////                }
////                robot.TServo.setPosition(servoPosition);
////                telemetry.addData("Servo Position", servoPosition);
////                telemetry.update();
////                sleep(200);
////            }
//
////End debugging with a step increment of 0.05
//
//    }
////End Definition and Initialization of steptestservo()
//
////Temp ******************************
////**************************
//
//////Begin  open and close of outtakeclaw 12122024 finetuned
////
////                    if (gamepad2.left_trigger > 0.3) { //open
////                        robot.OClaw.setPosition(0.32); //12122024
////                    }
////                    if (gamepad2.right_trigger > 0.3) { //close
////                        robot.OClaw.setPosition(0.548); // 0.543 hold
////                    }
////
//////End open and close of outtakeclaw
//
////End Definition and Initialization of gamepad
//
//
//
//
//    //Begin  Wristxpitch do not use it any more
////            if (gamepad1.dpad_up && !move) { //up if arm is Horizontal, the the wrist is vertical up and down
////                robot.Wristxpitch.setPosition(0.05); // Wristxpitch  12122024
////            }
////            if (gamepad1.dpad_down && !move) { //down
////                robot.Wristxpitch.setPosition(0.65); // Wristxpitch 12122024 0.65
////            }
////End   Wristxpitch
////            if (gamepad2.a && !move ) { //down
////                moveVSlideToPosition(POSITION_A_BOTTOM);
////            }
////            if (gamepad2.y && !move) { //up prepare forchameber
////                moveVSlideToPosition(-POSITION_Y_LOW);
////            }
////            if (gamepad2.right_bumper && !move){ //upforchameber
////                moveVSlideToPosition(-POSITION_Y_HIGH);
////            }
//
//    //      HAND SPECIALIST   48444442243  JULIA MAYBERRY
//    //for up
//    //for down
//
//    // ...（其他原始代码逻辑保持不变）
////
////
//    //Begin  Wristxpitch do not use it any more
////            if (gamepad1.dpad_up && !move) { //up if arm is Horizontal, the the wrist is vertical up and down
////                robot.Wristxpitch.setPosition(0.05); // Wristxpitch  12122024
////            }
////            if (gamepad1.dpad_down && !move) { //down
////                robot.Wristxpitch.setPosition(0.65); // Wristxpitch 12122024 0.65
////            }
////End   Wristxpitch
////            if (gamepad2.a && !move ) { //down
////                moveVSlideToPosition(POSITION_A_BOTTOM);
////            }
////            if (gamepad2.y && !move) { //up prepare forchameber
////                moveVSlideToPosition(-POSITION_Y_LOW);
////            }
////            if (gamepad2.right_bumper && !move){ //upforchameber
////                moveVSlideToPosition(-POSITION_Y_HIGH);
////            }
//
//    //      HAND SPECIALIST   48444442243  JULIA MAYBERRY
//    //for up
//    //for down
////****************************************************************************************
////
//
//
////Temp *************************
//    public void moveDriveTrain() {
//        double y = gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x;
//        double rx = (gamepad1.right_stick_x*0.5);
//        double fl = y - x - rx;
//        double bl = y + x - rx;
//        double fr = y + x + rx;
//        double br = y - x + rx;
//        robot.LFMotor.setPower(fl*DriveTrains_ReducePOWER);
//        robot.LBMotor.setPower(bl*DriveTrains_ReducePOWER);
//        robot.RFMotor.setPower(fr*DriveTrains_ReducePOWER);
//        robot.RBMotor.setPower(br*DriveTrains_ReducePOWER);
//    }
//
////Begin Definition and Initialization of Vertical Slides by gamepad2.left_stick_y
//
//    public void liftVertSlidesHigh () {
//        double liftVertSlides_y = -gamepad2.left_stick_y;
//        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.VSMotorL.setPower(liftVertSlides_y*0.45);
//        robot.VSMotorR.setPower(liftVertSlides_y*0.45);
//        robot.VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//        robot.VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//
//        //up joystick makes the slides rotate clockwise on the out right side
//        //when looking at the robots right side from the outside wall the slide pulley spins clockwise/to the right when the joystick is pushed up
//    }
//
////End Definition and Initialization of Vertical Slides by gamepad2.left_stick_y
//
////Begin Definition and Initialization of Horizontal Slides by gamepad2.left_stick_x  extrude slides long
//
//    public void extrHoriSlidesLong() {
//        double liftVertSlides_y = gamepad2.left_stick_x;
//        robot.HSMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.HSMotor.setPower(liftVertSlides_y*0.45);
//        robot.HSMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//
//        //up joystick makes the slides rotate clockwise on the out right side
//        //when looking at the robots right side from the outside wall the slide pulley spins clockwise/to the right when the joystick is pushed up
//    }
//
////End Definition and Initialization of Horizontal Slides by gamepad2.left_stick_x
//
////Begin Definition and Initialization of Vertical Slides
//    private void moveVSlideToPosition ( int targetPosition){
//        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("targetPosition", targetPosition);
//        telemetry.addData("liftMotorL.getCurrentPosition()",robot.VSMotorL.getCurrentPosition());
//        telemetry.addData("liftMotorR.getCurrentPosition()",robot.VSMotorR.getCurrentPosition());
//        telemetry.update();
//        robot.VSMotorL.setTargetPosition(-targetPosition);
//        robot.VSMotorR.setTargetPosition(-targetPosition);
//        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.VSMotorL.setPower(+SLIDE_POWER_V);
//        robot.VSMotorR.setPower(+SLIDE_POWER_V);
//        move = true;
//        while (robot.VSMotorL.isBusy() && robot.VSMotorR.isBusy() && move) {
//            // Wait until the motor reaches the target position
//        }
////        while (robot.VSMotorR.isBusy() && move) {
//        //           // Wait until the motor reaches the target position
//        //       }
//        telemetry.addData("targetPosition", targetPosition);
//        telemetry.addData("after while liftMotorL.getCurrentPosition()",robot.VSMotorL.getCurrentPosition());
//        telemetry.addData("after while liftMotorR.getCurrentPosition()",robot.VSMotorR.getCurrentPosition());
//        telemetry.update();
//
//        robot.VSMotorL.setPower(0);
//        robot.VSMotorR.setPower(0);
//        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//        robot.VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//        // Fine-tune the position using a PID-like approach
////        holdSlidePosition(targetPosition);
//        move = false;
//    }
////////////////////////////
//    private void moveVSlideToPositionPID(int targetPosition) {
//        // Set motors to use encoders
//        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Initialize PID Controllers
//        pidControllerL.reset();
//        pidControllerR.reset();
//        pidControllerL.setSetpoint(targetPosition);
//        pidControllerR.setSetpoint(targetPosition);
//        pidControllerL.setTolerance(10); // Allowable position error
//        pidControllerR.setTolerance(10);
//
//        move = true;
//
//        // Main control loop
//        while (move) {
//            int currentPositionL = robot.VSMotorL.getCurrentPosition();
//            int currentPositionR = robot.VSMotorR.getCurrentPosition();
//
//            // Calculate PID outputs
//            double powerL = pidControllerL.performPID(currentPositionL);
//            double powerR = pidControllerR.performPID(currentPositionR);
//
//            // Set motor power based on PID outputs
//            robot.VSMotorL.setPower(powerL);
//            robot.VSMotorR.setPower(powerR);
//
//            // Telemetry for debugging
//            telemetry.addData("Target Position", targetPosition);
//            telemetry.addData("Current Position L", currentPositionL);
//            telemetry.addData("Current Position R", currentPositionR);
//            telemetry.addData("Power L", powerL);
//            telemetry.addData("Power R", powerR);
//            telemetry.update();
//
//            // Check if both motors are on target
//            if (pidControllerL.onTarget() && pidControllerR.onTarget()) {
//                move = false;
//            }
//            telemetry.addData("Target Position", targetPosition);
//            telemetry.addData("Current Position L", currentPositionL);
//            telemetry.addData("Current Position R", currentPositionR);
//            telemetry.addData("Power L", powerL);
//            telemetry.addData("Power R", powerR);
//            telemetry.update();
//        }
//
//        // Stop motors and set braking behavior
//        robot.VSMotorL.setPower(0);
//        robot.VSMotorR.setPower(0);
//        robot.VSMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.VSMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//
//
/////////////////////////////
//    private void holdSlidePosition(int targetPosition) {
//        final double HOLD_POWER = 0.1; // Minimal power to hold the position
//        final int POSITION_TOLERANCE = 10; // Allowable deviation from the target
//
//        while (true) {
//            int currentPositionL = robot.VSMotorL.getCurrentPosition();
//            int currentPositionR = robot.VSMotorR.getCurrentPosition();
//
//            // Check if the slide is within the tolerance
//            if (Math.abs(currentPositionL + targetPosition) <= POSITION_TOLERANCE &&
//                    Math.abs(currentPositionR + targetPosition) <= POSITION_TOLERANCE) {
//                robot.VSMotorL.setPower(0);
//                robot.VSMotorR.setPower(0);
//            } else {
//                // Apply minimal power to correct the position
//                robot.VSMotorL.setPower(HOLD_POWER);
//                robot.VSMotorR.setPower(HOLD_POWER);
//            }
//
//            // Optionally break the loop based on a condition or timer
//            // Example: break if a stop flag is set
//            if (!move) {
//                break;
//            }
//
//            // Add telemetry to monitor holding behavior
//            telemetry.addData("Holding Position L", currentPositionL);
//            telemetry.addData("Holding Position R", currentPositionR);
//            telemetry.update();
//        }
//    }
////End Definition and Initialization of Vertical Slides
//
////Begin Definition and Initialization of Horizontal Slides
//    private void moveHSlideToPosition ( int targetPosition){
//        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("targetPosition", targetPosition);
//        telemetry.addData("robot.HSMotor.getCurrentPosition()",robot.HSMotor.getCurrentPosition());
//        telemetry.update();
//        robot.HSMotor.setTargetPosition(-targetPosition);
//        robot.HSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.HSMotor.setPower(+SLIDE_POWER_H);
//        move = true;
//        while (robot.HSMotor.isBusy()  && move) {
//            // Wait until the motor reaches the target position
//        }
////        while (robot.VSMotorR.isBusy() && move) {
//        //           // Wait until the motor reaches the target position
//        //       }
//        telemetry.addData("targetPosition", targetPosition);
//        telemetry.addData("after while HSMotor.getCurrentPosition()",robot.HSMotor.getCurrentPosition());
//        telemetry.update();
//        robot.HSMotor.setPower(0);
//        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.HSMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//
//        move = false;
//    }
//
////End Definition and Initialization of Horizontal Slides
//
//
//
//    public void moveDriveTrain_FieldCentric() {
//        double y = gamepad1.left_stick_y * (1); // Remember, Y stick value is reversed
//        double x = -gamepad1.left_stick_x * (1);
//        double rx = -gamepad1.right_stick_x * (1); //*(0.5) is fine
//
//        // This button choice was made so that it is hard to hit on accident,
//        // it can be freely changed based on preference.
//        // The equivalent button is start on Xbox-style controllers.
//        //******************************************temp
////        if (gamepad1.back) {
////            robot.imu.resetYaw();
////        }
////******************************************temp
//
//        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//
//        robot.LFMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
//        robot.LBMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
//        robot.RFMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
//        robot.RBMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
//    }
//
//    public void moveDriveTrain_RobotCentric() {
//        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double robot_x = gamepad1.left_stick_x;
//        double robot_rx = gamepad1.right_stick_x*0.5; // If a smooth turn is required 0.5
//
//        double fl = robot_y - robot_x - robot_rx;
//        double bl = robot_y + robot_x - robot_rx;
//        double fr = robot_y + robot_x + robot_rx;
//        double br = robot_y - robot_x + robot_rx;
//
//        robot.LFMotor.setPower(fl * speedMultiplier);
//        robot.LBMotor.setPower(bl * speedMultiplier);
//        robot.RFMotor.setPower(fr * speedMultiplier);
//        robot.RBMotor.setPower(br * speedMultiplier);
//
//    }
//    public void RobotCentricDriveTrain () {
//        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double robot_x = gamepad1.left_stick_x;
//        double robot_rx = gamepad1.right_stick_x*0.5; // If a smooth turn is required 0.5
//
//        double fl = robot_y - robot_x - robot_rx;
//        double bl = robot_y + robot_x - robot_rx;
//        double fr = robot_y + robot_x + robot_rx;
//        double br = robot_y - robot_x + robot_rx;
//
//        robot.LFMotor.setPower(fl * speedMultiplier);
//        robot.LBMotor.setPower(bl * speedMultiplier);
//        robot.RFMotor.setPower(fr * speedMultiplier);
//        robot.RBMotor.setPower(br * speedMultiplier);
//
//    }
//
////Begin Definition and Initialization of Horizontal Slides
//
//
////    private void moveHSlideToPosition ( int targetPosition){
////
////        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        telemetry.addData("targetPosition", targetPosition);
////        telemetry.addData("liftMotorR.getCurrentPosition()",robot.HSMotor.getCurrentPosition());
////        telemetry.update();
////        robot.HSMotor.setTargetPosition(-targetPosition);
////        robot.HSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.HSMotor.setPower(+SLIDE_POWER_H);
////
////        move = true;
////
////        while (robot.HSMotor.isBusy() &&  move) {
////            // Wait until the motor reaches the target position
////        }
////
////        robot.HSMotor.setPower(0);
////        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.HSMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
////        move = false;
////    }
//
//
////End Definition and Initialization of Horizontal Slides
//
//
//
//
//}
//
//
//
//
//
//
//
//
