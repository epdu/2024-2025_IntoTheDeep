/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */
package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants_CS.OArmRearSpecimenPick;
import static org.firstinspires.ftc.teamcode.Constants_CS.OClawOpen;
import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_A_BOTTOM;
import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_Y_HIGH;
import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_Y_HIGHHH;
import static org.firstinspires.ftc.teamcode.Constants_CS.SLIDE_POWER_V;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import java.util.Locale;
import static org.firstinspires.ftc.teamcode.Constants_CS.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.notUsing.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import java.util.Locale;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import java.util.Locale;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */

@Autonomous(name="AAAAA Right 4 specimens 2 preloaded", group="Linear OpMode")
//@Disabled
//face to the bar right space for 4 of them only
// two preload and push two home
//faster version
public class SpecimenFourTwoPlusTwo extends LinearOpMode {


    public float DriveTrains_ReducePOWER=0.75f;
    //   DriveTrains_ReducePOWER = 0.75f;
//    DriveTrains_ReducePOWER = speedLimiterSlower;//************************
    HardwareTeletubbies robot = new HardwareTeletubbies();
    public String fieldOrRobotCentric = "robot";
    boolean move = false;
    boolean movementActive = false;

    private volatile boolean isRunning = true;
    ElapsedTime delayTimer = new ElapsedTime();
    // 在类顶部声明PID控制器
    // 状态变量
    private boolean pidActive = false; // PID 控制是否激活
    private int pidTargetPosition = 0; // PID 控制目标位置
    private PIDController pidController = new PIDController(0.005, 0.0000005, 0.0002);// (0.005, 0.0000005, 0.0002) good for target 300 (1.9, 0.014, 4.9)
    // Tune these values  POSITION_B_EXTRUDETransfer = 600;//horizontal slides  out //600 is too much
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    public double targetX =0.0;
    public double targetY =0.0;
    public double targetH =0.0;
    public double moveSpeed =0.0;
    public double targetmoveAccuracyX=0.0;
    public double targetmoveAccuracyY=0.0;
    public double targetangleAccuracy=0.0;
    public double targettimeoutS=0.0;

    //Starting location0
    public double GlobalX = 0;
    public double GlobalY = 0;
    public double GlobalH = 0;
    private boolean pidActiveVS = false; // PID 控制是否激活
    private int pidTargetPositionVS = 0; // PID 控制目标位置
    private PIDController pidControllerVS = new PIDController(0.005, 0.0000005, 0.0002);// (0.005, 0.0000005, 0.0002) good for target 300 (1.9, 0.014, 4.9)
    // Tune these values  POSITION_B_EXTRUDETransfer = 600;//horizontal slides  out //600 is too much

    // pid for HSlides
    int controlMode = 1;
    private boolean pidActiveHS = false; // PID 控制是否激活
    private int pidTargetPositionHS = 0; // PID 控制目标位置
    private PIDController pidControllerHS = new PIDController(0.005, 0.0000005, 0.0002);// (0.005, 0.0000005, 0.0002) good for target 300 (1.9, 0.014, 4.9)
    // Tune these values  POSITION_B_EXTRUDETransfer = 600;//horizontal slides  out //600 is too much


    private Pose2d beginPose = null;

    private PinpointDrive drive;     // 定义驱动系统
    Gyro gyro = new Gyro(); // 创建 Gyro 类的对象
    private ElapsedTime runtime = new ElapsedTime();
    public String armPositionCuzBorS ="NOLL"; //new variable for it and arm will go back of robo

    public static double lastOArmLPosition = 0.0;
    public static double lastOArmRPosition = 0.0;
    public int ii=0;
    //    verticalSlide verticalslide;
//    Intake intake;
//    Outtake outtake;
//    GoToHSlidePos goToHSlidePos;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        gyro.robot.init(hardwareMap);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        robot.odo.resetPosAndIMU();
        robot.odo.recalibrateIMU();
        robot.odo.resetPosAndIMU();
        Pose2D pos = robot.odo.getPosition();// Pose2D is for Gobilda, Pose2d is RR
        Pose2D currentPose = robot.odo.getPosition();
        String data = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("X Position", currentPose.getX(DistanceUnit.MM));
        telemetry.addData("Y Position", currentPose.getY(DistanceUnit.MM));
        telemetry.addData("Heading (rad)", currentPose.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        waitForStart();
////////////////////////////////////////////////////////////////////////////////////02142025
////////////KpX=0.0015; KpY=0.0037; for 0.7 power /////////////////////////////////////////////////////////////

///////////////////////////////////////////move robot close to chamber and adjust slides high///////////
// method one
//        startDriveMovement(610, 0, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.1);
//       //this one does not change distance but speed
//        startVSlideMovement(POSITION_Y_HIGH);
//        robot.OArmL.setPosition(0.97);//  good
//        robot.OArmR.setPosition(0.97);//
//        lastOArmLPosition= robot.OArmL.getPosition();
//        lastOArmRPosition= robot.OArmL.getPosition();
//        while (opModeIsActive()) {
//            updateVSlidePIDControl();
//            myGoToPosSingle(targetX, targetY, targetH, moveSpeed);
//
//            if (!pidActiveVS && !movementActive) {
//                break;
//            }
//        }
//        sleep(2000);
//
        // 0 80 130 180?
        //0 80 160 200
// method two
        startDriveMovement(810, 0, Math.toRadians(0), 0.7, 3, 3, Math.toRadians(1), 0.1);
        startVSlideMovement(POSITION_Y_HIGH);
        robot.OArmL.setPosition(0.97);//  good
        robot.OArmR.setPosition(0.97);//
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmL.getPosition();
        while (opModeIsActive()) {
            updateVSlidePIDControl(); // 更新滑轨位置
            if (movementActive) {
                myGoToPosSingle(targetX, targetY, targetH, moveSpeed); // 更新驱动位置
            }
            if (!movementActive && !pidActiveVS){
                break;
            }
        }
        myGoToPos(800, 0, Math.toRadians(0), 0.7, 3, 3, Math.toRadians(1), 0.2);
        //from 810 to 800  0.5- 0.3
        myGoToPos(820, 0, Math.toRadians(0), 0.4, 3, 3, Math.toRadians(1), 0.2);

//        myGoToPos(430, 0, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.2);
//        //430  530 now 630 change
//        myGoToPos(600, 0, Math.toRadians(0), 0.4, 3, 3, Math.toRadians(1), 0.2);
        // 600  700 the last myGoToPos control distance well
        // 0.3-- 0.2
        // increase power to 0.95 800-->750
        ////////////////////////////////////in front of chamber and ready to hang////////////////////////////
        goToVSlidePos(POSITION_Y_HIGHHH,0.4);
        robot.OClaw.setPosition(OClawOpen);
        goToHSlidePos(0,0.2);
        myGoToPos(570, 0, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.3);
        robot.OArmL.setPosition(OArmRearSpecimenPick);
        robot.OArmR.setPosition(OArmRearSpecimenPick);
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmR.getPosition();
        goToVSlidePos(POSITION_A_BOTTOM,0.8);

        //////////////////////////////finishing hanging the FIRST and reset claw and arms/////////////////////////////
        ///////FIRSTFIRSTFIRSTFIRSTFIRSTFIRSTFIRSTFIRSTFIRSTFIRSTFIRSTFIRSTFIRSTFIRST////////
////////////////////////////////////////navigate to the left specimen and push it to home////////////////////////////
        myGoToPos(500, -850, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 1.01);
        goToHSlidePos(0,0.1);
        myGoToPos(1200, -850, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 1.0);
        myGoToPos(1200, -1180, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.5);
        // time 0.3- 0.4 y 1170 -1195 back to 1175
        myGoToPos(250, -1180, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 1.3);
 //       goToHSlidePos(0,0.2);
        myGoToPos(250, -850, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.35);
        myGoToPos(1300, -1200, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 1.4);
        myGoToPos(1300, -1450, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.5);
        myGoToPos(250, -1450, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 1.25);
        myGoToPos(250, -800, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.3);
//        sleep(100);
        myGoToPos(-60, -800, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.8);
        // -69 -- -60
        sleep(150);
        robot.OClaw.setPosition(OClawSpecimenChambers);
        sleep(250);
        robot.OArmL.setPosition(0.97);//  good
        robot.OArmR.setPosition(0.97);//
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmL.getPosition();
        myGoToPos(200, -800, Math.toRadians(0), 0.85, 3, 3, Math.toRadians(1), 0.8);
        myGoToPos(0, 80, Math.toRadians(0), 0.85, 3, 3, Math.toRadians(1), 1.2);
////        ///////////////////////////move Synchronous both drive train and slides//////////////////////////
        startDriveMovement(620, 80, Math.toRadians(0), 0.7, 3, 3, Math.toRadians(1), 0.1);
        startVSlideMovement(POSITION_Y_HIGH);
        robot.OArmL.setPosition(0.97);//  good
        robot.OArmR.setPosition(0.97);//
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmL.getPosition();
        while (opModeIsActive()) {
            updateVSlidePIDControl(); // 更新滑轨位置
            if (movementActive) {
                myGoToPosSingle(targetX, targetY, targetH, moveSpeed); // 更新驱动位置
            }
            if (!movementActive && !pidActiveVS){
                break;
            }
        }
        myGoToPos(755, 80, Math.toRadians(0), 0.7, 3, 3, Math.toRadians(1), 0.2);
        myGoToPos(860, 80, Math.toRadians(0), 0.4, 3, 3, Math.toRadians(1), 0.2);
// 570 -- 820  620-620-820 no hard crash, but still move backward 12cm away 620-7400-820 no hard crash, but still move backward 4cm away
// 620-790-820 with crash, but still move backward 4cm away 620-780-860 good for previous run but not now
        //reduce it t obe 620-760-860
        //reduce it t obe 620-750-860
        //reduce it t obe 620-740-860
//        ////////////////////////////////////in front of chamber and ready to hang////////////////////////////
        goToVSlidePos(POSITION_Y_HIGHHH,0.4);
        robot.OClaw.setPosition(OClawOpen);
        goToHSlidePos(0,0.1);
        myGoToPos(570, 80, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.5);
        robot.OArmL.setPosition(OArmRearSpecimenPick);
        robot.OArmR.setPosition(OArmRearSpecimenPick);
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmL.getPosition();
        goToVSlidePos(POSITION_A_BOTTOM,0.8);

////////////////////////////////////finishing hanging SECOND and reset claw and arms/////////////////////////////
        ///////////SECOND SECOND SECOND SECOND SECOND SECOND SECOND SECOND SECOND SECOND SECOND SECOND SECOND SECOND SECOND ///
        myGoToPos(200, -900, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 1.19);
//        sleep(300);
        myGoToPos(-60, -900, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.55);
        // -69 -- -60
        sleep(150);
        robot.OClaw.setPosition(OClawSpecimenChambers);
        sleep(250);
        robot.OArmL.setPosition(0.97);//  good
        robot.OArmR.setPosition(0.97);//
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmL.getPosition();
        myGoToPos(200, -900, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.5);
        myGoToPos(0, 160, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 1.19);
////        ///////////////////////////move Synchronous both drive train and slides//////////////////////////
        startDriveMovement(620, 160, Math.toRadians(0), 0.7, 3, 3, Math.toRadians(1), 0.1);
        startVSlideMovement(POSITION_Y_HIGH);
        robot.OArmL.setPosition(0.97);//  good
        robot.OArmR.setPosition(0.97);//
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmL.getPosition();
        while (opModeIsActive()) {
            updateVSlidePIDControl(); // 更新滑轨位置
            if (movementActive) {
                myGoToPosSingle(targetX, targetY, targetH, moveSpeed); // 更新驱动位置
            }
            if (!movementActive && !pidActiveVS){
                break;
            }
        }
        myGoToPos(750, 160, Math.toRadians(0), 0.7, 3, 3, Math.toRadians(1), 0.2);
        myGoToPos(860, 160, Math.toRadians(0), 0.4, 3, 3, Math.toRadians(1), 0.2);
// 570 -- 820  620-620-820 no hard crash, but still move backward 12cm away 620-7400-820 no hard crash, but still move backward 4cm away
// 620-790-820 with crash, but still move backward 4cm away 620-750-860 ?with
//        ////////////////////////////////////in front of chamber and ready to hang////////////////////////////
        goToVSlidePos(POSITION_Y_HIGHHH,0.3);
        robot.OClaw.setPosition(OClawOpen);
        goToHSlidePos(0,0.1);
        robot.OArmL.setPosition(OArmRearSpecimenPick);
        robot.OArmR.setPosition(OArmRearSpecimenPick);
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmL.getPosition();
        goToVSlidePos(POSITION_A_BOTTOM,0.8);
        myGoToPos(500, 160, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.5);
//        myGoToPos(570, 250, Math.toRadians(0), 0.7, 3, 3, Math.toRadians(1), 0.8);
////////////////////////////////////finishing hanging THIRD and reset claw and arms/////////////////////////////
        ///////////////// THIRD   THIRD  THIRD  THIRD  THIRD  THIRD  THIRD  THIRD
        myGoToPos(200, -900, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 1.19);
        myGoToPos(-60, -900, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.55);
        // -69 -- -60
        sleep(150);
        robot.OClaw.setPosition(OClawSpecimenChambers);
        sleep(250);
        robot.OArmL.setPosition(0.97);//  good
        robot.OArmR.setPosition(0.97);//
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmL.getPosition();
        myGoToPos(200, -900, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.55);
        myGoToPos(0, 200, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 1.25);
////        ///////////////////////////move Synchronous both drive train and slides//////////////////////////
        startDriveMovement(620, 200, Math.toRadians(0), 0.7, 3, 3, Math.toRadians(1), 0.1);
        startVSlideMovement(POSITION_Y_HIGH);
        robot.OArmL.setPosition(0.97);//  good
        robot.OArmR.setPosition(0.97);//
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmL.getPosition();
        while (opModeIsActive()) {
            updateVSlidePIDControl(); // 更新滑轨位置
            if (movementActive) {
                myGoToPosSingle(targetX, targetY, targetH, moveSpeed); // 更新驱动位置
            }
            if (!movementActive && !pidActiveVS){
                break;
            }
        }
        myGoToPos(770, 200, Math.toRadians(0), 0.7, 3, 3, Math.toRadians(1), 0.3);
 //760 good
        myGoToPos(890, 200, Math.toRadians(0), 0.4, 3, 3, Math.toRadians(1), 0.3);
// 570 -- 820  620-620-820 no hard crash, but still move backward 12cm away 620-7400-820 no hard crash, but still move backward 4cm away
// 620-790-820 with crash, but still move backward 4cm away 620-750-860 ?with
//        ////////////////////////////////////in front of chamber and ready to hang////////////////////////////
        goToVSlidePos(POSITION_Y_HIGHHH,0.4);
//        sleep(100);
        robot.OClaw.setPosition(OClawOpen);
//        goToHSlidePos(0,0.2);
        robot.OArmL.setPosition(0.46);
        robot.OArmR.setPosition(0.46);
        lastOArmLPosition= robot.OArmL.getPosition();
        lastOArmRPosition= robot.OArmL.getPosition();
//        robot.OArmL.setPosition(OArmRearSpecimenPick);
//        robot.OArmR.setPosition(OArmRearSpecimenPick);
//        goToHSlidePos(0,0.1);
        goToVSlidePos(POSITION_A_BOTTOM,0.8);
//        myGoToPos(570, 200, Math.toRadians(0), 0.95, 3, 3, Math.toRadians(1), 0.5);
//        myGoToPos(570, 300, Math.toRadians(0), 0.7, 3, 3, Math.toRadians(1), 0.8);
        sleep(1000);

/////////////////////////////////////////////////////////////////////////////////////02142025/
// //////////////////////////////////finishing hanging FOUR and reset claw and arms/////////////////////////////

///////////////////////try speend 0.8///////////////




    }


    ///////////////////////////////////////
    public void startVSlideMovement(int targetPosition) {
        startVSlidePIDControl(targetPosition); // 启动PID
    }
    public void startDriveMovement(double x, double y, double h, double speed, double moveAccuracyX, double moveAccuracyY, double angleAccuracy, double timeoutS)
    {
        this.targetX = x;
        this.targetY = y;
        this.targetH = h;
        this.moveSpeed = speed;
        this.targetmoveAccuracyX=moveAccuracyX;
        this.targetmoveAccuracyY=moveAccuracyY;
        this.targetangleAccuracy=angleAccuracy;
        this.targettimeoutS=timeoutS;
        this.movementActive = true;
//    myGoToPos(targetX, targetY,targetH,moveSpeed,targetmoveAccuracyX,targetmoveAccuracyY,targetangleAccuracy,targettimeoutS);

    }

    public void goToVSlidePos(int targetPosition, double timeoutS) {
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // 初始化 PID 控制器
        pidControllerVS.reset();
        pidControllerVS.enable();
        pidControllerVS.setSetpoint(targetPosition);
        pidControllerVS.setTolerance(10); // 允许误差范围
        pidActiveVS = true; // 激活 PID 控制

        runtime.reset(); // 重置计时器

        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            if (!pidActiveVS) return;
            int currentPositionL = robot.VSMotorL.getCurrentPosition();

            // 计算 PID 输出
            double powerL = pidControllerVS.performPID(currentPositionL);

            // 控制电机功率
            robot.VSMotorL.setPower(Range.clip(powerL * 0.9, -1.0, 1.0));
            robot.VSMotorR.setPower(Range.clip(powerL * 0.9, -1.0, 1.0));

            // 输出 Telemetry 信息
            telemetry.addData("PID Target", targetPosition);
            telemetry.addData("Current Position VL", currentPositionL);
            telemetry.addData("Power V", powerL);
            telemetry.update();

            // 检查是否达到目标位置
            if (!pidActiveVS && Math.abs(robot.VSMotorL.getCurrentPosition() - pidTargetPositionVS) > 10) {
                double holdPowerVS = pidControllerVS.performPID(robot.VSMotorL.getCurrentPosition());
                robot.VSMotorL.setPower(holdPowerVS);
                robot.VSMotorR.setPower(holdPowerVS);
                pidActiveVS = false; // 停止 PID 控制
                break;
            }

        }

        robot.VSMotorL.setPower(0.0);
        robot.VSMotorR.setPower(0.0); // 防止震荡时继续保持微功率
    }

    //////////////////////////////////////////////////////////////
    /////////////////////////////////////
    //////////////////////////////////////////////////////////////
    public void myGoToPos(double x, double y, double h, double speed, double moveAccuracyX, double moveAccuracyY, double angleAccuracy, double timeoutS) {
        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)
        integralSum = 0;
        integralSumX = 0;
        integralSumY = 0;
        refresh();
        feedfowardX = -x + GlobalX;
        feedfowardY = -y + GlobalY;
        feedfoward = -h + GlobalH;

        double distanceToTarget = Math.hypot(-x + GlobalX, -y + GlobalY);
        double absoluteTurnAngle = Math.atan2(-y + GlobalY, -x + GlobalX);
        double relativeAngleToTarget = angleWrapRad(-absoluteTurnAngle + GlobalH);//////////////////??????????????
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(-h + GlobalH);
        double correctFactor = correctFactorCoeff;
        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor * Math.abs(relativeTurnAngle);
        double initialSpeed = 0.2;
        double movementXpower = initialSpeed * relativeXToTarget / maxPower;
        double movementYpower = initialSpeed * relativeYToTarget / maxPower;
        double movementTurnPower = initialSpeed * correctFactor * relativeTurnAngle / maxPower;

        runtime.reset();
        robot.LFMotor.setPower(Range.clip(movementXpower - movementYpower - movementTurnPower, -initialSpeed, initialSpeed));
        robot.LBMotor.setPower(Range.clip(movementXpower + movementYpower - movementTurnPower, -initialSpeed, initialSpeed));
        robot.RFMotor.setPower(Range.clip(movementXpower + movementYpower + movementTurnPower, -initialSpeed, initialSpeed));
        robot.RBMotor.setPower(Range.clip(movementXpower - movementYpower + movementTurnPower, -initialSpeed, initialSpeed));
//        sleep(5);
        initialDistanceToTarget = distanceToTarget;
        while (((Math.abs(-x + GlobalX) > moveAccuracyX || Math.abs(-y + GlobalY) > moveAccuracyY || Math.abs(angleWrapRad(-h + GlobalH)) > angleAccuracy)) && opModeIsActive() && (runtime.seconds() < timeoutS)) {
            // while(true){

            myGoToPosSingle(x, y, h, speed);

//            Pose2D pos = odo.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);
//            telemetry.update();


        }
    }

    ///////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////
    public void myGoToPosSingle(double x, double y, double h, double speed){

        refresh();
        //math to calculate distances to the target
        double distanceToTarget = Math.hypot(-x + GlobalX, -y + GlobalY);
        double absoluteTurnAngle = Math.atan2(-y + GlobalY, -x+ GlobalX);
        double relativeAngleToTarget = angleWrapRad(-absoluteTurnAngle + GlobalH);// changed both to be --
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(-h+GlobalH);

        double correctFactor = correctFactorCoeff;
        if (initialDistanceToTarget>1200) { correctFactor = 3.5*correctFactorCoeff;}
        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor*Math.abs(relativeTurnAngle) ;

//        double movementXpower = relativeXToTarget / maxPower * speed;
//        double movementYpower = relativeYToTarget / maxPower * speed;

        double PIDX = myPIDControlX(x, GlobalX)*Math.signum(Math.cos(GlobalH));
        double PIDY = myPIDControlY(y, GlobalY)*Math.signum(Math.cos(GlobalH));
        double PIDH = myPIDControlH(h, GlobalH);
        double movementXpower = PIDX * speed * (Math.abs(relativeXToTarget)/maxPower) ;
        double movementYpower = PIDY * speed * (Math.abs(relativeYToTarget)/maxPower);
        double movementTurnPower = PIDH * speed * (correctFactor*Math.abs(relativeTurnAngle)/maxPower);




        robot.LFMotor.setPower(Range.clip(movementXpower - movementYpower - movementTurnPower, -speed, speed));
        robot.LBMotor.setPower(Range.clip(movementXpower + movementYpower - movementTurnPower, -speed, speed));
        robot.RFMotor.setPower(Range.clip(movementXpower + movementYpower + movementTurnPower, -speed, speed));
        robot.RBMotor.setPower(Range.clip(movementXpower - movementYpower + movementTurnPower, -speed, speed));


        telemetry.addData("distanceToTarget", distanceToTarget);
        telemetry.addData("movementXpower", movementXpower);
        telemetry.addData("movementYpower", movementYpower);
        telemetry.addData("movementTurnPower", movementTurnPower);
        telemetry.addData("relativeYToTarget", relativeYToTarget);
        telemetry.addData("absoluteAngleToTarget", absoluteTurnAngle);
        telemetry.addData("relativeAngleToTarget", relativeAngleToTarget);
        telemetry.addData("GlobalX", GlobalX);
        telemetry.addData("GlobalY", GlobalY);
        telemetry.addData("GlobalH", Math.toDegrees(GlobalH));
        telemetry.update();
        if (((Math.abs(-x + GlobalX) <= targetmoveAccuracyX && Math.abs(-y + GlobalY) <= targetmoveAccuracyY && Math.abs(angleWrapRad(-h + GlobalH)) <= targetangleAccuracy)) || (runtime.seconds() >= targettimeoutS))
        {
            movementActive = false;
        }
    }



    public void refresh(){
        robot.odo.update();
        Pose2D pos = robot.odo.getPosition();
        GlobalX = pos.getX(DistanceUnit.MM);
        GlobalY = pos.getY(DistanceUnit.MM);
        GlobalH = pos.getHeading(AngleUnit.RADIANS);
    }

    ///////////////////////////
    double integralSum = 0;
    double feedfoward = 0;
    double Kp = 1.313; //1.213-1.3213-1.4213-1.5213 good    0.99    0.59 for 2102025 0.6
    double Ki = 0.01;// 0.028   0.2 for2102025 0.32
    double Kd = 0.0000217;// 0.017 for 21020250.17
    double Kf = 0.025;//0.25
    private double lastError = 0;
    double integralSumX = 0;
    double KpX=0.0019; //power 0.7,,,,,, KpX=0.005; 0.0045 0.0037 0.003 0.0023 0.002 0.0017  over shoot, 0.001 too less
    double KiX=0.0000005;   //Kxp/KYp ratio is affected by the robot weight balance
    double KdX=0.0002;// KXf/KYf ratio is affected by the robot weight balance
    double feedfowardX = 0;
    private double lastErrorX = 0;

    double integralSumY = 0;
    double KpY=0.0028; // power 0.7,,,,,, 0.0035 a little bit less 0.0037 A LITTLE BIT MORE
    double KiY=0.000000005;   //Kxp/KYp ratio is affected by the robot weight balance
    double KdY=0.000002;// KXf/KYf ratio is affected by the robot weight balance
    double feedfowardY = 0;
    private double lastErrorY = 0;


    //////////////////////////
    /*


    double integralSum = 0;
    double feedfoward = 0;
    double Kp = 0.6;
    double Ki = 0.32;
    double Kd = 0.17;
    double Kf = 0.25;
    private double lastError = 0;



    double integralSumX = 0;
    double KpX=0.04;
    double KiX=0.002;   //Kxp/KYp ratio is affected by the robot weight balance
    double KdX=0.008;// KXf/KYf ratio is affected by the robot weight balance
    double feedfowardX = 0;
    private double lastErrorX = 0;

    double integralSumY = 0;
    double KpY=0.04;
    double KiY=0.002;   //Kxp/KYp ratio is affected by the robot weight balance
    double KdY=0.008;// KXf/KYf ratio is affected by the robot weight balance
    double feedfowardY = 0;
    private double lastErrorY = 0;
    */

    double correctFactorCoeff = 300;
    double initialDistanceToTarget = 0;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timerX = new ElapsedTime();
    ElapsedTime timerY = new ElapsedTime();
    public double myPIDControlX(double reference, double state) {
        double error = -reference + state;
        integralSumX += error*timerX.seconds();
        double derivative = (error - lastErrorX) / (timerX.seconds());
        lastErrorX = error;
        timerX.reset();
        double output = (error*KpX) + (derivative*KdX) + (integralSumX*KiX);
        telemetry.addData("targetX", targetX);
        telemetry.addData("outputX", output);
        telemetry.addData("errorX", error);
        telemetry.update();
        return output;
    }

    public double myPIDControlY(double reference, double state) {
        double error = -reference + state;
        integralSumY += error*timerY.seconds();
        double derivative = (error - lastErrorY) / (timerY.seconds());
        lastErrorY = error;
        timerY.reset();
        double output = (error*KpY) + (derivative*KdY) + (integralSumY*KiY);
//        telemetry.addData("reference", reference);
        telemetry.addData("targetY", targetY);
        telemetry.addData("outputY", output);
        telemetry.addData("errorY", error);
        telemetry.update();
        return output;
    }
    /////////////////////tuning PID/////////////////////////
    public double myPIDControlH(double reference, double state) {
        double error = angleWrapRad(-reference + state);
        integralSum += error*timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error*Kp) + (derivative*Kd) + (integralSum*Ki) + (feedfoward*Kf);
        telemetry.addData("targetH", targetH);
        telemetry.addData("outputH", output);
        telemetry.addData("errorH", error);
        telemetry.update();
        return output;
    }

    ////////////////////////////////////////////////////////////

    private void moveVSlideToPosition ( int targetPosition){
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("liftMotorL.getCurrentPosition()",robot.VSMotorL.getCurrentPosition());
//        telemetry.addData("liftMotorR.getCurrentPosition()",robot.VSMotorR.getCurrentPosition());
        telemetry.update();
        robot.VSMotorL.setTargetPosition(-targetPosition);
//        robot.VSMotorR.setTargetPosition(-targetPosition);
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.VSMotorL.setPower(+SLIDE_POWER_V);
        robot.VSMotorR.setPower(+SLIDE_POWER_V);
        move = true;
        while (robot.VSMotorL.isBusy() && move) {
            // Wait until the motor reaches the target position
        }
//        while (robot.VSMotorR.isBusy() && move) {
        //           // Wait until the motor reaches the target position
        //       }
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("after while liftMotorL.getCurrentPosition()",robot.VSMotorL.getCurrentPosition());
        telemetry.addData("after while liftMotorR.getCurrentPosition()",robot.VSMotorR.getCurrentPosition());
        telemetry.update();

        robot.VSMotorL.setPower(0);
        robot.VSMotorR.setPower(0);
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.VSMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        robot.VSMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        move = false;
    }
    public void moveDriveTrain(double x, double y, double h, double speed, double moveAccuracyX, double moveAccuracyY, double angleAccuracy, double timeoutS){
        myGoToPos(x, y, h,speed,moveAccuracyX,moveAccuracyY,angleAccuracy,timeoutS);
    }
    public void goToHSlidePos(int targetPosition, double timeoutS) {
        // 初始化 PID 控制器
        pidControllerHS.reset();
        pidControllerHS.enable();
        pidControllerHS.setSetpoint(targetPosition);
        pidControllerHS.setTolerance(10); // 允许误差范围
        pidActiveHS = true; // 激活 PID 控制
        runtime.reset(); // 重置计时器

        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            if (!pidActiveHS) return;
            int currentPositionH = robot.HSMotor.getCurrentPosition();

            // 计算 PID 输出
            double powerH = pidControllerHS.performPID(currentPositionH);

            // 控制电机功率
            robot.HSMotor.setPower(Range.clip(powerH * 0.6, -1.0, 1.0));

            // 输出 Telemetry 信息
            telemetry.addData("PID Target", targetPosition);
            telemetry.addData("Current Position H", currentPositionH);
            telemetry.addData("Power H", powerH);
            telemetry.update();

            // 检查是否达到目标位置
            if (!pidActiveHS && Math.abs(robot.HSMotor.getCurrentPosition() - pidTargetPositionHS) > 10) {
                robot.HSMotor.setPower(0.1); // 保持抗重力的微小功率
                pidActiveHS = false; // 停止 PID 控制
                break;
            }
        }

        robot.HSMotor.setPower(0); // 防止震荡时继续保持微功率
    }


    ///////////////////////////
    private void holdHSlidePosition(int targetPosition) {
        final double HOLD_POWER = 0.1; // Minimal power to hold the position
        final int POSITION_TOLERANCE = 10; // Allowable deviation from the target

        while (true) {
            int currentPositionH = robot.HSMotor.getCurrentPosition();
            // Check if the slide is within the tolerance
            if (Math.abs(currentPositionH + targetPosition) <= POSITION_TOLERANCE) {
                robot.HSMotor.setPower(0);
            } else {
                // Apply minimal power to correct the position
                robot.HSMotor.setPower(HOLD_POWER);
            }
            // Optionally break the loop based on a condition or timer
            // Example: break if a stop flag is set
            if (!move) {
                break;
            }
            // Add telemetry to monitor holding behavior
            telemetry.addData("Holding Position H", currentPositionH);
            telemetry.update();
        }
    }




    ///////////startVSlidePIDControl///////////////

    /// 初始化 PID 控制器
    private void startVSlidePIDControl(int targetPosition) {
        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        int currentPositionL = robot.VSMotorL.getCurrentPosition();

        // 计算 PID 输出
        double powerL = pidControllerVS.performPID(currentPositionL);
        robot.VSMotorL.setPower(powerL*0.8); // change it to make it move faster both at the same time
        robot.VSMotorR.setPower(powerL*0.8); // change it to make it move faster

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
        if (pidActiveVS && Math.abs(robot.VSMotorL.getCurrentPosition() - pidTargetPositionVS) < 10) {
            double holdPowerVS = pidControllerVS.performPID(robot.VSMotorL.getCurrentPosition());
            robot.VSMotorL.setPower(holdPowerVS);
            robot.VSMotorR.setPower(holdPowerVS);
            pidActiveVS = false; // 停止 PID 控制
        }

    }

//////////////startVSlidePIDControl/////////////


///////////startHSlidePIDControl///////////////

    /// 初始化 PID 控制器
    private void startHSlidePIDControl(int targetPosition) {
        robot.HSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidControllerHS.reset();
        pidControllerHS.enable();
        pidControllerHS.setSetpoint(targetPosition);
        pidControllerHS.setTolerance(10); // 允许误差范围
        pidTargetPositionHS = targetPosition;
        pidActiveHS = true; // 激活 PID 控制
    }
    // 在主循环中调用的非阻塞 PID 控制逻辑
    private void updateHSlidePIDControl() {
        if (!pidActiveHS) return; // 如果 PID 未激活，直接返回

        int currentPositionH = robot.HSMotor.getCurrentPosition();

        // 计算 PID 输出
        double powerH = pidControllerHS.performPID(currentPositionH);
        robot.HSMotor.setPower(powerH*0.6);


        // 输出 Telemetry 信息
        telemetry.addData("PID Target", pidTargetPositionHS);
        telemetry.addData("Current Position H", currentPositionH);
        telemetry.addData("Power H", powerH);
        telemetry.update();

//        // 如果达到目标位置，停止滑轨运动，但保持抗重力功率
//        if (pidControllerHS.onTarget()) {
//            robot.VSMotorL.setPower(0.1); // 保持位置的最小功率
//            robot.VSMotorR.setPower(0.1);
//            pidActiveHS = false; // 停止 PID 控制
//        }
        // 在 updateVSlidePIDControl 中加入抗重力逻辑
        if (!pidActiveHS && Math.abs(robot.HSMotor.getCurrentPosition() - pidTargetPositionHS) > 10) {
            double holdPowerHS = pidControllerHS.performPID(robot.HSMotor.getCurrentPosition());
            robot.HSMotor.setPower(holdPowerHS);
            pidActiveHS = false; // 停止 PID 控制
        }


    }
//////////////startHSlidePIDControl/////////////


    public void goToPos(double x, double y, double h, double speed, double moveAccuracyX, double moveAccuracyY, double angleAccuracy, double timeoutS) {
        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)
        integralSum = 0;
        integralSumX = 0;
        integralSumY = 0;
        refresh();
        feedfowardX = x - GlobalX;
        feedfowardY = y - GlobalY;
        feedfoward = h - GlobalH;

        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteTurnAngle = Math.atan2(y - GlobalY, x - GlobalX);
        double relativeAngleToTarget = angleWrapRad(absoluteTurnAngle - GlobalH);
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(h - GlobalH);
        double correctFactor = correctFactorCoeff;
        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor * Math.abs(relativeTurnAngle);
        double initialSpeed = 0.2;
        double movementXpower = initialSpeed * relativeXToTarget / maxPower;
        double movementYpower = initialSpeed * relativeYToTarget / maxPower;
        double movementTurnPower = initialSpeed * correctFactor * relativeTurnAngle / maxPower;

        runtime.reset();
        robot.LFMotor.setPower(Range.clip(movementXpower - movementYpower - movementTurnPower, -initialSpeed, initialSpeed));
        robot.LBMotor.setPower(Range.clip(movementXpower + movementYpower + movementTurnPower, -initialSpeed, initialSpeed));
        robot.RFMotor.setPower(Range.clip(movementXpower + movementYpower - movementTurnPower, -initialSpeed, initialSpeed));
        robot.RBMotor.setPower(Range.clip(movementXpower - movementYpower + movementTurnPower, -initialSpeed, initialSpeed));
//        sleep(5);
        initialDistanceToTarget = distanceToTarget;
        while (((Math.abs(x - GlobalX) > moveAccuracyX || Math.abs(y - GlobalY) > moveAccuracyY || Math.abs(angleWrapRad(h - GlobalH)) > angleAccuracy)) && opModeIsActive() && (runtime.seconds() < timeoutS)) {
            // while(true){

            goToPosSingle(x, y, h, speed);

//            Pose2D pos = odo.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);
//            telemetry.update();


        }
    }


    public void goToPosSingle(double x, double y, double h, double speed){

        refresh();
        //math to calculate distances to the target
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteTurnAngle = Math.atan2(y - GlobalY, x- GlobalX);
        double relativeAngleToTarget = angleWrapRad(absoluteTurnAngle - GlobalH);
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(h-GlobalH);

        double correctFactor = correctFactorCoeff;
        if (initialDistanceToTarget>1200) { correctFactor = 3.5*correctFactorCoeff;}
        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor*Math.abs(relativeTurnAngle) ;

//        double movementXpower = relativeXToTarget / maxPower * speed;
//        double movementYpower = relativeYToTarget / maxPower * speed;

        double PIDX = PIDControlX(x, GlobalX)*Math.signum(Math.cos(GlobalH));
        double PIDY = PIDControlY(y, GlobalY)*Math.signum(Math.cos(GlobalH));
        double PIDH = PIDControlH(h, GlobalH);
        double movementXpower = PIDX * speed * (Math.abs(relativeXToTarget)/maxPower) ;
        double movementYpower = PIDY * speed * (Math.abs(relativeYToTarget)/maxPower);
        double movementTurnPower = PIDH * speed * (correctFactor*Math.abs(relativeTurnAngle)/maxPower);


        telemetry.addData("distanceToTarget", distanceToTarget);
        telemetry.addData("movementXpower", movementXpower);
        telemetry.addData("movementYpower", movementYpower);
        telemetry.addData("movementTurnPower", movementTurnPower);
        telemetry.addData("relativeYToTarget", relativeYToTarget);
        telemetry.addData("absoluteAngleToTarget", absoluteTurnAngle);
        telemetry.addData("relativeAngleToTarget", relativeAngleToTarget);
        telemetry.addData("GlobalX", GlobalX);
        telemetry.addData("GlobalY", GlobalY);
        telemetry.addData("GlobalH", Math.toDegrees(GlobalH));
        telemetry.update();

        robot.LFMotor.setPower(Range.clip(movementXpower - movementYpower - movementTurnPower, -speed, speed));
        robot.LBMotor.setPower(Range.clip(movementXpower + movementYpower + movementTurnPower, -speed, speed));
        robot.RFMotor.setPower(Range.clip(movementXpower + movementYpower - movementTurnPower, -speed, speed));
        robot.RBMotor.setPower(Range.clip(movementXpower - movementYpower + movementTurnPower, -speed, speed));

    }

    public void moveForward(double x, double speed) {
        while((x - GlobalH) > Math.toRadians(5)) {
            robot.LFMotor.setPower(-speed);
            robot.LBMotor.setPower(-speed);
            robot.RFMotor.setPower(speed);
            robot.RBMotor.setPower(speed);
            refresh();
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("GlobalH", GlobalH);
            telemetry.update();
        }
        robot.LFMotor.setPower(0);
        robot.LBMotor.setPower(0);
        robot.RFMotor.setPower(0);
        robot.RBMotor.setPower(0);
    }

    // used to mantain angle values between Pi and -Pi
    public double angleWrapRad(double angle)
    {
        while (angle > Math.PI)
        {
            angle -= Math.PI * 2;
        }
        while (angle < -Math.PI)
        {
            angle += Math.PI * 2;
        }

        return angle;
    }

    public double PIDControlH(double reference, double state) {
        double error = angleWrapRad(reference - state);
        integralSum += error*timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error*Kp) + (derivative*Kd) + (integralSum*Ki) + (feedfoward*Kf);
        return output;
    }

    public double PIDControlX(double reference, double state) {
        double error = reference - state;
        integralSumX += error*timerX.seconds();
        double derivative = (error - lastErrorX) / (timerX.seconds());
        lastErrorX = error;
        timerX.reset();
        double output = (error*KpX) + (derivative*KdX) + (integralSumX*KiX);
        return output;
    }

    public double PIDControlY(double reference, double state) {
        double error = reference - state;
        integralSumY += error*timerY.seconds();
        double derivative = (error - lastErrorY) / (timerY.seconds());
        lastErrorY = error;
        timerY.reset();
        double output = (error*KpY) + (derivative*KdY) + (integralSumY*KiY);
        return output;
    }

    public void goToTest(double x, double y, double h, double speed, double moveAccuracyX, double moveAccuracyY, double angleAccuracy, double timeoutS) {
        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)
        integralSum = 0;
        integralSumX = 0;
        integralSumY = 0;
        refresh();
        feedfowardX = x - GlobalX;
        feedfowardY = y - GlobalY;
        feedfoward = h - GlobalH;

        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteTurnAngle = Math.atan2(y - GlobalY, x - GlobalX);
        double relativeAngleToTarget = angleWrapRad(absoluteTurnAngle - GlobalH);
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(h - GlobalH);
        double correctFactor =  correctFactorCoeff;
        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor * Math.abs(relativeTurnAngle);
//        double initialSpeed=0.2;
        double movementXpower = relativeXToTarget / maxPower ;
        double movementYpower = relativeYToTarget / maxPower ;
        double movementTurnPower = correctFactor * relativeTurnAngle / maxPower;

        runtime.reset();
        robot.LFMotor.setPower(movementXpower - movementYpower - movementTurnPower);
        robot.LBMotor.setPower(movementXpower + movementYpower + movementTurnPower);
        robot.RFMotor.setPower(movementXpower + movementYpower - movementTurnPower);
        robot.RBMotor.setPower(movementXpower - movementYpower + movementTurnPower);
//        sleep(5);
        initialDistanceToTarget = distanceToTarget;
        while (((Math.abs(x - GlobalX) > moveAccuracyX || Math.abs(y - GlobalY) > moveAccuracyY || Math.abs(angleWrapRad(h - GlobalH)) > angleAccuracy)) && opModeIsActive() && (runtime.seconds() < timeoutS)){
            // while(true){

            goToPosSingle(x, y, h, speed);

//            Pose2D pos = odo.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);
//            telemetry.update();


        }

        //stop all movement at the end of while loop
    }

    public void goToPosShortDis(double x, double y, double h, double speed, double moveAccuracyX, double moveAccuracyY, double angleAccuracy){

        refresh();
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteTurnAngle = Math.atan2(y - GlobalY, x - GlobalX);
        double relativeAngleToTarget = angleWrapRad(absoluteTurnAngle - GlobalH);
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(h - GlobalH);

        double correctFactor=300;
        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor * Math.abs(relativeTurnAngle);

        double movementXpower = speed * relativeXToTarget / maxPower ;
        double movementYpower = speed * relativeYToTarget / maxPower ;
        double movementTurnPower = speed * correctFactor * relativeTurnAngle / maxPower;

        while (Math.abs(x - GlobalX) > moveAccuracyX || Math.abs(y - GlobalY) > moveAccuracyY || Math.abs(angleWrapRad(h - GlobalH)) > angleAccuracy) {
            // while(true){

            robot.LFMotor.setPower(Range.clip(movementXpower - movementYpower - movementTurnPower, -speed, speed));
            robot.LBMotor.setPower(Range.clip(movementXpower + movementYpower + movementTurnPower, -speed, speed));
            robot.RFMotor.setPower(Range.clip(movementXpower + movementYpower - movementTurnPower, -speed, speed));
            robot.RBMotor.setPower(Range.clip(movementXpower - movementYpower + movementTurnPower, -speed, speed));

            telemetry.addData("movementXpower", movementXpower);
            telemetry.addData("movementYpower", movementYpower);
            telemetry.addData("movementTurnPower", movementTurnPower);
            telemetry.addData("relativeYToTarget", relativeYToTarget);
            telemetry.addData("sign:", Math.signum(Math.cos(GlobalH)));
            telemetry.addData("maxPower", maxPower);
            telemetry.addData("GlobalX", GlobalX);
            telemetry.addData("GlobalY", GlobalY);
            telemetry.addData("GlobalH", Math.toDegrees(GlobalH));
            telemetry.update();
            refresh();
        }
    }

    public void goToPosStop (){
        robot.LFMotor.setPower(0);
        robot.LBMotor.setPower(0);
        robot.RFMotor.setPower(0);
        robot.RBMotor.setPower(0);
    }
    /*
    public void goToStart(double x, double y, double h, double speed, int sleep_time){

        refresh();
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteTurnAngle = Math.atan2(y - GlobalY, x - GlobalX);
        double relativeAngleToTarget = angleWrapRad(absoluteTurnAngle - GlobalH);
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(h - GlobalH);

        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor * Math.abs(relativeTurnAngle);

        double movementXpower = speed * relativeXToTarget / maxPower ;
        double movementYpower = speed * relativeYToTarget / maxPower ;
        double movementTurnPower = speed * correctFactor * relativeTurnAngle / maxPower;

            FLMotor.setPower(Range.clip(movementXpower - movementYpower - movementTurnPower, -speed, speed));
            FRMotor.setPower(Range.clip(movementXpower + movementYpower + movementTurnPower, -speed, speed));
            BLMotor.setPower(Range.clip(movementXpower + movementYpower - movementTurnPower, -speed, speed));
            BRMotor.setPower(Range.clip(movementXpower - movementYpower + movementTurnPower, -speed, speed));
        sleep(sleep_time);
    }
*/
//    public void makeDroppieWork(int position){
//        droppie.setTargetPosition(position); //-1400
//        droppie.setPower(-0.9);
//        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

//    public void makeIntakieWork(int pos){
//        intakie.setTargetPosition(pos);//800
//        intakie.setPower(1);//0.8);
//        intakie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

//    public void makeBobbyWork(double power){
//        bobby.setPower(power);//-0.6
//    }
//
//    public void makeFlipityWork(double pos){
//        flipity.setPosition(pos);//0.8387);
//    }
//
//    public void makeFlopityWork(double pos){
//        flopity.setPosition(pos);//0.8387);
//    }
//
//    public void makeIndulgeyWork(double power){
//        indulgey.setPower(power);
//    }
}


// vertical distance 43 inches 109.22 cm - 1092.2 mm
// horizontal distance  odometer at 26.5 inches 67.31 cm - 673.1 mm
//          start of wheel 31 inches 78.74 - 787.4 mm
//Angle 180 degrees - pi