//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.google.android.gms.gcm.Task;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
//
//public class verticalSlide Task{
//    private boolean taskActive = false;
//    private int targetPosition;
//    private static final double SLIDE_POWER = 0.8;
//    HardwareTeletubbies robot = new HardwareTeletubbies();
//    public String fieldOrRobotCentric = "robot";
//    boolean move = false;
//    boolean movementActive = false;
//   private volatile boolean isRunning = true;
//    ElapsedTime delayTimer = new ElapsedTime();
//    // 在类顶部声明PID控制器
//    // 状态变量
//    private boolean pidActiveVS = false; // PID 控制是否激活
//    private int pidTargetPositionVS = 0; // PID 控制目标位置
//    private PIDController pidControllerVS = new PIDController(0.005, 0.0000005, 0.0002);// (0.005, 0.0000005, 0.0002) good for target 300 (1.9, 0.014, 4.9)
//    // Tune these values  POSITION_B_EXTRUDETransfer = 600;//horizontal slides  out //600 is too much
//
//    // pid for HSlides
//    int controlMode = 1;
//    private ElapsedTime runtime = new ElapsedTime();
//    public String armPositionCuzBorS ="NOLL"; //new variable for it and arm will go back of robo
//
//
//    public verticalSlide() {
//        // 空构造函数，稍后通过 `init()` 传入 HardwareMap
//    }
//
//    public void setPosition(int position) {
//        targetPosition = position;
//        robot.VSMotorL.setTargetPosition(targetPosition);
//        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.VSMotorL.setPower(SLIDE_POWER);
//        robot.VSMotorR.setPower(SLIDE_POWER);
//        taskActive = true;
//    }
//    private void startVSlidePIDControl(int targetPosition,double vSlidesAccuracy) {
//        robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.VSMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        pidControllerVS.reset();
//        pidControllerVS.enable();
//        pidControllerVS.setSetpoint(targetPosition);
//        pidControllerVS.setTolerance(10); // 允许误差范围
//        pidTargetPositionVS = targetPosition;
//        pidActiveVS = true; // 激活 PID 控制
//        if (!pidActiveVS) return; // 如果 PID 未激活，直接返回
//        int currentPositionL = robot.VSMotorL.getCurrentPosition();
//// 计算 PID 输出
//        double powerL = pidControllerVS.performPID(currentPositionL);
//        robot.VSMotorL.setPower(powerL*0.8); // change it to make it move faster both at the same time
//        robot.VSMotorR.setPower(powerL*0.8); // change it to make it move faster
//
//        // 输出 Telemetry 信息
//        telemetry.addData("PID Target", pidTargetPositionVS);
//        telemetry.addData("Current Position L", currentPositionL);
//        telemetry.addData("Power L", powerL);
//        telemetry.update();
//
//        // 在 updateVSlidePIDControl 中加入抗重力逻辑
//        while(!pidActiveVS && Math.abs(robot.VSMotorL.getCurrentPosition() - pidTargetPositionVS) > vSlidesAccuracy) {
//            updateVSlidePIDControl();
//        }
//
//
//    }
//    // 在主循环中调用的非阻塞 PID 控制逻辑
//    private void updateVSlidePIDControl() {
//        if (!pidActiveVS) return; // 如果 PID 未激活，直接返回
//
//        int currentPositionL = robot.VSMotorL.getCurrentPosition();
//
//        // 计算 PID 输出
//        double powerL = pidControllerVS.performPID(currentPositionL);
//        robot.VSMotorL.setPower(powerL*0.8); // change it to make it move faster both at the same time
//        robot.VSMotorR.setPower(powerL*0.8); // change it to make it move faster
//
//        // 输出 Telemetry 信息
//        telemetry.addData("PID Target", pidTargetPositionVS);
//        telemetry.addData("Current Position L", currentPositionL);
//        telemetry.addData("Power L", powerL);
//        telemetry.update();
//
//        // 在 updateVSlidePIDControl 中加入抗重力逻辑
//        if (!pidActiveVS && Math.abs(robot.VSMotorL.getCurrentPosition() - pidTargetPositionVS) > 10) {
//            double holdPowerVS = pidControllerVS.performPID(robot.VSMotorL.getCurrentPosition());
//            robot.VSMotorL.setPower(holdPowerVS);
//            robot.VSMotorR.setPower(holdPowerVS);
//            pidActiveVS = false; // 停止 PID 控制
//        }
//
//    }
//
//    @Override
//    public boolean isTaskActive() {
//        return taskActive;
//    }
//
//    @Override
//    public void runTask() {
//        if (taskActive) {
//            if (!robot.VSMotorL.isBusy()) {
//                robot.VSMotorL.setPower(0); // 目标到达后停止
//                robot.VSMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                taskActive = false;
//            }
//        }
//    }
//}