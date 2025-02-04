//package org.firstinspires.ftc.teamcode;
//
//import com.google.android.gms.gcm.Task;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class verticalSlide extends Task {
//    private DcMotor slideMotor;
//    private boolean taskActive = false;
//    private int targetPosition;
//    private static final double SLIDE_POWER = 0.8;
//
//    public Slide() {
//        // 空构造函数，稍后通过 `init()` 传入 HardwareMap
//    }
//
//    public void init(HardwareMap hardwareMap) {
//        slideMotor = hardwareMap.get(DcMotor.class, "VSMotorL");
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void setPosition(int position) {
//        targetPosition = position;
//        slideMotor.setTargetPosition(targetPosition);
//        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideMotor.setPower(SLIDE_POWER);
//        taskActive = true;
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
//            if (!slideMotor.isBusy()) {
//                slideMotor.setPower(0); // 目标到达后停止
//                slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                taskActive = false;
//            }
//        }
//    }
//}