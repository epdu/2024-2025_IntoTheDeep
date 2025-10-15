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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import static org.firstinspires.ftc.teamcode.Constants_CS.POSITION_Y_LOW;


@Autonomous(name = "AA Test Motors")
//V1 with pid for both slides but not odo
public class testMotors extends LinearOpMode {

    boolean move = false;
    public DcMotorEx uno;
    public DcMotorEx dos;
    public DcMotorEx trois;
    public DcMotorEx quatre;


    private volatile boolean isRunning = true;
    ElapsedTime delayTimer = new ElapsedTime();
    /*
    package mypackage; // 与 Gyro 类的包名一致
            Gyro gyro = new Gyro(); // 创建 Gyro 类的对象
            gyro.turn();            // 调用 turn() 方法
     */

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        uno   = hardwareMap.get(DcMotorEx.class, "LFMotor");//02022024 control hub port 0
        dos  = hardwareMap.get(DcMotorEx.class, "RFMotor"); //02022024 control hub port 1
        trois   = hardwareMap.get(DcMotorEx.class, "LBMotor");//02022024 control hub port 2
        quatre  = hardwareMap.get(DcMotorEx.class, "RBMotor");//02022024 control hub port 3

        uno.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trois.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        quatre.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        uno.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dos.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trois.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        quatre.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//

        waitForStart();
        while (opModeIsActive()){
            uno.setPower(1); // Set desired power
            dos.setPower(1);
            trois.setPower(1);
            quatre.setPower(1);


            sleep(5000);

            uno.setPower(0);
            dos.setPower(0);
            trois.setPower(0);
            quatre.setPower(0);


            telemetry.addData("Motor 1 Position", uno.getCurrentPosition());
            telemetry.addData("Motor 2 Position", dos.getCurrentPosition());
            telemetry.addData("Motor 3 Position", trois.getCurrentPosition());
            telemetry.addData("Motor 4 Position", quatre.getCurrentPosition());
            telemetry.update();
        }
    }
}








