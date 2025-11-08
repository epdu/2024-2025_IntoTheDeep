package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class ScrimAuto extends LinearOpMode {

    public float DriveTrains_ReducePOWER = 0.7f;
    HardwareScrim robot = new HardwareScrim();

    public String fieldOrRobotCentric = "robot";
    boolean move = false;
    boolean movementActive = false;

    Gyro gyro = new Gyro(); // 创建 Gyro 类的对象
    private volatile boolean isRunning = true;
    ElapsedTime delayTimer = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
//        gyro.robot.init(hardwareMap);

        waitForStart();
        robot.LBMotor.setPower(-0.6);
        robot.LFMotor.setPower(-0.6);
        robot.RBMotor.setPower(-0.6);
        robot.RFMotor.setPower(-0.6);
        sleep(400);
        robot.LBMotor.setPower(0);
        robot.LFMotor.setPower(0);
        robot.RBMotor.setPower(0);
        robot.RFMotor.setPower(0);

        stop();
    }
}
