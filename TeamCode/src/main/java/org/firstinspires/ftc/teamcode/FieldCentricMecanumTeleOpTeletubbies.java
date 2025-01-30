package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentricMecanumTeleOpTeletubbies")
public class FieldCentricMecanumTeleOpTeletubbies extends LinearOpMode {
    public static final double DriveTrains_ReducePOWER =  0.9 ;
    private static final int POSITION_Y = 1600;//1322 //1000 one stage
    private static final int POSITION_A = 100;
    private static final int POSITION_PrepareForHing = 1600;
    private static final int POSITION_ForHing = 400;
    private static final double SLIDE_POWER = 0.8; // Adjust as needed
    HardwareTeletubbies robot = new HardwareTeletubbies();
    public String fieldOrRobotCentric="robot";
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            moveDriveTrain_FieldCentric();// tune the power for easy to control it

        }
    }
public void moveDriveTrain_FieldCentric() {
        double y = gamepad1.left_stick_y * (1); // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * (1);
        double rx = -gamepad1.right_stick_x * (1); //*(0.5) is fine


        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            robot.imu.resetYaw();
        }

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
}