//package org.firstinspires.ftc.teamcode;
//
//import android.graphics.Path;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@Autonomous
//public class ServoSynching extends LinearOpMode {
//
//    HardWare robot = new HardWare();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            robot.zero.setPosition(1);
//            robot.one.setPosition(0);
//            sleep(2000);
//            robot.zero.setPosition(0);
//            robot.one.setPosition(1);
//            sleep(2000);
//            robot.zero.setPosition(0.2);
//            robot.one.setPosition(0.8);
//            sleep(2000);
//            robot.zero.setPosition(0.8);
//            robot.one.setPosition(0.2);
//            sleep(2000);
//            posSer(0.4, 0.6);
//        }
//
//    }
//
//    public void posSer(double posOne, double posTwo){
//        robot.zero.setPosition(posOne);
//        robot.one.setPosition(posTwo);
//        sleep(2000);
//    }
//
//}
