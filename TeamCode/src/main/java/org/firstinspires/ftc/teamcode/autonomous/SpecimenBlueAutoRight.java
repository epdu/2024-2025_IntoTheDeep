package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleops.Claw;
import org.firstinspires.ftc.teamcode.teleops.ScoringArm;

@Autonomous
public class SpecimenBlueAutoRight extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Claw scoringClaw = new Claw(hardwareMap);
        ScoringArm scoringArm = new ScoringArm(hardwareMap);
        Pose2d beginPose = new Pose2d(-17.5, 66, -Math.PI/2);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        waitForStart();
        Actions.runBlocking(scoringClaw.close());
        Actions.runBlocking(scoringArm.score());
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(0,42))
                        .build()
        );
        //Actions.runBlocking(
        //        drive.actionBuilder(beginPose)
        //                .strafeTo(new Vector2d())
        //);
    }
}
