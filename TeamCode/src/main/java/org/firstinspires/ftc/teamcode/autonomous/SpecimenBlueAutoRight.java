package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;

@Autonomous
public class SpecimenBlueAutoRight extends OpMode {
    SpecimenClaw scoringClaw;
    ScoringArm scoringArm;
    Pose2d beginPose;
    PinpointDrive drive;

    public void init() {
        scoringClaw = new SpecimenClaw(hardwareMap);
        scoringArm = new ScoringArm(hardwareMap);
        beginPose = new Pose2d(-17.5, 66, -Math.PI / 2);
        drive = new PinpointDrive(hardwareMap, beginPose);
    }

    public void start() {
        //Actions.runBlocking(scoringClaw.close());
        //Actions.runBlocking(scoringArm.score());
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-6, 42))
                        .strafeTo(new Vector2d(-6, 38))
                        .build()
        );
        //Actions.runBlocking(scoringClaw.open());
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setTangent(Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-24, 48), Math.PI)
                        .splineToConstantHeading(new Vector2d(-42, 14), Math.PI)
                        .build()
        );
        //Actions.runBlocking(scoringClaw.close());
    }

    public void loop() {}
}
