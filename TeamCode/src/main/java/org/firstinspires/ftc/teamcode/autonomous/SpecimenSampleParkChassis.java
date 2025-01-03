package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw;
@Autonomous
public class    SpecimenSampleParkChassis extends OpMode {
    Pose2d beginPose;
    PinpointDrive drive;
    ScoringArm scoringArm;
    SpecimenClaw specimenClaw;

    public void init() {
        specimenClaw = new SpecimenClaw(hardwareMap);
        beginPose = new Pose2d(-17.5, 66, -Math.PI / 2);
        drive = new PinpointDrive(hardwareMap, beginPose);
        scoringArm = new ScoringArm(hardwareMap);
    }
    public void start() {
        Actions.runBlocking(scoringArm.collect());
        Actions.runBlocking(specimenClaw.open());
        Actions.runBlocking(specimenClaw.close());
        Actions.runBlocking(scoringArm.score());
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .setTangent(0.0)
                .splineToConstantHeading(new Vector2d(2.0,40.0), -Math.PI/2)
                .strafeTo(new Vector2d(2.0, 30.0))
                .build());
        Actions.runBlocking(specimenClaw.open());
        Actions.runBlocking(drive.actionBuilder(drive.getPose())
                .strafeTo(new Vector2d(-4.0, 54.0))
                .build());
        Actions.runBlocking(specimenClaw.close());
        Actions.runBlocking(scoringArm.collect());
        Actions.runBlocking(specimenClaw.approach());
        Actions.runBlocking(drive.actionBuilder(drive.getPose())
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(new Vector2d(-27.0,59.0), Math.PI), Math.PI)
                .strafeTo(new Vector2d(-35.0, 59.0))
                .build());
        Actions.runBlocking(specimenClaw.open());
        Actions.runBlocking(specimenClaw.close());
        Actions.runBlocking(scoringArm.score());
        Actions.runBlocking(drive.actionBuilder(drive.getPose())
                .setTangent(0.0)
                .splineToLinearHeading(new Pose2d(new Vector2d(-4.0,55.0), -Math.PI/2), -Math.PI/2)
                .strafeTo(new Vector2d(-4.0, 28.0))
                .build());
        Actions.runBlocking(specimenClaw.open());
        Actions.runBlocking(drive.actionBuilder(drive.getPose())
                .waitSeconds(1.0)
                .strafeTo(new Vector2d(-6.0, 44.0))
                .waitSeconds(1.0)
                //.setTangent(Math.PI / 2)
                //.splineToConstantHeading(new Vector2d(-24.0, 48.0), Math.PI)
                //.splineToConstantHeading(new Vector2d(-49.0, 14.0), Math.PI)
                .build());
        //Actions.runBlocking(drive.actionBuilder(drive.getPose())
//
        //        .strafeTo(new Vector2d(-46.0,60.0)) //go to 1st sample
        //        .strafeTo(new Vector2d(-46.0,7.0)) //go to 1st sample
        //        .strafeTo(new Vector2d(-60.0,7.0)) //strafe over
        //        .strafeTo(new Vector2d(-55.0,60.0))//go to 2nd sample
        //        .strafeTo(new Vector2d(-55.0,7.0))//go to 2nd sample
        //        .strafeTo(new Vector2d(-64.0,7.0)) //strafe over
        //        .strafeTo(new Vector2d(-64.0,60.0)) //go to 3rd sample
        //        .build());

    }
    public void loop() {
    }

}

