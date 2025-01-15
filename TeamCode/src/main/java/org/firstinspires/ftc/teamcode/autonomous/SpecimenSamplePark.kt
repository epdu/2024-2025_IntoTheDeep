package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw

@Autonomous
class SpecimenSamplePark : OpMode() {
    var scoringClaw: SpecimenClaw? = null
    var scoringArm: ScoringArm? = null
    var beginPose: Pose2d? = null
    var drive: PinpointDrive? = null

    //BTWWWW heading is in radians jsyk\
    //MATH.PI IS 90 DEGREES Left
    //North is looking at baskets
    override fun init() {
        scoringClaw = SpecimenClaw(hardwareMap)
        scoringArm = ScoringArm(hardwareMap)
        beginPose = Pose2d(-17.5, 66.0, -Math.PI / 2)
        drive = PinpointDrive(hardwareMap, beginPose)
    }

    override fun start() {
        runBlocking(scoringClaw!!.close())
        runBlocking(scoringArm!!.score())
        runBlocking(
            drive!!.actionBuilder(beginPose)
                .setTangent(0.0)
                .splineToConstantHeading(Vector2d(-4.0, 42.0), -Math.PI / 2)
                .strafeTo(Vector2d(-4.0, 35.0)) // clip it in
                .build()
        )
        runBlocking(scoringClaw!!.open())
        runBlocking(
            drive!!.actionBuilder(drive!!.getPose()) //.setTangent(Math.PI/2)
                .strafeTo(Vector2d(-6.0, 54.0)) // backs up
                .build()
        )
        runBlocking(scoringClaw!!.approach())
        runBlocking(scoringArm!!.collect())
        runBlocking(
            drive!!.actionBuilder(drive!!.getPose())
                .setTangent(Math.PI / 2)
                .splineToLinearHeading(
                    Pose2d(
                        Vector2d(-38.0, 61.0),
                        Rotation2d.fromDouble(Math.PI)
                    ), Math.PI
                )
                .build()
        )
        runBlocking(scoringClaw!!.close())
        runBlocking(scoringArm!!.score())
        runBlocking(
            drive!!.actionBuilder(drive!!.pose)
                .setTangent(0.0)
                .splineToLinearHeading(
                    Pose2d(
                        Vector2d(-4.0, 42.0),
                        Rotation2d.fromDouble(-Math.PI / 2)
                    ), -Math.PI / 2
                )
                .strafeTo(Vector2d(-4.0, 35.0)) //.setTangent(Math.PI/2)
                //.turnTo(-Math.PI / 2)
                //.strafeTo(new Vector2d(-2, 54))
                //.waitSeconds(1)
                //.strafeTo(new Vector2d(3, 42)) //go to front of bar
                //.strafeTo(new Vector2d(3, 36)) // clip it in
                //.waitSeconds(0.5)
                .build()
        )
        runBlocking(
            drive!!.actionBuilder(drive!!.getPose())
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(Vector2d(-24.0, 48.0), Math.PI)
                .splineToConstantHeading(Vector2d(-42.0, 14.0), Math.PI)
                .build()
        )

        //Actions.runBlocking(scoringClaw.open());
        //Actions.runBlocking(drive.actionBuilder(drive.getPose())
        //        .strafeTo(new Vector2d(-6, 57)) // backs up
        //        .build());
        //Actions.runBlocking(scoringClaw.close());
        ////it stops here idk why
        ////Actions.runBlocking(scoringArm.collect());
        //// goes to first sample
        //Actions.runBlocking(drive.actionBuilder(drive.getPose())
        //        .setTangent(Math.PI / 2)
        //        .splineToConstan
        //        hctHeading(new Vector2d(-24, 48), Math.PI)
        //        .splineToConstantHeading(new Vector2d(-42, 14), Math.PI)
        //        .build());
    }

    override fun loop() {
    }
}
