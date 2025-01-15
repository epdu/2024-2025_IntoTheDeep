package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw

@Autonomous
class SpecimenParkBlueRight : OpMode() {
    var scoringClaw: SpecimenClaw? = null
    var scoringArm: ScoringArm? = null
    var beginPose: Pose2d? = null
    var drive: PinpointDrive? = null

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
                .strafeTo(Vector2d(-4.0, 42.0))
                .strafeTo(Vector2d(-4.0, 35.0))
                .build()
        )
        runBlocking(scoringClaw!!.open())
        drive!!.updatePoseEstimate()
        runBlocking(
            drive!!.actionBuilder(drive!!.pose) //.setTangent(Math.PI/2)
                .strafeTo(Vector2d(-6.0, 48.0))
                .strafeTo(
                    Vector2d(
                        -60.5,
                        64.0
                    )
                ) //.splineToConstantHeading(new Vector2d(-24, 48), Math.PI)
                //.splineToConstantHeading(new Vector2d(-42, 28), Math.PI)
                .build()
        )
        runBlocking(scoringClaw!!.close())
        runBlocking(scoringArm!!.collect())
    }

    override fun loop() {}
}
