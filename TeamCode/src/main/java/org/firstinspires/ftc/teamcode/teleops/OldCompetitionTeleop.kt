package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive
import org.firstinspires.ftc.teamcode.subsystems.CollectionArm
import org.firstinspires.ftc.teamcode.subsystems.SampleClaw
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm.ArmState
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw.ClawState

@TeleOp(name = "OldCompetitionTeleop")
class OldCompetitionTeleop : OpMode() {
    private lateinit var drive: PinpointDrive
    private lateinit var g1: PandaGamepad
    private lateinit var g2: PandaGamepad
    private lateinit var scoringClaw: SpecimenClaw
    private lateinit var sampleClaw: SampleClaw
    private lateinit var collectionArm: CollectionArm
    private lateinit var scoringArm: ScoringArm
    private lateinit var climbing: DcMotor
    private var headingOffset: Double = 0.0
    private val dash: FtcDashboard = FtcDashboard.getInstance()
    private var runningActions: MutableList<Action> = ArrayList()
    private var lastTime: Double = 0.0


    override fun init() {
        drive = PinpointDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        g1 = PandaGamepad(gamepad1)
        g2 = PandaGamepad(gamepad2)
        scoringClaw = SpecimenClaw(hardwareMap)
        sampleClaw = SampleClaw(hardwareMap)
        collectionArm = CollectionArm(hardwareMap)
        scoringArm = ScoringArm(hardwareMap)
        climbing = hardwareMap.get(DcMotor::class.java, "climbing")

    }

    override fun start() {
        lastTime = runtime
    }

    fun getDeltaTime(): Double {
        val newTime = runtime
        val deltaTime: Double = newTime - lastTime
        lastTime = newTime
        return deltaTime
    }

    fun nearestCompass(currentHeading: Double, direction: String): Double {
        //Finds nearest 90 degree increment to current heading
        var headOut: Double = 0.0

        if ((currentHeading < 90) and (currentHeading <= 0)) headOut = 90.0
        else if ((currentHeading < 180) and (currentHeading <= 90)) headOut = 180.0
        else if ((currentHeading < -90) and (currentHeading <= -180)) headOut = -90.0
        else if ((currentHeading < 0) and (currentHeading <= -90)) headOut = 0.0

        if (direction == "right") return headOut
        else if (direction == "left") return headOut - 90
        else return 0.0
    }

    override fun loop() {
        //update delta time
        val deltaTime = getDeltaTime()
        telemetry.addData("Delta Time", deltaTime)

        //update gamepad values
        g1.update()
        g2.update()

        //run and update actions
        val packet = PandaTelemetryPacket(telemetry)

        val newActions: MutableList<Action> = ArrayList()
        for (action in runningActions) {
            action.preview(packet.fieldOverlay())
            telemetry.addLine(action.toString())
            if (action.run(packet)) {
                newActions.add(action)
            }
        }
        runningActions = newActions

        dash.sendTelemetryPacket(packet)

        //update drive Pose
        drive.updatePoseEstimate()

        /* driver 1 */
        val rawHeading = drive.getPinpoint().heading
        val heading: Rotation2d = Rotation2d.fromDouble(rawHeading - headingOffset)

        val input = Vector2d(
            g1.leftStickY.component.toDouble(),
            -g1.leftStickX.component.toDouble()
        )
        if (g1.rightBumper.isInactive() and g1.leftBumper.isInactive()) { //don't set drive if bumpers
            if (g1.y.isActive()) {
                drive.setDrivePowers(
                    PoseVelocity2d(
                        heading.inverse().times(input),
                        ((gamepad1.left_trigger - gamepad1.right_trigger) * 1 / 2).toDouble()
                    )
                )
            } else {
                drive.setDrivePowers(
                    PoseVelocity2d(
                        heading.inverse().times(input),
                        ((gamepad1.left_trigger - gamepad1.right_trigger) * 1 / 4).toDouble()
                    )
                )
            }
        }
        //Climbing****************************
        if (g1.dpadUp.isActive()) climbing.power = 1.0
        else if (g1.dpadDown.isActive()) climbing.power = -1.0
        else climbing.power = 0.0

        if (g1.b.justPressed()) headingOffset = rawHeading
        /*90 degree turn idea
        if (g1.leftBumper.justPressed()) {
            //Turn left to the nearest 90 degree increment
            curPose = drive.getPinpoint().positionRR
            var headTo = nearestCompass(rawHeading-headingOffset, "left")
            headTo += -headingOffset
            runBlocking(
                drive.actionBuilder(curPose)
                    .turnTo(headTo)
                    .build()
                    .ligmaBalls();
            )
        }
        if (g1.rightBumper.justPressed()){
            //Turn right to the nearest 90 degree increment
            curPose = Pose2d(0.0, 0.0, rawHeading-headingOffset)
            var headTo = nearestCompass(rawHeading-headingOffset, "right")
            headTo += -headingOffset
            runBlocking(
                drive.actionBuilder(curPose)
                    .turnTo(headTo)
                    .build()
            )
        }*/

        /* driver 2 */
        // Specimen Arm***********************************************************
        telemetry.addData("scoringArm position", scoringArm.targetPosition)
        if (g2.dpadDown.justPressed()) {
            val notCollecting = scoringArm.targetPosition - scoringArm.scoringArmOffset >= ArmState.ThroughBars.position-100
            if (notCollecting) {
                runningActions.add(scoringArm.collect())
                runningActions.add(
                    SequentialAction(
                        scoringClaw.close(),
                        scoringClaw.approach()
                    )
                )
            } else if (scoringClaw.clawState == ClawState.Close) {
                runningActions.add(scoringClaw.approach())
            } else {
                runningActions.add(scoringClaw.close())
            }
        }
        if (g2.dpadLeft.justPressed()) {
            if (scoringClaw.clawState == ClawState.Close) {
                runningActions.add(scoringClaw.open())
            } else {
                runningActions.add(scoringClaw.close())
            }
        }
        if (g2.dpadRight.justPressed()) {
            runningActions.add(scoringArm.throughBars())
        }
        if (g2.dpadUp.justPressed()) {
            val notScoring = scoringArm.targetPosition - scoringArm.scoringArmOffset <= ArmState.ThroughBars.position+100
            if (notScoring) {
                runningActions.add(scoringArm.score())
                runningActions.add(scoringClaw.close())
            } else if (scoringClaw.clawState == ClawState.Close) {
                runningActions.add(scoringClaw.open())
            } else {
                runningActions.add(scoringClaw.close())
            }
        }
        if (g2.leftBumper.justActive()) {   //ONLY USE IN COLLECTION POSE
            runningActions.add(scoringClaw.inBox())
            scoringArm.resetArmPosition()
            runningActions.add(scoringArm.collect())
        }

        if (g2.leftStickY.isActive()) runningActions.add(scoringArm.manual(g2.leftStickY.component * deltaTime))


        //Collection System*************************************
        if (g2.rightTrigger.isActive()) runningActions.add(collectionArm.deployArm())
        if (g2.y.justPressed()) {
            runningActions.add(sampleClaw.close())
            runningActions.add(collectionArm.retract())
        }
        if (g2.x.justPressed()) runningActions.add(sampleClaw.open())
        if (g2.a.justPressed()) {
            runningActions.add(sampleClaw.open())
            runningActions.add(collectionArm.extend())
        }
        if (g2.b.justPressed()) runningActions.add(sampleClaw.close())

        if (g2.rightStickY.isActive()) runningActions.add(collectionArm.manual(g2.rightStickY.component * deltaTime))


    }

    //protected abstract val allianceColor: AllianceColor
}
