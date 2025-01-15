package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class SpecimenClaw(hardwareMap: HardwareMap) {

    /**
     * @param leftPosition the position of the left part of the claw (looking back-to-front top-down)
     * @param rightPosition the position of the right part of the claw (looking back-to-front top-down)
     */
    enum class ClawState(val leftPosition: Double, val rightPosition: Double) {
        Open(0.495, 0.645),
        Close(0.43, 0.73),
        InBox(0.88, 0.0),
        Approach(0.6, 0.6)
    }
    var clawState: ClawState = ClawState.InBox

    private val leftClaw: Servo = hardwareMap.get(Servo::class.java, "leftClaw")
    private val rightClaw: Servo = hardwareMap.get(Servo::class.java, "rightClaw")

    /**
     * Start: sets the claw state and position;
     * IsFinished: duration is up
     *
     * @param duration the amount of time to run the action for
     * @param state the state (and associated position) to set the claw to
     */
    inner class SetState(val duration: Double, val state: ClawState) : Action {
        private var startTime = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            if (startTime < 0) {
                startTime = now()
                clawState = state
                leftClaw.position = state.leftPosition
                rightClaw.position = state.rightPosition
            }
            val elapsedTime = now() - startTime
            return elapsedTime < duration
        }
    }

    fun close(duration: Double = .2): Action = SetState(duration, ClawState.Close)
    fun open(duration: Double = .2): Action = SetState(duration, ClawState.Open)

    /**
     * positions the claw to fit inside the 18" cube
     */
    fun inBox(duration: Double = .2): Action = SetState(duration, ClawState.InBox)

    /**
     * positions the claw to approach the specimen when picking it up
     */
    fun approach(duration: Double = .2): Action = SetState(duration, ClawState.Approach)
}