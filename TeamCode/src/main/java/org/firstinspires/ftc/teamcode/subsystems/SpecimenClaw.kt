package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class SpecimenClaw(hardwareMap: HardwareMap) {

    enum class ClawState(val leftPosition: Double, val rightPosition: Double) {
        Open(0.495, 0.645),
        Close(0.43, 0.73),
        InBox(0.88, 0.0),
        Approach(0.6, 0.6)
    }

    var clawState: ClawState = ClawState.InBox

    private val leftClaw: Servo = hardwareMap.get(Servo::class.java, "leftClaw")
    private val rightClaw: Servo = hardwareMap.get(Servo::class.java, "rightClaw")

    inner class SetPosition(val duration: Double, val state: ClawState) : Action {
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

    fun close(): Action = SetPosition(2.0, ClawState.Close)
    fun open(): Action = SetPosition(1.0, ClawState.Open)
    fun inBox(): Action = SetPosition(1.0, ClawState.InBox)
    fun approach(): Action = SetPosition(1.0, ClawState.Approach)
}