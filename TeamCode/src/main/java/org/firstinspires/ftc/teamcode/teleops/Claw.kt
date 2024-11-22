package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class Claw(hardwareMap: HardwareMap) {

    private val leftClaw: Servo = hardwareMap.get(Servo::class.java, "leftClaw")
    private val rightClaw: Servo = hardwareMap.get(Servo::class.java, "rightClaw")
    var clawState: ClawState = ClawState.InBox


    inner class SetPosition(val dt: Double, val state: ClawState) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            if (beginTs < 0) {
                beginTs = now()
                leftClaw.position = state.leftPosition
                rightClaw.position = state.rightPosition
                clawState = state
            }
            val t = now() - beginTs
            p.put("test", true)

            return t < dt
        }
    }

    enum class ClawState(val leftPosition: Double, val rightPosition: Double) {
        Open(0.495, 0.645),
        Close(0.43, 0.73),
        InBox(0.83, 0.0),
        Approach(0.6, 0.6)
    }


    fun close(): Action = SetPosition(2.0, ClawState.Close)
    fun open(): Action = SetPosition(1.0, ClawState.Open)
    fun inBox(): Action = SetPosition(1.0, ClawState.InBox)
    fun approach(): Action = SetPosition(1.0, ClawState.Approach)
}