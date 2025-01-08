package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class SampleClaw(hardwareMap: HardwareMap) {

    enum class SampleClawState(val position: Double) {
        Open(0.5),
        Close(0.1)
    }
    var sampleClawState: SampleClawState = SampleClawState.Close

    private val pulley: Servo = hardwareMap.get(Servo::class.java, "sampleCollection")

    inner class SetPosition(val dt: Double, val state: SampleClawState) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            if (beginTs < 0) {
                beginTs = now()
                pulley.position = state.position
                sampleClawState = state
            }
            val t = now() - beginTs
            p.put("test", true)

            return t < dt
        }
    }

    fun close(): Action = SetPosition(1.0, SampleClawState.Close)
    fun open(): Action = SetPosition(1.0, SampleClawState.Open)
}