package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class CollectionArm(hardwareMap: HardwareMap) {
    private val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "collectionArm")
    private val motorPower = 0.6
    private val collectionValue = -2800 // amount and direction for collection
    private val retractionValue = 2800 // amount and direction for retraction

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    inner class Extend : Action {
        private var initialized = false
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                motor.power = motorPower
                motor.targetPosition = motor.currentPosition + collectionValue
                initialized = true
            }
            val pos = motor.currentPosition
            packet.put("collectionArmPosition", pos)
            return motor.isBusy
        }
    }

    inner class Retract : Action {
        private var initialized = false
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                motor.power = motorPower
                motor.targetPosition = motor.currentPosition - retractionValue
                initialized = true
            }
            val pos = motor.currentPosition
            packet.put("collectionArmPosition", pos)
            return motor.isBusy
        }
    }
    fun extend(): Action {
        return Extend()
    }

    fun retract(): Action {
        return Retract()
    }
}