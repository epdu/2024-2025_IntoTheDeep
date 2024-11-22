package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class CollectionArm(hardwareMap: HardwareMap) {
    private val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "collectionArm")
    private val motorPower = 0.8
    private val extendRetractDelta = 1300
    private var pos = 0
    private val deploy: Servo = hardwareMap.get(Servo::class.java, "armRelease")
    private val deployStart = 0.5
    private var targetPosition = 0.0

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        deploy.position = deployStart
        motor.power = motorPower
    }

    inner class DeployArm(val dt: Double, private val dropPosition: Double) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            if (beginTs < 0) {
                beginTs = now()
                deploy.position = dropPosition
            }
            val t = now() - beginTs
            p.put("test", true)

            return t < dt
        }
    }

    inner class Extend : Action {
        private var initialized = false
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                motor.power = motorPower
                motor.targetPosition = motor.currentPosition - extendRetractDelta
                initialized = true
            }
            pos = motor.currentPosition
            packet.put("collectionArmPosition", pos)
            return motor.isBusy
        }
    }

    inner class Retract : Action {
        private var initialized = false
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                motor.power = motorPower
                motor.targetPosition = motor.currentPosition + extendRetractDelta
                initialized = true
            }
            pos = motor.currentPosition
            packet.put("collectionArmPosition", pos)
            return motor.isBusy
        }
    }

    inner class Manual(private val input: Double) : Action {
        val maxSpeed = 600.0

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            targetPosition += input * maxSpeed
            motor.targetPosition = targetPosition.toInt()
            packet.put("Target Position", motor.targetPosition)
            packet.put("Current Position", motor.currentPosition)
            return false
        }
    }


    fun extend(): Action {
        return Extend()
    }

    fun retract(): Action {
        return Retract()
    }

    fun manual(input: Double): Action = Manual(input)

    fun deployArm(): Action = DeployArm(1.0, 0.2)

}