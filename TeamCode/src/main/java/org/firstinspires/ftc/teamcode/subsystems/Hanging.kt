package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class Hanging(hardwareMap: HardwareMap) {
    private val hanging = hardwareMap.get(DcMotor::class.java, "hanging")

    init {
        hanging.direction = DcMotorSimple.Direction.FORWARD
    }

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class ArmStateHanging(val position: Int) {
        Down(-1), //very accurate numbers,
        Up(-1),
        Manual(-1) //switch to manual for on init
    }

    var armState = ArmStateHanging.Manual

    private val power = 0.75

    var hangingArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPosition = 0.0 //on boot-up, don't try to go anywhere

    init {
        hanging.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hanging.direction = DcMotorSimple.Direction.REVERSE
        hanging.targetPosition = 0
        hanging.mode = DcMotor.RunMode.RUN_TO_POSITION
        hanging.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: ArmStateHanging) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                hanging.targetPosition = targetPosition.toInt() - hangingArmOffset
                armState = state
                initialized = true
            }
            hanging.currentPosition
            packet.put("Target Position", hanging.targetPosition)
            packet.put("Current Position", hanging.currentPosition)
            //TODO: make this collectionArm.isbusy so that it will actually do smth :)
            return hanging.isBusy
        }
    }

    /**
     * manually changes the position of the scoringArm (typically with a joystick)
     *
     * @param input the percent speed (-1 to 1) normalized by delta time (the time between each loop)
     */
    inner class Manual(private val input: Double) : Action {
        val maxSpeed = 800.0 //tics/second

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            armState = ArmStateHanging.Manual
            targetPosition += input * maxSpeed
            hanging.targetPosition = targetPosition.toInt() - hangingArmOffset
            packet.put("Target Position", hanging.targetPosition)
            packet.put("Current Position", hanging.currentPosition)
            return false
        }
    }
    fun resetArmPosition() {
        hangingArmOffset = ArmStateHanging.Down.position - hanging.currentPosition
    }


    fun offGround(): Action = SetState(ArmStateHanging.Up)
    fun down(): Action = SetState(ArmStateHanging.Down)
    fun manual(input: Double): Action = Manual(input)
}