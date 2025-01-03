package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class NewCollectionArm(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class ArmState(val position: Int) {
        Up(1600),
        OffGround(1600),
        Drop(60),
        Manual(-1) //switch to manual for on init
    }

    var armState = ArmState.Manual

    private val collectionArm = hardwareMap.get(DcMotor::class.java, "CollectionArm")

    private val power = 0.75

    var collectionArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPosition = 0.0

    init {
        collectionArm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        collectionArm.direction = DcMotorSimple.Direction.REVERSE
        collectionArm.targetPosition = 0
        collectionArm.mode = DcMotor.RunMode.RUN_TO_POSITION
        collectionArm.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: ArmState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                collectionArm.targetPosition = targetPosition.toInt() - collectionArmOffset
                armState = state
                initialized = true
            }
            collectionArm.currentPosition
            packet.put("Target Position", collectionArm.targetPosition)
            packet.put("Current Position", collectionArm.currentPosition)
            //TODO: make this scoringArm.isbusy so that it will actually do smth :)
            return collectionArm.isBusy
        }
    }

    /**
     * manually changes the position of the scoringArm (typically with a joystick)
     *
     * @param input the percent speed (-1 to 1) normalized by delta time (the time between each loop)
     */
    inner class Manual(private val input: Double) : Action {
        val maxSpeed = 600.0 //tics/second

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            armState = ArmState.Manual
            targetPosition += input * maxSpeed
            collectionArm.targetPosition = targetPosition.toInt() - collectionArmOffset
            packet.put("Target Position", collectionArm.targetPosition)
            packet.put("Current Position", collectionArm.currentPosition)
            return false
        }
    }



    fun offGround(): Action = SetState(ArmState.OffGround)
    fun up(): Action = SetState(ArmState.Up)
    fun drop(): Action = SetState(ArmState.Drop)
    fun manual(input: Double): Action = Manual(input)
}