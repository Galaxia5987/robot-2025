package frc.robot.lib.network_tables

import edu.wpi.first.networktables.NetworkTableInstance

class NetworkTableIOReal : NetworkTableIO {
    override fun getDoubleFromTopic(topicName: String): Double =
        NetworkTableInstance.getDefault()
            .getDoubleTopic(topicName)
            .getEntry(0.0)
            .asDouble

    override fun getIntFromTopic(topicName: String): Int =
        NetworkTableInstance.getDefault()
            .getIntegerTopic(topicName)
            .getEntry(0)
            .asLong
            .toInt()

    override fun getStringTopic(topicName: String): String =
        NetworkTableInstance.getDefault()
            .getStringTopic(topicName)
            .getEntry("")
            .get()
}
