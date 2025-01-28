package frc.robot.lib.network_tables

interface NetworkTableIO {
    fun getDoubleFromTopic(topicName: String): Double = 0.0

    fun getIntFromTopic(topicName: String): Int = 0

    fun getStringTopic(topicName: String): String = ""
}
