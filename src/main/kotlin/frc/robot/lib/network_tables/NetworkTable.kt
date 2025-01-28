package frc.robot.lib.network_tables

class NetworkTable(private val io: NetworkTableIO) {

    fun getDoubleFromTopic(topicName: String): Double =
        io.getDoubleFromTopic(topicName)

    fun getIntFromTopic(topicName: String): Int =
        io.getDoubleFromTopic(topicName).toInt()

    fun getStringTopic(topicName: String): String = io.getStringTopic(topicName)
}
