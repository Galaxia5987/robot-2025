package frc.robot.lib.network_tables

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.littletonrobotics.junction.networktables.LoggedNetworkString

class NetworkTableIOSim : NetworkTableIO {

    private val doubleTopics = mutableMapOf<String, LoggedNetworkNumber>()
    private val stringTopics = mutableMapOf<String, LoggedNetworkString>()

    override fun getDoubleFromTopic(topicName: String): Double {
        val simTopicValue =
            doubleTopics.getOrPut(topicName) {
                LoggedNetworkNumber("/Tuning/$topicName", 0.0)
            }
        return simTopicValue.get()
    }

    override fun getIntFromTopic(topicName: String): Int {
        val simTopicValue =
            doubleTopics.getOrPut(topicName) {
                LoggedNetworkNumber("/Tuning/$topicName", 0.0)
            }
        return simTopicValue.get().toInt()
    }

    override fun getStringTopic(topicName: String): String {
        val simTopicValue =
            stringTopics.getOrPut(topicName) {
                LoggedNetworkString("/Tuning/$topicName", "")
            }
        return simTopicValue.get()
    }
}
