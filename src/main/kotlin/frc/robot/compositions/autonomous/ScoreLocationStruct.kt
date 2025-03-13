package frc.robot.compositions.autonomous

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.util.struct.Struct
import java.nio.ByteBuffer

class ScoreLocationStruct : Struct<ScoreLocation> {
    override fun getTypeClass() = ScoreLocation::class.java

    override fun getTypeName() = "ScoreLocation"

    override fun getSize() = Struct.kSizeInt8 + Pose2d.struct.size + Struct.kSizeInt8

    override fun getSchema() = "char name;Pose2d pose;int8 tag_id"

    override fun unpack(bb: ByteBuffer): ScoreLocation {
        val name = bb.getChar().toString()
        val pose = Pose2d.struct.unpack(bb)
        val tagId = bb.getInt()
        return ScoreLocation(name, pose, tagId)
    }

    override fun pack(bb: ByteBuffer, value: ScoreLocation) {
        bb.putChar(value.name.first())
        Pose2d.struct.pack(bb, value.pose)
        bb.putInt(value.tagId)
    }

    override fun isImmutable() = true
}