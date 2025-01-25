package frc.robot

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d

fun Translation2d.getRotationToTranslation(other: Translation2d): Rotation2d =
    (this - other).angle

fun Pose2d.flip(): Pose2d = FlippingUtil.flipFieldPose(this)

fun Pose2d.flipIfNeeded(): Pose2d = if (IS_RED) this.flip() else this

fun Pose2d.withTranslation(translation: Translation2d): Pose2d = Pose2d(translation, this.rotation)

fun Pose2d.withRotation(rotation: Rotation2d): Pose2d = Pose2d(this.translation, rotation)

fun Pose2d.toTransform(): Transform2d = Transform2d(this.translation, this.rotation)

fun Pose3d.toTransform(): Transform3d = Transform3d(this.translation, this.rotation)

fun Translation2d.flip(): Translation2d = FlippingUtil.flipFieldPosition(this)

fun Translation2d.flipIfNeeded(): Translation2d =
    if (IS_RED) this.flip() else this

fun Translation2d.toTransform(): Transform2d = Transform2d(this, Rotation2d())

fun Translation2d.toPose(): Pose2d = Pose2d(this, Rotation2d())

fun Rotation2d.flip(): Rotation2d = FlippingUtil.flipFieldRotation(this)

fun Rotation2d.flipIfNeeded(): Rotation2d = if (IS_RED) this.flip() else this

fun Rotation2d.toTransform(): Transform2d = Transform2d(Translation2d(), this)

fun Rotation2d.toPose(): Pose2d = Pose2d(Translation2d(), this)

fun Transform2d.toPose(): Pose2d = Pose2d(this.translation, this.rotation)

fun Transform3d.toPose(): Pose3d = Pose3d(this.translation, this.rotation)