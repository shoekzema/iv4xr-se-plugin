package spaceEngineers.model.extensions

import spaceEngineers.model.CharacterMovementType
import spaceEngineers.model.NumberVec3
import spaceEngineers.model.Vec3F


fun Vec3F.normalizeAsMovement(characterMovementType: CharacterMovementType): Vec3F {
    return normalized() * characterMovementType.speed
}

fun Vec3F.normalizeAsWalk(): Vec3F {
    return normalizeAsMovement(CharacterMovementType.WALK)
}

fun Vec3F.normalizeAsRun(): Vec3F {
    return normalizeAsMovement(CharacterMovementType.RUN)
}

fun Vec3F.normalizeAsSprint(): Vec3F {
    return normalizeAsMovement(CharacterMovementType.SPRINT)
}

fun NumberVec3<Float>.crossProduct(b: NumberVec3<Float>): Vec3F {
    val a = this
    return Vec3F(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    )
}

fun NumberVec3<Float>.toArray(): FloatArray {
    return floatArrayOf(x, y, z)
}

public inline fun Iterable<Vec3F>.sum(): Vec3F {
    var sum = Vec3F.ZERO
    for (element in this) {
        sum += element
    }
    return sum
}
