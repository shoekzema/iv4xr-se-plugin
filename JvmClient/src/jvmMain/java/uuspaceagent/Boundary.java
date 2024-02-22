package uuspaceagent;

import eu.iv4xr.framework.spatial.Vec3;

public class Boundary {
    Vec3 lowerBounds, upperBounds;

    public Boundary(Vec3 pos, float size) {
        lowerBounds = pos;
        upperBounds = new Vec3(pos.x + size, pos.y + size, pos.z + size);
    }

    public boolean intersects(Boundary other) {
        if (other.lowerBounds.x > this.upperBounds.x || other.upperBounds.x < this.lowerBounds.x)
            return false;
        if (other.lowerBounds.y > this.upperBounds.y || other.upperBounds.y < this.lowerBounds.y)
            return false;
        if (other.lowerBounds.z > this.upperBounds.z || other.upperBounds.z < this.lowerBounds.z)
            return false;
        return true;
    }

    public boolean contains(Boundary other) {
        if (other.lowerBounds.x < this.lowerBounds.x || other.upperBounds.x > this.upperBounds.x)
            return false;
        if (other.lowerBounds.y < this.lowerBounds.y || other.upperBounds.y > this.upperBounds.y)
            return false;
        if (other.lowerBounds.z < this.lowerBounds.z || other.upperBounds.z > this.upperBounds.z)
            return false;
        return true;
    }

    public boolean contains(Vec3 point) {
        return (point.x >= lowerBounds.x && point.x <= upperBounds.x
                &&
                point.y >= lowerBounds.y && point.y <= upperBounds.y
                &&
                point.z >= lowerBounds.z && point.z <= upperBounds.z);
    }
    public boolean contains(DPos3 point) {
        return (point.x >= lowerBounds.x && point.x <= upperBounds.x
                &&
                point.y >= lowerBounds.y && point.y <= upperBounds.y
                &&
                point.z >= lowerBounds.z && point.z <= upperBounds.z);
    }
}
