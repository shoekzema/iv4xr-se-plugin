package uuspaceagent;

import eu.iv4xr.framework.spatial.Vec3;

public class Boundary {
    /**
     * The lower boundary of the voxel boundary
     */
    Vec3 position;
    float size;

    public Boundary(Vec3 pos, float size) {
        position = pos;
        this.size = size;
        //upperBounds = new Vec3(pos.x + size, pos.y + size, pos.z + size);
    }

    public float size() { return size; } // Because a boundary is always cubic, we can just take the width
    //public float size() { return upperBounds.x - position.x; } // Because a boundary is always cubic, we can just take the width

    public Vec3 upperBounds() { return Vec3.add(position, new Vec3(size)); }

    public Vec3 center() { return Vec3.add(position, new Vec3(size / 2)); }
    //public Vec3 center() { return Vec3.mul(Vec3.add(position, upperBounds), 0.5f); }

    public boolean intersects(Boundary other) {
        Vec3 this_upperBounds = this.upperBounds();
        Vec3 other_upperBounds = other.upperBounds();
        if (other.position.x > this_upperBounds.x || other_upperBounds.x < this.position.x)
            return false;
        if (other.position.y > this_upperBounds.y || other_upperBounds.y < this.position.y)
            return false;
        if (other.position.z > this_upperBounds.z || other_upperBounds.z < this.position.z)
            return false;
        return true;
    }

    public boolean contains(Boundary other) {
        Vec3 this_upperBounds = this.upperBounds();
        Vec3 other_upperBounds = other.upperBounds();
        if (other.position.x < this.position.x || other_upperBounds.x > this_upperBounds.x)
            return false;
        if (other.position.y < this.position.y || other_upperBounds.y > this_upperBounds.y)
            return false;
        if (other.position.z < this.position.z || other_upperBounds.z > this_upperBounds.z)
            return false;
        return true;
    }

    public boolean contains(Vec3 point) {
        Vec3 upperBounds = upperBounds();
        return (point.x >= position.x && point.x <= upperBounds.x
                &&
                point.y >= position.y && point.y <= upperBounds.y
                &&
                point.z >= position.z && point.z <= upperBounds.z);
    }
    public boolean contains(DPos3 point) {
        Vec3 upperBounds = upperBounds();
        return (point.x >= position.x && point.x <= upperBounds.x
                &&
                point.y >= position.y && point.y <= upperBounds.y
                &&
                point.z >= position.z && point.z <= upperBounds.z);
    }

    public boolean sphereIntersects(Vec3 center, float radius) {
        Vec3 closest = new Vec3(center.x, center.y, center.z);

        if (center.x < position.x) closest.x = position.x;
        else if(center.x > position.x + size) closest.x = position.x + size;

        if (center.y < position.y) closest.y = position.y;
        else if(center.y > position.y + size) closest.y = position.y + size;

        if (center.z < position.z) closest.z = position.z;
        else if(center.z > position.z + size) closest.z = position.z + size;

        return Vec3.dist(center, closest) <= radius;
    }

    public boolean sphereContains(Vec3 center, float radius) {
        Vec3 furthest = new Vec3(center.x, center.y, center.z);

        if (center.x < position.x) furthest.x = position.x + size;
        else if(center.x > position.x + size) furthest.x = position.x;

        if (center.y < position.y) furthest.y = position.y + size;
        else if(center.y > position.y + size) furthest.y = position.y;

        if (center.z < position.z) furthest.z = position.z + size;
        else if(center.z > position.z + size) furthest.z = position.z;

        return Vec3.dist(center, furthest) <= radius;
    }
}
