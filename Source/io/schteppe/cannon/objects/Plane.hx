package io.schteppe.cannon.objects;

import io.schteppe.cannon.math.Quaternion;
import io.schteppe.cannon.math.Vec3;

/**
 * @class CANNON.Plane
 * @extends CANNON.Shape
 * @param Vec3 normal
 * @brief A plane, facing in the Z direction.
 * @description A plane, facing in the Z direction. The plane has its surface at z=0 and everything below z=0 is assumed to be solid plane. To make the plane face in some other direction than z, you must put it inside a RigidBody and rotate that body. See the demos.
 * @author schteppe
 */

class Plane extends Shape {

    public var worldNormal:Vec3;

    var tempNormal:Vec3;

    public function new(){
        super();

        this.type = Shape.types.PLANE;

        // World oriented normal
        this.worldNormal = new Vec3();
        this.worldNormalNeedsUpdate = true;

        tempNormal = new Vec3();
    }

    public function computeWorldNormal(quat){
        var n = this.worldNormal;
        n.set(0,0,1);
        quat.vmult(n,n);
        this.worldNormalNeedsUpdate = false;
    }

    public override function calculateLocalInertia(mass:Float,target:Vec3 = null):Vec3{
        if (target == null) target = new Vec3();
        return target;
    }

    public override function volume(){
        return Math.POSITIVE_INFINITY; // The plane is infinite...
    }

    public override function calculateWorldAABB(pos:Vec3,quat:Quaternion,min:Vec3,max:Vec3){
        // The plane AABB is infinite, except if the normal is pointing along any axis
        tempNormal.set(0,0,1); // Default plane normal is z
        quat.vmult(tempNormal,tempNormal);
        min.set(Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY);
        max.set(Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY);

        if(tempNormal.x == 1){ max.x = pos.x; }
        if(tempNormal.y == 1){ max.y = pos.y; }
        if(tempNormal.z == 1){ max.z = pos.z; }

        if(tempNormal.x == -1){ min.x = pos.x; }
        if(tempNormal.y == -1){ min.y = pos.y; }
        if(tempNormal.z == -1){ min.z = pos.z; }

    }
}
