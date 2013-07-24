package io.schteppe.cannon.objects;

import io.schteppe.cannon.math.Quaternion;
import io.schteppe.cannon.math.Vec3;

/**
 * @brief Spherical rigid body
 * @class CANNON.Sphere
 * @extends CANNON.Shape
 * @param float radius
 * @author schteppe / http://github.com/schteppe
 */

class Sphere extends Shape {

    public var radius:Float;

    public function new(radius:Float = -1.0){
        super();

        /**
         * @property float radius
         * @memberof CANNON.Sphere
         */
        this.radius = radius < 0.0 ? radius : 1.0;
        this.type = Shape.types.SPHERE;
    }

    public override function calculateLocalInertia(mass:Float,target:Vec3 = null):Vec3{
        if (target == null) target = new Vec3();
        var I = 2.0*mass*this.radius*this.radius/5.0;
        target.x = I;
        target.y = I;
        target.z = I;
        return target;
    }

    public override function volume():Float{
        return 4.0 * Math.PI * this.radius / 3.0;
    }

    public override function computeBoundingSphereRadius(){
        this.boundingSphereRadiusNeedsUpdate = false;
        this.boundingSphereRadius = this.radius;
    }

    public override function calculateWorldAABB(pos:Vec3,quat:Quaternion,min:Vec3,max:Vec3){
        var r = this.radius;
        min.x = pos.x - r;
        max.x = pos.x + r;
        min.y = pos.y - r;
        max.y = pos.y + r;
        min.z = pos.z - r;
        max.z = pos.z + r;
    }
}
