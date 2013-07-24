package io.schteppe.cannon.objects;

import io.schteppe.cannon.math.Quaternion;
import io.schteppe.cannon.math.Vec3;

/**
 * @class CANNON.Shape
 * @author schteppe
 * @brief Base class for shapes
 * @todo Should have a mechanism for caching bounding sphere radius instead of calculating it each time
 */
class Shape {

    public var type:Int;

    public var aabbmin:Vec3;
    public var aabbmax:Vec3;

    public var boundingSphereRadius:Float;
    public var boundingSphereRadiusNeedsUpdate:Bool;

    //FIXME: Change this back to ConvexPolyhedron
    public var convexPolyhedronRepresentation:Dynamic;//ConvexPolyhedron;

    var Shape_calculateTransformedInertia_localInertia:Vec3;
    var Shape_calculateTransformedInertia_worldInertia:Vec3;

    // Update flags for Plane and ConvexPoly
    public var worldVerticesNeedsUpdate:Bool;
    public var worldNormalNeedsUpdate:Bool;
    public var worldFaceNormalsNeedsUpdate:Bool;

    public function new(){

        /**
         * @property int type
         * @memberof CANNON.Shape
         * @brief The type of this shape. Must be set to an int > 0 by subclasses.
         * @see CANNON.Shape.types
         */
        this.type = 0;

        this.aabbmin = new Vec3();
        this.aabbmax = new Vec3();

        this.boundingSphereRadius = 0;
        this.boundingSphereRadiusNeedsUpdate = true;

        Shape_calculateTransformedInertia_localInertia = new Vec3();
        Shape_calculateTransformedInertia_worldInertia = new Vec3();
    }

    /**
     * @method computeBoundingSphereRadius
     * @memberof CANNON.Shape
     * @brief Computes the bounding sphere radius. The result is stored in the property .boundingSphereRadius
     * @return float
     */
    public function computeBoundingSphereRadius(){
        throw "computeBoundingSphereRadius() not implemented for shape type "+this.type;
    }

    /**
     * @method getBoundingSphereRadius
     * @memberof CANNON.Shape
     * @brief Returns the bounding sphere radius. The result is stored in the property .boundingSphereRadius
     * @return float
     */
    public function getBoundingSphereRadius(){
        if (this.boundingSphereRadiusNeedsUpdate) {
            this.computeBoundingSphereRadius();
        }
        return this.boundingSphereRadius;
    }

    /**
     * @method volume
     * @memberof CANNON.Shape
     * @brief Get the volume of this shape
     * @return float
     */
    public function volume():Float {
        throw "volume() not implemented for shape type " + this.type;
        return 0;
    }

    /**
     * @method calculateLocalInertia
     * @memberof CANNON.Shape
     * @brief Calculates the inertia in the local frame for this shape.
     * @return Vec3
     * @see http://en.wikipedia.org/wiki/List_of_moments_of_inertia
     */
    public function calculateLocalInertia(mass:Float,target:Vec3 = null):Vec3{
        throw "calculateLocalInertia() not implemented for shape type " + this.type;
        return null;
    }

    /**
     * @method calculateTransformedInertia
     * @memberof CANNON.Shape
     * @brief Calculates inertia in a specified frame for this shape.
     * @return Vec3
     */
    public function calculateTransformedInertia(mass,quat,target):Vec3{
        if (target == null) target = new Vec3();

        // Compute inertia in the world frame
        //quat.normalize();
        var localInertia = Shape_calculateTransformedInertia_localInertia;
        var worldInertia = Shape_calculateTransformedInertia_worldInertia;
        this.calculateLocalInertia(mass,localInertia);

        // @todo Is this rotation OK? Check!
        quat.vmult(localInertia,worldInertia);
        target.x = Math.abs(worldInertia.x);
        target.y = Math.abs(worldInertia.y);
        target.z = Math.abs(worldInertia.z);
        return target;
    }

    // Calculates the local aabb and sets the result to .aabbmax and .aabbmin
    public function calculateLocalAABB(){
        throw "calculateLocalAABB is not implemented for this Shape yet!";
    }

    public function calculateWorldAABB(pos:Vec3, quat:Quaternion, min:Vec3, max:Vec3) {
        throw "calculateWorldAABB is not implemented for this Shape yet!";
    }

    /**
     * @property Object types
     * @memberof CANNON.Shape
     * @brief The available shape types.
     */
    public static var types:Dynamic ={
        SPHERE:1,
        PLANE:2,
        BOX:4,
        COMPOUND:8,
        CONVEXPOLYHEDRON:16
    };
}
