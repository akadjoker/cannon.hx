package io.schteppe.cannon.objects;

import io.schteppe.cannon.math.Quaternion;
import io.schteppe.cannon.math.Vec3;

/**
 * @class CANNON.Box
 * @brief A 3d box shape.
 * @param Vec3 halfExtents
 * @author schteppe
 * @extends CANNON.Shape
 */
class Box extends Shape {

    public var halfExtents:Vec3;

    var worldCornerTempPos:Vec3;
    var worldCornerTempNeg:Vec3;

    public function new(halfExtents:Vec3){
        super();

        worldCornerTempPos = new Vec3();
        worldCornerTempNeg = new Vec3();

        /**
        * @property Vec3 halfExtents
        * @memberof CANNON.Box
        */
        this.halfExtents = halfExtents;
        this.type = Shape.types.BOX;

        /**
        * @property CANNON.ConvexPolyhedron convexPolyhedronRepresentation
        * @brief Used by the contact generator to make contacts with other convex polyhedra for example
        * @memberof CANNON.Box
        */
        this.convexPolyhedronRepresentation = null;

        this.updateConvexPolyhedronRepresentation();
    }

    /**
     * @method updateConvexPolyhedronRepresentation
     * @memberof CANNON.Box
     * @brief Updates the local convex polyhedron representation used for some collisions.
     */
    public function updateConvexPolyhedronRepresentation(){
        var sx:Float = this.halfExtents.x;
        var sy:Float = this.halfExtents.y;
        var sz:Float = this.halfExtents.z;

        var h = new ConvexPolyhedron([new Vec3(-sx,-sy,-sz),
                                             new Vec3( sx,-sy,-sz),
                                             new Vec3( sx, sy,-sz),
                                             new Vec3(-sx, sy,-sz),
                                             new Vec3(-sx,-sy, sz),
                                             new Vec3( sx,-sy, sz),
                                             new Vec3( sx, sy, sz),
                                             new Vec3(-sx, sy, sz)],
                                             [[3,2,1,0], // -z
                                              [4,5,6,7], // +z
                                              [5,4,1,0], // -y
                                              [2,3,6,7], // +y
                                              [0,4,7,3], // -x
                                              [1,2,5,6], // +x
                                              ],
                                            [new Vec3( 0, 0,-1),
                                             new Vec3( 0, 0, 1),
                                             new Vec3( 0,-1, 0),
                                             new Vec3( 0, 1, 0),
                                             new Vec3(-1, 0, 0),
                                             new Vec3( 1, 0, 0)]);
        this.convexPolyhedronRepresentation = h;
    }

    public override function calculateLocalInertia(mass:Float,target:Vec3 = null):Vec3{
        if (target == null) target = new Vec3();
        var e = this.halfExtents;
        target.x = 1.0 / 12.0 * mass * (   2*e.y*2*e.y + 2*e.z*2*e.z );
        target.y = 1.0 / 12.0 * mass * (   2*e.x*2*e.x + 2*e.z*2*e.z );
        target.z = 1.0 / 12.0 * mass * (   2*e.y*2*e.y + 2*e.x*2*e.x );
        return target;
    }

    /**
     * @method getSideNormals
     * @memberof CANNON.Box
     * @brief Get the box 6 side normals
     * @param bool includeNegative If true, this function returns 6 vectors. If false, it only returns 3 (but you get 6 by reversing those 3)
     * @param Quaternion quat Orientation to apply to the normal vectors. If not provided, the vectors will be in respect to the local frame.
     * @return array
     */
    public function getSideNormals(sixTargetVectors:Array<Vec3>,quat:Quaternion = null):Array<Vec3>{
        var sides = sixTargetVectors;
        var ex = this.halfExtents;
        sides[0].set(  ex.x,     0,     0);
        sides[1].set(     0,  ex.y,     0);
        sides[2].set(     0,     0,  ex.z);
        sides[3].set( -ex.x,     0,     0);
        sides[4].set(     0, -ex.y,     0);
        sides[5].set(     0,     0, -ex.z);

        if (quat != null) {
            var nsides:Int = sides.length;
            for(i in 0...nsides){
                quat.vmult(sides[i],sides[i]);
            }
        }

        return sides;
    }

    public override function volume():Float{
        return 8.0 * this.halfExtents.x * this.halfExtents.y * this.halfExtents.z;
    }

    public override function computeBoundingSphereRadius(){
        this.boundingSphereRadius = this.halfExtents.norm();
        this.boundingSphereRadiusNeedsUpdate = false;
    }

    public function forEachWorldCorner(pos:Vec3,quat:Quaternion,call_back:Dynamic){
        var e = this.halfExtents;
        var corners:Array<Array<Float>> =
                      [[  e.x,  e.y,  e.z],
                       [ -e.x,  e.y,  e.z],
                       [ -e.x, -e.y,  e.z],
                       [ -e.x, -e.y, -e.z],
                       [  e.x, -e.y, -e.z],
                       [  e.x,  e.y, -e.z],
                       [ -e.x,  e.y, -e.z],
                       [  e.x, -e.y,  e.z]];
        var n:Int = corners.length;
        for(i in 0...n){
            worldCornerTempPos.set(corners[i][0],corners[i][1],corners[i][2]);
            quat.vmult(worldCornerTempPos,worldCornerTempPos);
            pos.vadd(worldCornerTempPos,worldCornerTempPos);
            call_back(worldCornerTempPos.x,
                     worldCornerTempPos.y,
                     worldCornerTempPos.z);
        }
    }

    public override function calculateWorldAABB(pos:Vec3,quat:Quaternion,min:Vec3,max:Vec3){
        // Get each axis max
        min.set(Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY);
        max.set(Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY);
        this.forEachWorldCorner(pos,quat,function(x:Float,y:Float,z:Float){
            if(x > max.x){
                max.x = x;
            }
            if(y > max.y){
                max.y = y;
            }
            if(z > max.z){
                max.z = z;
            }

            if(x < min.x){
                min.x = x;
            }
            if(y < min.y){
                min.y = y;
            }
            if(z < min.z){
                min.z = z;
            }
        });
    }
}
