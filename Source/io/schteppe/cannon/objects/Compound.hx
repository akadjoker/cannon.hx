package io.schteppe.cannon.objects;

import io.schteppe.cannon.math.Quaternion;
import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.objects.Shape;
/**
 * @class CANNON.Compound
 * @extends CANNON.Shape
 * @brief A shape made of several other shapes.
 * @author schteppe
 */
class Compound extends Shape {

    public var childShapes:Array<Shape>;
    public var childOffsets:Array<Vec3>;
    public var childOrientations:Array<Quaternion>;

    var Compound_calculateLocalInertia_mr2:Vec3;
    var Compound_calculateLocalInertia_childInertia:Vec3;
    var aabbmaxTemp:Vec3;
    var aabbminTemp:Vec3;
    var childPosTemp:Vec3;
    var childQuatTemp:Quaternion;

    public function new(){
        super();
        this.type = Shape.types.COMPOUND;
        this.childShapes = [];
        this.childOffsets = [];
        this.childOrientations = [];

        Compound_calculateLocalInertia_mr2 = new Vec3();
        Compound_calculateLocalInertia_childInertia = new Vec3();
        aabbmaxTemp = new Vec3();
        aabbminTemp = new Vec3();
        childPosTemp = new Vec3();
        childQuatTemp = new Quaternion();
    }

    /**
     * @method addChild
     * @memberof CANNON.Compound
     * @brief Add a child shape.
     * @param CANNON.Shape shape
     * @param Vec3 offset
     * @param Quaternion orientation
     */
    public function addChild(shape:Shape,offset:Vec3,orientation:Quaternion){
        if (offset == null) offset = new Vec3();
        if (orientation == null) orientation = new Quaternion();
        this.childShapes.push(shape);
        this.childOffsets.push(offset);
        this.childOrientations.push(orientation);
    }

    public override function volume():Float{
        var r = 0.0;
        var Nchildren = this.childShapes.length;
        for(i in 0...Nchildren){
            r += this.childShapes[i].volume();
        }
        return r;
    }

    public override function calculateLocalInertia(mass:Float,target:Vec3 = null):Vec3{
        if (target == null) target = new Vec3();

        // Calculate the total volume, we will spread out this objects' mass on the sub shapes
        var V:Float = this.volume();
        var childInertia = Compound_calculateLocalInertia_childInertia;
        var Nchildren = this.childShapes.length;
        for(i in 0...Nchildren){
            // Get child information
            var b:Shape = this.childShapes[i];
            var o:Vec3 = this.childOffsets[i];
            var q:Quaternion = this.childOrientations[i];
            var m:Float = b.volume() / V * mass;

            // Get the child inertia, transformed relative to local frame
            ////var inertia = b.calculateTransformedInertia(m,q);
            b.calculateLocalInertia(m,childInertia); // Todo transform!
            ////console.log(childInertia,m,b.volume(),V);

            //// Add its inertia using the parallel axis theorem, i.e.
            //// I += I_child;    
            //// I += m_child * r^2

            target.vadd(childInertia,target);
            var mr2 = Compound_calculateLocalInertia_mr2;
            mr2.set(m*o.x*o.x,
                    m*o.y*o.y,
                    m*o.z*o.z);
            target.vadd(mr2,target);
        }

        return target;
    }

    public override function computeBoundingSphereRadius(){
        var r:Float = 0.0;
        var csl:Int = this.childShapes.length;
        for(i in 0...csl){
            var si = this.childShapes[i];
            if(si.boundingSphereRadiusNeedsUpdate){
                si.computeBoundingSphereRadius();
            }
            var candidate:Float = this.childOffsets[i].norm() + si.boundingSphereRadius;
            if(r < candidate){
                r = candidate;
            }
        }
        this.boundingSphereRadius = r;
        this.boundingSphereRadiusNeedsUpdate = false;
    }

    public override function calculateWorldAABB(pos:Vec3, quat:Quaternion, min:Vec3, max:Vec3) {
        var N=this.childShapes.length;
        min.set(Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY);
        max.set(Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY);
        // Get each axis max
        for(i in 0...N){

            // Accumulate transformation to child
            this.childOffsets[i].copy(childPosTemp);
            quat.vmult(childPosTemp,childPosTemp);
            pos.vadd(childPosTemp,childPosTemp);

            quat.mult(this.childOrientations[i],childQuatTemp);

            // Get child AABB
            this.childShapes[i].calculateWorldAABB(childPosTemp,
                                                   childQuatTemp,//this.childOrientations[i],
                                                   aabbminTemp,
                                                   aabbmaxTemp);

            if(aabbminTemp.x < min.x){
                min.x = aabbminTemp.x;
            }
            if(aabbminTemp.y < min.y){
                min.y = aabbminTemp.y;
            }
            if(aabbminTemp.z < min.z){
                min.z = aabbminTemp.z;
            }

            if(aabbmaxTemp.x > max.x){
                max.x = aabbmaxTemp.x;
            }
            if(aabbmaxTemp.y > max.y){
                max.y = aabbmaxTemp.y;
            }
            if(aabbmaxTemp.z > max.z){
                max.z = aabbmaxTemp.z;
            }
        }
    }
}
