package io.schteppe.cannon.collision;

import io.schteppe.cannon.math.Quaternion;
import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.objects.Body;
import io.schteppe.cannon.objects.Plane;
import io.schteppe.cannon.objects.Shape;
import io.schteppe.cannon.objects.Sphere;
import io.schteppe.cannon.world.World;

/**
 * @class CANNON.Broadphase
 * @author schteppe
 * @brief Base class for broadphase implementations
 */
class Broadphase {

    public var world:World;
    public var useBoundingBoxes:Bool;

    var Broadphase_needBroadphaseCollision_STATIC_OR_KINEMATIC:Int;
    var Broadphase_collisionPairs_r:Vec3;
    var Broadphase_collisionPairs_normal:Vec3;
    var Broadphase_collisionPairs_quat:Quaternion;
    var Broadphase_collisionPairs_relpos:Vec3;
    var Broadphase_makePairsUnique_temp:Map<String, Int>;
    var Broadphase_makePairsUnique_p1:Array<Body>;
    var Broadphase_makePairsUnique_p2:Array<Body>;

    public function new(){
        /**
        * @property CANNON.World world
        * @brief The world to search for collisions in.
        * @memberof CANNON.Broadphase
        */
        this.world = null;

        /**
         * If set to true, the broadphase uses bounding boxes for intersection test, else it uses bounding spheres.
         * @property bool useBoundingBoxes
         * @memberof CANNON.Broadphase
         */
        this.useBoundingBoxes = false;

        Broadphase_needBroadphaseCollision_STATIC_OR_KINEMATIC = Body.STATIC | Body.KINEMATIC;
        Broadphase_collisionPairs_r = new Vec3(); // Temp objects
        Broadphase_collisionPairs_normal =  new Vec3();
        Broadphase_collisionPairs_quat =  new Quaternion();
        Broadphase_collisionPairs_relpos  =  new Vec3();
        Broadphase_makePairsUnique_temp = new Map<String, Int>();
        Broadphase_makePairsUnique_p1 = [];
        Broadphase_makePairsUnique_p2 = [];
    }

    /**
     * @method collisionPairs
     * @memberof CANNON.Broadphase
     * @brief Get the collision pairs from the world
     * @param CANNON.World world The world to search in
     * @param Array p1 Empty array to be filled with body objects
     * @param Array p2 Empty array to be filled with body objects
     * @return array An array with two subarrays of body indices
     */
    public function collisionPairs(world:World,p1:Array<Body>,p2:Array<Body>){
        throw "collisionPairs not implemented for this BroadPhase class!";
    }

    public function supportsStaticGeometry():Bool {
        return false;
    }

    /**
     * @method needBroadphaseCollision
     * @memberof CANNON.Broadphase
     * @brief Check if a body pair needs to be intersection tested at all.
     * @param CANNON.Body bodyA
     * @param CANNON.Body bodyB
     * @return bool
     */
    public function needBroadphaseCollision(bodyA:Body,bodyB:Body):Bool{

        // Check collision filter masks
        if( (bodyA.collisionFilterGroup & bodyB.collisionFilterMask)==0 || (bodyB.collisionFilterGroup & bodyA.collisionFilterMask)==0){
            return false;
        }

        // Check motionstate
        if(((bodyA.motionstate & Broadphase_needBroadphaseCollision_STATIC_OR_KINEMATIC)!=0 || bodyA.isSleeping()) &&
           ((bodyB.motionstate & Broadphase_needBroadphaseCollision_STATIC_OR_KINEMATIC)!=0 || bodyB.isSleeping())) {
            // Both bodies are static, kinematic or sleeping. Skip.
            return false;
        }

        // Two particles don't collide
        if(bodyA.shape == null && bodyB.shape == null){
            return false;
        }

        // Two planes don't collide
        if(Std.is(bodyA.shape, Plane) && Std.is(bodyB.shape, Plane)){
            return false;
        }

        return true;
    }

    /**
     * @method intersectionTest
     * @memberof CANNON.Broadphase
     * @brief Check if a body pair needs to be intersection tested at all.
     * @param CANNON.Body bodyA
     * @param CANNON.Body bodyB
     * @return bool
     */
    public function intersectionTest(bi:Body,bj:Body,pairs1:Array<Body>,pairs2:Array<Body>):Bool{
        if(this.useBoundingBoxes){
            this.doBoundingBoxBroadphase(bi,bj,pairs1,pairs2);
        } else {
            this.doBoundingSphereBroadphase(bi,bj,pairs1,pairs2);
        }
        return true;
    }

    /**
     * @method doBoundingSphereBroadphase
     * @memberof CANNON.Broadphase
     * @brief Check if the bounding spheres of two bodies are intersecting.
     * @param CANNON.Body bi
     * @param CANNON.Body bj
     * @param Array pairs1 bi is appended to this array if intersection
     * @param Array pairs2 bj is appended to this array if intersection
     */
    public function doBoundingSphereBroadphase(bi:Body,bj:Body,pairs1:Array<Body>,pairs2:Array<Body>){
        // Local fast access
        var types = Shape.types;
        var BOX_SPHERE_COMPOUND_CONVEX:Int = types.SPHERE | types.BOX | types.COMPOUND | types.CONVEXPOLYHEDRON;
        var PLANE:Int = types.PLANE;
        var STATIC_OR_KINEMATIC:Int = Body.STATIC | Body.KINEMATIC;

        // Temp vecs
        var r = Broadphase_collisionPairs_r;
        var normal = Broadphase_collisionPairs_normal;
        var quat = Broadphase_collisionPairs_quat;
        var relpos = Broadphase_collisionPairs_relpos;

        var bishape = bi.shape, bjshape = bj.shape;
        if(bishape != null && bjshape != null){
            var ti = bishape.type; var tj = bjshape.type;

            // --- Box / sphere / compound / convexpolyhedron collision ---
            if(((ti & BOX_SPHERE_COMPOUND_CONVEX) != 0) && ((tj & BOX_SPHERE_COMPOUND_CONVEX) != 0)){
                // Rel. position
                bj.position.vsub(bi.position,r);

                // Update bounding spheres if needed
                if(bishape.boundingSphereRadiusNeedsUpdate){
                    bishape.computeBoundingSphereRadius();
                }
                if(bjshape.boundingSphereRadiusNeedsUpdate){
                    bjshape.computeBoundingSphereRadius();
                }

                var boundingRadiusSum:Float = bishape.boundingSphereRadius + bjshape.boundingSphereRadius;
                if(r.norm2() < boundingRadiusSum*boundingRadiusSum){
                    pairs1.push(bi);
                    pairs2.push(bj);
                }

                // --- Sphere/box/compound/convexpoly versus plane ---
            } else if(((ti & BOX_SPHERE_COMPOUND_CONVEX) != 0) && ((tj & types.PLANE) != 0) || ((tj & BOX_SPHERE_COMPOUND_CONVEX) != 0) && ((ti & types.PLANE) != 0)){
                var planeBody = (ti == PLANE) ? bi : bj; // Plane
                var otherBody = (ti != PLANE) ? bi : bj; // Other

                var otherShape = otherBody.shape;
                var planeShape = cast(planeBody.shape, Plane);

                // Rel. position
                otherBody.position.vsub(planeBody.position,r);

                if(planeShape.worldNormalNeedsUpdate){
                    planeShape.computeWorldNormal(planeBody.quaternion);
                }

                normal = planeShape.worldNormal;

                if(otherShape.boundingSphereRadiusNeedsUpdate){
                    otherShape.computeBoundingSphereRadius();
                }

                var q:Float = r.dot(normal) - otherShape.boundingSphereRadius;
                if(q < 0.0){
                    pairs1.push(bi);
                    pairs2.push(bj);
                }
            }
        } else {
            // Particle without shape
            if(bishape == null && bjshape == null){
                // No collisions between 2 particles
            } else {
                var particle = bishape != null ? bj : bi;
                var other = bishape != null ? bi : bj;
                var otherShape = other.shape;
                var type = otherShape.type;

                if((type & BOX_SPHERE_COMPOUND_CONVEX) != 0){
                    if(type == types.SPHERE){ // particle-sphere
                        particle.position.vsub(other.position, relpos);
                        var otherSphere:Sphere = cast(otherShape, Sphere);
                        if(otherSphere.radius*otherSphere.radius >= relpos.norm2()){
                            pairs1.push(particle);
                            pairs2.push(other);
                        }
                    } else if(type==types.CONVEXPOLYHEDRON || type==types.BOX || type==types.COMPOUND){

                        if(otherShape.boundingSphereRadiusNeedsUpdate){
                            otherShape.computeBoundingSphereRadius();
                        }
                        var R:Float = otherShape.boundingSphereRadius;
                        particle.position.vsub(other.position,relpos);
                        if(R*R >= relpos.norm2()){
                            pairs1.push(particle);
                            pairs2.push(other);
                        }
                    }
                } else if(type == types.PLANE){
                    // particle/plane
                    var plane = other;
                    normal.set(0,0,1);
                    plane.quaternion.vmult(normal,normal);
                    particle.position.vsub(plane.position,relpos);
                    if(normal.dot(relpos)<=0.0){
                        pairs1.push(particle);
                        pairs2.push(other);
                    }
                }
            }
        }
    }

    /**
     * @method doBoundingBoxBroadphase
     * @memberof CANNON.Broadphase
     * @brief Check if the bounding boxes of two bodies are intersecting.
     * @param CANNON.Body bi
     * @param CANNON.Body bj
     * @param Array pairs1
     * @param Array pairs2
     */
    public function doBoundingBoxBroadphase(bi:Body,bj:Body,pairs1:Array<Body>,pairs2:Array<Body>){
        var bishape = bi.shape;
        var bjshape = bj.shape;

        if(bi.aabbNeedsUpdate){
            bi.computeAABB();
        }
        if(bj.aabbNeedsUpdate){
            bj.computeAABB();
        }

        if(bishape != null && bjshape != null){
            // Check AABB / AABB
            if( !(  bi.aabbmax.x < bj.aabbmin.x ||
                    bi.aabbmax.y < bj.aabbmin.y ||
                    bi.aabbmax.z < bj.aabbmin.z ||
                    bi.aabbmin.x > bj.aabbmax.x ||
                    bi.aabbmin.y > bj.aabbmax.y ||
                    bi.aabbmin.z > bj.aabbmax.z   ) ){
                pairs1.push(bi);
                pairs2.push(bj);
            }
        } else {
            // Particle without shape
            if(bishape == null && bjshape == null){
                // No collisions between 2 particles
            } else {
                // particle vs AABB
                var p:Body =      bishape == null ? bi : bj;
                var other:Body =  bishape == null ? bj : bi;

                if(Std.is(other.shape, Plane)){
                    //console.log(p.position.z+"<"+other.aabbmin.z+" = ",p.position.z < other.aabbmin.z);
                }

                if( !(  p.position.x < other.aabbmin.x ||
                        p.position.y < other.aabbmin.y ||
                        p.position.z < other.aabbmin.z ||
                        p.position.x > other.aabbmax.x ||
                        p.position.y > other.aabbmax.y ||
                        p.position.z > other.aabbmax.z   ) ){
                    pairs1.push(bi);
                    pairs2.push(bj);
                }
            }
        }
    }

    public function addStaticBody(
            bi:Body):Void {
        throw "Static geometry not impelemented for this broadphase type.";
    }

    /**
     * @method makePairsUnique
     * @memberof CANNON.Broadphase
     * @brief Removes duplicate pairs from the pair arrays.
     * @param Array pairs1
     * @param Array pairs2
     */
    public function makePairsUnique (pairs1:Array<Body>, pairs2:Array<Body>) {
        var t:Map<String, Int> = Broadphase_makePairsUnique_temp;
        var p1 = Broadphase_makePairsUnique_p1;
        var p2 = Broadphase_makePairsUnique_p2;
        var N = pairs1.length;

        for(i in 0...N){
            p1[i] = pairs1[i];
            p2[i] = pairs2[i];
        }

        pairs1.splice(0, pairs1.length);
        pairs2.splice(0, pairs2.length);

        for(i in 0...N){
            var id1 = p1[i].id;
            var id2 = p2[i].id;
            var idx = id1 < id2 ? "" + id1 + "," + id2 : "" + id2 + "," + id1;
            t.set(idx, i);
        }

        for (idx in t) {
            pairs1.push(p1[idx]);
            pairs2.push(p2[idx]);
        }

        for (id in t.keys()) {
            t.remove(id);
        }
    }
}
