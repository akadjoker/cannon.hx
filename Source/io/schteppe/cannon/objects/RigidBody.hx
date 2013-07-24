package io.schteppe.cannon.objects;

import io.schteppe.cannon.material.Material;
import io.schteppe.cannon.math.Quaternion;
import io.schteppe.cannon.math.Vec3;

/**
 * @class CANNON.RigidBody
 * @brief Rigid body base class
 * @param float mass
 * @param CANNON.Shape shape
 * @param CANNON.Material material
 */
class RigidBody extends Particle {

    public var that:Dynamic;

    var RigidBody_applyForce_r:Vec3;
    var RigidBody_applyForce_rotForce:Vec3;
    var RigidBody_applyImpulse_r:Vec3;
    var RigidBody_applyImpulse_velo:Vec3;
    var RigidBody_applyImpulse_rotVelo:Vec3;

    public function new(mass:Float,shape:Shape,material:Material = null){
        super(mass,material);

        var that = this;

        /**
         * @property Vec3 tau
         * @memberof CANNON.RigidBody
         * @brief Rotational force on the body, around center of mass
         */
        this.tau = new Vec3();

        /**
         * @property Quaternion quaternion
         * @memberof CANNON.RigidBody
         * @brief Orientation of the body
         */
        this.quaternion = new Quaternion();

        /**
         * @property Quaternion initQuaternion
         * @memberof CANNON.RigidBody
         */
        this.initQuaternion = new Quaternion();

        /**
         * @property Vec3 angularVelocity
         * @memberof CANNON.RigidBody
         */
        this.angularVelocity = new Vec3();

        /**
         * @property Vec3 initAngularVelocity
         * @memberof CANNON.RigidBody
         */
        this.initAngularVelocity = new Vec3();

        /**
         * @property CANNON.Shape shape
         * @memberof CANNON.RigidBody
         */
        this.shape = shape;

        /**
         * @property Vec3 inertia
         * @memberof CANNON.RigidBody
         */
        this.inertia = new Vec3();
        shape.calculateLocalInertia(mass,this.inertia);

        this.inertiaWorld = new Vec3();
        this.inertia.copy(this.inertiaWorld);
        this.inertiaWorldAutoUpdate = false;

        /**
         * @property Vec3 intInertia
         * @memberof CANNON.RigidBody
         */
        this.invInertia = new Vec3(this.inertia.x>0 ? 1.0/this.inertia.x : 0,
                                          this.inertia.y>0 ? 1.0/this.inertia.y : 0,
                                          this.inertia.z>0 ? 1.0/this.inertia.z : 0);
        this.invInertiaWorld = new Vec3();
        this.invInertia.copy(this.invInertiaWorld);
        this.invInertiaWorldAutoUpdate = false;

        /**
         * @property float angularDamping
         * @memberof CANNON.RigidBody
         */
        this.angularDamping = 0.01; // Perhaps default should be zero here?

        /**
         * @property Vec3 aabbmin
         * @memberof CANNON.RigidBody
         */
        this.aabbmin = new Vec3();

        /**
         * @property Vec3 aabbmax
         * @memberof CANNON.RigidBody
         */
        this.aabbmax = new Vec3();

        /**
         * @property bool aabbNeedsUpdate
         * @memberof CANNON.RigidBody
         * @brief Indicates if the AABB needs to be updated before use.
         */
        this.aabbNeedsUpdate = true;

        this.wlambda = new Vec3();
    }

    public override function computeAABB(){
        this.shape.calculateWorldAABB(this.position,
                                      this.quaternion,
                                      this.aabbmin,
                                      this.aabbmax);
        this.aabbNeedsUpdate = false;
    }

    /**
     * Apply force to a world point. This could for example be a point on the RigidBody surface. Applying force this way will add to Body.force and Body.tau.
     * @param  Vec3 force The amount of force to add.
     * @param  Vec3 worldPoint A world point to apply the force on.
     */
    public function applyForce(force:Vec3,worldPoint:Vec3){
        // Compute point position relative to the body center
        var r = RigidBody_applyForce_r;
        worldPoint.vsub(this.position,r);

        // Compute produced rotational force
        var rotForce = RigidBody_applyForce_rotForce;
        r.cross(force,rotForce);

        // Add linear force
        this.force.vadd(force,this.force);

        // Add rotational force
        this.tau.vadd(rotForce,this.tau);
    }

    /**
     * Apply impulse to a world point. This could for example be a point on the RigidBody surface. An impulse is a force added to a body during a short period of time (impulse = force * time). Impulses will be added to Body.velocity and Body.angularVelocity.
     * @param  Vec3 impulse The amount of impulse to add.
     * @param  Vec3 worldPoint A world point to apply the force on.
     */
    public function applyImpulse(impulse:Vec3,worldPoint:Vec3){
        // Compute point position relative to the body center
        var r = RigidBody_applyImpulse_r;
        worldPoint.vsub(this.position,r);

        // Compute produced central impulse velocity
        var velo:Vec3 = RigidBody_applyImpulse_velo;
        impulse.copy(velo);
        velo.mult(this.invMass,velo);

        // Add linear impulse
        this.velocity.vadd(velo, this.velocity);

        // Compute produced rotational impulse velocity
        var rotVelo = RigidBody_applyImpulse_rotVelo;
        r.cross(impulse,rotVelo);
        rotVelo.x *= this.invInertia.x;
        rotVelo.y *= this.invInertia.y;
        rotVelo.z *= this.invInertia.z;

        // Add rotational Impulse
        this.angularVelocity.vadd(rotVelo, this.angularVelocity);
    }
}
