package io.schteppe.cannon.objects;

import io.schteppe.cannon.material.Material;
import io.schteppe.cannon.math.Quaternion;
import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.utils.EventTarget;

/**
 * @class CANNON.Body
 * @brief Base class for all body types.
 * @param string type
 * @extends CANNON.EventTarget
 * @event collide The body object dispatches a "collide" event whenever it collides with another body. Event parameters are "with" (the body it collides with) and "contact" (the contact equation that is generated).
 */

class Body extends EventTarget {

    // This is set by world when a body is created
    public var index:Int;

    // Particle properties
    public var position:Vec3;
    public var initPosition:Vec3;
    public var velocity:Vec3;
    public var initVelocity:Vec3;
    public var force:Vec3;
    public var mass:Float;
    public var invMass:Float;
    public var material:Material;
    public var linearDamping:Float;
    public var motionstate:Int;
    public var allowSleep:Bool;
    public var sleepState:Int;
    public var sleepSpeedLimit:Float;
    public var sleepTimeLimit:Int;
    public var timeLastSleepy:Float;

    // RigidBody properties
    public var tau:Vec3;
    public var quaternion:Quaternion;
    public var initQuaternion:Quaternion;
    public var angularVelocity:Vec3;
    public var initAngularVelocity:Vec3;
    public var shape:Shape;
    public var inertia:Vec3;
    public var inertiaWorld:Vec3;
    public var inertiaWorldAutoUpdate:Bool;
    public var invInertia:Vec3;
    public var invInertiaWorld:Vec3;
    public var invInertiaWorldAutoUpdate:Bool;
    public var angularDamping:Float;
    public var aabbmin:Vec3;
    public var aabbmax:Vec3;
    public var aabbNeedsUpdate:Bool;
    public var wlambda:Vec3;

    public var id:Int;
    public var type:String;
    public var world:Dynamic;
    public var preStep:Dynamic;
    public var postStep:Dynamic;
    public var vlambda:Vec3;
    public var collisionFilterGroup:Int;
    public var collisionFilterMask:Int;

    public function new(type:String){
        super();

        this.type = type;

        /**
        * @property CANNON.World world
        * @memberof CANNON.Body
        * @brief Reference to the world the body is living in
        */
        this.world = null;

        /**
        * @property function preStep
        * @memberof CANNON.Body
        * @brief Callback function that is used BEFORE stepping the system. Use it to apply forces, for example. Inside the function, "this" will refer to this CANNON.Body object.
        * @todo dispatch an event from the World instead
        */
        this.preStep = null;

        /**
        * @property function postStep
        * @memberof CANNON.Body
        * @brief Callback function that is used AFTER stepping the system. Inside the function, "this" will refer to this CANNON.Body object.
        * @todo dispatch an event from the World instead
        */
        this.postStep = null;

        this.vlambda = new Vec3();

        this.collisionFilterGroup = 1;
        this.collisionFilterMask = 1;
    }

    public function wakeUp() { }
    public function sleepTick(time:Float) { }
    public function isAwake():Bool { return false; }
    public function isSleepy():Bool{ return false; }
    public function isSleeping():Bool { return false; }
    public function computeAABB() { }
 
    /*
     * @brief A dynamic body is fully simulated. Can be moved manually by the user, but normally they move according to forces. A dynamic body can collide with all body types. A dynamic body always has finite, non-zero mass.
     */
    public static var DYNAMIC:Int = 1;

    /*
     * @brief A static body does not move during simulation and behaves as if it has infinite mass. Static bodies can be moved manually by setting the position of the body. The velocity of a static body is always zero. Static bodies do not collide with other static or kinematic bodies.
     */
    public static var STATIC:Int = 2;

    /*
     * A kinematic body moves under simulation according to its velocity. They do not respond to forces. They can be moved manually, but normally a kinematic body is moved by setting its velocity. A kinematic body behaves as if it has infinite mass. Kinematic bodies do not collide with other static or kinematic bodies.
     */
    public static var KINEMATIC:Int = 4;
}
