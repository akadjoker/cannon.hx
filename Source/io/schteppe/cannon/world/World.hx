package io.schteppe.cannon.world;

import io.schteppe.cannon.constraints.Constraint;
import io.schteppe.cannon.constraints.ContactEquation;
import io.schteppe.cannon.constraints.Equation;
import io.schteppe.cannon.constraints.FrictionEquation;
import io.schteppe.cannon.material.ContactMaterial;
import io.schteppe.cannon.material.Material;
import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.math.Quaternion;
import io.schteppe.cannon.objects.Body;
import io.schteppe.cannon.objects.Particle;
import io.schteppe.cannon.objects.RigidBody;
import io.schteppe.cannon.objects.Shape;
import io.schteppe.cannon.solver.Solver;
import io.schteppe.cannon.utils.EventTarget;
import io.schteppe.cannon.solver.GSSolver;

/**
 * The physics world, where all bodies live.
 * An instance of the CANNON.World dispatches the following events:
 * - "preStep", before integration.
 * - "postStep", after integration.
 * - "collide", when a contact occurs.
 * @class CANNON.World
 * @extends CANNON.EventTarget
 */

class World extends EventTarget {

    public var default_dt:Float;
    public var bodies:Array<Body>;
    public var allowSleep:Bool;
    public var contacts:Array<Dynamic>;
    public var frictionEquations:Array<Dynamic>;
    public var quatNormalizeSkip:Int;
    public var quatNormalizeFast:Bool;
    public var time:Float;
    public var stepnumber:Int;
    public var last_dt:Float;
    public var nextId:Int;
    public var gravity:Vec3;
    public var broadphase:Dynamic;
    public var th:World;
    public var solver:GSSolver;
    public var constraints:Array<Dynamic>;
    public var contactgen:ContactGenerator;
    public var collisionMatrix:Array<Dynamic>;
    public var collisionMatrixPrevious:Array<Dynamic>;
    public var materials:Array<Material>;
    public var contactmaterials:Array<ContactMaterial>;
    public var mats2cmat:Array<Dynamic>;
    public var defaultMaterial:Material;
    public var defaultContactMaterial:ContactMaterial;
    public var doProfiling:Bool;
    public var profile:Dynamic;
    public var subsystems:Array<Dynamic>;

    var World_step_postStepEvent:Dynamic;
    var World_step_preStepEvent:Dynamic;
    var World_step_collideEvent:Dynamic;
    var World_step_oldContacts:Array<Dynamic>;
    var World_step_frictionEquationPool:Dynamic;
    var World_step_p1:Array<Body>;
    var World_step_p2:Array<Body>;
    var World_step_gvec:Vec3;
    var World_step_vi:Vec3;
    var World_step_vj:Vec3;
    var World_step_wi:Vec3;
    var World_step_wj:Vec3;
    var World_step_t1:Vec3;
    var World_step_t2:Vec3;
    var World_step_rixn:Vec3;
    var World_step_rjxn:Vec3;
    var World_step_step_q:Quaternion;
    var World_step_step_w:Quaternion;
    var World_step_step_wq:Quaternion;

    public function new() {
        World_step_postStepEvent = {type:"postStep"}; // Reusable event objects to save memory
        World_step_preStepEvent = {type:"preStep"};
        World_step_collideEvent = {type:"collide", "with":null, contact:null };
        World_step_oldContacts = []; // Pools for unused objects
        World_step_frictionEquationPool = [];
        World_step_p1 = []; // Reusable arrays for collision pairs
        World_step_p2 = [];
        World_step_gvec = new Vec3(); // Temporary vectors and quats
        World_step_vi = new Vec3();
        World_step_vj = new Vec3();
        World_step_wi = new Vec3();
        World_step_wj = new Vec3();
        World_step_t1 = new Vec3();
        World_step_t2 = new Vec3();
        World_step_rixn = new Vec3();
        World_step_rjxn = new Vec3();
        World_step_step_q = new Quaternion();
        World_step_step_w = new Quaternion();
        World_step_step_wq = new Quaternion();

        super();

        // @property bool allowSleep
        // @brief Makes bodies go to sleep when they've been inactive
        // @memberof CANNON.World
        allowSleep = false;

        // @property Array contacts
        // @brief All the current contacts (instances of CANNON.ContactEquation) in the world.
        // @memberof CANNON.World
        contacts = [];
        frictionEquations = [];

        // @property int quatNormalizeSkip
        // @brief How often to normalize quaternions. Set to 0 for every step, 1 for every second etc.. A larger value increases performance. If bodies tend to explode, set to a smaller value (zero to be sure nothing can go wrong).
        // @memberof CANNON.World
        quatNormalizeSkip = 0;

        // @property bool quatNormalizeFast
        // @brief Set to true to use fast quaternion normalization. It is often enough accurate to use. If bodies tend to explode, set to false.
        // @memberof CANNON.World
        // @see Quaternion.normalizeFast
        // @see Quaternion.normalize
        quatNormalizeFast = false;

        // @property float time
        // @brief The wall-clock time since simulation start
        // @memberof CANNON.World
        time = 0.0;

        // @property int stepnumber
        // @brief Number of timesteps taken since start
        // @memberof CANNON.World
        stepnumber = 0;

        /// Default and last timestep sizes
        default_dt = 1/60;
        last_dt = this.default_dt;

        nextId = 0;
        // @property Vec3 gravity
        // @memberof CANNON.World
        gravity = new Vec3();

        // @property CANNON.Broadphase broadphase
        // @memberof CANNON.World
        broadphase = null;

        // @property Array bodies
        // @memberof CANNON.World
        bodies = [];

        th = this;

        // @property CANNON.Solver solver
        // @memberof CANNON.World
        solver = new GSSolver();

        // @property Array constraints
        // @memberof CANNON.World
        constraints = [];

        // @property CANNON.ContactGenerator contactgen
        // @memberof CANNON.World
        contactgen = new ContactGenerator();

        // @property Collision "matrix", size (Nbodies * (Nbodies.length + 1))/2 
        //  @brief It's actually a triangular-shaped array of whether two bodies are touching this step, for reference next step
        // @memberof CANNON.World
        collisionMatrix = [];
        // @property Collision "matrix", size (Nbodies * (Nbodies.length + 1))/2 
        //  @brief collisionMatrix from the previous step
        //  @memberof CANNON.World
        collisionMatrixPrevious = [];

        // @property Array materials
        // @memberof CANNON.World
        materials = []; // References to all added materials

        // @property Array contactmaterials
        // @memberof CANNON.World
        contactmaterials = []; // All added contact materials

        mats2cmat = []; // Hash: (mat1_id, mat2_id) => contactmat_id

        defaultMaterial = new Material("default");

        // @property CANNON.ContactMaterial defaultContactMaterial
        // @brief This contact material is used if no suitable contactmaterial is found for a contact.
        // @memberof CANNON.World
        defaultContactMaterial = new ContactMaterial(this.defaultMaterial,this.defaultMaterial,0.3,0.0);

        // @property bool doProfiling
        // @memberof CANNON.World
        doProfiling = false;

        // @property Object profile
        // @memberof CANNON.World
        profile = {
            solve:0,
            makeContactConstraints:0,
            broadphase:0,
            integrate:0,
            nearphase:0,
        };

        // @property Array subystems
        // @memberof CANNON.World
        subsystems = [];
    }

    // @method getContactMaterial
    // @memberof CANNON.World
    // @brief Get the contact material between materials m1 and m2
    // @param CANNON.Material m1
    // @param CANNON.Material m2
    // @return CANNON.Contactmaterial The contact material if it was found.
    public function getContactMaterial(m1:Material, m2:Material):ContactMaterial {
        if (m1 != null && m1 != null) {
            var i = m1.id;
            var j = m2.id;

            if(i<j){
                var temp:Int = i;
                i = j;
                j = temp;
            }

            return this.contactmaterials[this.mats2cmat[i + j * this.materials.length]];
        }
        return null;
    }

    // @method numObjects
    // @memberof CANNON.World
    // @brief Get number of objects in the world.
    // @return int
    function numObjects(){
        return this.bodies.length;
    }

    // Keep track of contacts for current and previous timestep
    // 0: No contact between i and j
    // 1: Contact
    function collisionMatrixGet(i:Int,j:Int,current:Bool = true){
        if(j > i){
            var temp = j;
            j = i;
            i = temp;
        }
        // Reuse i for the index
        i = (i * (i + 1) >> 1) + j - 1;
        return current ? this.collisionMatrix[i] : this.collisionMatrixPrevious[i];
    }

    function collisionMatrixSet(i:Int,j:Int,value:Dynamic,current:Bool = true){
        if(j > i){
            var temp = j;
            j = i;
            i = temp;
        }
        // Reuse i for the index
        i = (i*(i + 1)>>1) + j-1;
        if (current) {
            this.collisionMatrix[i] = value;
        }
        else {
            this.collisionMatrixPrevious[i] = value;
        }
    }

    // transfer old contact state data to T-1
    function collisionMatrixTick(){
        var temp = this.collisionMatrixPrevious;
        this.collisionMatrixPrevious = this.collisionMatrix;
        this.collisionMatrix = temp;
        var l:Int = this.collisionMatrix.length;
        for (i in 0...l) {
            this.collisionMatrix[i]=0;
        }
    }

    // @method add
    // @memberof CANNON.World
    // @brief Add a rigid body to the simulation.
    // @param CANNON.Body body
    // @todo If the simulation has not yet started, why recrete and copy arrays for each body? Accumulate in dynamic arrays in this case.
    // @todo Adding an array of bodies should be possible. This would save some loops too
    public function add(body:Body){
        body.id = this.id();
        body.index = this.bodies.length;
        this.bodies.push(body);
        body.world = this;
        body.position.copy(body.initPosition);
        body.velocity.copy(body.initVelocity);
        body.timeLastSleepy = this.time;
        if (Std.is(body, RigidBody)) {
            var rigidBody:RigidBody = cast(body, RigidBody);
            rigidBody.angularVelocity.copy(rigidBody.initAngularVelocity);
            rigidBody.quaternion.copy(rigidBody.initQuaternion);
        }

        var n = this.numObjects();
        // FIXME
        //this.collisionMatrix.length = n*(n-1)>>1;
    }

    // @method addConstraint
    // @memberof CANNON.World
    // @brief Add a constraint to the simulation.
    // @param CANNON.Constraint c
    public function addConstraint(c){
        this.constraints.push(c);
        c.id = this.id();
    }

    // @method removeConstraint
    // @memberof CANNON.World
    // @brief Removes a constraint
    // @param CANNON.Constraint c
    public function removeConstraint(c){
        var idx = Lambda.indexOf(this.constraints, c);
        if(idx!=-1){
            this.constraints.splice(idx,1);
        }
    }

    // @method id
    // @memberof CANNON.World
    // @brief Generate a new unique integer identifyer
    // @return int
    function id():Int{
        return this.nextId++;
    }

    // @method remove
    // @memberof CANNON.World
    // @brief Remove a rigid body from the simulation.
    // @param CANNON.Body body
    public function remove(body:Body){
        body.world = null;
        var n = this.numObjects()-1;
        var bodies = this.bodies;
        bodies.splice(body.index, 1);
        for(i in (body.index)...n) {
            bodies[i].index=i;
        }
        // FIXME
        throw "Not implemented.";
        //TODO: Maybe splice out the correct elements?
        //this.collisionMatrixPrevious.length = 
        //this.collisionMatrix.length = n*(n-1)>>1;*/
    }

    // @method addMaterial
    // @memberof CANNON.World
    // @brief Adds a material to the World. A material can only be added once, it's added more times then nothing will happen.
    // @param CANNON.Material m
    public function addMaterial(m:Material){
        if(m.id == -1){
            var n = this.materials.length;
            this.materials.push(m);
            m.id = this.materials.length-1;

            // Increase size of collision matrix to (n+1)*(n+1)=n*n+2*n+1 elements, it was n*n last.
            for(i in 0...(2*n+1)){
                this.mats2cmat.push(-1);
            }
        }
    }

    // @method addContactMaterial
    // @memberof CANNON.World
    // @brief Adds a contact material to the World
    // @param CANNON.ContactMaterial cmat
    public function addContactMaterial(cmat:ContactMaterial) {

        // Add materials if they aren't already added
        this.addMaterial(cmat.materials[0]);
        this.addMaterial(cmat.materials[1]);

        // Save (material1,material2) -> (contact material) reference for easy access later
        // Make sure i>j, ie upper right matrix
        var i,j;
        if(cmat.materials[0].id > cmat.materials[1].id){
            i = cmat.materials[0].id;
            j = cmat.materials[1].id;
        } else {
            j = cmat.materials[0].id;
            i = cmat.materials[1].id;
        }

        // Add contact material
        this.contactmaterials.push(cmat);
        cmat.id = this.contactmaterials.length-1;

        // Add current contact material to the material table
        this.mats2cmat[i+this.materials.length*j] = cmat.id; // index of the contact material
    }

    function _now():Float{
        return Date.now().getTime();
    }

    // @method step
    // @memberof CANNON.World
    // @brief Step the simulation
    // @param float dt

    public function step(dt:Float) {
        var world = this;
        var that = this;
        var contacts = this.contacts;
        var p1 = World_step_p1;
        var p2 = World_step_p2;
        var N:Int = this.numObjects();
        var bodies:Array<Body> = this.bodies;
        var solver:Solver = this.solver;
        var gravity:Vec3 = this.gravity;
        var doProfiling = this.doProfiling;
        var profile = this.profile;
        var DYNAMIC:Int = Body.DYNAMIC;
        var now = this._now;
        var profilingStart:Float = 0;
        var constraints = this.constraints;
        var frictionEquation = FrictionEquation;
        var frictionEquationPool = World_step_frictionEquationPool;
        var gnorm:Float = gravity.norm();
        var gx:Float = gravity.x;
        var gy:Float = gravity.y;
        var gz:Float = gravity.z;
        var i:Int=0;

        if(doProfiling){
            profilingStart = now();
        }

        //if(dt===undefined){
        //    dt = this.last_dt || this.default_dt;
        //}

        // Add gravity to all objects
        for(bi in bodies){
            if ((bi.motionstate & DYNAMIC) != 0) { // Only for dynamic bodies
                var f = bi.force;
                var m = bi.mass;
                f.x += m*gx;
                f.y += m*gy;
                f.z += m*gz;
            }
        }

        // Update subsystems
        var Nsubsystems:Int=this.subsystems.length;
        for(i in 0...Nsubsystems){
            this.subsystems[i].update();
        }

        // 1. Collision detection
        if(doProfiling){ profilingStart = now(); }
        p1.splice(0, p1.length); // Clean up pair arrays from last step
        p2.splice(0, p2.length);
        this.broadphase.collisionPairs(this,p1,p2);
        if(doProfiling){ profile.broadphase = now() - profilingStart; }

        this.collisionMatrixTick();

        // Generate contacts
        if(doProfiling){ profilingStart = now(); }
        var oldcontacts = World_step_oldContacts;
        var NoldContacts = contacts.length;

        for(i in 0...NoldContacts){
            oldcontacts.push(contacts[i]);
        }
        contacts.splice(0, contacts.length);

        this.contactgen.getContacts(p1,p2,
                                    this,
                                    contacts,
                                    oldcontacts // To be reused
                                    );
        if(doProfiling){
            profile.nearphase = now() - profilingStart;
        }

        // Loop over all collisions
        if(doProfiling){
            profilingStart = now();
        }
        var ncontacts = contacts.length;

        // Transfer FrictionEquation from current list to the pool for reuse
        var NoldFrictionEquations = this.frictionEquations.length;
        for(i in 0...NoldFrictionEquations){
            frictionEquationPool.push(this.frictionEquations[i]);
        }
        this.frictionEquations.splice(0, this.frictionEquations.length);

        for(k in 0...ncontacts){

            // Current contact
            var c:ContactEquation = contacts[k];

            // Get current collision indeces
            var bi = c.bi; var bj = c.bj;

            // Resolve indeces
            var i = Lambda.indexOf(bodies, bi); var j = Lambda.indexOf(bodies, bj);

            // Get collision properties
            var cm:ContactMaterial = this.getContactMaterial(bi.material, bj.material);
            if (cm == null) cm = this.defaultContactMaterial;
            var mu:Float = cm.friction;
            var e:Float = cm.restitution;

            // g = ( xj + rj - xi - ri ) .dot ( ni )
            var gvec = World_step_gvec;
            gvec.set(bj.position.x + c.rj.x - bi.position.x - c.ri.x,
                     bj.position.y + c.rj.y - bi.position.y - c.ri.y,
                     bj.position.z + c.rj.z - bi.position.z - c.ri.z);
            var g:Float = gvec.dot(c.ni); // Gap, negative if penetration

            // Action if penetration
            if (g < 0.0) { 
                c.restitution = cm.restitution;
                c.penetration = g;
                c.stiffness = cm.contactEquationStiffness;
                c.regularizationTime = cm.contactEquationRegularizationTime;

                solver.addEquation(c);

                // Add friction constraint equation
                if(mu > 0){

                    // Create 2 tangent equations
                    var mug:Float = mu*gnorm;
                    var reducedMass:Float = (bi.invMass + bj.invMass);
                    if(reducedMass > 0.0){
                        reducedMass = 1.0 / reducedMass;
                    }
                    var pool = frictionEquationPool;
                    var c1:FrictionEquation = pool.length > 0 ? pool.pop() : new FrictionEquation(bi,bj,mug*reducedMass);
                    var c2:FrictionEquation = pool.length > 0 ? pool.pop() : new FrictionEquation(bi,bj,mug*reducedMass);
                    this.frictionEquations.push(c1);
                    this.frictionEquations.push(c2);

                    c1.bi = c2.bi = bi;
                    c1.bj = c2.bj = bj;
                    c1.minForce = c2.minForce = -mug*reducedMass;
                    c1.maxForce = c2.maxForce = mug*reducedMass;

                    // Copy over the relative vectors
                    c.ri.copy(c1.ri);
                    c.rj.copy(c1.rj);
                    c.ri.copy(c2.ri);
                    c.rj.copy(c2.rj);

                    // Construct tangents
                    c.ni.tangents(c1.t,c2.t);

                    // Add equations to solver
                    solver.addEquation(c1);
                    solver.addEquation(c2);
                }

                // Now we know that i and j are in contact. Set collision matrix state
                this.collisionMatrixSet(i,j,1,true);

                if(this.collisionMatrixGet(i,j,true)!=this.collisionMatrixGet(i,j,false)){
                    // First contact!
                    // We reuse the collideEvent object, otherwise we will end up creating new objects for each new contact, even if there's no event listener attached.
                    World_step_collideEvent.with = bj;
                    World_step_collideEvent.contact = c;
                    bi.dispatchEvent(World_step_collideEvent);

                    World_step_collideEvent.with = bi;
                    bj.dispatchEvent(World_step_collideEvent);

                    bi.wakeUp();
                    bj.wakeUp();
                }
            }
        }
        if(doProfiling){
            profile.makeContactConstraints = now() - profilingStart;
        }

        if(doProfiling){
            profilingStart = now();
        }

        // Add user-added constraints
        var Nconstraints = constraints.length;
        for(i in 0...Nconstraints){
            var c:Constraint = constraints[i];
            c.update();
            var Neq:Int = c.equations.length;
            for(j in 0...Neq){
                var eq:Equation = c.equations[j];
                solver.addEquation(eq);
            }
        }

        // Solve the constrained system
        solver.solve(dt,this);

        if(doProfiling){
            profile.solve = now() - profilingStart;
        }

        // Remove all contacts from solver
        solver.removeAllEquations();

        // Apply damping, see http://code.google.com/p/bullet/issues/detail?id=74 for details
        var pow = Math.pow;
        for(i in 0...N){
            var bi = bodies[i];
            if((bi.motionstate & DYNAMIC) != 0){ // Only for dynamic bodies
                var ld = pow(1.0 - bi.linearDamping,dt);
                var v = bi.velocity;
                v.mult(ld,v);
                var av = bi.angularVelocity;
                if(av != null){
                    var ad = pow(1.0 - bi.angularDamping,dt);
                    av.mult(ad,av);
                }
            }
        }

        this.dispatchEvent(World_step_postStepEvent);

        // Invoke pre-step callbacks
        for(i in 0...N){
            var bi = bodies[i];
            if(bi.preStep != null){
                bi.preStep(bi);
            }
        }

        // Leap frog
        // vnew = v + h*f/m
        // xnew = x + h*vnew
        if(doProfiling){
            profilingStart = now();
        }
        var q:Quaternion = World_step_step_q;
        var w:Quaternion = World_step_step_w;
        var wq:Quaternion = World_step_step_wq;
        var stepnumber:Int = this.stepnumber;
        var DYNAMIC_OR_KINEMATIC:Int = (Body.DYNAMIC | Body.KINEMATIC);
        var quatNormalize:Bool = stepnumber % (this.quatNormalizeSkip+1) == 0;
        var quatNormalizeFast:Bool = this.quatNormalizeFast;
        var half_dt:Float = dt * 0.5;
        var PLANE:Int = Shape.types.PLANE;
        var CONVEX:Int = Shape.types.CONVEXPOLYHEDRON;

        for(i in 0...N){
            var b:Body = bodies[i];
            var s:Shape = b.shape;
            var force:Vec3 = b.force;
            var tau:Vec3 = b.tau;
            if((b.motionstate & DYNAMIC_OR_KINEMATIC) != 0){ // Only for dynamic
                var velo:Vec3 = b.velocity;
                var angularVelo:Vec3 = b.angularVelocity;
                var pos:Vec3 = b.position;
                var quat:Quaternion = b.quaternion;
                var invMass:Float = b.invMass;
                var invInertia:Vec3 = b.invInertia;
                velo.x += force.x * invMass * dt;
                velo.y += force.y * invMass * dt;
                velo.z += force.z * invMass * dt;

                if(b.angularVelocity != null){
                    angularVelo.x += tau.x * invInertia.x * dt;
                    angularVelo.y += tau.y * invInertia.y * dt;
                    angularVelo.z += tau.z * invInertia.z * dt;
                }

                // Use new velocity  - leap frog
                if(!b.isSleeping()){
                    pos.x += velo.x * dt;
                    pos.y += velo.y * dt;
                    pos.z += velo.z * dt;

                    if(b.angularVelocity != null){
                        w.set(angularVelo.x, angularVelo.y, angularVelo.z, 0.0);
                        w.mult(quat,wq);
                        quat.x += half_dt * wq.x;
                        quat.y += half_dt * wq.y;
                        quat.z += half_dt * wq.z;
                        quat.w += half_dt * wq.w;
                        if(quatNormalize){
                            if(quatNormalizeFast){
                                quat.normalizeFast();
                            } else {
                                quat.normalize();
                            }
                        }
                    }

                    if(b.aabbmin != null){
                        b.aabbNeedsUpdate = true;
                    }
                }

                if(s!=null){
                    switch(s.type){
                    case 2: //PLANE:
                        s.worldNormalNeedsUpdate = true;
                        break;
                    case 16: //CONVEX:
                        s.worldFaceNormalsNeedsUpdate = true;
                        s.worldVerticesNeedsUpdate = true;
                        break;
                    }
                }
            }
            b.force.set(0,0,0);
            if(b.tau != null){
                b.tau.set(0,0,0);
            }
        }

        if(doProfiling){
            profile.integrate = now() - profilingStart;
        }

        // Update world time
        this.time += dt;
        this.stepnumber += 1;

        this.dispatchEvent(World_step_postStepEvent);

        // Invoke post-step callbacks
        for(i in 0...N){
            var bi = bodies[i];
            var postStep = bi.postStep;
            if (postStep != null) {
                postStep(bi);
            }
        }

        // Update world inertias
        // @todo should swap autoUpdate mechanism for .xxxNeedsUpdate
        for(i in 0...N){
            var b = bodies[i];
            if(b.inertiaWorldAutoUpdate){
                b.quaternion.vmult(b.inertia,b.inertiaWorld);
            }
            if(b.invInertiaWorldAutoUpdate){
                b.quaternion.vmult(b.invInertia,b.invInertiaWorld);
            }
        }

        // Sleeping update
        if(this.allowSleep){
            for(i in 0...N){
                bodies[i].sleepTick(this.time);
            }
        }
    }
}
