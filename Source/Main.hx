package;

import flash.display.Sprite;
import flash.events.Event;

import io.schteppe.cannon.world.World;
import io.schteppe.cannon.collision.NaiveBroadphase;
import io.schteppe.cannon.objects.Plane;
import io.schteppe.cannon.objects.RigidBody;
import io.schteppe.cannon.objects.Sphere;
import io.schteppe.cannon.objects.Box;
import io.schteppe.cannon.objects.Compound;
import io.schteppe.cannon.objects.Cylinder;

class Main extends Sprite {

    public function new() {
        super ();

        // Setup our world
        var world = new World();
        world.gravity.set(0,0,-9.82);
        world.broadphase = new NaiveBroadphase();
        world.solver.iterations = 3;
        //world.solver.setSpookParams(1000,3);
        world.solver.tolerance = 0.01;

        // Create a sphere
        var mass:Float = 5;
        var radius:Float = 1;
        var damping:Float = 0.01;

        var sphereShape = new Sphere(radius);
        var sphereBody = new RigidBody(mass, sphereShape);
        sphereBody.linearDamping = damping;
        sphereBody.position.set(0,0,10);
        world.add(sphereBody);

        // Create a plane
        var groundShape = new Plane();
        var groundBody = new RigidBody(0,groundShape);
        world.add(groundBody);

        // Step the simulation
        this.addEventListener(Event.ENTER_FRAME, function(e:Event) {
            world.step(1.0/60.0);
            trace("Sphere z position: " + sphereBody.position.z);
        });
    }

}
