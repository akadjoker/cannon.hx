package io.schteppe.cannon.collision;
import io.schteppe.cannon.objects.Body;
import io.schteppe.cannon.world.World;

/**
 * @class CANNON.NaiveBroadphase
 * @brief Naive broadphase implementation, used in lack of better ones.
 * @description The naive broadphase looks at all possible pairs without restriction, therefore it has complexity N^2 (which is bad)
 * @extends CANNON.Broadphase
 */
class NaiveBroadphase extends Broadphase {

    public function new(){
        super();
    }

    /**
     * @method collisionPairs
     * @memberof CANNON.NaiveBroadphase
     * @brief Get all the collision pairs in the physics world
     * @param CANNON.World world
     * @param Array pairs1
     * @param Array pairs2
     */
    public override function collisionPairs(world:World,pairs1:Array<Body>,pairs2:Array<Body>){
        var bodies:List<Body> = world.bodies;
        var n:Int = bodies.length;
        var i:Int; var j:Int; var bi:Body; var bj:Body;

        // Naive N^2 ftw!
        for(bi in bodies) {
            for(bj in bodies) {
                if(!this.needBroadphaseCollision(bi,bj)){
                    continue;
                }

                this.intersectionTest(bi,bj,pairs1,pairs2);
            }
        }
    }
}
