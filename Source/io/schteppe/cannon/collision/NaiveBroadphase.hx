package io.schteppe.cannon.collision;
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
    public override function collisionPairs(world:World,pairs1:Array<Dynamic>,pairs2:Array<Dynamic>){
        var bodies = world.bodies;
        var n = bodies.length;
        var i, j, bi, bj;

        // Naive N^2 ftw!
        for(i in 0...n){
            for(j in 0...i){

                bi = bodies[i];
                bj = bodies[j];

                if(!this.needBroadphaseCollision(bi,bj)){
                    continue;
                }

                this.intersectionTest(bi,bj,pairs1,pairs2);
            }
        }
    }
}
