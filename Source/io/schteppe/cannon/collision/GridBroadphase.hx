package io.schteppe.cannon.collision;

import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.objects.Body;
import io.schteppe.cannon.objects.Plane;
import io.schteppe.cannon.objects.Shape;
import io.schteppe.cannon.world.World;

/**
 * @class CANNON.GridBroadphase
 * @brief Axis aligned uniform grid broadphase.
 * @extends CANNON.Broadphase
 * @todo Needs support for more than just planes and spheres.
 * @param Vec3 aabbMin
 * @param Vec3 aabbMax
 * @param int nx Number of boxes along x
 * @param int ny Number of boxes along y
 * @param int nz Number of boxes along z
 */

class GridBroadphase extends Broadphase {

    var GridBroadphase_collisionPairs_d:Vec3;
    var GridBroadphase_collisionPairs_binPos:Vec3;

    var nx:Int;
    var ny:Int;
    var nz:Int;
    var aabbMin:Vec3;
    var aabbMax:Vec3;
    var bins:Array<Array<Body>>;

    public function new(aabbMin:Vec3 = null,aabbMax:Vec3 = null,nx:Int = 10,ny:Int = 10,nz:Int = 10){
        super();

        this.aabbMin = aabbMin != null ? aabbMin : new Vec3(100, 100, 100);
        this.aabbMax = aabbMax != null ? aabbMax : new Vec3(100, 100, 100);
        this.nx = nx;
        this.ny = ny;
        this.nz = nz;
        this.bins = new Array<Array<Body>>();

        GridBroadphase_collisionPairs_d = new Vec3();
        GridBroadphase_collisionPairs_binPos = new Vec3();
    }

    /**
     * @method collisionPairs
     * @memberof CANNON.GridBroadphase
     * @brief Get all the collision pairs in the physics world
     * @param CANNON.World world
     * @param Array pairs1
     * @param Array pairs2
     */

    public override function collisionPairs(world:World,p1:Array<Body>,p2:Array<Body>){
        var N:Int = world.numObjects();
        var bodies:Array<Body> = world.bodies;

        var max:Vec3 = this.aabbMax;
        var min:Vec3 = this.aabbMin;
        var nx:Float = this.nx;
        var ny:Float = this.ny;
        var nz:Float = this.nz;

        var xmax:Float = max.x;
        var ymax:Float = max.y;
        var zmax:Float = max.z;
        var xmin:Float = min.x;
        var ymin:Float = min.y;
        var zmin:Float = min.z;

        var xmult:Float = nx / (xmax - xmin);
        var ymult:Float = ny / (ymax - ymin);
        var zmult:Float = nz / (zmax - zmin);

        var binsizeX:Float = (xmax - xmin) / nx;
        var binsizeY:Float = (ymax - ymin) / ny;
        var binsizeZ:Float = (zmax - zmin) / nz;

        var types:Dynamic = Shape.types;
        var SPHERE:Int =            types.SPHERE;
        var PLANE:Int =             types.PLANE;
        var BOX:Int =               types.BOX;
        var COMPOUND:Int =          types.COMPOUND;
        var CONVEXPOLYHEDRON:Int =  types.CONVEXPOLYHEDRON;

        var bins = this.bins;
        var Nbins:Int = this.nx * this.ny * this.nz;

        // Reset bins
        var binsLength:Int = bins.length - 1;
        for(i in binsLength...Nbins){
            bins.push([]);
        }
        for(i in 0...Nbins){
            bins[i].splice(0, bins[i].length);
        }

        var floor = Math.floor;

        // Put all bodies into the bins
        for(i in 0...N){
            var bi:Body = bodies[i];
            var si:Shape = bi.shape;

            switch(si.type){
                case 2://PLANE:
                    // Put in all bins for now
                    // @todo put only in bins that are actually intersecting the plane
                    var psi:Plane = cast(si, Plane);
                    var d = GridBroadphase_collisionPairs_d;
                    var binPos = GridBroadphase_collisionPairs_binPos;
                    var binRadiusSquared:Float = (binsizeX*binsizeX + binsizeY*binsizeY + binsizeZ*binsizeZ) * 0.25;

                    var planeNormal:Vec3 = psi.worldNormal;
                    if(psi.worldNormalNeedsUpdate){
                        psi.computeWorldNormal(bi.quaternion);
                    }

                    for(j in 0...this.nx){
                        for(k in 0...this.ny){
                            for(l in 0...this.nz){
                                var xi:Int = j;
                                var yi:Int = k;
                                var zi:Int = l;

                                binPos.set(xi*binsizeX+xmin, yi*binsizeY+ymin, zi*binsizeZ+zmin);
                                binPos.vsub(bi.position, d);

                                if(d.dot(planeNormal) < binRadiusSquared){
                                    var idx:Int = xi * ( this.ny - 1 ) * ( this.nz - 1 ) + yi * ( this.nz - 1 ) + zi;
                                    bins[ idx ].push( bi );
                                }
                            }
                        }
                    }

                default:
                    // Put in bin
                    // check if overlap with other bins
                    var x:Float = bi.position.x;
                    var y:Float = bi.position.y;
                    var z:Float = bi.position.z;
                    var r:Float = si.boundingSphereRadius;

                    var xi1:Int = floor(xmult * (x - r - xmin));
                    var yi1:Int = floor(ymult * (y - r - ymin));
                    var zi1:Int = floor(zmult * (z - r - zmin));
                    var xi2:Int = floor(xmult * (x + r - xmin));
                    var yi2:Int = floor(ymult * (y + r - ymin));
                    var zi2:Int = floor(zmult * (z + r - zmin));

                    for(j in xi1...xi2+1){
                        for(k in yi1...yi2+1){
                            for(l in zi1...zi2+1){
                                var xi:Int = j;
                                var yi:Int = k;
                                var zi:Int = l;
                                var idx:Int = xi * ( this.ny - 1 ) * (this.nz - 1 ) + yi * ( this.nz - 1 ) + zi;
                                if(idx >= 0 && idx < Nbins){
                                    bins[ idx ].push( bi );
                                }
                            }
                        }
                    }
            }
        }

        // Check each bin
        for(i in 0...Nbins){
            var bin:Array<Dynamic> = bins[i];

            // Do N^2 broadphase inside
            var NbodiesInBin:Int = bin.length;
            for(j in 0...NbodiesInBin){
                var bi = bin[j];

                for(k in 0...j){
                    var bj = bin[k];
                    if(this.needBroadphaseCollision(bi,bj)){
                        this.intersectionTest(bi,bj,p1,p2);
                    }
                }
            }
        }

        this.makePairsUnique(p1,p2);
    }
}
