package io.schteppe.cannon.collision;

import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.objects.Body;
import io.schteppe.cannon.objects.Plane;
import io.schteppe.cannon.objects.Shape;
import io.schteppe.cannon.objects.Sphere;
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
    var binLengths:Array<Int>;

    public function new(aabbMin:Vec3 = null,aabbMax:Vec3 = null,nx:Int = 10,ny:Int = 10,nz:Int = 10){
        super();

        this.aabbMin = aabbMin != null ? aabbMin : new Vec3(100, 100, 100);
        this.aabbMax = aabbMax != null ? aabbMax : new Vec3(100, 100, 100);

        this.nx = nx;
        this.ny = ny;
        this.nz = nz;

        var nbins:Int = this.nx * this.ny * this.nz;
        if (nbins <= 0) {
            throw "GridBroadphase: Each dimension's n must be >0";
        }

        this.bins = new Array<Array<Body>>();
        this.binLengths = []; //Rather than continually resizing arrays (thrashing the memory), just record length and allow them to grow
        //this.bins.length = nbins;
        //this.binLengths.length = nbins;
        for (i in 0...nbins) {
            this.bins[i]=[];
            this.binLengths[i] = 0;
        }

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
        var bodies:List<Body> = world.bodies;

        var max:Vec3 = this.aabbMax;
        var min:Vec3 = this.aabbMin;
        var nx:Float = this.nx;
        var ny:Float = this.ny;
        var nz:Float = this.nz;

        var xstep:Int = this.ny * this.nz;
        var ystep:Int = this.nz;
        var zstep:Int = 1;

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

        var binRadius:Float = Math.sqrt(binsizeX * binsizeX + binsizeY * binsizeY + binsizeZ * binsizeZ) * 0.5;

        function addBoxToBins(
                x0:Float, y0:Float, z0:Float,
                x1:Float, y1:Float, z1:Float,
                bi:Body):Void {
            var ceil = Math.ceil;
            var floor = Math.floor;

            var xoff0:Int = floor((x0 - xmin) * xmult);
            var yoff0:Int = floor((y0 - ymin) * ymult);
            var zoff0:Int = floor((z0 - zmin) * zmult);
            var xoff1:Int = ceil((x1 - xmin) * xmult);
            var yoff1:Int = ceil((y1 - ymin) * ymult);
            var zoff1:Int = ceil((z1 - zmin) * zmult);

            if (xoff0 < 0) xoff0 = 0; else if (xoff0 >= this.nx) xoff0 = this.nx - 1;
            if (yoff0 < 0) yoff0 = 0; else if (yoff0 >= this.ny) yoff0 = this.ny - 1;
            if (zoff0 < 0) zoff0 = 0; else if (zoff0 >= this.nz) zoff0 = this.nz - 1;
            if (xoff1 < 0) xoff1 = 0; else if (xoff1 >= this.nx) xoff1 = this.nx - 1;
            if (yoff1 < 0) yoff1 = 0; else if (yoff1 >= this.ny) yoff1 = this.ny - 1;
            if (zoff1 < 0) zoff1 = 0; else if (zoff1 >= this.nz) zoff1 = this.nz - 1;

            //    console.log("Adding bi "+(bi.adust_object && bi.adust_object.constructor && bi.adust_object.constructor.name)+" to " + xoff0 + "-" + xoff1 + "," + yoff0 + "-" + yoff1 + "," + zoff0 + "-" + zoff1);
            xoff0 *= xstep;
            yoff0 *= ystep;
            zoff0 *= zstep;
            xoff1 *= xstep;
            yoff1 *= ystep;
            zoff1 *= zstep;

            var xoff:Int = xoff0;
            while (xoff <= xoff1) {
                var yoff:Int = yoff0;
                while (yoff <= yoff1) {
                    var zoff:Int = zoff0;
                    while (zoff <= zoff1) {
                        var idx:Int = xoff+yoff+zoff;
                        bins[idx][binLengths[idx]++] = bi;
                        zoff += zstep;
                    }
                    yoff += ystep;
                }
                xoff += xstep;
            }
        }

        var types:Dynamic = Shape.types;
        var SPHERE:Int =            types.SPHERE;
        var PLANE:Int =             types.PLANE;
        var BOX:Int =               types.BOX;
        var COMPOUND:Int =          types.COMPOUND;
        var CONVEXPOLYHEDRON:Int =  types.CONVEXPOLYHEDRON;

        var bins = this.bins;
        //var Nbins:Int = this.nx * this.ny * this.nz;
        var binLengths:Array<Int> = this.binLengths;
        var Nbins:Int = this.bins.length;

        // Reset bins
        for(i in 0...Nbins){
            binLengths[i] = 0;
        }

        var ceil = Math.ceil;
        var min = Math.min;
        var max = Math.max;

        // Put all bodies into the bins
        for(bi in bodies) {
            var si:Shape = bi.shape;

            switch(si.type) {
                case 1://SPHERE
                    // Put in bin
                    // check if overlap with other bins
                    var sphere:Sphere = cast(si, Sphere);
                    var x:Float = bi.position.x;
                    var y:Float = bi.position.y;
                    var z:Float = bi.position.z;
                    var r:Float = sphere.radius;

                    addBoxToBins(x-r, y-r, z-r, x+r, y+r, z+r, bi);
                case 2://PLANE:
                    var plane:Plane = cast(si, Plane);
                    if(plane.worldNormalNeedsUpdate){
                        plane.computeWorldNormal(bi.quaternion);
                    }
                    var planeNormal:Vec3 = plane.worldNormal;

                    //Relative position from origin of plane object to the first bin
                    //Incremented as we iterate through the bins
                    var xreset:Float = xmin + binsizeX * 0.5 - bi.position.x;
                    var yreset:Float = ymin + binsizeY * 0.5 - bi.position.y;
                    var zreset:Float = zmin + binsizeZ * 0.5 - bi.position.z;

                    var d:Vec3 = GridBroadphase_collisionPairs_d;
                    d.set(xreset, yreset, zreset);

                    var xoff:Int = 0;
                    for (xi in 0...this.nx) {
                        var yoff:Int = 0;
                        for (yi in 0...this.ny) {
                            var zoff:Int = 0;
                            for (zi in 0...this.nz) {
                                if (d.dot(planeNormal) < binRadius) {
                                    var idx:Int = xoff + yoff + zoff;
                                    bins[idx][binLengths[idx]++] = bi;
                                }
                                zoff += zstep; d.z += binsizeZ;
                            }
                            yoff += ystep; d.z = zreset; d.y += binsizeY;
                        }
                        xoff += xstep; d.y = yreset; d.x += binsizeX;
                    }
                default:
                    if (bi.aabbNeedsUpdate) {
                        bi.computeAABB();
                    }

                    addBoxToBins(
                        bi.aabbmin.x,
                        bi.aabbmin.y,
                        bi.aabbmin.z,
                        bi.aabbmax.x,
                        bi.aabbmax.y,
                        bi.aabbmax.z,
                        bi);
                    }
        }

        // Check each bin
        for (i in 0...Nbins) {
            var binLength:Int = binLengths[i];
            if (binLength > 1) {
                var bin:Array<Body> = bins[i];

                // Do N^2 broadphase inside
                for(xi in 0...binLength){
                    var bi = bin[xi];

                    for(yi in 0...xi){
                        var bj = bin[yi];
                        if(this.needBroadphaseCollision(bi,bj)){
                            this.intersectionTest(bi,bj,p1,p2);
                        }
                    }
                }
            }
        }

        this.makePairsUnique(p1,p2);
    }
}
