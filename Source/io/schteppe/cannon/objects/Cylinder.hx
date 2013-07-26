package io.schteppe.cannon.objects;

import io.schteppe.cannon.math.Vec3;

/**
 * @class CANNON.Cylinder
 * @extends CANNON.ConvexPolyhedron
 * @author schteppe / https://github.com/schteppe
 * @param float radiusTop
 * @param float radiusBottom
 * @param float height
 * @param int numSegments The number of segments to build the cylinder out of
 */
class Cylinder extends ConvexPolyhedron {

    public function new( radiusTop, radiusBottom, height , numSegments ) {
        var N = numSegments;
        var verts = [];
        var normals = [];
        var faces = [];
        var bottomface = [];
        var topface = [];
        var cos = Math.cos;
        var sin = Math.sin;

        // First bottom point
        verts.push(new Vec3(radiusBottom*cos(0),
                                   radiusBottom*sin(0),
                                   -height*0.5));
        bottomface.push(0);

        // First top point
        verts.push(new Vec3(radiusTop*cos(0),
                                   radiusTop*sin(0),
                                   height*0.5));
        topface.push(1);

        for(i in 0...N){
            var theta:Float = 2.0*Math.PI/N * (i+1.0);
            var thetaN:Float = 2.0*Math.PI/N * (i+0.5);
            if(i<N-1){
                // Bottom
                verts.push(new Vec3(radiusBottom*cos(theta),
                                           radiusBottom*sin(theta),
                                           -height*0.5));
                bottomface.push(2*i+2);
                // Top
                verts.push(new Vec3(radiusTop*cos(theta),
                                           radiusTop*sin(theta),
                                           height*0.5));
                topface.push(2*i+3);
                // Normal
                normals.push(new Vec3(cos(thetaN),
                                             sin(thetaN),
                                             0));
                // Face
                faces.push([2*i+2, 2*i+3, 2*i+1,2*i]);
            } else {
                faces.push([0,1, 2*i+1, 2*i]); // Connect
                // Normal
                normals.push(new Vec3(cos(thetaN),sin(thetaN),0));
            }
        }
        faces.push(topface);
        normals.push(new Vec3(0,0,1));

        // Reorder bottom face
        var temp = [];
        var bfl:Int = bottomface.length;
        for(i in 0...bfl){
            temp.push(bottomface[bottomface.length - i - 1]);
        }
        faces.push(temp);
        normals.push(new Vec3(0,0,-1));

        this.type = Shape.types.CONVEXPOLYHEDRON;
        super(verts, faces, normals);
    }
}
