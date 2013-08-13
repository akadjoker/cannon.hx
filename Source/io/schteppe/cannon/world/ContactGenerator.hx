package io.schteppe.cannon.world;

import io.schteppe.cannon.constraints.ContactEquation;
import io.schteppe.cannon.math.Quaternion;
import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.objects.Body;
import io.schteppe.cannon.objects.ConvexPolyhedron;
import io.schteppe.cannon.objects.RigidBody;
import io.schteppe.cannon.objects.Shape;
import io.schteppe.cannon.objects.Sphere;
import io.schteppe.cannon.utils.Vec3Pool;

/**
 * @class CANNON.ContactGenerator
 * @brief Helper class for the World. Generates ContactEquations.
 * @todo Sphere-ConvexPolyhedron contacts
 * @todo Contact reduction
 */

class ContactGenerator {

    /**
     * @property bool contactReduction
     * @memberof CANNON.ContactGenerator
     * @brief Turns on or off contact reduction. Can be handy to turn off when debugging new collision types.
     */
    var contactReduction:Bool = false;

    // Contact point objects that can be reused
    var contactPointPool:Array<Dynamic>;

    var v3pool:Vec3Pool;

    var point_on_plane_to_sphere:Vec3;
    var plane_to_sphere_ortho:Vec3;
    var pointInPolygon_edge:Vec3;
    var pointInPolygon_edge_x_normal:Vec3;
    var pointInPolygon_vtp:Vec3;
    var box_to_sphere:Vec3;
    var sphereBox_ns:Vec3;
    var sphereBox_ns1:Vec3;
    var sphereBox_ns2:Vec3;
    var sphereBox_sides:Array<Vec3>;
    var sphereBox_sphere_to_corner:Vec3;
    var sphereBox_side_ns:Vec3;
    var sphereBox_side_ns1:Vec3;
    var sphereBox_side_ns2:Vec3;
    var convex_to_sphere:Vec3;
    var sphereConvex_edge:Vec3;
    var sphereConvex_edgeUnit:Vec3;
    var sphereConvex_sphereToCorner:Vec3;
    var sphereConvex_worldCorner:Vec3;
    var sphereConvex_worldNormal:Vec3;
    var sphereConvex_worldPoint:Vec3;
    var sphereConvex_worldSpherePointClosestToPlane:Vec3;
    var sphereConvex_penetrationVec:Vec3;
    var sphereConvex_sphereToWorldPoint:Vec3;
    var planeBox_normal:Vec3;
    var plane_to_corner:Vec3;
    var recurseCompound_v3pool:Array<Vec3>;
    var recurseCompound_quatpool:Array<Quaternion>;
    var planeConvex_v:Vec3;
    var planeConvex_normal:Vec3;
    var planeConvex_relpos:Vec3;
    var planeConvex_projected:Vec3;
    var convexConvex_sepAxis:Vec3;
    var convexConvex_q:Vec3;
    var particlePlane_normal:Vec3;
    var particlePlane_relpos:Vec3;
    var particlePlane_projected:Vec3;
    var particleSphere_normal:Vec3;
    var cqj:Quaternion;
    var particleConvex_local:Vec3;
    var particleConvex_normal:Vec3;
    var particleConvex_penetratedFaceNormal:Vec3;
    var particleConvex_vertexToParticle:Vec3;
    var particleConvex_worldPenetrationVec:Vec3;

    public function new() {
        contactPointPool = [];

        v3pool = new Vec3Pool();

        point_on_plane_to_sphere = new Vec3();
        plane_to_sphere_ortho = new Vec3();
        pointInPolygon_edge = new Vec3();
        pointInPolygon_edge_x_normal = new Vec3();
        pointInPolygon_vtp = new Vec3();
        box_to_sphere = new Vec3();
        sphereBox_ns = new Vec3();
        sphereBox_ns1 = new Vec3();
        sphereBox_ns2 = new Vec3();
        sphereBox_sides = [new Vec3(),new Vec3(),new Vec3(),new Vec3(),new Vec3(),new Vec3()];
        sphereBox_sphere_to_corner = new Vec3();
        sphereBox_side_ns = new Vec3();
        sphereBox_side_ns1 = new Vec3();
        sphereBox_side_ns2 = new Vec3();
        convex_to_sphere = new Vec3();
        sphereConvex_edge = new Vec3();
        sphereConvex_edgeUnit = new Vec3();
        sphereConvex_sphereToCorner = new Vec3();
        sphereConvex_worldCorner = new Vec3();
        sphereConvex_worldNormal = new Vec3();
        sphereConvex_worldPoint = new Vec3();
        sphereConvex_worldSpherePointClosestToPlane = new Vec3();
        sphereConvex_penetrationVec = new Vec3();
        sphereConvex_sphereToWorldPoint = new Vec3();
        planeBox_normal = new Vec3();
        plane_to_corner = new Vec3();
        recurseCompound_v3pool = [];
        recurseCompound_quatpool = [];
        planeConvex_v = new Vec3();
        planeConvex_normal = new Vec3();
        planeConvex_relpos = new Vec3();
        planeConvex_projected = new Vec3();
        convexConvex_sepAxis = new Vec3();
        convexConvex_q = new Vec3();
        particlePlane_normal = new Vec3();
        particlePlane_relpos = new Vec3();
        particlePlane_projected = new Vec3();
        particleSphere_normal = new Vec3();
        cqj = new Quaternion();
        particleConvex_local = new Vec3();
        particleConvex_normal = new Vec3();
        particleConvex_penetratedFaceNormal = new Vec3();
        particleConvex_vertexToParticle = new Vec3();
        particleConvex_worldPenetrationVec = new Vec3();
    }

    /*
     * Make a contact object.
     * @return object
     * @todo reuse old contact point objects
     */
    function makeResult(bi,bj){
        if(contactPointPool.length > 0){
            var c = contactPointPool.pop();
            c.bi = bi;
            c.bj = bj;
            return c;
        } else {
            return new ContactEquation(bi,bj);
        }
    }

    /*
     * Swaps the body references in the contact
     * @param object r
     */
    function swapResult(r:ContactEquation){
        var temp:Vec3 = r.ri;
        r.ri = r.rj;
        r.rj = temp;
        r.ni.negate(r.ni);
        var temp2:Body = r.bi;
        r.bi = r.bj;
        r.bj = temp2;
    }

    function sphereSphere(result:Array<Dynamic>,si:Dynamic,sj:Dynamic,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        // We will have only one contact in this case
        var r = makeResult(bi,bj);

        // Contact normal
        bj.position.vsub(xi, r.ni);
        r.ni.normalize();

        // Contact point locations
        r.ni.copy(r.ri);
        r.ni.copy(r.rj);
        r.ri.mult(si.radius, r.ri);
        r.rj.mult(-sj.radius, r.rj);
        result.push(r);
    }

    function spherePlane(result:Array<Dynamic>,si:Dynamic,sj:Dynamic,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        // We will have one contact in this case
        var r = makeResult(bi,bj);

        // Contact normal
        r.ni.set(0,0,1);
        qj.vmult(r.ni,r.ni);
        r.ni.negate(r.ni); // body i is the sphere, flip normal
        r.ni.normalize();

        // Vector from sphere center to contact point
        r.ni.mult(si.radius,r.ri);

        // Project down sphere on plane
        xi.vsub(xj, point_on_plane_to_sphere);
        r.ni.mult(r.ni.dot(point_on_plane_to_sphere), plane_to_sphere_ortho);
        point_on_plane_to_sphere.vsub(plane_to_sphere_ortho, r.rj); // The sphere position projected to plane
        if (plane_to_sphere_ortho.norm2() <= si.radius * si.radius) {
            result.push(r);
        }
    }

    // See http://bulletphysics.com/Bullet/BulletFull/SphereTriangleDetector_8cpp_source.html
    function pointInPolygon(verts:Array<Vec3>, normal:Vec3, p:Vec3):Bool{
        var positiveResult:Bool = false;
        var N = verts.length;
        for(i in 0...N){
            var v:Vec3 = verts[i];

            // Get edge to the next vertex
            var edge = pointInPolygon_edge;
            verts[(i+1) % (N)].vsub(v,edge);

            // Get cross product between polygon normal and the edge
            var edge_x_normal = pointInPolygon_edge_x_normal;
            //var edge_x_normal = new Vec3();
            edge.cross(normal,edge_x_normal);

            // Get vector between point and current vertex
            var vertex_to_p = pointInPolygon_vtp;
            p.vsub(v,vertex_to_p);

            // This dot product determines which side of the edge the point is
            var r:Float = edge_x_normal.dot(vertex_to_p);

            // If all such dot products have same sign, we are inside the polygon.
            if(!positiveResult || (r>0 && positiveResult) || (r<=0 && !positiveResult)){
                if(!positiveResult){
                    positiveResult = r>0;
                }
                continue;
            } else {
                return false; // Encountered some other sign. Exit.
            }
        }

        // If we got here, all dot products were of the same sign.
        return true;
    }

    function sphereBox(result:Array<Dynamic>,si:Dynamic,sj:Dynamic,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        // we refer to the box as body j
        var sides:Array<Vec3> = sphereBox_sides;
        xi.vsub(xj,box_to_sphere);
        sj.getSideNormals(sides,qj);
        var R:Float =     si.radius;
        var penetrating_sides:Array<Dynamic> = [];

        // Check side (plane) intersections
        var found:Bool = false;

        // Store the resulting side penetration info
        var side_ns:Vec3 = sphereBox_side_ns;
        var side_ns1:Vec3 = sphereBox_side_ns1;
        var side_ns2:Vec3 = sphereBox_side_ns2;
        var side_h:Float = Math.POSITIVE_INFINITY;
        var side_penetrations:Int = 0;
        var side_dot1:Float = 0;
        var side_dot2:Float = 0;
        var side_distance:Float = Math.POSITIVE_INFINITY;
        var side_distance_set:Bool = false;
        var nsides:Int = sides.length;
        for (idx in 0...nsides) {
            if (found) break;
            // Get the plane side normal (ns)
            var ns = sphereBox_ns;
            sides[idx].copy(ns);

            var h:Float = ns.norm();
            ns.normalize();

            // The normal/distance dot product tells which side of the plane we are
            var dot:Float = box_to_sphere.dot(ns);

            if(dot<h+R && dot>0){
                // Intersects plane. Now check the other two dimensions
                var ns1:Vec3 = sphereBox_ns1;
                var ns2:Vec3 = sphereBox_ns2;
                sides[(idx+1)%3].copy(ns1);
                sides[(idx+2)%3].copy(ns2);
                var h1:Float = ns1.norm();
                var h2:Float = ns2.norm();
                ns1.normalize();
                ns2.normalize();
                var dot1:Float = box_to_sphere.dot(ns1);
                var dot2:Float = box_to_sphere.dot(ns2);
                if(dot1<h1 && dot1>-h1 && dot2<h2 && dot2>-h2){
                    var dist:Float = Math.abs(dot-h-R);
                    if(!side_distance_set || dist < side_distance){
                        side_distance = dist;
                        side_distance_set = true;
                        side_dot1 = dot1;
                        side_dot2 = dot2;
                        side_h = h;
                        ns.copy(side_ns);
                        ns1.copy(side_ns1);
                        ns2.copy(side_ns2);
                        side_penetrations++;
                    }
                }
            }
        }
        if(side_penetrations != 0){
            found = true;
            var r = makeResult(bi,bj);
            side_ns.mult(-R,r.ri); // Sphere r
            side_ns.copy(r.ni);
            r.ni.negate(r.ni); // Normal should be out of sphere
            side_ns.mult(side_h,side_ns);
            side_ns1.mult(side_dot1,side_ns1);
            side_ns.vadd(side_ns1,side_ns);
            side_ns2.mult(side_dot2,side_ns2);
            side_ns.vadd(side_ns2,r.rj);
            result.push(r);
        }

        // Check corners
        var rj:Vec3 = v3pool.get();
        var sphere_to_corner = sphereBox_sphere_to_corner;
        for (j in 0...2) {
            if (found) break; 
            for (k in 0...2) {
                if (found) break;  
                for (l in 0...2) {
                    if (found) break;  
                    rj.set(0,0,0);
                    if(j != 0){
                        rj.vadd(sides[0],rj);
                    } else {
                        rj.vsub(sides[0],rj);
                    }
                    if(k != 0){
                        rj.vadd(sides[1],rj);
                    } else {
                        rj.vsub(sides[1],rj);
                    }
                    if(l != 0){
                        rj.vadd(sides[2],rj);
                    } else {
                        rj.vsub(sides[2],rj);
                    }

                    // World position of corner
                    xj.vadd(rj,sphere_to_corner);
                    sphere_to_corner.vsub(xi,sphere_to_corner);

                    if(sphere_to_corner.norm2() < R*R){
                        found = true;
                        var r = makeResult(bi,bj);
                        sphere_to_corner.copy(r.ri);
                        r.ri.normalize();
                        r.ri.copy(r.ni);
                        r.ri.mult(R,r.ri);
                        rj.copy(r.rj);
                        result.push(r);
                    }
                }
            }
        }
        v3pool.release(rj);
        rj = null;

        // Check edges
        var edgeTangent = v3pool.get();
        var edgeCenter = v3pool.get();
        var r = v3pool.get(); // r = edge center to sphere center
        var orthogonal = v3pool.get();
        var dist = v3pool.get();
        var Nsides = sides.length;
        for(j in 0...Nsides){
            if (found) break;
            for (k in 0...Nsides) {
                if (found) break; 
                if(j%3 != k%3){
                    // Get edge tangent
                    sides[k].cross(sides[j],edgeTangent);
                    edgeTangent.normalize();
                    sides[j].vadd(sides[k], edgeCenter);
                    xi.copy(r);
                    r.vsub(edgeCenter,r);
                    r.vsub(xj,r);
                    var orthonorm:Float = r.dot(edgeTangent); // distance from edge center to sphere center in the tangent direction
                    edgeTangent.mult(orthonorm,orthogonal); // Vector from edge center to sphere center in the tangent direction

                    // Find the third side orthogonal to this one
                    var l = 0;
                    while(l==j%3 || l==k%3){
                        l++;
                    }

                    // vec from edge center to sphere projected to the plane orthogonal to the edge tangent
                    xi.copy(dist);
                    dist.vsub(orthogonal,dist);
                    dist.vsub(edgeCenter,dist);
                    dist.vsub(xj,dist);

                    // Distances in tangent direction and distance in the plane orthogonal to it
                    var tdist:Float = Math.abs(orthonorm);
                    var ndist:Float = dist.norm();

                    if(tdist < sides[l].norm() && ndist<R){
                        found = true;
                        var res = makeResult(bi,bj);
                        edgeCenter.vadd(orthogonal,res.rj); // box rj
                        res.rj.copy(res.rj);
                        dist.negate(res.ni);
                        res.ni.normalize();

                        res.rj.copy(res.ri);
                        res.ri.vadd(xj,res.ri);
                        res.ri.vsub(xi,res.ri);
                        res.ri.normalize();
                        res.ri.mult(R,res.ri);

                        result.push(res);
                    }
                }
            }
        }
        v3pool.releaseGroup([edgeTangent,edgeCenter,r,orthogonal,dist]);
    }

    function sphereConvex(result:Array<Dynamic>,si:Dynamic,sj:ConvexPolyhedron,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        xi.vsub(xj,convex_to_sphere);
        var normals:Array<Vec3> = sj.faceNormals;
        var faces:Array<Dynamic> = sj.faces;
        var verts:Array<Vec3> = sj.vertices;
        var R:Float =     si.radius;
        var penetrating_sides:Array<Dynamic> = [];
        var found:Bool = false;

        // Check corners
        for(i in 0...verts.length){
            var v:Vec3 = verts[i];
            // World position of corner
            var worldCorner = sphereConvex_worldCorner;
            qj.vmult(v,worldCorner);
            xj.vadd(worldCorner,worldCorner);
            var sphere_to_corner = sphereConvex_sphereToCorner;
            worldCorner.vsub(xi, sphere_to_corner);
            if(sphere_to_corner.norm2()<R*R){
                found = true;
                var r = makeResult(bi,bj);
                sphere_to_corner.copy(r.ri);
                r.ri.normalize();
                r.ri.copy(r.ni);
                r.ri.mult(R,r.ri);
                worldCorner.vsub(xj,r.rj);
                result.push(r);
                return;
            }
        }

        // Check side (plane) intersections

        var nfaces:Int = faces.length;
        for (i in 0...nfaces) {
            if (found) break; 
            var normal = normals[i];
            var face = faces[i];

            var worldNormal = sphereConvex_worldNormal;
            qj.vmult(normal,worldNormal);

            var worldPoint = sphereConvex_worldPoint;
            qj.vmult(verts[face[0]],worldPoint);
            worldPoint.vadd(xj,worldPoint); // Arbitrary point in the face

            var worldSpherePointClosestToPlane = sphereConvex_worldSpherePointClosestToPlane;
            worldNormal.mult(-R,worldSpherePointClosestToPlane);
            xi.vadd(worldSpherePointClosestToPlane,worldSpherePointClosestToPlane);

            var penetrationVec = sphereConvex_penetrationVec;
            worldSpherePointClosestToPlane.vsub(worldPoint,penetrationVec);
            var penetration:Float = penetrationVec.dot(worldNormal);

            var sphereToWorldPoint = sphereConvex_sphereToWorldPoint;
            xi.vsub(worldPoint,sphereToWorldPoint);

            if(penetration<0 && sphereToWorldPoint.dot(worldNormal)>0){
                // Intersects plane. Now check if the sphere is inside the face polygon
                var faceVerts = []; // Face vertices, in world coords 
                var Nverts:Int = face.length;
                for(j in 0...Nverts){
                    var worldVertex = v3pool.get();
                    qj.vmult(verts[face[j]], worldVertex);
                    xj.vadd(worldVertex,worldVertex);
                    faceVerts.push(worldVertex);
                }

                if(pointInPolygon(faceVerts,worldNormal,xi)){ // Is the sphere center in the face polygon?
                    found = true;
                    var r = makeResult(bi,bj);
                    worldNormal.mult(-R,r.ri); // Sphere r
                    worldNormal.negate(r.ni); // Normal should be out of sphere

                    var penetrationVec2 = v3pool.get();
                    worldNormal.mult(-penetration,penetrationVec2);
                    var penetrationSpherePoint = v3pool.get();
                    worldNormal.mult(-R,penetrationSpherePoint);

                    //xi.vsub(xj).vadd(penetrationSpherePoint).vadd(penetrationVec2 , r.rj);
                    xi.vsub(xj,r.rj);
                    r.rj.vadd(penetrationSpherePoint,r.rj);
                    r.rj.vadd(penetrationVec2 , r.rj);

                    v3pool.release(penetrationVec2);
                    v3pool.release(penetrationSpherePoint);

                    result.push(r);

                    // Release world vertices
                    var Nfaceverts:Int = faceVerts.length;
                    for(j in 0...Nfaceverts){
                        v3pool.release(faceVerts[j]);
                    }

                    return; // We only expect *one* face contact
                } else {
                    // Edge?
                    for(j in 0...face.length){

                        // Get two world transformed vertices
                        var v1 = v3pool.get();
                        var v2 = v3pool.get();
                        qj.vmult(verts[face[(j+1)%face.length]], v1);
                        qj.vmult(verts[face[(j+2)%face.length]], v2);
                        xj.vadd(v1, v1);
                        xj.vadd(v2, v2);

                        // Construct edge vector
                        var edge = sphereConvex_edge;
                        v2.vsub(v1,edge);

                        // Construct the same vector, but normalized
                        var edgeUnit = sphereConvex_edgeUnit;
                        edge.unit(edgeUnit);

                        // p is xi projected onto the edge
                        var p = v3pool.get();
                        var v1_to_xi = v3pool.get();
                        xi.vsub(v1, v1_to_xi);
                        var dot:Float = v1_to_xi.dot(edgeUnit);
                        edgeUnit.mult(dot, p);
                        p.vadd(v1, p);

                        // Compute a vector from p to the center of the sphere
                        var xi_to_p = v3pool.get();
                        p.vsub(xi, xi_to_p);

                        // Collision if the edge-sphere distance is less than the radius
                        // AND if p is in between v1 and v2
                        if(dot > 0 && dot*dot<edge.norm2() && xi_to_p.norm2() < R*R){ // Collision if the edge-sphere distance is less than the radius
                            // Edge contact!
                            var r = makeResult(bi,bj);
                            p.vsub(xj,r.rj);

                            p.vsub(xi,r.ni);
                            r.ni.normalize();

                            r.ni.mult(R,r.ri);
                            result.push(r);

                            // Release world vertices
                            var Nfaceverts:Int = faceVerts.length;
                            for(j in 0...Nfaceverts){
                                v3pool.release(faceVerts[j]);
                            }

                            v3pool.release(v1);
                            v3pool.release(v2);
                            v3pool.release(p);
                            v3pool.release(xi_to_p);
                            v3pool.release(v1_to_xi);

                            return;
                        }

                        v3pool.release(v1);
                        v3pool.release(v2);
                        v3pool.release(p);
                        v3pool.release(xi_to_p);
                        v3pool.release(v1_to_xi);
                    }
                }

                // Release world vertices
                var Nfaceverts:Int = faceVerts.length;
                for(j in 0...Nfaceverts){
                    v3pool.release(faceVerts[j]);
                }
            }
        }
    }

    function planeBox(result:Array<Dynamic>,si:Dynamic,sj:Dynamic,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        planeConvex(result,si,sj.convexPolyhedronRepresentation,xi,xj,qi,qj,bi,bj);
    }

    /*
     * Go recursive for compound shapes
     * @param Shape si
     * @param CompoundShape sj
     */
    function recurseCompound(result:Array<Dynamic>,si:Dynamic,sj:Dynamic,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        var v3pool:Array<Vec3> = recurseCompound_v3pool;
        var quatPool:Array<Quaternion> = recurseCompound_quatpool;
        var nr = 0;
        var Nchildren:Int = sj.childShapes.length;
        for(i in 0...Nchildren){
            var r:Array<Dynamic> = [];
            var newQuat:Quaternion = quatPool.pop();
            if (newQuat == null) newQuat = new Quaternion();
            var newPos:Vec3 = v3pool.pop();
            if (newPos == null) newPos = new Vec3();
            qj.mult(sj.childOrientations[i],newQuat); // Can't reuse these since nearPhase() may recurse
            newQuat.normalize();
            //var newPos = xj.vadd(qj.vmult(sj.childOffsets[i]));
            qj.vmult(sj.childOffsets[i],newPos);
            xj.vadd(newPos,newPos);
            nearPhase(r,
                      si,
                      sj.childShapes[i],
                      xi,
                      newPos,//xj.vadd(qj.vmult(sj.childOffsets[i])), // Transform the shape to its local frame
                      qi,
                      newQuat, // Accumulate orientation
                      bi,
                      bj);
            // Release vector and quat
            quatPool.push(newQuat);

            var tempVec = newPos;

            if(si == null){
                nr+= r.length;
            }
            for(j in 0...r.length){
                // The "rj" vector is in world coords, though we must add the world child offset vector.
                //r[j].rj.vadd(qj.vmult(sj.childOffsets[i]),r[j].rj);
                qj.vmult(sj.childOffsets[i],tempVec);
                r[j].rj.vadd(tempVec,r[j].rj);
                result.push(r[j]);
            }

            v3pool.push(newPos);
        }
    }

    function planeConvex(result:Array<Dynamic>,si:Dynamic,sj:Dynamic,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        // Simply return the points behind the plane.
        var v = planeConvex_v;
        var normal = planeConvex_normal;
        normal.set(0,0,1);
        qi.vmult(normal,normal); // Turn normal according to plane orientation
        var relpos = planeConvex_relpos;
        for(i in 0...sj.vertices.length){
            sj.vertices[i].copy(v);
            // Transform to world coords
            qj.vmult(v,v);
            xj.vadd(v,v);
            v.vsub(xi,relpos);

            var dot:Float = normal.dot(relpos);
            if(dot<=0.0){
                // Get vertex position projected on plane
                var projected = planeConvex_projected;
                normal.mult(normal.dot(v),projected);
                v.vsub(projected,projected);

                var r = makeResult(bi,bj);
                normal.copy( r.ni ); // Contact normal is the plane normal out from plane

                projected.copy(r.ri); // From plane to vertex projected on plane

                // rj is now just the vertex position
                v.vsub(xj,r.rj);

                result.push(r);
            }
        }
    }

    function convexConvex(result:Array<Dynamic>,si:Dynamic,sj:Dynamic,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        var sepAxis:Vec3 = convexConvex_sepAxis;
        if(si.findSeparatingAxis(sj,xi,qi,xj,qj,sepAxis)){
            var res = [];
            var q = convexConvex_q;
            si.clipAgainstHull(xi,qi,sj,xj,qj,sepAxis,-100,100,res);
            //console.log(res.length);
            for(j in 0...res.length){
                var r = makeResult(bi,bj);
                sepAxis.negate(r.ni);
                res[j].normal.negate(q);
                q.mult(res[j].depth,q);
                res[j].point.vadd(q,r.ri);
                res[j].point.copy(r.rj);
                // Contact points are in world coordinates. Transform back to relative
                r.rj.vsub(xj,r.rj);
                r.ri.vsub(xi,r.ri);
                result.push(r);
            }
        }
    }

    function particlePlane(result:Array<Dynamic>,si:Dynamic,sj:Dynamic,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        var normal = particlePlane_normal;
        normal.set(0,0,1);
        bj.quaternion.vmult(normal,normal); // Turn normal according to plane orientation
        var relpos = particlePlane_relpos;
        xi.vsub(bj.position,relpos);
        var dot:Float = normal.dot(relpos);
        if(dot<=0.0){
            var r = makeResult(bi,bj);
            normal.copy( r.ni ); // Contact normal is the plane normal
            r.ni.negate(r.ni);
            r.ri.set(0,0,0); // Center of particle

            // Get particle position projected on plane
            var projected = particlePlane_projected;
            normal.mult(normal.dot(xi),projected);
            xi.vsub(projected,projected);
            //projected.vadd(bj.position,projected);

            // rj is now the projected world position minus plane position
            projected.copy(r.rj);
            result.push(r);
        }
    }

    function particleSphere(result:Array<Dynamic>,si:Dynamic,sj:Dynamic,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        // The normal is the unit vector from sphere center to particle center
        var normal = particleSphere_normal;
        normal.set(0,0,1);
        xi.vsub(xj,normal);
        var lengthSquared:Float = normal.norm2();

        if(lengthSquared <= sj.radius * sj.radius){
            var r = makeResult(bi,bj);
            normal.normalize();
            normal.copy(r.rj);
            r.rj.mult(sj.radius,r.rj);
            normal.copy( r.ni ); // Contact normal
            r.ni.negate(r.ni);
            r.ri.set(0,0,0); // Center of particle
            result.push(r);
        }
    }

    // WIP
    function particleConvex(result:Array<Dynamic>,si:Dynamic,sj:Dynamic,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        var penetratedFaceIndex:Int = -1;
        var penetratedFaceNormal:Vec3 = particleConvex_penetratedFaceNormal;
        var worldPenetrationVec:Vec3 = particleConvex_worldPenetrationVec;
        var minPenetration:Float = Math.POSITIVE_INFINITY;
        var minPenetrationSet:Bool = false;
        var numDetectedFaces:Int = 0;

        // Convert particle position xi to local coords in the convex
        var local = particleConvex_local;
        xi.copy(local);
        local.vsub(xj,local); // Convert position to relative the convex origin
        qj.conjugate(cqj);
        cqj.vmult(local,local);

        if(sj.pointIsInside(local)){

            if(sj.worldVerticesNeedsUpdate){
                sj.computeWorldVertices(xj,qj);
            }
            if(sj.worldFaceNormalsNeedsUpdate){
                sj.computeWorldFaceNormals(qj);
            }

            // For each world polygon in the polyhedra
            var nfaces:Int = sj.faces.length;
            for(i in 0...nfaces){

                // Construct world face vertices
                var verts = [ sj.worldVertices[ sj.faces[i][0] ] ];
                var normal = sj.worldFaceNormals[i];

                // Check how much the particle penetrates the polygon plane.
                xi.vsub(verts[0],particleConvex_vertexToParticle);
                var penetration:Float = -normal.dot(particleConvex_vertexToParticle);
                if(!minPenetrationSet || Math.abs(penetration)<Math.abs(minPenetration)){
                    minPenetration = penetration;
                    penetratedFaceIndex = i;
                    minPenetrationSet = true;
                    normal.copy(penetratedFaceNormal);
                    numDetectedFaces++;
                }
            }

            if(penetratedFaceIndex!=-1){
                // Setup contact
                var r = makeResult(bi,bj);
                penetratedFaceNormal.mult(minPenetration, worldPenetrationVec);

                // rj is the particle position projected to the face
                worldPenetrationVec.vadd(xi,worldPenetrationVec);
                worldPenetrationVec.vsub(xj,worldPenetrationVec);
                worldPenetrationVec.copy(r.rj);
                //var projectedToFace = xi.vsub(xj).vadd(worldPenetrationVec);
                //projectedToFace.copy(r.rj);

                //qj.vmult(r.rj,r.rj);
                penetratedFaceNormal.negate( r.ni ); // Contact normal
                r.ri.set(0,0,0); // Center of particle
                result.push(r);
            } else {
                trace("Point found inside convex, but did not find penetrating face!");
            }
        }
    }

    /*
     * Near phase calculation, get the contact point, normal, etc.
     * @param array result The result one will get back with all the contact point information
     * @param Shape si Colliding shape. If not given, particle is assumed.
     * @param Shape sj
     * @param Vec3 xi Position of the center of mass
     * @param Vec3 xj
     * @param Quaternion qi Rotation around the center of mass
     * @param Quaternion qj
     * @todo All collision cases
     */
    function nearPhase(result:Array<Dynamic>,si:Shape,sj:Shape,xi:Vec3,xj:Vec3,qi:Quaternion,qj:Quaternion,bi:Body,bj:Body){
        var swapped = false;
        var types = Shape.types;
        var SPHERE = types.SPHERE;
        var PLANE = types.PLANE;
        var BOX = types.BOX;
        var COMPOUND = types.COMPOUND;
        var CONVEXPOLYHEDRON = types.CONVEXPOLYHEDRON;

        if(si != null && sj != null){
            if(si.type > sj.type){
                var temp:Shape;
                temp=sj;
                sj=si;
                si=temp;

                var temp2:Vec3;
                temp2=xj;
                xj=xi;
                xi=temp2;

                var temp3:Quaternion;
                temp3=qj;
                qj=qi;
                qi=temp3;

                var temp4:Body;
                temp4=bj;
                bj=bi;
                bi=temp4;

                swapped = true;
            }
        } else {
            // Particle!
            if(si !=null && sj == null){
                var temp:Shape;
                temp=sj;
                sj=si;
                si=temp;

                var temp2:Vec3;
                temp2=xj;
                xj=xi;
                xi=temp2;

                var temp3:Quaternion;
                temp3=qj;
                qj=qi;
                qi=temp3;

                var temp4:Body;
                temp4=bj;
                bj=bi;
                bi=temp4;

                swapped = true;
            }
        }

        if(si !=null && sj != null){
            if(si.type == SPHERE){

                switch(sj.type){
                case 1: //Shape.types.SPHERE: // sphere-sphere
                    sphereSphere(result,si,sj,xi,xj,qi,qj,bi,bj);
                case 2: //Shape.types.PLANE: // sphere-plane
                    spherePlane(result, si, sj, xi, xj, qi, qj, bi, bj);
                case 4: //Shape.types.BOX: // sphere-box
                    sphereBox(result,si,sj,xi,xj,qi,qj,bi,bj);
                case 8: //Shape.types.COMPOUND: // sphere-compound
                    recurseCompound(result,si,sj,xi,xj,qi,qj,bi,bj);
                case 16: //Shape.types.CONVEXPOLYHEDRON: // sphere-convexpolyhedron
                    throw "Not Implemented";
                    //sphereConvex(result,si,sj,xi,xj,qi,qj,bi,bj);
                default:
                    trace("Collision between CANNON.Shape.types.SPHERE and "+sj.type+" not implemented yet.");
                }

            } else if(si.type == types.PLANE){

                switch(sj.type){
                case 2: //types.PLANE: // plane-plane
                    throw "Plane-plane collision... wait, you did WHAT?";
                case 4: //types.BOX: // plane-box
                    planeBox(result,si,sj,xi,xj,qi,qj,bi,bj);
                case 8: //types.COMPOUND: // plane-compound
                    recurseCompound(result,si,sj,xi,xj,qi,qj,bi,bj);
                case 16: //types.CONVEXPOLYHEDRON: // plane-convex polyhedron
                    planeConvex(result,si,sj,xi,xj,qi,qj,bi,bj);
                default:
                    trace("Collision between CANNON.Shape.types.PLANE and "+sj.type+" not implemented yet.");
                }

            } else if(si.type==types.BOX){

                switch(sj.type){
                case 4: //types.BOX: // box-box
                    // Do convex/convex instead
                    nearPhase(result,si.convexPolyhedronRepresentation,sj.convexPolyhedronRepresentation,xi,xj,qi,qj,bi,bj);
                case 8: //types.COMPOUND: // box-compound
                    recurseCompound(result,si,sj,xi,xj,qi,qj,bi,bj);
                case 16: //types.CONVEXPOLYHEDRON: // box-convexpolyhedron
                    // Do convex/convex instead
                    nearPhase(result,si.convexPolyhedronRepresentation,sj,xi,xj,qi,qj,bi,bj);
                default:
                    trace("Collision between CANNON.Shape.types.BOX and "+sj.type+" not implemented yet.");
                }

            } else if(si.type==types.COMPOUND){

                switch(sj.type){
                case 8: //types.COMPOUND: // compound-compound
                    recurseCompound(result,si,sj,xi,xj,qi,qj,bi,bj);
                case 16: //types.CONVEXPOLYHEDRON: // compound-convex polyhedron
                    // Must swap
                    var r = [];
                    recurseCompound(r,sj,si,xj,xi,qj,qi,bj,bi);
                    for(ri in 0...r.length){
                        swapResult(r[ri]);
                        result.push(r[ri]);
                    }
                default:
                    trace("Collision between CANNON.Shape.types.COMPOUND and "+sj.type+" not implemented yet.");
                }

            } else if(si.type==types.CONVEXPOLYHEDRON){

                switch(sj.type){
                case 16: //types.CONVEXPOLYHEDRON: // convex polyhedron - convex polyhedron
                    convexConvex(result,si,sj,xi,xj,qi,qj,bi,bj);
                default:
                    trace("Collision between CANNON.Shape.types.CONVEXPOLYHEDRON and "+sj.type+" not implemented yet.");
                }

            }

        } else {

            // Particle!
            switch(sj.type){
            case 2: //types.PLANE: // Particle vs plane
                particlePlane(result,si,sj,xi,xj,qi,qj,bi,bj);
            case 1: //types.SPHERE: // Particle vs sphere
                particleSphere(result,si,sj,xi,xj,qi,qj,bi,bj);
            case 4: //types.BOX: // Particle vs box
                particleConvex(result,si,sj.convexPolyhedronRepresentation,xi,xj,qi,qj,bi,bj);
            case 16: //types.CONVEXPOLYHEDRON: // particle-convex
                particleConvex(result,si,sj,xi,xj,qi,qj,bi,bj);
            case 8: //types.COMPOUND: // particle-compound
                recurseCompound(result,si,sj,xi,xj,qi,qj,bi,bj);
            default:
                trace("Collision between CANNON.Particle and "+sj.type+" not implemented yet.");
            }
        }

        // Swap back if we swapped bodies in the beginning
        var Nresults = result.length;
        for(i in 0...Nresults){
            swapResult(result[i]);
        }
    }

    /**
     * @method reduceContacts
     * @memberof CANNON.ContactGenerator
     * @brief Removes unnecessary members of an array of CANNON.ContactPoint.
     */
    public function reduceContacts(contacts){

    }

    /**
     * @method getContacts
     * @memberof CANNON.ContactGenerator
     * @param array p1 Array of body indices
     * @param array p2 Array of body indices
     * @param CANNON.World world
     * @param array result Array to store generated contacts
     * @param array oldcontacts Optional. Array of reusable contact objects
     */
    public function getContacts(p1:Array<Body>,p2:Array<Body>,world:World,result:Array<Dynamic>,oldcontacts:Array<Dynamic>){
        // Save old contact objects
        contactPointPool = oldcontacts;

        var N:Int = p1.length;
        for(k in 0...N){
            // Get current collision indeces
            var bi:Body = p1[k];
            var bj:Body = p2[k];

            // Get contacts
            nearPhase(  result,
                        bi.shape,
                        bj.shape,
                        bi.position,
                        bj.position,
                        bi.quaternion,
                        bj.quaternion,
                        bi,
                        bj
                        );
        }
    }
}
