package io.schteppe.cannon.constraints;

import io.schteppe.cannon.math.Mat3;
import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.objects.Body;

/**
 * @class CANNON.ContactEquation
 * @brief Contact/non-penetration constraint equation
 * @author schteppe
 * @param CANNON.Body bj
 * @param CANNON.Body bi
 * @extends CANNON.Equation
 */
class ContactEquation extends Equation {

    public var restitution:Float;
    public var ri:Vec3;
    public var rj:Vec3;

    public var penetration:Float;
    public var penetrationVec:Vec3;
    public var ni:Vec3;
    public var rixn:Vec3;
    public var rjxn:Vec3;
    public var invIi:Mat3;
    public var invIj:Mat3;
    public var biInvInertiaTimesRixn:Vec3;
    public var bjInvInertiaTimesRjxn:Vec3;

    var ContactEquation_computeB_temp1:Vec3; // Temp vectors
    var ContactEquation_computeB_temp2:Vec3;
    var ContactEquation_computeB_zero:Vec3;
    var computeC_temp1:Vec3;
    var computeC_temp2:Vec3;
    var ContactEquation_addToWlambda_temp1:Vec3;
    var ContactEquation_addToWlambda_temp2:Vec3;
    var computeGWlambda_ulambda:Vec3;

    public function new(bi:Body,bj:Body){
        super(bi,bj,0,1e6);

        /**
         * @property float restitution
         * @memberof CANNON.ContactEquation
         */
        this.restitution = 0.0; // "bounciness": u1 = -e*u0

        /**
         * @property Vec3 ri
         * @memberof CANNON.ContactEquation
         * @brief World-oriented vector that goes from the center of bi to the contact point in bi.
         */
        this.ri = new Vec3();

        /**
         * @property Vec3 rj
         * @memberof CANNON.ContactEquation
         */
        this.rj = new Vec3();

        this.penetrationVec = new Vec3();

        this.ni = new Vec3();
        this.rixn = new Vec3();
        this.rjxn = new Vec3();

        this.invIi = new Mat3();
        this.invIj = new Mat3();

        // Cache
        this.biInvInertiaTimesRixn =  new Vec3();
        this.bjInvInertiaTimesRjxn =  new Vec3();

        ContactEquation_computeB_temp1 = new Vec3();
        ContactEquation_computeB_temp2 = new Vec3();
        ContactEquation_computeB_zero = new Vec3();
        computeC_temp1 = new Vec3();
        computeC_temp2 = new Vec3();
        ContactEquation_addToWlambda_temp1 = new Vec3();
        ContactEquation_addToWlambda_temp2 = new Vec3();
        computeGWlambda_ulambda = new Vec3();
    }

    /**
     * @method reset
     * @memberof CANNON.ContactEquation
     * @brief To be run before object reuse
     */
    public function reset(){
        this.invInertiaTimesRxnNeedsUpdate = true;
    }

    public override function computeB(h:Float):Float {
        var a:Float = this.a;
        var b:Float = this.b;
        var bi:Body = this.bi;
        var bj:Body  = this.bj;
        var ri:Vec3 = this.ri;
        var rj:Vec3 = this.rj;
        var rixn:Vec3 = this.rixn;
        var rjxn:Vec3 = this.rjxn;

        var zero = ContactEquation_computeB_zero;

        var vi:Vec3 = bi.velocity;
        var wi:Vec3 = (bi.angularVelocity!=null) ? bi.angularVelocity : zero;
        var fi:Vec3 = bi.force;
        var taui:Vec3 = (bi.tau!=null) ? bi.tau : zero;

        var vj:Vec3 = bj.velocity;
        var wj:Vec3 = (bj.angularVelocity!=null) ? bj.angularVelocity : zero;
        var fj:Vec3 = bj.force;
        var tauj:Vec3 = (bj.tau!=null) ? bj.tau : zero;

        var penetrationVec:Vec3 = this.penetrationVec;
        var invMassi:Float = bi.invMass;
        var invMassj:Float = bj.invMass;

        var invIi:Mat3 = this.invIi;
        var invIj:Mat3 = this.invIj;

        if(bi.invInertia != null){
            invIi.setTrace(bi.invInertia);
        } else {
            invIi.identity(); // ok?
        }
        if(bj.invInertia != null){
            invIj.setTrace(bj.invInertia);
        } else {
            invIj.identity(); // ok?
        }

        var n:Vec3 = this.ni;

        // Caluclate cross products
        ri.cross(n,rixn);
        rj.cross(n,rjxn);

        // Calculate q = xj+rj -(xi+ri) i.e. the penetration vector
        var penetrationVec:Vec3 = this.penetrationVec;
        penetrationVec.set(0,0,0);
        penetrationVec.vadd(bj.position,penetrationVec);
        penetrationVec.vadd(rj,penetrationVec);
        penetrationVec.vsub(bi.position,penetrationVec);
        penetrationVec.vsub(ri,penetrationVec);

        var Gq:Float = n.dot(penetrationVec);//-Math.abs(this.penetration);

        var invIi_vmult_taui = ContactEquation_computeB_temp1;
        var invIj_vmult_tauj = ContactEquation_computeB_temp2;
        invIi.vmult(taui,invIi_vmult_taui);
        invIj.vmult(tauj,invIj_vmult_tauj);

        // Compute iteration
        var ePlusOne:Float = this.restitution + 1;
        var GW:Float = ePlusOne * vj.dot(n) - ePlusOne * vi.dot(n) + wj.dot(rjxn) - wi.dot(rixn);
        var GiMf:Float = fj.dot(n)*invMassj - fi.dot(n)*invMassi + rjxn.dot(invIj_vmult_tauj) - rixn.dot(invIi_vmult_taui);
        var B:Float = - Gq * a - GW * b - h*GiMf;

        return B;
    }

    // Compute C = GMG+eps in the SPOOK equation
    public override function computeC():Float{
        var bi:Body = this.bi;
        var bj:Body = this.bj;
        var rixn:Vec3 = this.rixn;
        var rjxn:Vec3 = this.rjxn;
        var invMassi:Float = bi.invMass;
        var invMassj:Float = bj.invMass;

        var C:Float = invMassi + invMassj + this.eps;

        var invIi:Mat3 = this.invIi;
        var invIj:Mat3 = this.invIj;


        ////if(bi.invInertia){
        ////    invIi.setTrace(bi.invInertia);
        ////} else {
        ////    invIi.identity(); // ok?
        ////}
        ////if(bj.invInertia){
        ////    invIj.setTrace(bj.invInertia);
        ////} else {
        ////    invIj.identity(); // ok?
        ////}


        // Compute rxn * I * rxn for each body
        invIi.vmult(rixn, this.biInvInertiaTimesRixn);
        invIj.vmult(rjxn, this.bjInvInertiaTimesRjxn);


        ////invIi.vmult(rixn,computeC_temp1);
        ////invIj.vmult(rjxn,computeC_temp2);
        
        ////C += computeC_temp1.dot(rixn);
        ////C += computeC_temp2.dot(rjxn);

        C += this.biInvInertiaTimesRixn.dot(rixn);
        C += this.bjInvInertiaTimesRjxn.dot(rjxn);

        return C;
    }

    public override function computeGWlambda():Float{
        var bi:Body = this.bi;
        var bj:Body = this.bj;
        var ulambda:Vec3 = computeGWlambda_ulambda;

        var GWlambda:Float = 0.0;

        bj.vlambda.vsub(bi.vlambda, ulambda);
        GWlambda += ulambda.dot(this.ni);

        // Angular
        if(bi.wlambda != null){
            GWlambda -= bi.wlambda.dot(this.rixn);
        }
        if(bj.wlambda != null){
            GWlambda += bj.wlambda.dot(this.rjxn);
        }

        return GWlambda;
    }

    public override function addToWlambda(deltalambda:Float):Void{
        var bi:Body = this.bi;
        var bj:Body = this.bj;
        var rixn:Vec3 = this.rixn;
        var rjxn:Vec3 = this.rjxn;
        var invMassi:Float = bi.invMass;
        var invMassj:Float = bj.invMass;
        var n:Vec3 = this.ni;
        var temp1:Vec3 = ContactEquation_addToWlambda_temp1;
        var temp2:Vec3 = ContactEquation_addToWlambda_temp2;


        // Add to linear velocity
        n.mult(invMassi * deltalambda, temp2);
        bi.vlambda.vsub(temp2,bi.vlambda);
        n.mult(invMassj * deltalambda, temp2);
        bj.vlambda.vadd(temp2,bj.vlambda);

        // Add to angular velocity
        if(bi.wlambda != null){
            this.biInvInertiaTimesRixn.mult(deltalambda,temp1);

            bi.wlambda.vsub(temp1,bi.wlambda);
        }
        if(bj.wlambda != null){
            this.bjInvInertiaTimesRjxn.mult(deltalambda,temp1);
            bj.wlambda.vadd(temp1,bj.wlambda);
        }
    }
}
