package io.schteppe.cannon.constraints;
import io.schteppe.cannon.math.Mat3;
import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.objects.Body;

/**
 * @class CANNON.FrictionEquation
 * @brief Constrains the slipping in a contact along a tangent
 * @author schteppe
 * @param CANNON.Body bi
 * @param CANNON.Body bj
 * @param float slipForce should be +-F_friction = +-mu * F_normal = +-mu * m * g
 * @extends CANNON.Equation
 */

class FrictionEquation extends Equation {

    public var ri:Vec3;
    public var rj:Vec3;
    public var t:Vec3;
    public var rixt:Vec3;
    public var rjxt:Vec3;
    public var wixri:Vec3;
    public var wjxrj:Vec3;
    public var invIi:Mat3;
    public var invIj:Mat3;
    public var relVel:Vec3;
    public var relForce:Vec3;
    public var biInvInertiaTimesRixt:Vec3;
    public var bjInvInertiaTimesRjxt:Vec3;

    var FrictionEquation_computeGWlambda_ulambda:Vec3;
    var FrictionEquation_addToWlambda_tmp:Vec3;
    var FrictionEquation_computeB_temp1:Vec3;
    var FrictionEquation_computeB_temp2:Vec3;
    var FrictionEquation_computeB_zero:Vec3;

    public function new(bi:Body,bj:Body,slipForce:Float){
        super(bi, bj, -slipForce, slipForce);

        this.ri = new Vec3();
        this.rj = new Vec3();
        this.t = new Vec3(); // tangent


        // The following is just cache
        this.rixt = new Vec3();
        this.rjxt = new Vec3();
        this.wixri = new Vec3();
        this.wjxrj = new Vec3();

        this.invIi = new Mat3();
        this.invIj = new Mat3();

        this.relVel = new Vec3();
        this.relForce = new Vec3();

        this.biInvInertiaTimesRixt =  new Vec3();
        this.bjInvInertiaTimesRjxt =  new Vec3();

        FrictionEquation_computeGWlambda_ulambda = new Vec3();
        FrictionEquation_addToWlambda_tmp = new Vec3();
        FrictionEquation_computeB_temp1 = new Vec3();
        FrictionEquation_computeB_temp2 = new Vec3();
        FrictionEquation_computeB_zero = new Vec3();
    }

    public override function computeB(h:Float):Float{
        var a = this.a;
        var b = this.b;
        var bi = this.bi;
        var bj = this.bj;
        var ri = this.ri;
        var rj = this.rj;
        var rixt = this.rixt;
        var rjxt = this.rjxt;
        var wixri = this.wixri;
        var wjxrj = this.wjxrj;
        var zero = FrictionEquation_computeB_zero;

        var vi = bi.velocity;
        var wi = (bi.angularVelocity!=null) ? bi.angularVelocity : zero;
        var fi = bi.force;
        var taui = (bi.tau!=null) ? bi.tau : zero;

        var vj = bj.velocity;
        var wj = (bj.angularVelocity!=null) ? bj.angularVelocity : zero;
        var fj = bj.force;
        var tauj = (bj.tau!=null) ? bj.tau : zero;

        var relVel = this.relVel;
        var relForce = this.relForce;
        var invMassi = bi.invMass;
        var invMassj = bj.invMass;

        var invIi = this.invIi;
        var invIj = this.invIj;

        var t = this.t;

        var invIi_vmult_taui = FrictionEquation_computeB_temp1;
        var invIj_vmult_tauj = FrictionEquation_computeB_temp2;

        if(bi.invInertia != null){
            invIi.setTrace(bi.invInertia);
        }
        if(bj.invInertia != null){
            invIj.setTrace(bj.invInertia);
        }


        // Caluclate cross products
        ri.cross(t,rixt);
        rj.cross(t,rjxt);

        wi.cross(ri,wixri);
        wj.cross(rj,wjxrj);

        invIi.vmult(taui,invIi_vmult_taui);
        invIj.vmult(tauj,invIj_vmult_tauj);

        var Gq = 0; // we do only want to constrain motion
        var GW = vj.dot(t) - vi.dot(t) + wjxrj.dot(t) - wixri.dot(t); // eq. 40
        var GiMf = fj.dot(t)*invMassj - fi.dot(t)*invMassi + rjxt.dot(invIj_vmult_tauj) - rixt.dot(invIi_vmult_taui);

        var B = - Gq * a - GW * b - h*GiMf;

        return B;
    }

    // Compute C = G * Minv * G + eps
    //var FEcomputeC_temp1 = new Vec3();
    //var FEcomputeC_temp2 = new Vec3();
    public override function computeC():Float{
        var bi = this.bi;
        var bj = this.bj;
        var rixt = this.rixt;
        var rjxt = this.rjxt;
        var invMassi = bi.invMass;
        var invMassj = bj.invMass;
        var C = invMassi + invMassj + this.eps;
        var invIi = this.invIi;
        var invIj = this.invIj;

        ////if(bi.invInertia){
        ////    invIi.setTrace(bi.invInertia);
        ////}
        ////if(bj.invInertia){
        ////    invIj.setTrace(bj.invInertia);
        ////}

        // Compute rxt * I * rxt for each body
        ////invIi.vmult(rixt,FEcomputeC_temp1);
        ////invIj.vmult(rjxt,FEcomputeC_temp2);
        ////C += FEcomputeC_temp1.dot(rixt);
        ////C += FEcomputeC_temp2.dot(rjxt);

        invIi.vmult(rixt,this.biInvInertiaTimesRixt);
        invIj.vmult(rjxt,this.bjInvInertiaTimesRjxt);
        C += this.biInvInertiaTimesRixt.dot(rixt);
        C += this.bjInvInertiaTimesRjxt.dot(rjxt);

        return C;
    }

    public override function computeGWlambda():Float{

        // Correct at all ???

        var bi = this.bi;
        var bj = this.bj;

        var GWlambda = 0.0;
        var ulambda = FrictionEquation_computeGWlambda_ulambda;
        bj.vlambda.vsub(bi.vlambda,ulambda);
        GWlambda += ulambda.dot(this.t);

        // Angular
        if(bi.wlambda != null){
            GWlambda -= bi.wlambda.dot(this.rixt);
        }
        if(bj.wlambda != null){
            GWlambda += bj.wlambda.dot(this.rjxt);
        }

        return GWlambda;
    }

    public override function addToWlambda(deltalambda:Float):Void{
        var bi = this.bi;
        var bj = this.bj;
        var rixt = this.rixt;
        var rjxt = this.rjxt;
        var invMassi = bi.invMass;
        var invMassj = bj.invMass;
        var t = this.t;
        var tmp = FrictionEquation_addToWlambda_tmp;
        var wi = bi.wlambda;
        var wj = bj.wlambda;

        // Add to linear velocity
        t.mult(invMassi * deltalambda, tmp);
        bi.vlambda.vsub(tmp,bi.vlambda);

        t.mult(invMassj * deltalambda, tmp);
        bj.vlambda.vadd(tmp,bj.vlambda);

        // Add to angular velocity
        if(wi != null){
            ////var I = this.invIi;
            ////I.vmult(rixt,tmp);
            ////tmp.mult(deltalambda,tmp);
            this.biInvInertiaTimesRixt.mult(deltalambda,tmp);
            wi.vsub(tmp,wi);
        }
        if(wj != null){
            ////var I = this.invIj;
            ////I.vmult(rjxt,tmp);
            ////tmp.mult(deltalambda,tmp);
            this.bjInvInertiaTimesRjxt.mult(deltalambda,tmp);
            wj.vadd(tmp,wj);
        }
    }
}
