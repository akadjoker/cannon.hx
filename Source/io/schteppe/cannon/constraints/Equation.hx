package io.schteppe.cannon.constraints;

import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.objects.Body;

/**
 * @class CANNON.Equation
 * @brief Equation base class
 * @author schteppe
 * @param CANNON.Body bi
 * @param CANNON.Body bj
 * @param float minForce Minimum (read: negative max) force to be applied by the constraint.
 * @param float maxForce Maximum (read: positive max) force to be applied by the constraint.
 */
class Equation {

    public var id:Int;
    public var minForce:Float;
    public var maxForce:Float;
    public var bi:Body;
    public var bj:Body;
    public var stiffness:Float;
    public var regularizationTime:Int = 5;
    public var a:Float;
    public var b:Float;
    public var eps:Float;
    public var spookParamsNeedsUpdate:Bool;
    public var invInertiaTimesRxnNeedsUpdate:Bool;

    public function new(bi:Body,bj:Body,minForce:Float = -1.0,maxForce:Float = -1.0){
        this.id = -1;

        /**
         * @property float minForce
         * @memberof CANNON.Equation
         */
        this.minForce = minForce < 0 ? -1e6 : minForce;

        /**
         * @property float maxForce
         * @memberof CANNON.Equation
         */
        this.maxForce = maxForce < 0 ? 1e6 : maxForce;

        /**
         * @property CANNON.Body bi
         * @memberof CANNON.Equation
         */
        this.bi = bi;

        /**
         * @property CANNON.Body bj
         * @memberof CANNON.Equation
         */
        this.bj = bj;

        /**
         * @property float stiffness
         * @brief Corresponds to spring stiffness. Makes constraints stiffer, but harder to solve.
         * @memberof CANNON.Equation
         */
        this.stiffness = 1e7;

        /**
         * @property float regularizationTime
         * @brief Similar to damping. Represents the number of timesteps needed to stabilize the constraint.
         * @memberof CANNON.Equation
         */
        this.regularizationTime = 5;

        /**
         * @property float a
         * @brief SPOOK parameter
         * @memberof CANNON.Equation
         */
        this.a = 0.0;

        /**
         * @property float b
         * @brief SPOOK parameter
         * @memberof CANNON.Equation
         */
        this.b = 0.0;

        /**
         * @property float eps
         * @brief SPOOK parameter
         * @memberof CANNON.Equation
         */
        this.eps = 0.0;

        /**
         * @property bool spookParamsNeedsUpdate
         * @brief Set to true if you just changed stiffness or regularization. The parameters a,b,eps will be recalculated by the solver before solve.
         * @memberof CANNON.Equation
         */
        this.spookParamsNeedsUpdate = true;
    }

    /**
     * @method updateSpookParams
     * @brief Recalculates a,b,eps.
     * @memberof CANNON.Equation
     */
    public function updateSpookParams(h:Float){
        var d:Int = this.regularizationTime;
        var k:Float = this.stiffness;
        this.a = 4.0 / (h * (1 + 4 * d));
        this.b = (4.0 * d) / (1 + 4 * d);
        this.eps = 4.0 / (h * h * k * (1 + 4 * d));
    }

    public function computeB(h:Float):Float { 
        return 0.0;
    }
    public function computeC():Float {
        return 0.0;
    }
    public function computeGWlambda():Float {
        return 0.0;
    }
    public function addToWlambda(deltalambda:Float):Void {
    }
}
