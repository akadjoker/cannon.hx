package io.schteppe.cannon.constraints;

import io.schteppe.cannon.constraints.Equation;
import io.schteppe.cannon.objects.Body;

/**
 * @class CANNON.Constraint
 * @brief Constraint base class
 * @author schteppe
 * @param CANNON.Body bodyA
 * @param CANNON.Body bodyB
 */

class Constraint {

    public var equations:Array<Equation>;
    public var bodyA:Body;
    public var bodyB:Body;

    public function new(bodyA:Body,bodyB:Body){

        /**
         * @property Array equations
         * @memberOf CANNON.Constraint
         * @brief Equations to be solved in this constraint
         */
        this.equations = [];

        /**
         * @property CANNON.Body bodyA
         * @memberOf CANNON.Constraint
         */
        this.bodyA = bodyA;

        /**
         * @property CANNON.Body bodyB
         * @memberOf CANNON.Constraint
         */
        this.bodyB = bodyB;
    };

    /**
     * @method update
     * @memberOf CANNON.Constraint
     */
    public function update():Void {
        throw "method update() not implmemented in this Constraint subclass!";
    }
}
