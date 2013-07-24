package io.schteppe.cannon.solver;

import io.schteppe.cannon.constraints.Equation;
import io.schteppe.cannon.world.World;

/**
 * @class CANNON.Solver
 * @brief Constraint equation solver base class.
 * @author schteppe / https://github.com/schteppe
 */
class Solver {

    public var iterations:Int;
    public var tolerance:Float;

    var equations:Array<Equation>;

    public function new(){
        // All equations to be solved
        equations = [];
    }

    // Should be implemented in subclasses!
    public function solve(dt:Float,world:World){
        // Should return the number of iterations done!
        return 0;
    }

    public function addEquation(eq:Equation){
        this.equations.push(eq);
    }

    public function removeEquation(eq:Equation){
        var eqs = this.equations;
        var i = Lambda.indexOf(eqs, eq);
        if(i != -1){
            eqs.splice(i,1);
        }
    }

    public function removeAllEquations(){
        this.equations.splice(0, this.equations.length);
    }
}
