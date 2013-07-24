package io.schteppe.cannon.solver;

import io.schteppe.cannon.constraints.Equation;
import io.schteppe.cannon.math.Vec3;
import io.schteppe.cannon.objects.Body;
import io.schteppe.cannon.world.World;

/**
 * @class CANNON.Solver
 * @brief Constraint equation Gauss-Seidel solver.
 * @todo The spook parameters should be specified for each constraint, not globally.
 * @author schteppe / https://github.com/schteppe
 * @see https://www8.cs.umu.se/kurser/5DV058/VT09/lectures/spooknotes.pdf
 * @extends CANNON.Solver
 */
class GSSolver extends Solver {

    var GSSolver_solve_lambda:Array<Float>; // Just temporary number holders that we want to reuse each solve.
    var GSSolver_solve_invCs:Array<Float>;
    var GSSolver_solve_Bs:Array<Float>;

    public function new() {
       super();

        /**
        * @property int iterations
        * @brief The number of solver iterations determines quality of the constraints in the world. The more iterations, the more correct simulation. More iterations need more computations though. If you have a large gravity force in your world, you will need more iterations.
        * @todo write more about solver and iterations in the wiki
        * @memberof CANNON.GSSolver
        */
        this.iterations = 10;

        /**
         * When tolerance is reached, the system is assumed to be converged.
         * @property float tolerance
         */
        this.tolerance = 0.0;

        GSSolver_solve_lambda = [];
        GSSolver_solve_invCs = [];
        GSSolver_solve_Bs = [];
    }

    public override function solve(dt:Float,world:World):Int{
        //var d = this.d;
        //var ks = this.k;
        var iter:Int = 0;
        var maxIter:Int = this.iterations;
        var tolSquared:Float = this.tolerance * this.tolerance;
        //var a = this.a;
        //var b = this.b;
        var equations:Array<Equation> = this.equations;
        var Neq:Int = equations.length;
        var bodies:Array<Body> = world.bodies;
        var Nbodies:Int = bodies.length;
        var h:Float = dt;
        var q:Float;
        var B:Float;
        var invC:Float;
        var deltalambda:Float;
        var deltalambdaTot:Float;
        var GWlambda:Float;
        var lambdaj:Float;

        // Things that does not change during iteration can be computed once
        var invCs:Array<Float> = GSSolver_solve_invCs;
        var Bs:Array<Float> = GSSolver_solve_Bs;
        var lambda:Array<Float> = GSSolver_solve_lambda;
        invCs.splice(0, invCs.length);
        Bs.splice(0, Bs.length);
        lambda.splice(0, lambda.length);
        for(i in 0...Neq){
            var c = equations[i];
            if(c.spookParamsNeedsUpdate){
                c.updateSpookParams(h);
                c.spookParamsNeedsUpdate = false;
            }
            lambda[i] = 0.0;
            Bs[i] = c.computeB(h);
            invCs[i] = 1.0 / c.computeC();
        }


        if(Neq != 0){

            // Reset vlambda
            for(i  in 0...Nbodies){
                var b = bodies[i];
                var vlambda:Vec3 = b.vlambda;
                var wlambda:Vec3 = b.wlambda;
                vlambda.set(0,0,0);
                if(wlambda != null){
                    wlambda.set(0, 0, 0);
                }
            }

            // Iterate over equations
            for(iter in 0...maxIter){

                // Accumulate the total error for each iteration.
                deltalambdaTot = 0.0;

                for(j in 0...Neq){

                    var c = equations[j];

                    // Compute iteration
                    B = Bs[j];
                    invC = invCs[j];
                    lambdaj = lambda[j];
                    GWlambda = c.computeGWlambda();
                    deltalambda = invC * ( B - GWlambda - c.eps * lambdaj );

                    // Clamp if we are not within the min/max interval
                    if(lambdaj + deltalambda < c.minForce){
                        deltalambda = c.minForce - lambdaj;
                    } else if(lambdaj + deltalambda > c.maxForce){
                        deltalambda = c.maxForce - lambdaj;
                    }
                    lambda[j] += deltalambda;

                    deltalambdaTot += deltalambda > 0.0 ? deltalambda : -deltalambda; // abs(deltalambda)

                    c.addToWlambda(deltalambda);
                }

                // If the total error is small enough - stop iterate
                if(deltalambdaTot*deltalambdaTot < tolSquared){
                    break;
                }
            }

            // Add result to velocity
            for(i in 0...Nbodies){
                var b = bodies[i];
                var v:Vec3 = b.velocity;
                var w:Vec3 = b.angularVelocity;
                v.vadd(b.vlambda, v);
                if(w != null){
                    w.vadd(b.wlambda, w);
                }
            }
        }

        return iter;
    }
}
