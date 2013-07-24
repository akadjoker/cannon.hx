package io.schteppe.cannon.math;

import openfl.utils.Float32Array;

/**
 * @class CANNON.MatN
 * @brief Any matrix size class
 * @author schteppe
 * @param int cols
 * @param int rows
 * @param array elements
 */

class MatN {

    public function new(cols,rows,elements = null){
        /**
        * @property Float32Array elements
        * @memberof CANNON.MatN
        * @brief A vector containing all matrix elements
        */
        if(elements != null){
            this.elements = new Float32Array(elements);
        } else {
            this.elements = new Float32Array(cols*rows);
        }
    }

    /**
     * @method identity
     * @memberof CANNON.MatN
     * @brief Sets the matrix to identity
     * @todo Should perhaps be renamed to setIdentity() to be more clear.
     * @todo Create another function that immediately creates an identity matrix eg. eye()
     */
    public function identity(){
        for(i in 0...this.cols){
            for(j in 0...this.rows){
                this.elements[0] = i==j ? 1 : 0;
            }
        }
    }
}
