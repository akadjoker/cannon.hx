package io.schteppe.cannon.math;

/**
 * @class CANNON.Mat3
 * @brief A 3x3 matrix.
 * @param array elements Array of nine elements. Optional.
 * @author schteppe / http://github.com/schteppe
 */
class Mat3 {

    var elements:Array<Float>;

    public function new(elements = null){
        /**
        * @property Array elements
        * @memberof CANNON.Mat3
        * @brief A vector of length 9, containing all matrix elements
        * The values in the array are stored in the following order:
        * | 0 1 2 |
        * | 3 4 5 |
        * | 6 7 8 |
        * 
        */
        if(elements != null){
            this.elements = elements;
        } else {
            this.elements = [0,0,0,0,0,0,0,0,0];
        }
    }

    /**
     * @method identity
     * @memberof CANNON.Mat3
     * @brief Sets the matrix to identity
     * @todo Should perhaps be renamed to setIdentity() to be more clear.
     * @todo Create another function that immediately creates an identity matrix eg. eye()
     */
    public function identity():Void{
        this.elements[0] = 1;
        this.elements[1] = 0;
        this.elements[2] = 0;

        this.elements[3] = 0;
        this.elements[4] = 1;
        this.elements[5] = 0;

        this.elements[6] = 0;
        this.elements[7] = 0;
        this.elements[8] = 1;
    }

    public function setZero():Void{
        var e = this.elements;
        e[0] = 0;
        e[1] = 0;
        e[2] = 0;
        e[3] = 0;
        e[4] = 0;
        e[5] = 0;
        e[6] = 0;
        e[7] = 0;
        e[8] = 0;
    }

    /**
     * @method setTrace
     * @memberof CANNON.Mat3
     * @brief Sets the matrix diagonal elements from a Vec3
     */
    public function setTrace(vec3:Vec3):Void{
        var e = this.elements;
        e[0] = vec3.x;
        e[4] = vec3.y;
        e[8] = vec3.z;
    }

    /**
     * @method vmult
     * @memberof CANNON.Mat3
     * @brief Matrix-Vector multiplication
     * @param Vec3 v The vector to multiply with
     * @param Vec3 target Optional, target to save the result in.
     */
    public function vmult(v:Vec3,target:Vec3 = null){
        if (target == null) target = new Vec3();

        var e = this.elements;
        var x:Float = v.x;
        var y:Float = v.y;
        var z:Float = v.z;
        target.x = e[0]*x + e[1]*y + e[2]*z;
        target.y = e[3]*x + e[4]*y + e[5]*z;
        target.z = e[6]*x + e[7]*y + e[8]*z;

        return target;
    }

    /**
     * @method smult
     * @memberof CANNON.Mat3
     * @brief Matrix-scalar multiplication
     * @param float s
     */
    public function smult(s:Float){
        for(i in 0...this.elements.length){
            this.elements[i] *= s;
        }
    }

    /**
     * @method mmult
     * @memberof CANNON.Mat3
     * @brief Matrix multiplication
     * @param CANNON.Mat3 m Matrix to multiply with from left side.
     * @return CANNON.Mat3 The result.
     */
    public function mmult(m:Mat3){
        var r = new Mat3();
        for(i in 0...3){
            for(j in 0...3){
                var sum:Float = 0.0;
                for(k in 0...3){
                    sum += m.elements[i+k*3] * this.elements[k+j*3];
                }
                r.elements[i+j*3] = sum;
            }
        }
        return r;
    }

    /**
     * @method solve
     * @memberof CANNON.Mat3
     * @brief Solve Ax=b
     * @param Vec3 b The right hand side
     * @param Vec3 target Optional. Target vector to save in.
     * @return Vec3 The solution x
     * @todo should reuse arrays
     */
    public function solve(b:Vec3,target:Vec3 = null){
        if (target == null) target = new Vec3();

        // Construct equations
        var nr:Int = 3; // num rows
        var nc:Int = 4; // num cols
        var eqns:Array<Float> = [];
        for(i in 0...(nr*nc)){
            eqns.push(0);
        }
        var i:Int;
        var j:Int;
        for (i in 0...3) {
            for (j in 0...3) {
                eqns[i + nc * j] = this.elements[i + 3 * j];
            }
        }
        eqns[3+4*0] = b.x;
        eqns[3+4*1] = b.y;
        eqns[3+4*2] = b.z;

        // Compute right upper triangular version of the matrix - Gauss elimination
        var n:Int = 3;
        var k:Int = n;
        var np:Int;
        var kp:Int = 4; // num rows
        var p:Int;
        var els;
        do {
            i = k - n;
            if (eqns[i+nc*i] == 0) {
                // the pivot is null, swap lines
                for (j in (i + 1)...k) {
                    if (eqns[i+nc*j] != 0) {
                        np = kp;
                        do {  // do ligne( i ) = ligne( i ) + ligne( k )
                            p = kp - np;
                            eqns[p+nc*i] += eqns[p+nc*j];
                        } while (--np > 0);
                        break;
                    }
                }
            }
            if (eqns[i+nc*i] != 0) {
                for (j in (i + 1)...k) {
                    var multiplier = eqns[i+nc*j] / eqns[i+nc*i];
                    np = kp;
                    do {  // do ligne( k ) = ligne( k ) - multiplier * ligne( i )
                        p = kp - np;
                        eqns[p+nc*j] = p <= i ? 0 : eqns[p+nc*j] - eqns[p+nc*i] * multiplier ;
                    } while (--np > 0);
                }
            }
        } while (--n > 0);

        // Get the solution
        target.z = eqns[2*nc+3] / eqns[2*nc+2];
        target.y = (eqns[1*nc+3] - eqns[1*nc+2]*target.z) / eqns[1*nc+1];
        target.x = (eqns[0*nc+3] - eqns[0*nc+2]*target.z - eqns[0*nc+1]*target.y) / eqns[0*nc+0];

        if(Math.isNaN(target.x) || Math.isNaN(target.y) || Math.isNaN(target.z) || !Math.isFinite(target.x) || !Math.isFinite(target.y) || !Math.isFinite(target.z)){
            throw "Could not solve equation! Got x=["+target.toString()+"], b=["+b.toString()+"], A=["+this.toString()+"]";
        }

        return target;
    }

    /**
     * @method e
     * @memberof CANNON.Mat3
     * @brief Get an element in the matrix by index. Index starts at 0, not 1!!!
     * @param int row 
     * @param int column
     * @param float value Optional. If provided, the matrix element will be set to this value.
     * @return float
     */
    // FIXME: Ugly hack
    public function e( row:Int , column:Int ,value = -9999999999.9):Float{
        if(value < -9999999999.0){
            return this.elements[column+3*row];
        } else {
            // Set value
            this.elements[column + 3 * row] = value;
            return value;
        }
    }

    /**
     * @method copy
     * @memberof CANNON.Mat3
     * @brief Copy the matrixt:Mat3 = null){
        if (target == nu
     * @param CANNON.Mat3 target Optional. Target to save the copy in.
     * @return CANNON.Mat3
     */
    public function copy(target:Mat3 = null):Mat3 {
        if (target == null) target = new Mat3();
        for(i in 0...this.elements.length){
            target.elements[i] = this.elements[i];
        }
        return target;
    }

    /**
     * @method toString
     * @memberof CANNON.Mat3
     * @brief Returns a string representation of the matrix.
     * @return string
     */
    public function toString():String{
        var r:String = "";
        var sep:String = ",";
        for(i in 0...9){
            r += this.elements[i] + sep;
        }
        return r;
    }

    /**
     * @method reverse
     * @memberof CANNON.Mat3
     * @brief reverse the matrix
     * @param CANNON.Mat3 target Optional. Target matrix to save in.
     * @return CANNON.Mat3 The solution x
     */
    public function reverse(target:Mat3 = null):Mat3{

        if (target == null) target = new Mat3();

        // Construct equations
        var nr:Int = 3; // num rows
        var nc:Int = 6; // num cols
        var eqns:Array<Float> = [];
        for(i in 0...(nr*nc)){
            eqns.push(0);
        }
        var i:Int;
        var j:Int;
        for(i in 0...3){
            for(j in 0...3){
                eqns[i+nc*j] = this.elements[i+3*j];
            }
        }
        eqns[3+6*0] = 1;
        eqns[3+6*1] = 0;
        eqns[3+6*2] = 0;
        eqns[4+6*0] = 0;
        eqns[4+6*1] = 1;
        eqns[4+6*2] = 0;
        eqns[5+6*0] = 0;
        eqns[5+6*1] = 0;
        eqns[5+6*2] = 1;

        // Compute right upper triangular version of the matrix - Gauss elimination
        var n:Int = 3; var k:Int = n; var np:Int;
        var kp:Int = nc; // num rows
        var p:Int;
        do {
            i = k - n;
            if (eqns[i+nc*i] == 0) {
                // the pivot is null, swap lines
                for (j in (i + 1)...k) {
                    if (eqns[i+nc*j] != 0) {
                        np = kp;
                        do { // do line( i ) = line( i ) + line( k )
                            p = kp - np;
                            eqns[p+nc*i] += eqns[p+nc*j];
                        } while (--np > 0);
                        break;
                    }
                }
            }
            if (eqns[i+nc*i] != 0) {
                for (j in (i + 1)...k) {
                    var multiplier:Float = eqns[i+nc*j] / eqns[i+nc*i];
                    np = kp;
                    do { // do line( k ) = line( k ) - multiplier * line( i )
                        p = kp - np;
                        eqns[p+nc*j] = p <= i ? 0 : eqns[p+nc*j] - eqns[p+nc*i] * multiplier ;
                    } while (--np > 0);
                }
            }
        } while (--n > 0);

        // eliminate the upper left triangle of the matrix
        i = 2;
        do {
            j = i-1;
            do {
                var multiplier:Float = eqns[i+nc*j] / eqns[i+nc*i];
                np = nc;
                do {
                    p = nc - np;
                    eqns[p+nc*j] =  eqns[p+nc*j] - eqns[p+nc*i] * multiplier ;
                } while (--np > 0);
            } while (j-- >= 0);
        } while (--i> 0);

        // operations on the diagonal
        i = 2;
        do {
            var multiplier:Float = 1 / eqns[i+nc*i];
            np = nc;
            do {
                p = nc - np;
                eqns[p+nc*i] = eqns[p+nc*i] * multiplier ;
            } while (--np > 0);
        } while (i-- >= 0);

        i = 2;
        do {
            j = 2;
            do {
                var pp:Float = eqns[nr+j+nc*i];
                if( Math.isNaN( pp ) || !Math.isFinite( pp ) ){
                    throw "Could not reverse! A=["+this.toString()+"]";
                }
                target.e( i , j , pp );
            } while (j-- > 0);
        } while (i-- >= 0);

        return target;
    }
}
