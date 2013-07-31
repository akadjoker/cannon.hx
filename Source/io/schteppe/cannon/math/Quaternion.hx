package io.schteppe.cannon.math;

/**
 * @class Quaternion
 * @brief A Quaternion describes a rotation in 3D space.
 * @description The Quaternion is mathematically defined as Q = x*i + y*j + z*k + w, where (i,j,k) are imaginary basis vectors. (x,y,z) can be seen as a vector related to the axis of rotation, while the real multiplier, w, is related to the amount of rotation.
 * @param float x Multiplier of the imaginary basis vector i.
 * @param float y Multiplier of the imaginary basis vector j.
 * @param float z Multiplier of the imaginary basis vector k.
 * @param float w Multiplier of the real part.
 * @see http://en.wikipedia.org/wiki/Quaternion
 */
class Quaternion {

    public var x:Float;
    public var y:Float;
    public var z:Float;
    public var w:Float;

    static var Quaternion_mult_va:Vec3 = new Vec3();
    static var Quaternion_mult_vb:Vec3 = new Vec3();
    static var Quaternion_mult_vaxvb:Vec3 = new Vec3();

    public function new(x = 0.0,y = 0.0,z = 0.0,w = 1.0){
        /**
        * @property float x
        * @memberof Quaternion
        */
        this.x = x;
        /**
        * @property float y
        * @memberof Quaternion
        */
        this.y = y;
        /**
        * @property float z
        * @memberof Quaternion
        */
        this.z = z;
        /**
        * @property float w
        * @memberof Quaternion
        * @brief The multiplier of the real quaternion basis vector.
        */
        this.w = w;
    }

    /**
     * @method set
     * @memberof Quaternion
     * @brief Set the value of the quaternion.
     * @param float x
     * @param float y
     * @param float z
     * @param float w
     */
    public function set(x,y,z,w):Void{
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    /**
     * @method toString
     * @memberof Quaternion
     * @brief Convert to a readable format
     * @return string
     */
    public function toString():String{
        return this.x+","+this.y+","+this.z+","+this.w;
    }

    /**
     * @method setFromAxisAngle
     * @memberof Quaternion
     * @brief Set the quaternion components given an axis and an angle.
     * @param Vec3 axis
     * @param float angle in radians
     */
    public function setFromAxisAngle(axis,angle):Void{
        var s:Float = Math.sin(angle*0.5);
        this.x = axis.x * s;
        this.y = axis.y * s;
        this.z = axis.z * s;
        this.w = Math.cos(angle*0.5);
    }

    // saves axis to targetAxis and returns 
    public function toAxisAngle(targetAxis):Dynamic{
        if (targetAxis == null) targetAxis = new Vec3();
        this.normalize(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
        var angle:Float = 2 * Math.acos(this.w);
        var s:Float = Math.sqrt(1-this.w*this.w); // assuming quaternion normalised then w is less than 1, so term always positive.
        if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
            // if s close to zero then direction of axis not important
            targetAxis.x = this.x; // if it is important that axis is normalised then replace with x=1; y=z=0;
            targetAxis.y = this.y;
            targetAxis.z = this.z;
        } else {
            targetAxis.x = this.x / s; // normalise axis
            targetAxis.y = this.y / s;
            targetAxis.z = this.z / s;
        }
        return [targetAxis,angle];
    }

    /**
     * @method setFromVectors
     * @memberof Quaternion
     * @brief Set the quaternion value given two vectors. The resulting rotation will be the needed rotation to rotate u to v.
     * @param Vec3 u
     * @param Vec3 v
     */
    public function setFromVectors(u,v){
        var a = u.cross(v);
        this.x = a.x;
        this.y = a.y;
        this.z = a.z;
        this.w = Math.sqrt(Math.pow(u.norm(),2) * Math.pow(v.norm(),2)) + u.dot(v);
        this.normalize();
    }

    /**
     * @method mult
     * @memberof Quaternion
     * @brief Quaternion multiplication
     * @param Quaternion q
     * @param Quaternion target Optional.
     * @return Quaternion
     */
    public function mult(q:Quaternion,target:Quaternion):Quaternion{
        if (target == null) target = new Quaternion();
        var w:Float = this.w;
        var va = Quaternion_mult_va;
        var vb = Quaternion_mult_vb;
        var vaxvb = Quaternion_mult_vaxvb;

        va.set(this.x,this.y,this.z);
        vb.set(q.x,q.y,q.z);
        target.w = w*q.w - va.dot(vb);
        va.cross(vb,vaxvb);

        target.x = w * vb.x + q.w*va.x + vaxvb.x;
        target.y = w * vb.y + q.w*va.y + vaxvb.y;
        target.z = w * vb.z + q.w*va.z + vaxvb.z;

        return target;
    }

    /**
     * @method inverse
     * @memberof Quaternion
     * @brief Get the inverse quaternion rotation.
     * @param Quaternion target
     * @return Quaternion
     */
    public function inverse(target):Quaternion{
        var x:Float = this.x; var y:Float = this.y; var z:Float = this.z; var w:Float = this.w;
        if (target == null) target = new Quaternion();

        this.conjugate(target);
        var inorm2:Float = 1/(x*x + y*y + z*z + w*w);
        target.x *= inorm2;
        target.y *= inorm2;
        target.z *= inorm2;
        target.w *= inorm2;

        return target;
    }

    /**
     * @method conjugate
     * @memberof Quaternion
     * @brief Get the quaternion conjugate
     * @param Quaternion target
     * @return Quaternion
     */
    public function conjugate(target):Quaternion{
        if (target == null) target = new Quaternion();

        target.x = -this.x;
        target.y = -this.y;
        target.z = -this.z;
        target.w = this.w;

        return target;
    }

    /**
     * @method normalize
     * @memberof Quaternion
     * @brief Normalize the quaternion. Note that this changes the values of the quaternion.
     */
    public function normalize():Void{
        var l = Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w);
        if ( l == 0.0 ) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 0;
        } else {
            l = 1 / l;
            this.x *= l;
            this.y *= l;
            this.z *= l;
            this.w *= l;
        }
    }

    /**
     * @method normalizeFast
     * @memberof Quaternion
     * @brief Approximation of quaternion normalization. Works best when quat is already almost-normalized.
     * @see http://jsperf.com/fast-quaternion-normalization
     * @author unphased, https://github.com/unphased
     */
    public function normalizeFast():Void {
        var f = (3.0-(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w))/2.0;
        if ( f == 0 ) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 0;
        } else {
            this.x *= f;
            this.y *= f;
            this.z *= f;
            this.w *= f;
        }
    }

    /**
     * @method vmult
     * @memberof Quaternion
     * @brief Multiply the quaternion by a vector
     * @param Vec3 v
     * @param Vec3 target Optional
     * @return Vec3
     */
    public function vmult(v,target):Vec3{
        if (target == null) target = new Vec3();
        if(this.w==0.0){
            target.x = v.x;
            target.y = v.y;
            target.z = v.z;
        } else {

            var x = v.x,
                y = v.y,
                z = v.z;

            var qx = this.x,
                qy = this.y,
                qz = this.z,
                qw = this.w;

            // q*v
            var ix =  qw * x + qy * z - qz * y,
            iy =  qw * y + qz * x - qx * z,
            iz =  qw * z + qx * y - qy * x,
            iw = -qx * x - qy * y - qz * z;

            target.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
            target.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
            target.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;
        }

        return target;
    }

    /**
     * @method copy
     * @memberof Quaternion
     * @param Quaternion target
     */
    public function copy(target):Void{
        target.x = this.x;
        target.y = this.y;
        target.z = this.z;
        target.w = this.w;
    }

    /**
     * @method toEuler
     * @memberof Quaternion
     * @brief Convert the quaternion to euler angle representation. Order: YZX, as this page describes: http://www.euclideanspace.com/maths/standards/index.htm
     * @param Vec3 target
     * @param string order Three-character string e.g. "YZX", which also is default.
     */
    public function toEuler(target:Vec3, order:String = null):Void{
        if (order == null) order = "YZX";

        var heading:Float = Math.NaN; var attitude:Float = 0.0; var bank:Float = 0.0;
        var x:Float = this.x; var y:Float = this.y; var z:Float = this.z; var w:Float = this.w;

        switch(order){
        case "YZX":
            var test:Float = x * y + z * w;
            if (test > 0.499) { // singularity at north pole
                heading = 2 * Math.atan2(x,w);
                attitude = Math.PI/2;
                bank = 0;
            }
            if (test < -0.499) { // singularity at south pole
                heading = -2 * Math.atan2(x,w);
                attitude = - Math.PI/2;
                bank = 0;
            }
            if(Math.isNaN(heading)){
                var sqx:Float = x*x;
                var sqy:Float = y*y;
                var sqz:Float = z*z;
                heading = Math.atan2(2*y*w - 2*x*z , 1.0 - 2*sqy - 2*sqz); // Heading
                attitude = Math.asin(2*test); // attitude
                bank = Math.atan2(2*x*w - 2*y*z , 1.0 - 2*sqx - 2*sqz); // bank
            }
        default:
            throw "Euler order "+order+" not supported yet.";
        }

        target.y = heading;
        target.z = attitude;
        target.x = bank;
    }
}
