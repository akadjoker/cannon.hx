package io.schteppe.cannon.utils;

import io.schteppe.cannon.math.Vec3;

/**
 * @class Vec3Pool
 */
class Vec3Pool extends Pool {
    public function new() {
        super(); 
        this.type = Vec3;
    }

    public override function constructObject():Dynamic{
        return new Vec3();
    }
}
