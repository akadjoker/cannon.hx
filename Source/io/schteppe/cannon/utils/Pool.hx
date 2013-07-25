package io.schteppe.cannon.utils;

/**
 * @class CANNON.ObjectPool
 * @brief For pooling objects that can be reused.
 */
class Pool {

    var objects:Array<Dynamic>;
    var type:Dynamic;

    public function new(){
        this.objects = [];
        this.type = null;
    }

    public function releaseGroup(group:Array<Dynamic>) {
        for(obj in group){
            this.objects.push(obj);
        }
    }

    public function release(obj:Dynamic) {
        this.objects.push(obj);
    }

    public function get():Dynamic{
        if(this.objects.length==0){
            return this.constructObject();
        } else {
            return this.objects.pop();
        }
    }

    public function constructObject():Dynamic{
        throw "constructObject() not implemented in this ObjectPool subclass yet!";
        return null;
    }
}