package io.schteppe.cannon.material;

/**
 * @class CANNON.Material
 * @brief Defines a physics material.
 * @param string name
 * @author schteppe
 */
class Material {

    public var name:String;
    public var id:Int;

    public function new(name:String){
        /**
        * @property string name
        * @memberof CANNON.Material
        */
        this.name = name;
        this.id = -1;
    }
}
