package io.schteppe.cannon.material;

/**
 * @class CANNON.ContactMaterial
 * @brief Defines what happens when two materials meet.
 * @param CANNON.Material m1
 * @param CANNON.Material m2
 * @param float friction
 * @param float restitution
 * @todo Contact solving parameters here too?
 */

class ContactMaterial {

    public var id = -1;
    public var materials:Array<Material>;
    public var friction:Float;
    public var restitution:Float;
    public var contactEquationStiffness:Float;
    public var contactEquationRegularizationTime:Int;
    public var frictionEquationStiffness:Float;
    public var frictionEquationRegularizationTime:Int;

    public function new(m1:Material, m2:Material, friction:Float = -1.0, restitution:Float = -1.0){

        /// Contact material index in the world, -1 until added to the world
        this.id = -1;

        /// The two materials participating in the contact
        this.materials = [m1,m2];

        /// Kinetic friction
        this.friction = friction > -0.1 ? friction : 0.3;

        /// Restitution
        this.restitution = restitution > -0.1 ? restitution : 0.3;

        // Parameters to pass to the constraint when it is created
        this.contactEquationStiffness = 1e7;
        this.contactEquationRegularizationTime = 3;
        this.frictionEquationStiffness = 1e7;
        this.frictionEquationRegularizationTime = 3;
    }
}
