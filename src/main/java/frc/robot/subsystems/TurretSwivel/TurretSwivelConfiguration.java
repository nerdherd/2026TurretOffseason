package frc.robot.subsystems.TurretSwivel;

import org.wpilib.math.geometry.Pose2d;

public record TurretSwivelConfiguration(double lowerAngleBound, double upperAngleBound, Pose2d relativePosition, double maxAngleTolerance, double maxVelocityTolerance) {
    public TurretSwivelConfiguration(){
        this(0, 0, Pose2d.ZERO, 0, 0);
    }
    /** Set lower angle bound of the turret
     * @param lowerAngleBound lower angle bound in degrees
     */
    public TurretSwivelConfiguration setLowerAngleBound(double lowerAngleBound){
        return new TurretSwivelConfiguration(lowerAngleBound, upperAngleBound, relativePosition, maxAngleTolerance, maxVelocityTolerance);
    }
    /** Set upper angle bound of the turret
     * @param upperAngleBound upper angle bound in degrees
     */
    public TurretSwivelConfiguration setUpperAngleBound(double upperAngleBound){
        return new TurretSwivelConfiguration(lowerAngleBound, upperAngleBound, relativePosition, maxAngleTolerance, maxVelocityTolerance);
    } 
    /** Set relative position of the turret
     * @param relativePosition offset of the turret from the center of the robot in meters, x+ is forward y+ is left
     */
    public TurretSwivelConfiguration setRelativePosition(Pose2d relativePosition){
        return new TurretSwivelConfiguration(lowerAngleBound, upperAngleBound, relativePosition, maxAngleTolerance, maxVelocityTolerance);
    }
    /** Set max angle tolerance of the turret
     * @param maxAngleTolerance upper bound in degrees for the difference between the current angle and the angle to the hub
     */
    public TurretSwivelConfiguration setMaxAngleTolerance(double maxAngleTolerance){
        return new TurretSwivelConfiguration(lowerAngleBound, upperAngleBound, relativePosition, maxAngleTolerance, maxVelocityTolerance);
    } 
    /** Set max velocity tolerance of the turret
     * @param maxVelocityTolerance upper bound in meters/second for how fast the turret can be spinning
     */
    public TurretSwivelConfiguration setMaxVelocityTolerance(double maxVelocityTolerance){
        return new TurretSwivelConfiguration(lowerAngleBound, upperAngleBound, relativePosition, maxAngleTolerance, maxVelocityTolerance);
    } 
}
