declare namespace B2 {
    interface Vec2 {
        x: number, y: number
    }

    class Vec2Vector {
        push_back(v: Vec2): void;
        get(i: number): Vec2;
        length(): number;
    }

    // interface Transform {
    //     p: Vec2, q: Vec2
    // }

    interface Color {
        r: number, g: number, b: number, a: number
    }
    // class Vec2 {
    //     constructor();
    //     constructor(x: number, y: number);
    //     SetX(v: number): void;
    //     GetX(): number;
    //     SetY(v: number): void;
    //     GetY(): number;
    //     SetZero(): void;
    //     Set(x: number, y: number): void;
    //     Length(): number;
    //     LengthSquared(): number;
    //     Normalize(): number;
    //     IsValid(): boolean;
    // }

    class Transform {
        constructor();
        constructor(pos: Vec2, rot: number);
        SetIdentity(): void;
        Set(position: Vec2, angle: number): void;
    }

    class MassData {
        mass: number;
        center: Vec2;
        I: number;
    }

    class AABB {
        constructor();
        lowerBound: Vec2;
        upperBound: Vec2;
        IsValid(): boolean;
        GetCenter(): Vec2;
        GetExtents(): Vec2;
        GetPerimeter(): number;
        //Combine(aabb: AABB): void;
        //Combine(aabb1: AABB, aabb2: AABB): void;
        Contains(aabb: AABB): boolean;
        RayCast(output: RayCastOutput, input: RayCastInput): boolean;
        TestOverlap(other: AABB): boolean;
    }

    class RayCastCallback {
        constructor();
        ReportFixture(fixture: Fixture, point: Vec2, normal: Vec2, fraction: number): number;
    }

    class QueryCallback {
        constructor();
        ReportFixture(fixture: Fixture): boolean;
    }

    class RayCastInput {
        p1: Vec2;
        p2: Vec2;
        maxFraction: number;
    }

    class RayCastOutput {
        normal: Vec2;
        fraction: number;
    }

    class Filter {
        categoryBits: number;
        maskBits: number;
        groupIndex: number;
    }

    class FixtureProxy {
        aabb: AABB;
        fixture: Fixture;
        childIndex: number;
        proxyId: number;
    }

    class ManifoldPoint {
        localPoint: Vec2;
        normalImpulse: number;
        tangentImpulse: number;
        id: ContactID;
    }

    class Manifold {
        localNormal: Vec2;
        localPoint: Vec2;
        //points: ManifoldPoint[];
        type: number;
        pointCount: number;
        GetPoint(id: number): ManifoldPoint;
    }

    class Contact {
        GetManifold(): Manifold;
        GetWorldManifold(worldManifold: WorldManifold): void;
        IsTouching(): boolean;
        SetEnabled(flag: boolean): void;
        IsEnabled(): boolean;
        GetNext(): Contact;
        GetFixtureA(): Fixture;
        GetChildIndexA(): number;
        GetFixtureB(): Fixture;
        GetChildIndexB(): number;
        SetFriction(friction: number): void;
        GetFriction(): number;
        ResetFriction(): void;
        SetRestitution(restitution: number): void;
        GetRestitution(): number;
        ResetRestitution(): void;
        SetTangentSpeed(speed: number): void;
        GetTangentSpeed(): number;
        Reset(fixtureA: Fixture, indexA: number, fixtureB: Fixture, indexB: number): void;
        Evaluate(manifold: Manifold, xfA: Transform, xfB: Transform): void;
        Dump(): void;
    }

    class ContactListener {
        constructor();
        BeginContact(contact: Contact): void;
        EndContact(contact: Contact): void;
        PreSolve(contact: Contact, oldManifold: Manifold): void;
        PostSolve(contact: Contact, impulse: ContactImpulse): void;
    }

    class ContactImpulse {
        normalImpulses: number[];
        tangentImpulses: number[];
        count: number;
    }

    class ContactFilter {
        ShouldCollide(fixtureA: Fixture, fixtureB: Fixture): boolean;
        RayCollide(userData: any, fixture: Fixture): boolean;
    }

    class ContactID {
        cf: ContactFeature;
        key: number;
    }

    class ContactFeature {
        indexA: number;
        indexB: number;
        typeA: number;
        typeB: number;
    }

    class WorldManifold {
        constructor();
        Initialize(manifold: Manifold, xfA: Transform, radiusA: number, xfB: Transform, radiusB: number): void;
        normal: Vec2;
        //points: Vec2[];
        //separations: number[];
        GetPoint(index: number): Vec2;
        GetSeparation(index: number): number;
    }

    class ContactEdge {
        other: Body;
        contact: Contact;
        prev: ContactEdge;
        next: ContactEdge;
    }

    class JointEdge {
        other: Body;
        joint: Joint;
        prev: JointEdge;
        next: JointEdge;
    }

    class Profile {
        step: number;
        collide: number;
        solve: number;
        solveInit: number;
        solveVelocity: number;
        solvePosition: number;
        broadphase: number;
        solveTOI: number;
    }

    class TimeStep {
        dt: number;
        inv_dt: number;
        dtRatio: number;
        velocityIterations: number;
        positionIterations: number;
        warmStarting: boolean;
    }

    class ContactManager {
        AddPair(proxyUserDataA: any, proxyUserDataB: any): void;
        FindNewContacts(): void;
        Destroy(c: Contact): void;
        Collide(): void;
    }

    class Draw {
        constructor();
        SetFlags(flags: number): void;
        GetFlags(): number;
        AppendFlags(flags: number): void;
        ClearFlags(flags: number): void;
        PushTransform(xf: Transform): void;
        PopTransform(xf: Transform): void;
        DrawPolygon(vertices: Vec2[], vertexCount: number, color: Color): void;
        DrawSolidPolygon(vertices: Vec2[], vertexCount: number, color: Color): void;
        DrawCircle(center: Vec2, radius: number, color: Color): void;
        DrawSolidCircle(center: Vec2, radius: number, axis: Vec2, color: Color): void;
    }

    class World {
        constructor(gravity: Vec2);
        SetDestructionListener(listener: any): void;
        SetContactFilter(filter: any): void;
        SetContactListener(listener: any): void;
        SetDebugDraw(debugDraw: B2.Draw): void;
        CreateBody(def: BodyDef): Body;
        DestroyBody(body: Body): void;
        CreateFixture(body: Body, fixtureDef: FixtureDef): Fixture;
        DestroyFixture(fixture: Fixture): void;
        CreateJoint(def: JointDef): Joint;
        DestroyJoint(joint: Joint): void;
        Step(timeStep: number, velocityIterations: number, positionIterations: number): void;
        ClearForces(): void;
        DebugDraw(): void;
        QueryAABB(callback: QueryCallback, aabb: AABB): void;
        RayCast(callback: RayCastCallback, point1: Vec2, point2: Vec2): void;
        GetBodyList(): Body;
        GetJointList(): Joint;
        GetContactList(): Contact;
        SetAllowSleeping(flag: boolean): void;
        GetAllowSleeping(): boolean;
        SetWarmStarting(flag: boolean): void;
        GetWarmStarting(): boolean;
        SetContinuousPhysics(flag: boolean): void;
        GetContinuousPhysics(): boolean;
        SetSubStepping(flag: boolean): void;
        GetSubStepping(): boolean;
        GetProxyCount(): number;
        GetBodyCount(): number;
        GetJointCount(): number;
        GetContactCount(): number;
        GetTreeHeight(): number;
        GetTreeBalance(): number;
        GetTreeQuality(): number;
        SetGravity(gravity: Vec2): void;
        GetGravity(): Vec2;
        IsLocked(): boolean;
        SetAutoClearForces(flag: boolean): void;
        GetAutoClearForces(): boolean;
        ShiftOrigin(newOrigin: Vec2): void;
        GetContactManager(): ContactManager;
        GetProfile(): Profile;
        Dump(): void;
    }

    class Shape {
        m_type: number;
        m_radius: number;
        GetType(): number;
        GetChildCount(): number;
        TestPoint(xf: Transform, p: Vec2): boolean;
        RayCast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: number): boolean;
        ComputeAABB(aabb: AABB, xf: Transform, childIndex: number): void;
        ComputeMass(massData: MassData, density: number): void;
        SetRadius(radius: number): void;
        GetRadius(): number;
    }

    class CircleShape extends Shape {
        constructor();
        m_p: Vec2;
        Clone(): CircleShape;
        GetChildCount(): number;
        TestPoint(transform: Transform, p: Vec2): boolean;
        RayCast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: number): boolean;
        ComputeAABB(aabb: AABB, transform: Transform, childIndex: number): void;
        ComputeMass(massData: MassData, density: number): void;
    }

    class EdgeShape extends Shape {
        constructor();
        Set(v1: Vec2, v2: Vec2): void;
        Clone(): EdgeShape;
        GetChildCount(): number;
        TestPoint(transform: Transform, p: Vec2): boolean;
        RayCast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: number): boolean;
        ComputeAABB(aabb: AABB, transform: Transform, childIndex: number): void;
        ComputeMass(massData: MassData, density: number): void;
    }

    class PolygonShape extends Shape {
        constructor();
        Clone(): PolygonShape;
        Set(vertices: any, count: number): void;
        SetAsBox(hx: number, hy: number): void;
        SetAsBoxWithCenterAndAngle(hx: number, hy: number, center: Vec2, angle: number): void;
        GetChildCount(): number;
        TestPoint(transform: Transform, p: Vec2): boolean;
        RayCast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: number): boolean;
        ComputeAABB(aabb: AABB, transform: Transform, childIndex: number): void;
        ComputeMass(massData: MassData, density: number): void;
        Validate(): boolean;
    }

    class FixtureDef {
        shape: Shape;
        userData: any;
        friction: number;
        restitution: number;
        density: number;
        isSensor: boolean;
    }

    class Fixture {
        GetType(): number;
        GetShape(): Shape;
        SetSensor(sensor: boolean): void;
        IsSensor(): boolean;
        SetFilterData(filter: Filter): void;
        GetFilterData(): Filter;
        Refilter(): void;
        GetBody(): Body;
        GetNext(): Fixture;
        GetUserData(): any;
        SetUserData(data: any): void;
        TestPoint(p: Vec2): boolean;
        RayCast(output: RayCastOutput, input: RayCastInput, childIndex: number): boolean;
        GetMassData(massData: MassData): void;
        SetDensity(density: number): void;
        GetDensity(): number;
        GetFriction(): number;
        SetFriction(friction: number): void;
        GetRestitution(): number;
        SetRestitution(restitution: number): void;
        GetAABB(childIndex: number): AABB;
        Dump(bodyIndex: number): void;
    }

    enum BodyType{
        b2_staticBody = 0,
        b2_kinematicBody,
        b2_dynamicBody,
        cc_animatedBody,
    }

    class BodyDef {
        type: BodyType;
        position: Vec2;
        angle: number;
        linearVelocity: Vec2;
        angularVelocity: number;
        linearDamping: number;
        angularDamping: number;
        allowSleep: boolean;
        awake: boolean;
        fixedRotation: boolean;
        bullet: boolean;
        active: boolean;
        userData: any;
        gravityScale: number;
    }

    class Body {
        CreateFixture (fixtureDef: FixtureDef): Fixture;
        CreateFixtureWithShape (shape: Shape, density: number): Fixture;
        DestroyFixture(fixture: Fixture): void;
        SetTransform(position: Vec2, angle: number): void;
        GetTransform(): Transform;
        GetPosition(): Vec2;
        SetPosition(pos: Vec2): void;
        GetAngle(): number;
        SetAngle(angle: number): void;
        GetWorldCenter(): Vec2;
        GetLocalCenter(): Vec2;
        SetLinearVelocity(v: Vec2): void;
        GetLinearVelocity(): Vec2;
        SetAngularVelocity(omega: number): void;
        GetAngularVelocity(): number;
        ApplyForce(force: Vec2, point: Vec2, wake: boolean): void;
        ApplyForceToCenter(force: Vec2, wake: boolean): void;
        ApplyTorque(torque: number, wake: boolean): void;
        ApplyLinearImpulse(impulse: Vec2, point: Vec2, wake: boolean): void;
        ApplyAngularImpulse(impulse: number, wake: boolean): void;
        GetMass(): number;
        GetInertia(): number;
        GetMassData(data: MassData): void;
        SetMassData(data: MassData): void;
        ResetMassData(): void;
        GetWorldPoint(localPoint: Vec2): Vec2;
        GetWorldVector(localVector: Vec2): Vec2;
        GetLocalPoint(worldPoint: Vec2): Vec2;
        GetLocalVector(worldVector: Vec2): Vec2;
        GetLinearVelocityFromWorldPoint(worldPoint: Vec2): Vec2;
        GetLinearVelocityFromLocalPoint(localPoint: Vec2): Vec2;
        GetLinearDamping(): number;
        SetLinearDamping(linearDamping: number): void;
        GetAngularDamping(): number;
        SetAngularDamping(angularDamping: number): void;
        GetGravityScale(): number;
        SetGravityScale(scale: number): void;
        SetType(type: BodyType): void;
        GetType(): BodyType;
        SetBullet(flag: boolean): void;
        IsBullet(): boolean;
        SetSleepingAllowed(flag: boolean): void;
        IsSleepingAllowed(): boolean;
        SetAwake(flag: boolean): void;
        IsAwake(): boolean;
        SetEnabled(flag: boolean): void;
        IsEnabled(): boolean;
        SetFixedRotation(flag: boolean): void;
        IsFixedRotation(): boolean;
        GetFixtureList(): Fixture;
        GetJointList(): JointEdge;
        GetContactList(): ContactEdge;
        GetNext(): Body;
        GetUserData(): any;
        SetUserData(data: any): void;
        GetWorld(): World;
        Dump(): void;
    }

    enum JointType {
        e_unknownJoint,
        e_revoluteJoint,
        e_prismaticJoint,
        e_distanceJoint,
        e_pulleyJoint,
        e_mouseJoint,
        e_gearJoint,
        e_wheelJoint,
        e_weldJoint,
        e_frictionJoint,
        e_ropeJoint,
        e_motorJoint
    }

    class JointDef {
        constructor(type: JointType);
        type: JointType;
        userData: any;
        collideConnected: boolean;
        SetBodyA(bodyA: Body): void;
        SetBodyB(bodyB: Body): void;
        GetBodyA(): Body;
        GetBodyB(): Body;
        SetCollideConnected(flag: boolean): void;
    }

    class Joint {
        GetType(): JointType;
        GetBodyA(): Body;
        GetBodyB(): Body;
        GetAnchorA(): Vec2;
        GetAnchorB(): Vec2;
        GetReactionForce(inv_dt: number): Vec2;
        GetReactionTorque(inv_dt: number): number;
        GetNext(): Joint;
        GetUserData(): any;
        SetUserData(data: any): void;
        IsActive(): boolean;
        GetCollideConnected(): boolean;
        Cast2DistanceJoint(): DistanceJoint;
        // Cast2FrictionJoint(): FrictionJoint;
        // Cast2GearJoint(): GearJoint;
        Cast2MotorJoint(): MotorJoint;
        Cast2MouseJoint(): MouseJoint;
        Cast2PrismaticJoint(): PrismaticJoint;
        // Cast2PulleyJoint(): PulleyJoint;
        Cast2RevoluteJoint(): RevoluteJoint;
        Cast2RopeJoint(): RopeJoint;
        Cast2WeldJoint(): WeldJoint;
        Cast2WheelJoint(): WheelJoint;
        Dump(): void;
    }

    class RevoluteJointDef extends JointDef {
        constructor();
        Initialize(bodyA: Body, bodyB: Body, anchor: Vec2): void;
        localAnchorA: Vec2;
        localAnchorB: Vec2;
        referenceAngle: number;
        enableLimit: boolean;
        lowerAngle: number;
        upperAngle: number;
        enableMotor: boolean;
        motorSpeed: number;
        maxMotorTorque: number;
    }

    class RevoluteJoint extends Joint {
        GetLocalAnchorA(): Vec2;
        GetLocalAnchorB(): Vec2;
        GetReferenceAngle(): number;
        GetJointAngle(): number;
        GetJointSpeed(): number;
        IsLimitEnabled(): boolean;
        EnableLimit(flag: boolean): void;
        GetLowerLimit(): number;
        GetUpperLimit(): number;
        SetLimits(lower: number, upper: number): void;
        IsMotorEnabled(): boolean;
        EnableMotor(flag: boolean): void;
        SetMotorSpeed(speed: number): void;
        GetMotorSpeed(): number;
        SetMaxMotorTorque(torque: number): void;
        GetMaxMotorTorque(): number;
        GetMotorTorque(inv_dt: number): number;
        Dump(): void;
    }

    class WeldJointDef extends JointDef {
        constructor();
        Initialize(bodyA: Body, bodyB: Body, anchor: Vec2): void;
        localAnchorA: Vec2;
        localAnchorB: Vec2;
        referenceAngle: number;
        frequencyHz: number;
        dampingRatio: number;
    }

    class WeldJoint extends Joint {
        GetLocalAnchorA(): Vec2;
        GetLocalAnchorB(): Vec2;
        GetReferenceAngle(): number;
        SetStiffness(stiffness: number): void;
        GetStiffness(): number;
        SetDamping(damping: number): void;
        GetDamping(): number;
        Dump(): void;
    }

    class WheelJointDef extends JointDef {
        constructor();
        Initialize(bodyA: Body, bodyB: Body, anchor: Vec2, axis: Vec2): void;
        localAnchorA: Vec2;
        localAnchorB: Vec2;
        localAxisA: Vec2;
        enableLimit: boolean;
        lowerTranslation: number;
        upperTranslation: number;
        enableMotor: boolean;
        maxMotorTorque: number;
        motorSpeed: number;
        stiffness: number;
        damping: number;
    }

    class WheelJoint extends Joint {
        GetLocalAnchorA(): Vec2;
        GetLocalAnchorB(): Vec2;
        GetLocalAxisA(): Vec2;
        GetJointTranslation(): number;
        GetJointSpeed(): number;
        IsMotorEnabled(): boolean;
        EnableMotor(flag: boolean): void;
        SetMotorSpeed(speed: number): void;
        GetMotorSpeed(): number;
        SetMaxMotorTorque(torque: number): void;
        GetMaxMotorTorque(): number;
        GetMotorTorque(inv_dt: number): number;
        SetStiffness(stiffness: number): void;
        GetStiffness(): number;
        SetDamping(damping: number): void;
        GetDamping(): number;
        Dump(): void;
    }

    class DistanceJointDef extends JointDef {
        constructor();
        Initialize(bodyA: Body, bodyB: Body, anchorA: Vec2, anchorB: Vec2): void;
        localAnchorA: Vec2;
        localAnchorB: Vec2;
        length: number;
        stiffness: number;
        damping: number;
    }

    class DistanceJoint extends Joint {
        GetLocalAnchorA(): Vec2;
        GetLocalAnchorB(): Vec2;
        SetLength(length: number): void;
        GetLength(): number;
        SetStiffness(stiffness: number): void;
        GetStiffness(): number;
        SetDamping(damping: number): void;
        GetDamping(): number;
        Dump(): void;
    }

    class MotorJointDef extends JointDef {
        constructor();
        Initialize(bodyA: Body, bodyB: Body): void;
        linearOffset: Vec2;
        angularOffset: number;
        maxForce: number;
        maxTorque: number;
        correctionFactor: number;
    }

    class MotorJoint extends Joint {
        SetLinearOffset(linearOffset: Vec2): void;
        GetLinearOffset(): Vec2;
        SetAngularOffset(angularOffset: number): void;
        GetAngularOffset(): number;
        SetMaxForce(force: number): void;
        GetMaxForce(): number;
        SetMaxTorque(torque: number): void;
        GetMaxTorque(): number;
        SetCorrectionFactor(factor: number): void;
        GetCorrectionFactor(): number;
        Dump(): void;
    }

    class MouseJointDef extends JointDef {
        constructor();
        localAnchorA: Vec2;
        localAnchorB: Vec2;
        localAxisA: Vec2;
        enableLimit: boolean;
        lowerTranslation: number;
        upperTranslation: number;
        enableMotor: boolean;
        maxMotorForce: number;
        motorSpeed: number;
        stiffness: number;
        damping: number;
    }

    class MouseJoint extends Joint {
        SetTarget(target: Vec2): void;
        GetTarget(): Vec2;
        SetMaxForce(force: number): void;
        GetMaxForce(): number;
        SetFrequency(hz: number): void;
        GetFrequency(): number;
        SetDampingRatio(ratio: number): void;
        GetDampingRatio(): number;
        Dump(): void;
    }

    class PrismaticJointDef extends JointDef {
        constructor();
        Initialize(bodyA: Body, bodyB: Body, anchor: Vec2, axis: Vec2): void;
        localAnchorA: Vec2;
        localAnchorB: Vec2;
        localAxisA: Vec2;
        referenceAngle: number;
        enableLimit: boolean;
        lowerTranslation: number;
        upperTranslation: number;
        enableMotor: boolean;
        maxMotorForce: number;
        motorSpeed: number;
    }

    class PrismaticJoint extends Joint {
        GetLocalAnchorA(): Vec2;
        GetLocalAnchorB(): Vec2;
        GetLocalAxisA(): Vec2;
        GetReferenceAngle(): number;
        GetJointTranslation(): number;
        GetJointSpeed(): number;
        IsLimitEnabled(): boolean;
        EnableLimit(flag: boolean): void;
        GetLowerLimit(): number;
        GetUpperLimit(): number;
        SetLimits(lower: number, upper: number): void;
        IsMotorEnabled(): boolean;
        EnableMotor(flag: boolean): void;
        SetMotorSpeed(speed: number): void;
        GetMotorSpeed(): number;
        SetMaxMotorForce(force: number): void;
        GetMaxMotorForce(): number;
        GetMotorForce(inv_dt: number): number;
        Dump(): void;
    }

    class RopeJointDef extends JointDef {
        constructor();
        localAnchorA: Vec2;
        localAnchorB: Vec2;
        maxLength: number;
    }

    class RopeJoint extends Joint {
        GetLocalAnchorA(): Vec2;
        GetLocalAnchorB(): Vec2;
        GetReactionForce(inv_dt: number): Vec2;
        GetReactionTorque(inv_dt: number): number;
        SetMaxLength(length: number): void;
        GetMaxLength(): number;
        GetLength(): number;
        Dump(): void;
    }
}
