#include <box2d.h>

//#define ENABLE_LIBTESS
#ifdef ENABLE_LIBTESS
        #include "tesselator.h"

        static void ConvexPartition(const std::vector<b2Vec2>& points, const std::vector<int>& pathVertCounts,
                                std::vector<b2Vec2>& resultPoints, std::vector<int>& resultPathVertCounts)
        {
                const int maxPolygonVerts = b2_maxPolygonVertices;
                const int vertexDimension = 2;
                
                // new Tessellation
                TESStesselator* tess = tessNewTess (nullptr);

                // add all paths as a tessellation contour
                const int pathCount = pathVertCounts.size();
                int addedContours = 0;
                int prevPathVertCount = 0;
                for (int pathID = 0; pathID < pathCount; ++pathID)
                {
                        const int pathVertCount = pathVertCounts[pathID];

                        if (pathVertCount < 3)
                                continue;

                        // Add path contour.
                        tessAddContour (tess, vertexDimension, points.data() + prevPathVertCount * sizeof(b2Vec2), sizeof(b2Vec2), pathVertCount);
                        addedContours++;
                        prevPathVertCount = pathVertCount;
                }

                // return if no contours added
                if (addedContours == 0)
                        return;

                // perform the tessellation
                tessTesselate(tess, TESS_WINDING_ODD, TESS_POLYGONS, maxPolygonVerts, vertexDimension, nullptr);

                // get the tessellation result
                const int elemCount = tessGetElementCount (tess);
                if (elemCount == 0)
                        return;

                const TESSindex* elements = tessGetElements(tess);
                const TESSreal* real = tessGetVertices(tess);
                std::vector<b2Vec2> buffer;
                buffer.resize(maxPolygonVerts);
                resultPoints.clear();
                resultPathVertCounts.clear();
                for (int elemID = 0; elemID < elemCount; ++elemID)
                {
                        const int* indices = &elements[elemID * maxPolygonVerts];
                        int bufSize = 0;
                        for (int i = 0; i < maxPolygonVerts && indices[i] != TESS_UNDEF; ++i)
                        {
                                const float& x = real[indices[i]*vertexDimension];
                                const float& y = real[indices[i]*vertexDimension + 1];

                                b2Vec2 newPoint(x, y);
                                if (bufSize > 0 && b2DistanceSquared(buffer[bufSize-1], newPoint) <= b2_linearSlop * b2_linearSlop)
                                        continue;
                                
                                buffer[bufSize] = newPoint;
                                ++bufSize;
                        }

                        if (bufSize < 3)
                                continue;
        
                        resultPoints.insert(resultPoints.end(), buffer.begin(), buffer.begin() + bufSize);
                        resultPathVertCounts.push_back(bufSize);
                }

                tessDeleteTess(tess);
        }
#endif

// //SetLinearFrequencyAndDampingRatio
// static void SetLinearFrequencyAndDampingRatio(b2Joint* j, float frequencyHertz, float dampingRatio) {
//         float stiffness, damping;
//         b2LinearStiffness(stiffness, damping, frequencyHertz, dampingRatio, j->GetBodyA(), j->GetBodyB());
//         //cast j to specific type based on its type
//         switch (j->GetType())
//         {
//         case b2JointType::e_distanceJoint:
//                 ((b2DistanceJoint*)j)->SetStiffness(stiffness);
//                 ((b2DistanceJoint*)j)->SetDamping(damping);
//                 break;
//         case b2JointType::e_weldJoint:
//                 ((b2WeldJoint*)j)->SetStiffness(stiffness);
//                 ((b2WeldJoint*)j)->SetDamping(damping);
//                 break;
//         case b2JointType::e_wheelJoint:
//                 ((b2WheelJoint*)j)->SetStiffness(stiffness);
//                 ((b2WheelJoint*)j)->SetDamping(damping);
//                 break;
//         case b2JointType::e_frictionJoint:
//         case b2JointType::e_ropeJoint:
//         case b2JointType::e_motorJoint:
//         case b2JointType::e_prismaticJoint:
//         case b2JointType::e_revoluteJoint:
//         case b2JointType::e_pulleyJoint:
//         case b2JointType::e_mouseJoint:
//         case b2JointType::e_gearJoint:
//         case b2JointType::e_unknownJoint:
//         default:
//                 break;
//         }
// }

//Get Float32 from pointer
static float GetFloat32(uint32 ptr, int id) {
        return *((float*)ptr + id);
}

static b2Vec2 TransformVector2(const b2Transform& transform, const b2Vec2& vector) {
        return b2Mul(transform, vector);
}

//Contact
static void ContactSetEnabled(uint32 ptr, bool enabled) {
        ((b2Contact*)ptr)->SetEnabled(enabled);
}

static bool ContactIsTouching(uint32 ptr) {
        return ((b2Contact*)ptr)->IsTouching();
}

static void ContactSetTangentSpeed(uint32 ptr, float speed) {
        ((b2Contact*)ptr)->SetTangentSpeed(speed);
}

static float ContactGetTangentSpeed(uint32 ptr) {
        return ((b2Contact*)ptr)->GetTangentSpeed();
}

static void ContactSetFriction(uint32 ptr, float friction) {
        ((b2Contact*)ptr)->SetFriction(friction);
}

static float ContactGetFriction(uint32 ptr) {
        return ((b2Contact*)ptr)->GetFriction();
}

static void ContactResetFriction(uint32 ptr) {
        ((b2Contact*)ptr)->ResetFriction();
}

static void ContactSetRestitution(uint32 ptr, float restitution) {
        ((b2Contact*)ptr)->SetRestitution(restitution);
}

static float ContactGetRestitution(uint32 ptr) {
        return ((b2Contact*)ptr)->GetRestitution();
}

static void ContactResetRestitution(uint32 ptr) {
        ((b2Contact*)ptr)->ResetRestitution();
}

static uint32 ContactGetFixtureA(uint32 ptr) {
        return (uint32)((b2Contact*)ptr)->GetFixtureA();
}

static uint32 ContactGetFixtureB(uint32 ptr) {
        return (uint32)((b2Contact*)ptr)->GetFixtureB();
}

static void ContactGetWorldManifold(uint32 ptr, uint32 worldManifoldPtr) {
        ((b2Contact*)ptr)->GetWorldManifold((b2WorldManifold*)worldManifoldPtr);
}

static uint32 ContactGetManifold(uint32 ptr) {
        return (uint32)((b2Contact*)ptr)->GetManifold();
}

//Manifold
static uint32 ManifoldGetType(uint32 ptr) {
        //0 = e_circles, e_faceA, e_faceB
        return ((b2Manifold*)ptr)->type;
}
static uint32 ManifoldGetPointCount(uint32 ptr) {
        return ((b2Manifold*)ptr)->pointCount;
}

static uint32 ManifoldGetManifoldPointPtr(uint32 ptr, int id) {
        return (uint32)(&(((b2Manifold*)ptr)->points[id]));
}

static float ManifoldGetLocalPointValueX(uint32 ptr) {
        return ((b2Manifold*)ptr)->localPoint.x;
}

static float ManifoldGetLocalPointValueY(uint32 ptr) {
        return ((b2Manifold*)ptr)->localPoint.y;
}

static float ManifoldGetLocalNormalValueX(uint32 ptr) {
        return ((b2Manifold*)ptr)->localNormal.x;
}

static float ManifoldGetLocalNormalValueY(uint32 ptr) {
        return ((b2Manifold*)ptr)->localNormal.y;
}

//ManifoldPoint
static float ManifoldPointGetLocalPointX(uint32 ptr) {
        return ((b2ManifoldPoint*)ptr)->localPoint.x;
}

static float ManifoldPointGetLocalPointY(uint32 ptr) {
        return ((b2ManifoldPoint*)ptr)->localPoint.y;
}

static float ManifoldPointGetNormalImpulse(uint32 ptr) {
        return ((b2ManifoldPoint*)ptr)->normalImpulse;
}

static float ManifoldPointGetTangentImpulse(uint32 ptr) {
        return ((b2ManifoldPoint*)ptr)->tangentImpulse;
}


//WorldManifold
static uint32 WorldManifoldNew() {
        return (uint32)(new b2WorldManifold());
}

static float WorldManifoldGetPointValueX(uint32 ptr, int id) {
        return ((b2WorldManifold*)ptr)->points[id].x;
}

static float WorldManifoldGetPointValueY(uint32 ptr, int id) {
        return ((b2WorldManifold*)ptr)->points[id].y;
}

static float WorldManifoldGetSeparationValue(uint32 ptr, int id) {
        return ((b2WorldManifold*)ptr)->separations[id];
}

static float WorldManifoldGetNormalValueX(uint32 ptr) {
        return ((b2WorldManifold*)ptr)->normal.x;
}

static float WorldManifoldGetNormalValueY(uint32 ptr) {
        return ((b2WorldManifold*)ptr)->normal.y;
}

static void WorldManifoldDelete(uint32 ptr) {
        if(ptr)
                delete ((b2WorldManifold*)ptr);
}

//ContactImpulse
static float ContactImpulseGetNormalImpulse(uint32 ptr, int id) {
        return ((b2ContactImpulse*)ptr)->normalImpulses[id];
}
static float ContactImpulseGetTangentImpulse(uint32 ptr, int id) {
        return ((b2ContactImpulse*)ptr)->tangentImpulses[id];
}
static int ContactImpulseGetCount(uint32 ptr) {
        return ((b2ContactImpulse*)ptr)->count;
}

//b2JointEdge
static uint32 JointEdgeGetOther(uint32 ptr) {
        return (uint32)((b2JointEdge*)ptr)->other;
}

static uint32 JointEdgeGetJoint(uint32 ptr) {
        return (uint32)((b2JointEdge*)ptr)->joint;
}

static uint32 JointEdgeGetPrev(uint32 ptr) {
        return (uint32)((b2JointEdge*)ptr)->prev;
}

static uint32 JointEdgeGetNext(uint32 ptr) {
        return (uint32)((b2JointEdge*)ptr)->next;
}
//b2FixtureDef
static uint32 FixtureDefNew() {
        return (uint32)(new b2FixtureDef());
}
static void FixtureDefDelete(uint32 ptr) {
        if(ptr)
                delete ((b2FixtureDef*)ptr);
}

static void FixtureDefSetAll(uint32 ptr, uint32 shapePtr, uint32 userData, float friction, float restitution, float density, bool isSensor, uint16 categoryBits, uint16 maskBits, int16 groupIndex) {
        ((b2FixtureDef*)ptr)->shape = (b2Shape*)shapePtr;
        ((b2FixtureDef*)ptr)->userData = (void*)userData;
        ((b2FixtureDef*)ptr)->friction = friction;
        ((b2FixtureDef*)ptr)->restitution = restitution;
        ((b2FixtureDef*)ptr)->density = density;
        ((b2FixtureDef*)ptr)->isSensor = isSensor;
        ((b2FixtureDef*)ptr)->filter.categoryBits = categoryBits;
        ((b2FixtureDef*)ptr)->filter.maskBits = maskBits;
        ((b2FixtureDef*)ptr)->filter.groupIndex = groupIndex;
}

static void FixtureDefSetShape(uint32 ptr, uint32 shapePtr) {
        ((b2FixtureDef*)ptr)->shape = (b2Shape*)shapePtr;
}
static void FixtureDefSetUserData(uint32 ptr, uint32 userData) {
        ((b2FixtureDef*)ptr)->userData = (void*)userData;
}
static void FixtureDefSetFriction(uint32 ptr, float friction) {
        ((b2FixtureDef*)ptr)->friction = friction;
}
static void FixtureDefSetRestitution(uint32 ptr, float restitution) {
        ((b2FixtureDef*)ptr)->restitution = restitution;
}
static void FixtureDefSetDensity(uint32 ptr, float density) {
        ((b2FixtureDef*)ptr)->density = density;
}
static void FixtureDefSetIsSensor(uint32 ptr, bool isSensor) {
        ((b2FixtureDef*)ptr)->isSensor = isSensor;
}
static void FixtureDefSetFilter(uint32 ptr, uint32 filterPtr) {
        ((b2FixtureDef*)ptr)->filter = *(b2Filter*)filterPtr;
}
static void FixtureDefSetFilterCategoryBits(uint32 ptr, uint16 categoryBits) {
        ((b2FixtureDef*)ptr)->filter.categoryBits = categoryBits;
}
static void FixtureDefSetFilterMaskBits(uint32 ptr, uint16 maskBits) {
        ((b2FixtureDef*)ptr)->filter.maskBits = maskBits;
}
static void FixtureDefSetFilterGroupIndex(uint32 ptr, int16 groupIndex) {
        ((b2FixtureDef*)ptr)->filter.groupIndex = groupIndex;
}
static uint32 FixtureDefGetUserData(uint32 ptr) {
        return (uint32)((b2FixtureDef*)ptr)->userData;
}
static float FixtureDefGetFriction(uint32 ptr) {
        return ((b2FixtureDef*)ptr)->friction;
}
static float FixtureDefGetRestitution(uint32 ptr) {
        return ((b2FixtureDef*)ptr)->restitution;
}
static float FixtureDefGetDensity(uint32 ptr) {
        return ((b2FixtureDef*)ptr)->density;
}
static bool FixtureDefGetIsSensor(uint32 ptr) {
        return ((b2FixtureDef*)ptr)->isSensor;
}
static uint32 FixtureDefGetFilter(uint32 ptr) {
        return (uint32)&(((b2FixtureDef*)ptr)->filter);
}
static uint16 FixtureDefGetFilterCategoryBits(uint32 ptr) {
        return ((b2FixtureDef*)ptr)->filter.categoryBits;
}
static uint16 FixtureDefGetFilterMaskBits(uint32 ptr) {
        return ((b2FixtureDef*)ptr)->filter.maskBits;
}
static int16 FixtureDefGetFilterGroupIndex(uint32 ptr) {
        return ((b2FixtureDef*)ptr)->filter.groupIndex;
}
static void FixtureDefGetShape(uint32 ptr, uint32 shapePtr) {
        ((b2FixtureDef*)ptr)->shape = (b2Shape*)shapePtr;
}

//b2Fixture
static uint32 FixtureGetType(uint32 ptr) {
        return (uint32)((b2Fixture*)ptr)->GetType();
}
static uint32 FixtureGetShape(uint32 ptr) {
        return (uint32)((b2Fixture*)ptr)->GetShape();
}
static b2Shape* FixtureGetShape1(uint32 ptr) {
        return ((b2Fixture*)ptr)->GetShape();
}
static void FixtureSetSensor(uint32 ptr, bool sensor) {
        ((b2Fixture*)ptr)->SetSensor(sensor);
}
static bool FixtureIsSensor(uint32 ptr) {
        return ((b2Fixture*)ptr)->IsSensor();
}
// static void FixtureSetFilterData(uint32 ptr, uint32 filterPtr) {
//         ((b2Fixture*)ptr)->SetFilterData(*(b2Filter*)filterPtr);
// }
// static uint32 FixtureGetFilterData(uint32 ptr) {
//         return (uint32)&(((b2Fixture*)ptr)->GetFilterData());
// }
static void FixtureSetFilterData(uint32 ptr, const b2Filter& filterPtr) {
        ((b2Fixture*)ptr)->SetFilterData(filterPtr);
}
static const b2Filter& FixtureGetFilterData(uint32 ptr) {
        return ((b2Fixture*)ptr)->GetFilterData();
}
static void FixtureRefilter(uint32 ptr) {
        ((b2Fixture*)ptr)->Refilter();
}
static uint32 FixtureGetBody(uint32 ptr) {
        return (uint32)((b2Fixture*)ptr)->GetBody();
}
static uint32 FixtureGetNext(uint32 ptr) {
        return (uint32)((b2Fixture*)ptr)->GetNext();
}
static uint32 FixtureGetUserData(uint32 ptr) {
        return (uint32)((b2Fixture*)ptr)->GetUserData();
}
static void FixtureSetUserData(uint32 ptr, uint32 data) {
        ((b2Fixture*)ptr)->SetUserData((void*)data);
}
static bool FixtureTestPoint(uint32 ptr, uint32 pPtr) {
        return ((b2Fixture*)ptr)->TestPoint(*(b2Vec2*)pPtr);
}
static bool FixtureRayCast(uint32 ptr, uint32 outputPtr, uint32 inputPtr, int32 childIndex) {
        return ((b2Fixture*)ptr)->RayCast((b2RayCastOutput*)outputPtr, *(b2RayCastInput*)inputPtr, childIndex);
}
static void FixtureGetMassData(uint32 ptr, uint32 massDataPtr) {
        ((b2Fixture*)ptr)->GetMassData((b2MassData*)massDataPtr);
}
static void FixtureSetDensity(uint32 ptr, float density) {
        ((b2Fixture*)ptr)->SetDensity(density);
}
static float FixtureGetDensity(uint32 ptr) {
        return ((b2Fixture*)ptr)->GetDensity();
}
static float FixtureGetFriction(uint32 ptr) {
        return ((b2Fixture*)ptr)->GetFriction();
}
static void FixtureSetFriction(uint32 ptr, float friction) {
        ((b2Fixture*)ptr)->SetFriction(friction);
}
static float FixtureGetRestitution(uint32 ptr) {
        return ((b2Fixture*)ptr)->GetRestitution();
}
static void FixtureSetRestitution(uint32 ptr, float restitution) {
        ((b2Fixture*)ptr)->SetRestitution(restitution);
}
static uint32 FixtureGetAABB(uint32 ptr, int32 childIndex) {
        return (uint32)&(((b2Fixture*)ptr)->GetAABB(childIndex));
}
static const b2AABB& FixtureGetAABB1(uint32 ptr, int32 childIndex) {
        return ((b2Fixture*)ptr)->GetAABB(childIndex);
}
static void FixtureDump(uint32 ptr, int32 bodyIndex) {
        ((b2Fixture*)ptr)->Dump(bodyIndex);
}

//temp CircleShape
static uint32 CircleShapeNew() {
        return (uint32)(new b2CircleShape());
}
static void CircleShapeDelete(uint32 ptr) {
        if(ptr)
                delete ((b2CircleShape*)ptr);
}
static void CircleShapeSetRadius(uint32 ptr, float radius) {
        ((b2CircleShape*)ptr)->m_radius = radius;
}
static float CircleShapeGetRadius(uint32 ptr) {
        return ((b2CircleShape*)ptr)->m_radius;
}
static void CircleShapeSetPosition(uint32 ptr, float x, float y) {
        ((b2CircleShape*)ptr)->m_p.x = x;
        ((b2CircleShape*)ptr)->m_p.y = y;
}
static const b2Vec2& CircleShapeGetPosition(uint32 ptr) {
        return ((b2CircleShape*)ptr)->m_p;
}

//temp b2Body
//CreateFixture
static uint32 BodyCreateFixture(uint32 ptr, uint32 fixtureDefPtr) {
        return (uint32)((b2Body*)ptr)->CreateFixture((b2FixtureDef*)fixtureDefPtr);
}
//DestroyFixture
static void BodyDestroyFixture(uint32 ptr, uint32 fixturePtr) {
        ((b2Body*)ptr)->DestroyFixture((b2Fixture*)fixturePtr);
}