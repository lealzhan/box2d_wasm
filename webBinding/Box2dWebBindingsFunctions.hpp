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

//SetLinearFrequencyAndDampingRatio
static void SetLinearFrequencyAndDampingRatio(b2Joint* j, float frequencyHertz, float dampingRatio) {
        float stiffness, damping;
        b2LinearStiffness(stiffness, damping, frequencyHertz, dampingRatio, j->GetBodyA(), j->GetBodyB());
        //cast j to specific type based on its type
        switch (j->GetType())
        {
        case b2JointType::e_distanceJoint:
                ((b2DistanceJoint*)j)->SetStiffness(stiffness);
                ((b2DistanceJoint*)j)->SetDamping(damping);
                break;
        case b2JointType::e_weldJoint:
                ((b2WeldJoint*)j)->SetStiffness(stiffness);
                ((b2WeldJoint*)j)->SetDamping(damping);
                break;
        case b2JointType::e_wheelJoint:
                ((b2WheelJoint*)j)->SetStiffness(stiffness);
                ((b2WheelJoint*)j)->SetDamping(damping);
                break;
        case b2JointType::e_frictionJoint:
        case b2JointType::e_ropeJoint:
        case b2JointType::e_motorJoint:
        case b2JointType::e_prismaticJoint:
        case b2JointType::e_revoluteJoint:
        case b2JointType::e_pulleyJoint:
        case b2JointType::e_mouseJoint:
        case b2JointType::e_gearJoint:
        case b2JointType::e_unknownJoint:
        default:
                break;
        }
}

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