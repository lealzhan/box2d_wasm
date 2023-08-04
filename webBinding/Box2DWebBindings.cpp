        #include <set>

        #include <box2d.h>
        #include "tesselator.h"

        #include <emscripten.h>
        #include <emscripten/bind.h>

        using namespace emscripten;

        //Get Float32 from pointer
        static float GetFloat32(uint32 ptr, int id) {
                return *((float*)ptr + id);
        }
        
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

        static uint32 ContactGetFixtureARawPtr(uint32 ptr) {
                return (uint32)((b2Contact*)ptr)->GetFixtureA();
        }

        static uint32 ContactGetFixtureBRawPtr(uint32 ptr) {
                return (uint32)((b2Contact*)ptr)->GetFixtureB();
        }

        static void ContactGetWorldManifoldRawPtr(uint32 ptr, uint32 worldManifoldPtr) {
                ((b2Contact*)ptr)->GetWorldManifold((b2WorldManifold*)worldManifoldPtr);
        }

        static uint32 ContactGetManifoldRawPtr(uint32 ptr) {
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
        
        //b2QueryCallbackWrapper
        struct b2QueryCallbackWrapper : public wrapper<b2QueryCallback> {
                EMSCRIPTEN_WRAPPER(b2QueryCallbackWrapper)
                bool ReportFixture(b2Fixture* fixture) override {
                        return call<bool>("ReportFixture", fixture);
                }
        };
        struct b2RayCastCallbackWrapper : public wrapper<b2RayCastCallback> {
                EMSCRIPTEN_WRAPPER(b2RayCastCallbackWrapper)
                float ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float fraction) override {
                        return call<float>("ReportFixture", fixture, point, normal, fraction);
                }
        };

        //b2ContactListenerWrapper
        struct b2ContactListenerWrapper : public wrapper<b2ContactListener> {
                private:
                std::set<b2Fixture*> _contactFixtures;

                public:
                void registerContactFixture(unsigned int fixturePtr) {
                        b2Fixture* fixture = (b2Fixture*)fixturePtr;
                        _contactFixtures.insert(fixture);
                }

                void unregisterContactFixture(unsigned int  fixturePtr) {
                        b2Fixture* fixture = (b2Fixture*)fixturePtr;
                        _contactFixtures.erase(fixture);
                }

                bool isIndexOf(unsigned int  fixturePtr) {
                        b2Fixture* fixture = (b2Fixture*)fixturePtr;
                        return _contactFixtures.find(fixture) != _contactFixtures.end();
                }

                EMSCRIPTEN_WRAPPER(b2ContactListenerWrapper)
                void BeginContact(uint32 contact) override {
                        if(isIndexOf((unsigned int)((b2Contact*)contact)->GetFixtureA()) || isIndexOf((unsigned int)((b2Contact*)contact)->GetFixtureB())) {
                               return call<void>("BeginContact", contact);
                        }
                }
                void EndContact(uint32 contact) override {
                        if(isIndexOf((unsigned int)((b2Contact*)contact)->GetFixtureA()) || isIndexOf((unsigned int)((b2Contact*)contact)->GetFixtureB())) {
                               return call<void>("EndContact", contact);
                        }
                }
                void PreSolve(uint32 contact, const b2Manifold* oldManifold) override {
                        if(isIndexOf((unsigned int)((b2Contact*)contact)->GetFixtureA()) || isIndexOf((unsigned int)((b2Contact*)contact)->GetFixtureB())) {
                               return call<void>("PreSolve", contact, oldManifold);
                        }
                }
                void PostSolve(uint32 contact, const b2ContactImpulse* impulse) override {
                        if(isIndexOf((unsigned int)((b2Contact*)contact)->GetFixtureA()) || isIndexOf((unsigned int)((b2Contact*)contact)->GetFixtureB())) {
                               return call<void>("PostSolve", contact, impulse);
                        }
                }
                
        };

        //b2DrawWrapper
        struct b2DrawWrapper : public wrapper<b2Draw> {
                EMSCRIPTEN_WRAPPER(b2DrawWrapper)
                void DrawPolygon(uint32 vertices, int32 vertexCount, const b2Color& color) override {
                        return call<void>("DrawPolygon", vertices, vertexCount, color);
                }
                void DrawSolidPolygon(uint32 vertices, int32 vertexCount, const b2Color& color) override {
                        return call<void>("DrawSolidPolygon", vertices, vertexCount, color);
                }
                void DrawCircle(const b2Vec2& center, float radius, const b2Color& color) override {
                        return call<void>("DrawCircle", center, radius, color);
                }
                void DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color) override {
                        return call<void>("DrawSolidCircle", center, radius, axis, color);
                }
                void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) override {
                        return call<void>("DrawSegment", p1, p2, color);
                }
                void DrawTransform(const b2Transform& xf) override {
                        return call<void>("DrawTransform", xf);
                }
                void DrawPoint(const b2Vec2& p, float size, const b2Color& color) override {
                        return call<void>("DrawPoint", p, size, color);
                }
        };

        void SetLinearFrequencyAndDampingRatio(b2Joint* j, float frequencyHertz, float dampingRatio){
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


        //----------------------------------------------------------------------------------------------------------------------
        EMSCRIPTEN_BINDINGS(b2) {

        constant("VERSION_MAJOR", b2_version.major);
        constant("VERSION_MINOR", b2_version.minor);
        constant("VERSION_REVISION", b2_version.revision);

        function("GetFloat32", &GetFloat32);
        function("ContactSetEnabled", &ContactSetEnabled);
        function("ContactIsTouching", &ContactIsTouching);
        function("ContactSetTangentSpeed", &ContactSetTangentSpeed);
        function("ContactGetTangentSpeed", &ContactGetTangentSpeed);
        function("ContactSetFriction", &ContactSetFriction);
        function("ContactGetFriction", &ContactGetFriction);
        function("ContactResetFriction", &ContactResetFriction);
        function("ContactSetRestitution", &ContactSetRestitution);
        function("ContactGetRestitution", &ContactGetRestitution);
        function("ContactResetRestitution", &ContactResetRestitution);
        function("ContactGetFixtureARawPtr", &ContactGetFixtureARawPtr);
        function("ContactGetFixtureBRawPtr", &ContactGetFixtureBRawPtr);
        function("ContactGetWorldManifoldRawPtr", &ContactGetWorldManifoldRawPtr);
        function("ContactGetManifoldRawPtr", &ContactGetManifoldRawPtr);

        function("ManifoldGetType", &ManifoldGetType);
        function("ManifoldGetPointCount", &ManifoldGetPointCount);
        function("ManifoldGetManifoldPointPtr", &ManifoldGetManifoldPointPtr);
        function("ManifoldGetLocalPointValueX", &ManifoldGetLocalPointValueX);
        function("ManifoldGetLocalPointValueY", &ManifoldGetLocalPointValueY);
        function("ManifoldGetLocalNormalValueX", &ManifoldGetLocalNormalValueX);
        function("ManifoldGetLocalNormalValueY", &ManifoldGetLocalNormalValueY);

        function("ManifoldPointGetLocalPointX", &ManifoldPointGetLocalPointX);
        function("ManifoldPointGetLocalPointY", &ManifoldPointGetLocalPointY);
        function("ManifoldPointGetNormalImpulse", &ManifoldPointGetNormalImpulse);
        function("ManifoldPointGetTangentImpulse", &ManifoldPointGetTangentImpulse);
        
        function("WorldManifoldGetPointValueX", &WorldManifoldGetPointValueX);
        function("WorldManifoldGetPointValueY", &WorldManifoldGetPointValueY);
        function("WorldManifoldGetSeparationValue", &WorldManifoldGetSeparationValue);
        function("WorldManifoldGetNormalValueX", &WorldManifoldGetNormalValueX);
        function("WorldManifoldGetNormalValueY", &WorldManifoldGetNormalValueY);
        function("WorldManifoldDelete", &WorldManifoldDelete);


        function("ConvexPartition", &ConvexPartition, allow_raw_pointers());        

        enum_<b2Shape::Type>("ShapeType")
                .value("e_circle", b2Shape::e_circle)
                .value("e_edge", b2Shape::e_edge)
                .value("e_polygon", b2Shape::e_polygon)
                .value("e_chain", b2Shape::e_chain)
                .value("e_typeCount", b2Shape::e_typeCount);
        
        // value object        
        register_vector<int32>("Int32Vector");
        value_object<b2Vec2>("Vec2")
                .field("x", &b2Vec2::x)
                .field("y", &b2Vec2::y);
        register_vector<b2Vec2>("Vec2Vector");
        value_object<b2Vec3>("Vec3")
                .field("x", &b2Vec3::x)
                .field("y", &b2Vec3::y)
                .field("z", &b2Vec3::z);
        value_object<b2Mat22>("Mat22")
                .field("ex", &b2Mat22::ex)
                .field("ey", &b2Mat22::ey);
        value_object<b2Mat33>("Mat33")
                .field("ex", &b2Mat33::ex)
                .field("ey", &b2Mat33::ey)
                .field("ez", &b2Mat33::ez); 
        value_object<b2Rot>("Rot")
                .field("s", &b2Rot::s)
                .field("c", &b2Rot::c);
        value_object<b2Transform>("Transform")
                .field("p", &b2Transform::p)
                .field("q", &b2Transform::q);
        value_object<b2Sweep>("Sweep")
                .field("localCenter", &b2Sweep::localCenter)
                .field("c0", &b2Sweep::c0)
                .field("c", &b2Sweep::c)
                .field("a0", &b2Sweep::a0)
                .field("a", &b2Sweep::a)
                .field("alpha0", &b2Sweep::alpha0);
        value_object<b2Color>("Color")
                .field("r", &b2Color::r)
                .field("g", &b2Color::g)
                .field("b", &b2Color::b);
        value_object<b2ContactFeature>("ContactFeature")
                .field("indexA", &b2ContactFeature::indexA)
                .field("indexB", &b2ContactFeature::indexB)
                .field("typeA", &b2ContactFeature::typeA)
                .field("typeB", &b2ContactFeature::typeB);
        value_object<b2ContactID>("ContactID")
                .field("cf", &b2ContactID::cf)
                .field("key", &b2ContactID::key);
        value_object<b2ManifoldPoint>("ManifoldPoint")
                .field("localPoint", &b2ManifoldPoint::localPoint)
                .field("normalImpulse", &b2ManifoldPoint::normalImpulse)
                .field("tangentImpulse", &b2ManifoldPoint::tangentImpulse)
                .field("id", &b2ManifoldPoint::id);
        // value_object<b2Manifold>("Manifold")
        //         .field("points", &b2Manifold::points)
        //         .field("localNormal", &b2Manifold::localNormal)
        //         .field("localPoint", &b2Manifold::localPoint)
        //         .field("type", &b2Manifold::type)
        //         .field("pointCount", &b2Manifold::pointCount);
        // value_object<b2WorldManifold>("WorldManifold")
        //         .field("normal", &b2WorldManifold::normal)
        //         .field("points", &b2WorldManifold::points)
        //         .field("separations", &b2WorldManifold::separations);
        value_object<b2ClipVertex>("ClipVertex")
                .field("v", &b2ClipVertex::v)
                .field("id", &b2ClipVertex::id);
        value_object<b2RayCastInput>("RayCastInput")
                .field("p1", &b2RayCastInput::p1)
                .field("p2", &b2RayCastInput::p2)
                .field("maxFraction", &b2RayCastInput::maxFraction);
        value_object<b2RayCastOutput>("RayCastOutput")
                .field("normal", &b2RayCastOutput::normal)
                .field("fraction", &b2RayCastOutput::fraction);
        // value_object<b2AABB>("AABB")
        //         .field("lowerBound", &b2AABB::lowerBound)
        //         .field("upperBound", &b2AABB::upperBound);
        value_object<b2MassData>("MassData")
                .field("mass", &b2MassData::mass)
                .field("center", &b2MassData::center)
                .field("I", &b2MassData::I);
        
        value_object<b2Filter>("Filter")
                .field("categoryBits", &b2Filter::categoryBits)
                .field("maskBits", &b2Filter::maskBits)
                .field("groupIndex", &b2Filter::groupIndex);
        // value_object<b2ContactImpulse>("ContactImpulse")
        //         .field("normalImpulses", &b2ContactImpulse::normalImpulses)
        //         .field("tangentImpulses", &b2ContactImpulse::tangentImpulses)
        //         .field("count", &b2ContactImpulse::count);

        //b2_maxPolygonVertices

        // //binding class b2Vec2
        // class_<b2Vec2>("Vec2")
        //         .constructor<>()
        //         .constructor<float, float>()
        //         // .property("x", &b2Vec2::GetX, &b2Vec2::SetX)
        //         // .property("y", &b2Vec2::GetY, &b2Vec2::SetY)
        //         .function("SetX", &b2Vec2::SetX)
        //         .function("GetX", &b2Vec2::GetX)
        //         .function("SetY", &b2Vec2::SetY)
        //         .function("GetY", &b2Vec2::GetY)
        //         .function("SetZero", &b2Vec2::SetZero)
        //         .function("Set", &b2Vec2::Set)
        //         .function("Length", &b2Vec2::Length)
        //         .function("LengthSquared", &b2Vec2::LengthSquared)
        //         .function("Normalize", &b2Vec2::Normalize)
        //         .function("IsValid", &b2Vec2::IsValid);



        function("SetLinearFrequencyAndDampingRatio", &SetLinearFrequencyAndDampingRatio, allow_raw_pointers());

        //binding class b2QueryCallback
        class_<b2QueryCallback>("QueryCallback")
                .function("ReportFixture", &b2QueryCallback::ReportFixture, allow_raw_pointers())
                .allow_subclass<b2QueryCallbackWrapper>("QueryCallbackWrapper", constructor<>());

        // //binding class b2RayCastCallback
        // class_<b2RayCastCallback>("RayCastCallback")
        //         .function("ReportFixture", &b2RayCastCallback::ReportFixture, allow_raw_pointers());

        // override b2RayCastCallback in js and assign to
        // PxControllerDesc.reportCallback
        // https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html#deriving-from-c-classes-in-javascript
        class_<b2RayCastCallback>("RayCastCallback")
                .function("ReportFixture", &b2RayCastCallback::ReportFixture, pure_virtual(), allow_raw_pointers())
                .allow_subclass<b2RayCastCallbackWrapper>("RayCastCallbackWrapper", constructor<>());

        //binding class b2ContactListener
        class_<b2ContactListener>("ContactListener")
                .function("BeginContact", &b2ContactListener::BeginContact, allow_raw_pointers())
                .function("EndContact", &b2ContactListener::EndContact, allow_raw_pointers())
                .function("PreSolve", &b2ContactListener::PreSolve, allow_raw_pointers())
                .function("PostSolve", &b2ContactListener::PostSolve, allow_raw_pointers())
                .function("registerContactFixture", &b2ContactListenerWrapper::registerContactFixture)
                .function("unregisterContactFixture", &b2ContactListenerWrapper::unregisterContactFixture)
                .function("isIndexOf", &b2ContactListenerWrapper::isIndexOf)
                .allow_subclass<b2ContactListenerWrapper>("ContactListenerWrapper", constructor<>());

        //binding class b2Draw
        class_<b2Draw>("Draw")
                .function("SetFlags", &b2Draw::SetFlags)
                .function("GetFlags", &b2Draw::GetFlags)
                .function("AppendFlags", &b2Draw::AppendFlags)
                .function("ClearFlags", &b2Draw::ClearFlags)
                .function("DrawPolygon", &b2Draw::DrawPolygon, allow_raw_pointers())
                .function("DrawSolidPolygon", &b2Draw::DrawSolidPolygon, allow_raw_pointers())
                .function("DrawCircle", &b2Draw::DrawCircle, allow_raw_pointers())
                .function("DrawSolidCircle", &b2Draw::DrawSolidCircle, allow_raw_pointers())
                .function("DrawSegment", &b2Draw::DrawSegment, allow_raw_pointers())
                .function("DrawTransform", &b2Draw::DrawTransform, allow_raw_pointers())
                .function("DrawPoint", &b2Draw::DrawPoint, allow_raw_pointers())
                .allow_subclass<b2DrawWrapper>("DrawWrapper", constructor<>());

        //b2WorldManifold
        class_<b2WorldManifold>("WorldManifold")
                .constructor<>()
                .function("Initialize", &b2WorldManifold::Initialize, allow_raw_pointers())
                .property("normal", &b2WorldManifold::normal)
                // .function("GetPoint0", optional_override([](b2WorldManifold* m) {
                //         return m->points[0];
                //         }), allow_raw_pointers())
                // .function("GetPoint1", optional_override([](b2WorldManifold* m) {
                //         return m->points[1];
                //         }), allow_raw_pointers())
                // .function("GetSeperation0", optional_override([](b2WorldManifold* m) {
                //         return m->separations[0];
                //         }), allow_raw_pointers())
                // .function("GetSeperation1", optional_override([](b2WorldManifold* m) {
                //         return m->separations[1];
                //         }), allow_raw_pointers())
                .function("GetPoint", optional_override([](b2WorldManifold* m, int32 i) {
                        return m->points[i];
                        }), allow_raw_pointers())
                .function("GetSeparation", optional_override([](b2WorldManifold* m, int32 i) {
                        return (m->separations[i]);
                        }), allow_raw_pointers())
                ;
                //.property("points", &b2WorldManifold::points)
                //.property("separations", &b2WorldManifold::separations);
        
        // //b2Filter
        // class_<b2Filter>("Filter")
        //         .constructor<>()
        //         .property("categoryBits", &b2Filter::categoryBits)
        //         .property("maskBits", &b2Filter::maskBits)
        //         .property("groupIndex", &b2Filter::groupIndex);

        //b2Manifold
        class_<b2Manifold>("Manifold")
                .constructor<>()
                // .property("points", &b2Manifold::points)
                .property("localNormal", &b2Manifold::localNormal)
                .property("localPoint", &b2Manifold::localPoint)
                .property("type", &b2Manifold::type)
                .property("pointCount", &b2Manifold::pointCount)
                .function("GetPoint", optional_override([](b2Manifold* m, int32 i) {
                        if(i >= 0 && i < m->pointCount) {
                                return &(m->points[i]);
                        } else {
                                return (b2ManifoldPoint*)nullptr;
                        }
                        }), allow_raw_pointers())
                ;
        //b2ContactImpulse
        class_<b2ContactImpulse>("ContactImpulse")
                .constructor<>()
                // .property("normalImpulses", &b2ContactImpulse::normalImpulses)
                // .property("tangentImpulses", &b2ContactImpulse::tangentImpulses)
                .property("count", &b2ContactImpulse::count);

        //binding class b2Contact
        class_<b2Contact>("Contact")
                .function("GetManifold", &b2Contact::GetManifold, allow_raw_pointers())
                .function("GetWorldManifold", &b2Contact::GetWorldManifold, allow_raw_pointers())
                .function("IsTouching", &b2Contact::IsTouching)
                .function("SetEnabled", &b2Contact::SetEnabled)
                .function("IsEnabled", &b2Contact::IsEnabled)
                .function("GetNext", &b2Contact::GetNext, allow_raw_pointers())
                .function("GetFixtureA", &b2Contact::GetFixtureA, allow_raw_pointers())
                .function("GetChildIndexA", &b2Contact::GetChildIndexA)
                .function("GetFixtureB", &b2Contact::GetFixtureB, allow_raw_pointers())
                .function("GetChildIndexB", &b2Contact::GetChildIndexB)
                .function("SetFriction", &b2Contact::SetFriction)
                .function("GetFriction", &b2Contact::GetFriction)
                .function("ResetFriction", &b2Contact::ResetFriction)
                .function("SetRestitution", &b2Contact::SetRestitution)
                .function("GetRestitution", &b2Contact::GetRestitution)
                .function("ResetRestitution", &b2Contact::ResetRestitution)
                .function("SetTangentSpeed", &b2Contact::SetTangentSpeed)
                .function("GetTangentSpeed", &b2Contact::GetTangentSpeed)
                .function("Evaluate", &b2Contact::Evaluate, allow_raw_pointers())
                ;

        //b2AABB
        class_<b2AABB>("AABB")
                .constructor<>()
                .function("IsValid", &b2AABB::IsValid)
                .function("GetCenter", &b2AABB::GetCenter)
                .function("GetExtents", &b2AABB::GetExtents)
                .function("GetPerimeter", &b2AABB::GetPerimeter)
                // .function("Combine", &b2AABB::Combine)
                // .function("CombineTwo", &b2AABB::Combine)
                .function("Contains", &b2AABB::Contains)
                .function("RayCast", &b2AABB::RayCast, allow_raw_pointers())
                .property("lowerBound", &b2AABB::lowerBound)
                .property("upperBound", &b2AABB::upperBound);

        //binding class b2World
        class_<b2World>("World")
                .constructor<b2Vec2>()
                .function("SetDestructionListener", &b2World::SetDestructionListener, allow_raw_pointers())
                .function("SetContactFilter", &b2World::SetContactFilter, allow_raw_pointers())
                .function("SetContactListener", &b2World::SetContactListener, allow_raw_pointers())
                .function("SetDebugDraw", &b2World::SetDebugDraw, allow_raw_pointers())
                .function("DebugDraw", &b2World::DebugDraw, allow_raw_pointers())
                .function("CreateBody", &b2World::CreateBody, allow_raw_pointers())
                .function("DestroyBody", &b2World::DestroyBody, allow_raw_pointers())
                .function("CreateJoint", &b2World::CreateJoint, allow_raw_pointers())
                .function("DestroyJoint", &b2World::DestroyJoint, allow_raw_pointers())
                .function("Step", &b2World::Step)
                .function("ClearForces", &b2World::ClearForces)
                // .function("DrawDebugData", &b2World::DrawDebugData)
                .function("QueryAABB", &b2World::QueryAABB, allow_raw_pointers())
                .function("RayCast", &b2World::RayCast, allow_raw_pointers())
                //.function("GetBodyList", &b2World::GetBodyList)
                .function("GetBodyList", optional_override([](b2World* f) {
                                if(f->GetBodyCount()) {
                                        return &(f->GetBodyList()[0]);
                                }else{
                                        return (b2Body*)nullptr;
                                }
                        }), allow_raw_pointers())
                // .function("GetJointList", &b2World::GetJointList)
                // .function("GetContactList", &b2World::GetContactList)
                .function("GetContactList", optional_override([](b2World* f) {
                                if(f->GetBodyCount()) {
                                        return &(f->GetContactList()[0]);
                                }else{
                                        return (b2Contact*)nullptr;
                                }
                        }), allow_raw_pointers())
                .function("SetAllowSleeping", &b2World::SetAllowSleeping)
                .function("GetAllowSleeping", &b2World::GetAllowSleeping)
                .function("SetWarmStarting", &b2World::SetWarmStarting)
                .function("GetWarmStarting", &b2World::GetWarmStarting)
                .function("SetContinuousPhysics", &b2World::SetContinuousPhysics)
                .function("GetContinuousPhysics", &b2World::GetContinuousPhysics)
                .function("SetSubStepping", &b2World::SetSubStepping)
                .function("GetSubStepping", &b2World::GetSubStepping)
                .function("GetProxyCount", &b2World::GetProxyCount)
                .function("GetBodyCount", &b2World::GetBodyCount)
                .function("GetJointCount", &b2World::GetJointCount)
                .function("GetContactCount", &b2World::GetContactCount)
                .function("GetTreeHeight", &b2World::GetTreeHeight)
                .function("GetTreeBalance", &b2World::GetTreeBalance)
                .function("GetTreeQuality", &b2World::GetTreeQuality)
                .function("SetGravity", &b2World::SetGravity)
                .function("GetGravity", &b2World::GetGravity)
                .function("IsLocked", &b2World::IsLocked)
                .function("SetAutoClearForces", &b2World::SetAutoClearForces)
                .function("GetAutoClearForces", &b2World::GetAutoClearForces)
                .function("ShiftOrigin", &b2World::ShiftOrigin)
                .function("GetContactManager", &b2World::GetContactManager)
                .function("GetProfile", &b2World::GetProfile)
                .function("Dump", &b2World::Dump);


        //binding class b2Shape
        class_<b2Shape>("Shape")
                .property("m_type", &b2Shape::m_type)
                .property("m_radius", &b2Shape::m_radius)
                .function("GetType", &b2Shape::GetType)
                .function("GetChildCount", &b2Shape::GetChildCount)
                .function("TestPoint", &b2Shape::TestPoint)
                .function("RayCast", &b2Shape::RayCast, allow_raw_pointers())
                .function("ComputeAABB", &b2Shape::ComputeAABB, allow_raw_pointers())
                .function("ComputeMass", &b2Shape::ComputeMass, allow_raw_pointers())
                .function("SetRadius", optional_override([](b2Shape* f, float r) {
                        f->m_radius = r;
                        }), allow_raw_pointers())
                .function("GetRadius", optional_override([](b2Shape* f) {
                        return f->m_radius;
                        }), allow_raw_pointers())
                ;

        //binding class b2CircleShape
        class_<b2CircleShape, base<b2Shape>>("CircleShape")
                .constructor<>()
                .property("m_p", &b2CircleShape::m_p)
                .function("Clone", &b2CircleShape::Clone, allow_raw_pointers())
                .function("GetChildCount", &b2CircleShape::GetChildCount)
                .function("TestPoint", &b2CircleShape::TestPoint)
                .function("RayCast", &b2CircleShape::RayCast, allow_raw_pointers())
                .function("ComputeAABB", &b2CircleShape::ComputeAABB, allow_raw_pointers())
                .function("ComputeMass", &b2CircleShape::ComputeMass, allow_raw_pointers());

        //binding class b2EdgeShape
        class_<b2EdgeShape, base<b2Shape>>("EdgeShape")
                .function("Clone", &b2EdgeShape::Clone, allow_raw_pointers())
                .function("GetChildCount", &b2EdgeShape::GetChildCount)
                .function("TestPoint", &b2EdgeShape::TestPoint)
                .function("RayCast", &b2EdgeShape::RayCast, allow_raw_pointers())
                .function("ComputeAABB", &b2EdgeShape::ComputeAABB, allow_raw_pointers())
                .function("ComputeMass", &b2EdgeShape::ComputeMass, allow_raw_pointers());

        //binding class b2PolygonShape
        class_<b2PolygonShape, base<b2Shape>>("PolygonShape")
                .constructor<>()
                .function("Clone", &b2PolygonShape::Clone, allow_raw_pointers())
                .function("GetChildCount", &b2PolygonShape::GetChildCount)
                //.function("Set", select_overload<bool(const b2Vec2*, int32)>(&b2PolygonShape::Set), allow_raw_pointers())
                .function("Set", optional_override([](b2PolygonShape* p, std::vector<b2Vec2> &vertices, int32 count) {
                        return p->Set(vertices.data(), count);
                        }), allow_raw_pointers())
                // .function("SetWithConvexHull", select_overload<void(const b2Hull&)>(&b2PolygonShape::Set))
                .function("TestPoint", &b2PolygonShape::TestPoint)
                .function("RayCast", &b2PolygonShape::RayCast, allow_raw_pointers())
                .function("ComputeAABB", &b2PolygonShape::ComputeAABB, allow_raw_pointers())
                .function("ComputeMass", &b2PolygonShape::ComputeMass, allow_raw_pointers())
                .function("Validate", &b2PolygonShape::Validate)
                .function("SetAsBox", select_overload<void(float, float)>(&b2PolygonShape::SetAsBox))
                .function("SetAsBoxWithCenterAndAngle", select_overload<void(float, float, const b2Vec2&, float)>(&b2PolygonShape::SetAsBox));


        //binding class b2FixtureDef
        class_<b2FixtureDef>("FixtureDef")
                .constructor<>()
                //.property("shape", &b2FixtureDef::getShape, &b2FixtureDef::setShape, allow_raw_pointers())
                .function("SetShape", optional_override([](b2FixtureDef* f, const b2Shape* s) {
                        f->shape = s;}), allow_raw_pointers())
                .function("GetShape", optional_override([](b2FixtureDef* f) {
                        return f->shape;}), allow_raw_pointers())
                // .property("userData", &b2FixtureDef::userData)
                .property("friction", &b2FixtureDef::friction)
                .property("restitution", &b2FixtureDef::restitution)
                .property("density", &b2FixtureDef::density)
                .property("isSensor", &b2FixtureDef::isSensor)
                .property("filter", &b2FixtureDef::filter);

        //binding class b2Fixture
        class_<b2Fixture>("Fixture")
                .function("GetType", &b2Fixture::GetType)
                .function("GetShape", &b2Fixture::GetShape, allow_raw_pointers())
                .function("SetSensor", &b2Fixture::SetSensor)
                .function("IsSensor", &b2Fixture::IsSensor)
                .function("SetFilterData", &b2Fixture::SetFilterData)
                .function("GetFilterData", &b2Fixture::GetFilterData)
                .function("Refilter", &b2Fixture::Refilter)
                .function("GetBody", &b2Fixture::GetBody, allow_raw_pointers())
                .function("GetNext", &b2Fixture::GetNext, allow_raw_pointers())
                // .function("GetUserData", &b2Fixture::GetUserData)
                // .function("SetUserData", &b2Fixture::SetUserData)
                .function("TestPoint", &b2Fixture::TestPoint)
                .function("RayCast", &b2Fixture::RayCast, allow_raw_pointers())
                .function("GetMassData", &b2Fixture::GetMassData, allow_raw_pointers())
                .function("SetDensity", &b2Fixture::SetDensity)
                .function("GetDensity", &b2Fixture::GetDensity)
                .function("GetFriction", &b2Fixture::GetFriction)
                .function("SetFriction", &b2Fixture::SetFriction)
                .function("GetRestitution", &b2Fixture::GetRestitution)
                .function("SetRestitution", &b2Fixture::SetRestitution)
                .function("GetAABB", &b2Fixture::GetAABB)
                .function("Dump", &b2Fixture::Dump);

        //bind b2BodyType
        enum_<b2BodyType>("BodyType")
                .value("b2_staticBody", b2_staticBody)
                .value("b2_kinematicBody", b2_kinematicBody)
                .value("b2_dynamicBody", b2_dynamicBody);

        //binding class b2BodyDef
        class_<b2BodyDef>("BodyDef")
                .constructor<>()
                .property("type", &b2BodyDef::type)
                .property("position", &b2BodyDef::position)
                .property("angle", &b2BodyDef::angle)
                .property("linearVelocity", &b2BodyDef::linearVelocity)
                .property("angularVelocity", &b2BodyDef::angularVelocity)
                .property("linearDamping", &b2BodyDef::linearDamping)
                .property("angularDamping", &b2BodyDef::angularDamping)
                .property("allowSleep", &b2BodyDef::allowSleep)
                .property("awake", &b2BodyDef::awake)
                .property("fixedRotation", &b2BodyDef::fixedRotation)
                .property("bullet", &b2BodyDef::bullet)
                //.property("userData", &b2BodyDef::userData)
                .property("gravityScale", &b2BodyDef::gravityScale);

        //binding class b2Body
        class_<b2Body>("Body")
                .function("CreateFixture", select_overload<b2Fixture*(const b2FixtureDef*)>(&b2Body::CreateFixture), allow_raw_pointers())
                // .function("CreateFixture", optional_override([](b2Body* body, const b2FixtureDef& f) {
                //         return body->CreateFixture(&f);
                //         }), allow_raw_pointers())
                .function("CreateFixtureWithShape", select_overload<b2Fixture*(const b2Shape*, float)>(&b2Body::CreateFixture), allow_raw_pointers())
                .function("DestroyFixture", &b2Body::DestroyFixture, allow_raw_pointers())
                .function("SetTransform", &b2Body::SetTransform)
                .function("GetTransform", &b2Body::GetTransform)
                .function("GetPosition", &b2Body::GetPosition)
                .function("GetAngle", &b2Body::GetAngle)
                .function("GetWorldCenter", &b2Body::GetWorldCenter)
                .function("GetLocalCenter", &b2Body::GetLocalCenter)
                .function("SetLinearVelocity", &b2Body::SetLinearVelocity)
                .function("GetLinearVelocity", &b2Body::GetLinearVelocity)
                .function("SetAngularVelocity", &b2Body::SetAngularVelocity)
                .function("GetAngularVelocity", &b2Body::GetAngularVelocity)
                .function("ApplyForce", &b2Body::ApplyForce)
                .function("ApplyForceToCenter", &b2Body::ApplyForceToCenter)
                .function("ApplyTorque", &b2Body::ApplyTorque)
                .function("ApplyLinearImpulse", &b2Body::ApplyLinearImpulse)
                .function("ApplyLinearImpulseToCenter", &b2Body::ApplyLinearImpulseToCenter)
                .function("ApplyAngularImpulse", &b2Body::ApplyAngularImpulse)
                .function("GetMass", &b2Body::GetMass)
                .function("GetInertia", &b2Body::GetInertia)
                .function("GetMassData", &b2Body::GetMassData, allow_raw_pointers())
                // .function("SetMassData", &b2Body::SetMassData, allow_raw_pointers())
                .function("SetMassData", optional_override([](b2Body* body, const b2MassData& f) {
                        return body->SetMassData(&f);
                        }), allow_raw_pointers())
                .function("ResetMassData", &b2Body::ResetMassData)
                .function("GetWorldPoint", &b2Body::GetWorldPoint)
                .function("GetWorldVector", &b2Body::GetWorldVector)
                .function("GetLocalPoint", &b2Body::GetLocalPoint)
                .function("GetLocalVector", &b2Body::GetLocalVector)
                .function("GetLinearVelocityFromWorldPoint", &b2Body::GetLinearVelocityFromWorldPoint)
                .function("GetLinearVelocityFromLocalPoint", &b2Body::GetLinearVelocityFromLocalPoint)
                .function("GetLinearDamping", &b2Body::GetLinearDamping)
                .function("SetLinearDamping", &b2Body::SetLinearDamping)
                .function("GetAngularDamping", &b2Body::GetAngularDamping)
                .function("SetAngularDamping", &b2Body::SetAngularDamping)
                .function("GetGravityScale", &b2Body::GetGravityScale)
                .function("SetGravityScale", &b2Body::SetGravityScale)
                .function("SetType", &b2Body::SetType)
                .function("GetType", &b2Body::GetType)
                .function("SetBullet", &b2Body::SetBullet)
                .function("IsBullet", &b2Body::IsBullet)
                .function("SetSleepingAllowed", &b2Body::SetSleepingAllowed)
                .function("IsSleepingAllowed", &b2Body::IsSleepingAllowed)
                .function("SetAwake", &b2Body::SetAwake)
                .function("IsAwake", &b2Body::IsAwake)
                .function("SetEnabled", &b2Body::SetEnabled)
                .function("IsEnabled", &b2Body::IsEnabled)
                .function("SetFixedRotation", &b2Body::SetFixedRotation)
                .function("IsFixedRotation", &b2Body::IsFixedRotation)
                .function("GetFixtureList", &b2Body::GetFixtureList, allow_raw_pointers())
                // .function("GetJointList", &b2Body::GetJointList)
                // .function("GetContactList", &b2Body::GetContactList)
                .function("GetNext", &b2Body::GetNext, allow_raw_pointers())
                // .function("GetUserData", &b2Body::GetUserData)
                // .function("SetUserData", &b2Body::SetUserData)
                .function("GetWorld", &b2Body::GetWorld, allow_raw_pointers())
                //.function("GetWorld", select_overload<const b2World*()>(&b2Body::GetWorld), allow_raw_pointers())
                .function("Dump", &b2Body::Dump);




        //binding class b2JointDef
        class_<b2JointDef>("JointDef")
                .constructor<>()
                .property("type", &b2JointDef::type)
                // .property("userData", &b2JointDef::userData)
                //https://stackoverflow.com/questions/17056628/emscripten-error-when-binding-class-with-2d-double-array
                // .property("bodyA", &b2JointDef::bodyA, allow_raw_pointers())
                // .property("bodyB", &b2JointDef::bodyB, allow_raw_pointers())
                .function("SetBodyA", optional_override([](b2JointDef* j, b2Body* b) {
                        j->bodyA = b;}), allow_raw_pointers())
                .function("GetBodyA", optional_override([](b2JointDef* j) {
                        return j->bodyA;}), allow_raw_pointers())
                .function("SetBodyB", optional_override([](b2JointDef* j, b2Body* b) {
                        j->bodyB = b;}), allow_raw_pointers())
                .function("GetBodyB", optional_override([](b2JointDef* j) {
                        return j->bodyB;}), allow_raw_pointers())
                .property("collideConnected", &b2JointDef::collideConnected);

        //binding class b2DistanceJointDef
        class_<b2DistanceJointDef, base<b2JointDef>>("DistanceJointDef")
                .constructor<>()
                .property("localAnchorA", &b2DistanceJointDef::localAnchorA)
                .property("localAnchorB", &b2DistanceJointDef::localAnchorB)
                .property("length", &b2DistanceJointDef::length)
                // .property("minLength", &b2DistanceJointDef::minLength)
                // .property("maxLength", &b2DistanceJointDef::maxLength)
                .property("stiffness", &b2DistanceJointDef::stiffness)
                .property("damping", &b2DistanceJointDef::damping);

        //binding class b2WeldJointDef
        class_<b2WeldJointDef, base<b2JointDef>>("WeldJointDef")
                .constructor<>()
                .property("localAnchorA", &b2WeldJointDef::localAnchorA)
                .property("localAnchorB", &b2WeldJointDef::localAnchorB)
                .property("referenceAngle", &b2WeldJointDef::referenceAngle)
                .property("stiffness", &b2WeldJointDef::stiffness)
                .property("damping", &b2WeldJointDef::damping);

        //binding class b2RevoluteJointDef
        class_<b2RevoluteJointDef, base<b2JointDef>>("RevoluteJointDef")
                .constructor<>()
                .property("localAnchorA", &b2RevoluteJointDef::localAnchorA)
                .property("localAnchorB", &b2RevoluteJointDef::localAnchorB)
                .property("referenceAngle", &b2RevoluteJointDef::referenceAngle)
                .property("enableLimit", &b2RevoluteJointDef::enableLimit)
                .property("lowerAngle", &b2RevoluteJointDef::lowerAngle)
                .property("upperAngle", &b2RevoluteJointDef::upperAngle)
                .property("enableMotor", &b2RevoluteJointDef::enableMotor)
                .property("motorSpeed", &b2RevoluteJointDef::motorSpeed)
                .property("maxMotorTorque", &b2RevoluteJointDef::maxMotorTorque);
        
        //binding class b2PrismaticJointDef
        class_<b2PrismaticJointDef, base<b2JointDef>>("PrismaticJointDef")
                .constructor<>()
                .property("localAnchorA", &b2PrismaticJointDef::localAnchorA)
                .property("localAnchorB", &b2PrismaticJointDef::localAnchorB)
                .property("localAxisA", &b2PrismaticJointDef::localAxisA)
                .property("referenceAngle", &b2PrismaticJointDef::referenceAngle)
                .property("enableLimit", &b2PrismaticJointDef::enableLimit)
                .property("lowerTranslation", &b2PrismaticJointDef::lowerTranslation)
                .property("upperTranslation", &b2PrismaticJointDef::upperTranslation)
                .property("enableMotor", &b2PrismaticJointDef::enableMotor)
                .property("motorSpeed", &b2PrismaticJointDef::motorSpeed)
                .property("maxMotorForce", &b2PrismaticJointDef::maxMotorForce);

        //binding class b2WheelJointDef
        class_<b2WheelJointDef, base<b2JointDef>>("WheelJointDef")
                .constructor<>()
                .property("localAnchorA", &b2WheelJointDef::localAnchorA)
                .property("localAnchorB", &b2WheelJointDef::localAnchorB)
                .property("localAxisA", &b2WheelJointDef::localAxisA)
                .property("enableLimit", &b2WheelJointDef::enableLimit)
                .property("lowerTranslation", &b2WheelJointDef::lowerTranslation)
                .property("upperTranslation", &b2WheelJointDef::upperTranslation)
                .property("enableMotor", &b2WheelJointDef::enableMotor)
                .property("maxMotorTorque", &b2WheelJointDef::maxMotorTorque)
                .property("motorSpeed", &b2WheelJointDef::motorSpeed)
                .property("stiffness", &b2WheelJointDef::stiffness)
                .property("damping", &b2WheelJointDef::damping);

        //binding class b2RopeJointDef
        class_<b2RopeJointDef, base<b2JointDef>>("RopeJointDef")
                .constructor<>()
                .property("localAnchorA", &b2RopeJointDef::localAnchorA)
                .property("localAnchorB", &b2RopeJointDef::localAnchorB)
                .property("maxLength", &b2RopeJointDef::maxLength);

        //binding class b2Joint
        class_<b2Joint>("Joint")
                .function("GetType", &b2Joint::GetType)
                .function("GetBodyA", &b2Joint::GetBodyA, allow_raw_pointers())
                .function("GetBodyB", &b2Joint::GetBodyB, allow_raw_pointers())
                .function("GetAnchorA", &b2Joint::GetAnchorA, allow_raw_pointers())
                .function("GetAnchorB", &b2Joint::GetAnchorB, allow_raw_pointers())
                .function("GetReactionForce", &b2Joint::GetReactionForce)
                .function("GetReactionTorque", &b2Joint::GetReactionTorque)
                // .function("GetNext", &b2Joint::GetNext)
                // .function("GetUserData", &b2Joint::GetUserData)
                // .function("SetUserData", &b2Joint::SetUserData)
                // .function("IsActive", &b2Joint::IsActive)
                .function("GetCollideConnected", &b2Joint::GetCollideConnected)
                .function("Cast2DistanceJoint", optional_override([](b2Joint* j) {
                        return (b2DistanceJoint*)j;}), allow_raw_pointers())
                .function("Cast2FrictionJoint", optional_override([](b2Joint* j) {
                        return (b2FrictionJoint*)j;}), allow_raw_pointers())
                .function("Cast2GearJoint", optional_override([](b2Joint* j) {
                        return (b2GearJoint*)j;}), allow_raw_pointers())
                .function("Cast2MotorJoint", optional_override([](b2Joint* j) {
                        return (b2MotorJoint*)j;}), allow_raw_pointers())
                .function("Cast2MouseJoint", optional_override([](b2Joint* j) {
                        return (b2MouseJoint*)j;}), allow_raw_pointers())
                .function("Cast2PrismaticJoint", optional_override([](b2Joint* j) {
                        return (b2PrismaticJoint*)j;}), allow_raw_pointers())
                .function("Cast2PulleyJoint", optional_override([](b2Joint* j) {
                        return (b2PulleyJoint*)j;}), allow_raw_pointers())
                .function("Cast2RevoluteJoint", optional_override([](b2Joint* j) {
                        return (b2RevoluteJoint*)j;}), allow_raw_pointers())
                .function("Cast2RopeJoint", optional_override([](b2Joint* j) {
                        return (b2RopeJoint*)j;}), allow_raw_pointers())
                .function("Cast2WeldJoint", optional_override([](b2Joint* j) {
                        return (b2WeldJoint*)j;}), allow_raw_pointers())
                .function("Cast2WheelJoint", optional_override([](b2Joint* j) {
                        return (b2WheelJoint*)j;}), allow_raw_pointers())
                .function("Dump", &b2Joint::Dump);

        //binding class b2DistanceJoint
        class_<b2DistanceJoint, base<b2Joint>>("DistanceJoint")
                .function("GetLocalAnchorA", &b2DistanceJoint::GetLocalAnchorA)
                .function("GetLocalAnchorB", &b2DistanceJoint::GetLocalAnchorB)
                .function("SetLength", &b2DistanceJoint::SetLength)
                .function("GetLength", &b2DistanceJoint::GetLength)
                // .function("SetMinLength", &b2DistanceJoint::SetMinLength)
                // .function("GetMinLength", &b2DistanceJoint::GetMinLength)
                // .function("SetMaxLength", &b2DistanceJoint::SetMaxLength)
                // .function("GetMaxLength", &b2DistanceJoint::GetMaxLength)
                .function("SetStiffness", &b2DistanceJoint::SetStiffness)
                .function("GetStiffness", &b2DistanceJoint::GetStiffness)
                .function("SetDamping", &b2DistanceJoint::SetDamping)
                .function("GetDamping", &b2DistanceJoint::GetDamping)
                .function("Dump", &b2DistanceJoint::Dump);

        //binding class b2FrictionJoint
        class_<b2FrictionJoint, base<b2Joint>>("FrictionJoint")
                .function("GetLocalAnchorA", &b2FrictionJoint::GetLocalAnchorA)
                .function("GetLocalAnchorB", &b2FrictionJoint::GetLocalAnchorB)
                .function("SetMaxForce", &b2FrictionJoint::SetMaxForce)
                .function("GetMaxForce", &b2FrictionJoint::GetMaxForce)
                .function("SetMaxTorque", &b2FrictionJoint::SetMaxTorque)
                .function("GetMaxTorque", &b2FrictionJoint::GetMaxTorque)
                .function("Dump", &b2FrictionJoint::Dump);

        //binding class b2GearJoint
        class_<b2GearJoint, base<b2Joint>>("GearJoint")
                .function("GetJoint1", &b2GearJoint::GetJoint1, allow_raw_pointers())
                .function("GetJoint2", &b2GearJoint::GetJoint2, allow_raw_pointers())
                .function("SetRatio", &b2GearJoint::SetRatio)
                .function("GetRatio", &b2GearJoint::GetRatio)
                .function("Dump", &b2GearJoint::Dump);

        //b2MotorJointDef
        class_<b2MotorJointDef, base<b2JointDef>>("MotorJointDef")
                .constructor<>()
                .property("linearOffset", &b2MotorJointDef::linearOffset)
                .property("angularOffset", &b2MotorJointDef::angularOffset)
                .property("maxForce", &b2MotorJointDef::maxForce)
                .property("maxTorque", &b2MotorJointDef::maxTorque)
                .property("correctionFactor", &b2MotorJointDef::correctionFactor);

        //binding class b2MotorJoint
        class_<b2MotorJoint, base<b2Joint>>("MotorJoint")
                .function("SetLinearOffset", &b2MotorJoint::SetLinearOffset)
                .function("GetLinearOffset", &b2MotorJoint::GetLinearOffset)
                .function("SetAngularOffset", &b2MotorJoint::SetAngularOffset)
                .function("GetAngularOffset", &b2MotorJoint::GetAngularOffset)
                .function("SetMaxForce", &b2MotorJoint::SetMaxForce)
                .function("GetMaxForce", &b2MotorJoint::GetMaxForce)
                .function("SetMaxTorque", &b2MotorJoint::SetMaxTorque)
                .function("GetMaxTorque", &b2MotorJoint::GetMaxTorque)
                .function("SetCorrectionFactor", &b2MotorJoint::SetCorrectionFactor)
                .function("GetCorrectionFactor", &b2MotorJoint::GetCorrectionFactor)
                .function("Dump", &b2MotorJoint::Dump);

        //binding class b2MouseJointDef
        class_<b2MouseJointDef, base<b2JointDef>>("MouseJointDef")
                .constructor<>()
                .property("target", &b2MouseJointDef::target)
                .property("maxForce", &b2MouseJointDef::maxForce)
                .property("frequencyHz", &b2MouseJointDef::frequencyHz)
                .property("dampingRatio", &b2MouseJointDef::dampingRatio);

        //binding class b2MouseJoint
        class_<b2MouseJoint, base<b2Joint>>("MouseJoint")
                .function("SetTarget", &b2MouseJoint::SetTarget)
                .function("GetTarget", &b2MouseJoint::GetTarget)
                .function("SetMaxForce", &b2MouseJoint::SetMaxForce)
                .function("GetMaxForce", &b2MouseJoint::GetMaxForce)
                .function("SetFrequency", &b2MouseJoint::SetFrequency)
                .function("GetFrequency", &b2MouseJoint::GetFrequency)
                .function("SetDampingRatio", &b2MouseJoint::SetDampingRatio)
                .function("GetDampingRatio", &b2MouseJoint::GetDampingRatio)
                .function("Dump", &b2MouseJoint::Dump);

        //binding class b2PrismaticJoint
        class_<b2PrismaticJoint, base<b2Joint>>("PrismaticJoint")
                .function("GetLocalAnchorA", &b2PrismaticJoint::GetLocalAnchorA)
                .function("GetLocalAnchorB", &b2PrismaticJoint::GetLocalAnchorB)
                .function("GetLocalAxisA", &b2PrismaticJoint::GetLocalAxisA)
                .function("GetReferenceAngle", &b2PrismaticJoint::GetReferenceAngle)
                .function("GetJointTranslation", &b2PrismaticJoint::GetJointTranslation)
                .function("GetJointSpeed", &b2PrismaticJoint::GetJointSpeed)
                .function("IsLimitEnabled", &b2PrismaticJoint::IsLimitEnabled)
                .function("EnableLimit", &b2PrismaticJoint::EnableLimit)
                .function("GetLowerLimit", &b2PrismaticJoint::GetLowerLimit)
                .function("GetUpperLimit", &b2PrismaticJoint::GetUpperLimit)
                .function("SetLimits", &b2PrismaticJoint::SetLimits)
                .function("IsMotorEnabled", &b2PrismaticJoint::IsMotorEnabled)
                .function("EnableMotor", &b2PrismaticJoint::EnableMotor)
                .function("SetMotorSpeed", &b2PrismaticJoint::SetMotorSpeed)
                .function("GetMotorSpeed", &b2PrismaticJoint::GetMotorSpeed)
                .function("SetMaxMotorForce", &b2PrismaticJoint::SetMaxMotorForce)
                .function("GetMaxMotorForce", &b2PrismaticJoint::GetMaxMotorForce)
                .function("GetMotorForce", &b2PrismaticJoint::GetMotorForce)
                .function("Dump", &b2PrismaticJoint::Dump);

        //binding class b2PulleyJoint
        class_<b2PulleyJoint, base<b2Joint>>("PulleyJoint")
                .function("GetGroundAnchorA", &b2PulleyJoint::GetGroundAnchorA)
                .function("GetGroundAnchorB", &b2PulleyJoint::GetGroundAnchorB)
                .function("GetLengthA", &b2PulleyJoint::GetLengthA)
                .function("GetLengthB", &b2PulleyJoint::GetLengthB)
                .function("GetRatio", &b2PulleyJoint::GetRatio)
                .function("GetCurrentLengthA", &b2PulleyJoint::GetCurrentLengthA)
                .function("GetCurrentLengthB", &b2PulleyJoint::GetCurrentLengthB)
                .function("Dump", &b2PulleyJoint::Dump);

        //binding class b2RevoluteJoint
        class_<b2RevoluteJoint, base<b2Joint>>("RevoluteJoint")
                .function("GetLocalAnchorA", &b2RevoluteJoint::GetLocalAnchorA)
                .function("GetLocalAnchorB", &b2RevoluteJoint::GetLocalAnchorB)
                .function("GetReferenceAngle", &b2RevoluteJoint::GetReferenceAngle)
                .function("GetJointAngle", &b2RevoluteJoint::GetJointAngle)
                .function("GetJointSpeed", &b2RevoluteJoint::GetJointSpeed)
                .function("IsLimitEnabled", &b2RevoluteJoint::IsLimitEnabled)
                .function("EnableLimit", &b2RevoluteJoint::EnableLimit)
                .function("GetLowerLimit", &b2RevoluteJoint::GetLowerLimit)
                .function("GetUpperLimit", &b2RevoluteJoint::GetUpperLimit)
                .function("SetLimits", &b2RevoluteJoint::SetLimits)
                .function("IsMotorEnabled", &b2RevoluteJoint::IsMotorEnabled)
                .function("EnableMotor", &b2RevoluteJoint::EnableMotor)
                .function("SetMotorSpeed", &b2RevoluteJoint::SetMotorSpeed)
                .function("GetMotorSpeed", &b2RevoluteJoint::GetMotorSpeed)
                .function("SetMaxMotorTorque", &b2RevoluteJoint::SetMaxMotorTorque)
                .function("GetMaxMotorTorque", &b2RevoluteJoint::GetMaxMotorTorque)
                .function("GetMotorTorque", &b2RevoluteJoint::GetMotorTorque)
                .function("Dump", &b2RevoluteJoint::Dump);

        //binding class b2WeldJoint
        class_<b2WeldJoint, base<b2Joint>>("WeldJoint")
                .function("GetLocalAnchorA", &b2WeldJoint::GetLocalAnchorA)
                .function("GetLocalAnchorB", &b2WeldJoint::GetLocalAnchorB)
                .function("GetReferenceAngle", &b2WeldJoint::GetReferenceAngle)
                .function("SetStiffness", &b2WeldJoint::SetStiffness)
                .function("GetStiffness", &b2WeldJoint::GetStiffness)
                .function("SetDamping", &b2WeldJoint::SetDamping)
                .function("GetDamping", &b2WeldJoint::GetDamping)
                .function("Dump", &b2WeldJoint::Dump);

        //binding class b2WheelJoint
        class_<b2WheelJoint, base<b2Joint>>("WheelJoint")
                .function("GetLocalAnchorA", &b2WheelJoint::GetLocalAnchorA)
                .function("GetLocalAnchorB", &b2WheelJoint::GetLocalAnchorB)
                .function("GetLocalAxisA", &b2WheelJoint::GetLocalAxisA)
                .function("GetJointTranslation", &b2WheelJoint::GetJointTranslation)
                // .function("GetJointSpeed", &b2WheelJoint::GetJointSpeed)
                .function("IsMotorEnabled", &b2WheelJoint::IsMotorEnabled)
                .function("EnableMotor", &b2WheelJoint::EnableMotor)
                .function("SetMotorSpeed", &b2WheelJoint::SetMotorSpeed)
                .function("GetMotorSpeed", &b2WheelJoint::GetMotorSpeed)
                .function("SetMaxMotorTorque", &b2WheelJoint::SetMaxMotorTorque)
                .function("GetMaxMotorTorque", &b2WheelJoint::GetMaxMotorTorque)
                .function("GetMotorTorque", &b2WheelJoint::GetMotorTorque)
                .function("SetStiffness", &b2WheelJoint::SetStiffness)
                .function("GetStiffness", &b2WheelJoint::GetStiffness)
                .function("SetDamping", &b2WheelJoint::SetDamping)
                .function("GetDamping", &b2WheelJoint::GetDamping)
                .function("Dump", &b2WheelJoint::Dump);

        //binding class RopeJoint
        class_<b2RopeJoint, base<b2Joint>>("RopeJoint")
                .function("GetLocalAnchorA", &b2RopeJoint::GetLocalAnchorA)
                .function("GetLocalAnchorB", &b2RopeJoint::GetLocalAnchorB)
                .function("GetReactionForce", &b2RopeJoint::GetReactionForce)
                .function("GetReactionTorque", &b2RopeJoint::GetReactionTorque)
                .function("SetMaxLength", &b2RopeJoint::SetMaxLength)
                .function("GetMaxLength", &b2RopeJoint::GetMaxLength)
                .function("GetLength", &b2RopeJoint::GetLength)
                .function("Dump", &b2RopeJoint::Dump);        
        }

        namespace emscripten {
        namespace internal {
        template <> void raw_destructor<b2Manifold>(b2Manifold *) {}
        template <> void raw_destructor<b2ContactImpulse>(b2ContactImpulse *) {}
        template <> void raw_destructor<b2Contact>(b2Contact *) {}
        template <> void raw_destructor<b2Shape>(b2Shape *) {}
        template <> void raw_destructor<b2CircleShape>(b2CircleShape *) {}
        template <> void raw_destructor<b2EdgeShape>(b2EdgeShape *) {}
        template <> void raw_destructor<b2PolygonShape>(b2PolygonShape *) {}
        template <> void raw_destructor<b2BodyDef>(b2BodyDef *) {}
        template <> void raw_destructor<b2Body>(b2Body *) {}
        template <> void raw_destructor<b2Fixture>(b2Fixture *) {}
        template <> void raw_destructor<b2Joint>(b2Joint *) {}
        template <> void raw_destructor<b2DistanceJoint>(b2DistanceJoint *) {}
        template <> void raw_destructor<b2FrictionJoint>(b2FrictionJoint *) {}
        template <> void raw_destructor<b2GearJoint>(b2GearJoint *) {}
        template <> void raw_destructor<b2MotorJoint>(b2MotorJoint *) {}
        template <> void raw_destructor<b2MouseJoint>(b2MouseJoint *) {}
        template <> void raw_destructor<b2PrismaticJoint>(b2PrismaticJoint *) {}
        template <> void raw_destructor<b2PulleyJoint>(b2PulleyJoint *) {}
        template <> void raw_destructor<b2RevoluteJoint>(b2RevoluteJoint *) {}
        template <> void raw_destructor<b2WeldJoint>(b2WeldJoint *) {}
        template <> void raw_destructor<b2WheelJoint>(b2WheelJoint *) {}
        } // namespace internal
        } // namespace emscripten