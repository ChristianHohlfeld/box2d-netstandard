using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Newtonsoft.Json.Linq;
using Box2D.NetStandard.Dynamics.Joints;
using Box2D.NetStandard.Dynamics.World;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Common;
using OpenTK;
using Box2D.NetStandard.Dynamics.Joints.Gear;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Joints.Weld;
using Box2D.NetStandard.Dynamics.Joints.Friction;
using Box2D.NetStandard.Dynamics.Joints.Revolute;
using System.Numerics;
using Vector2 = System.Numerics.Vector2;
using Box2D.NetStandard.Dynamics.Joints.Prismatic;
using Box2D.NetStandard.Dynamics.Joints.Distance;
using Box2D.NetStandard.Dynamics.Joints.Pulley;
using Box2D.NetStandard.Dynamics.Joints.Mouse;
using Box2D.NetStandard.Dynamics.Joints.Wheel;

namespace CocosSharp.RubeLoader
{
    public class Nb2dJson
    {
        public class Nb2dJsonCustomProperties
        {
            public Dictionary<string, int> m_customPropertyMap_int = new Dictionary<string, int>();
            public Dictionary<string, double> m_customPropertyMap_float = new Dictionary<string, double>();
            public Dictionary<string, string> m_customPropertyMap_string = new Dictionary<string, string>();
            public Dictionary<string, Vector2> m_customPropertyMap_vec2 = new Dictionary<string, Vector2>();
            public Dictionary<string, bool> m_customPropertyMap_bool = new Dictionary<string, bool>();
        }

        protected bool m_useHumanReadableFloats;
        protected int m_simulationPositionIterations;
        protected int m_simulationVelocityIterations;
        protected float m_simulationFPS;

        protected Dictionary<int, Body> m_indexToBodyMap;
        protected Dictionary<Body, int?> m_bodyToIndexMap;
        protected Dictionary<Joint, int?> m_jointToIndexMap;
        protected List<Body> m_bodies;
        protected List<Joint> m_joints;
        protected List<Nb2dJsonImage> m_images;

        //Dicctionary to associate body to name
        protected Dictionary<Body, string> m_bodyToNameMap;
        protected Dictionary<Fixture, string> m_fixtureToNameMap;
        protected Dictionary<Joint, string> m_jointToNameMap;
        protected Dictionary<Nb2dJsonImage, string> m_imageToNameMap;


        // This maps an item (Body, Fixture etc) to a set of custom properties.
        // Use null for world properties.
        protected Dictionary<object, Nb2dJsonCustomProperties> m_customPropertiesMap;

        protected HashSet<Body> m_bodiesWithCustomProperties;
        protected HashSet<Fixture> m_fixturesWithCustomProperties;
        protected HashSet<Joint> m_jointsWithCustomProperties;
        protected HashSet<Nb2dJsonImage> m_imagesWithCustomProperties;
        protected HashSet<World> m_worldsWithCustomProperties;


        public Nb2dJson()
            : this(true)
        {

        }

        public Nb2dJson(bool useHumanReadableFloats)
        {

            if (!useHumanReadableFloats)
            {
                // The floatToHex function is not giving the same results
                // as the original C++ version... not critical so worry about it
                // later.
                Console.WriteLine("Non human readable floats are not implemented yet");
                useHumanReadableFloats = true;
            }

            m_useHumanReadableFloats = useHumanReadableFloats;
            m_simulationPositionIterations = 3;
            m_simulationVelocityIterations = 8;
            m_simulationFPS = 60;

            m_indexToBodyMap = new Dictionary<int, Body>();
            m_bodyToIndexMap = new Dictionary<Body, int?>();
            m_jointToIndexMap = new Dictionary<Joint, int?>();
            m_bodies = new List<Body>();
            m_joints = new List<Joint>();
            m_images = new List<Nb2dJsonImage>();

            m_bodyToNameMap = new Dictionary<Body, string>();
            m_fixtureToNameMap = new Dictionary<Fixture, string>();
            m_jointToNameMap = new Dictionary<Joint, string>();
            m_imageToNameMap = new Dictionary<Nb2dJsonImage, string>();

            m_customPropertiesMap = new Dictionary<object, Nb2dJsonCustomProperties>();

            m_bodiesWithCustomProperties = new HashSet<Body>();
            m_fixturesWithCustomProperties = new HashSet<Fixture>();
            m_jointsWithCustomProperties = new HashSet<Joint>();
            m_imagesWithCustomProperties = new HashSet<Nb2dJsonImage>();
            m_worldsWithCustomProperties = new HashSet<World>();
        }

        //public JObject writeToValue( World world) {
        //if (null == world)
        //    return new JObject();

        //return b2j(world);
        //}

        //public String worldToString(World world, int indentFactor) {
        //    if (null == world)
        //        return "";

        //    return b2j(world).toString(indentFactor);
        //}

        public bool WriteToFile(World world, String filename, int indentFactor, StringBuilder errorMsg)
        {
            if (null == world || null == filename)
                return false;


            using (System.IO.TextWriter writeFile = new StreamWriter(filename))
            {
                try
                {
                    writeFile.WriteLine((world).ToString());
                }
                catch (Exception e)
                {
                    errorMsg.Append("Error writing JSON to file: " + filename + "  " + e.Message);
                    return false;
                }
            }

            return true;
        }

        public JObject B2n(World world, bool allowSleeping = true, bool warmStarting = true)
        {
            JObject worldValue = new JObject();

            m_bodyToIndexMap.Clear();
            m_jointToIndexMap.Clear();

            VecToJson("gravity", world.GetGravity(), worldValue);
            worldValue["allowSleep"] = allowSleeping;

            //worldValue["autoClearForces"] = world.;
            worldValue["warmStarting"] = warmStarting;
            //worldValue["continuousPhysics"] = world.isco;

            JArray customPropertyValue = WriteCustomPropertiesToJson(null);
            if (customPropertyValue.Count > 0)
                worldValue["customProperties"] = customPropertyValue;

            int i = 0;

            Body tmp = world.GetBodyList();
            while (tmp != null)
            {
                m_bodyToIndexMap.Add(tmp, i);
                worldValue.Add("body", B2n(tmp));
                tmp = tmp.GetNext();
                i++;
            }

            Joint tmpJoint = world.GetJointList();
            while (true)
            {
                if (tmpJoint.GetType() == typeof(GearJoint))
                    continue;
                m_jointToIndexMap.Add(tmpJoint, i);
                worldValue.Add("joint", B2n(world));
                i++;

                tmpJoint = tmpJoint.GetNext();
            }

            tmpJoint = world.GetJointList();

            while (true)
            {
                if (tmpJoint.GetType() == typeof(GearJoint))
                    continue;
                m_jointToIndexMap.Add(tmpJoint, i);
                worldValue.Add("joint", B2n(world));
                i++;

                tmpJoint = tmpJoint.GetNext();
            }


            // Currently the only efficient way to add images to a Jb2dJson
            // is by using the R.U.B.E editor. This code has not been tested,
            // but should work ok.
            i = 0;
            foreach (var image in m_imageToNameMap.Keys)
            {
                worldValue.Add("image", B2n(image));
            }

            m_bodyToIndexMap.Clear();
            m_jointToIndexMap.Clear();
            return worldValue;
        }

        JObject B2n(Nb2dJsonImage image)
        {
            JObject imageValue = new JObject();

            if (null != image.Body)
                imageValue["body"] = lookupBodyIndex(image.Body);
            else
                imageValue["body"] = -1;

            if (null != image.Name)
                imageValue["name"] = image.Name;
            if (null != image.File)
                imageValue["file"] = image.File;
            VecToJson("center", image.Center, imageValue);
            FloatToJson("angle", image.Angle, imageValue);
            FloatToJson("scale", image.Scale, imageValue);
            if (image.Flip)
                imageValue["flip"] = true;
            FloatToJson("opacity", image.Opacity, imageValue);
            imageValue["filter"] = image.Filter;
            FloatToJson("renderOrder", image.RenderOrder, imageValue);

            bool defaultColorTint = true;
            for (int i = 0; i < 4; i++)
            {
                if (image.ColorTint[i] != 255)
                {
                    defaultColorTint = false;
                    break;
                }
            }

            if (!defaultColorTint)
            {
                JArray array = (JArray)imageValue["colorTint"];
                for (int i = 0; i < 4; i++)
                    array[i] = image.ColorTint[i];
            }

            // image->updateCorners();
            for (int i = 0; i < 4; i++)
                VecToJson("corners", image.Corners[i], imageValue, i);

            // image->updateUVs();
            for (int i = 0; i < 2 * image.NumPoints; i++)
            {
                VecToJson("glVertexPointer", image.Points[i], imageValue, i);
                VecToJson("glTexCoordPointer", image.UvCoords[i], imageValue, i);
            }
            for (int i = 0; i < image.NumIndices; i++)
                VecToJson("glDrawElements", image.Indices[i], imageValue, i);

            JArray customPropertyValue = WriteCustomPropertiesToJson(image);
            if (customPropertyValue.Count > 0)
                imageValue["customProperties"] = customPropertyValue;

            return imageValue;
        }

        public JObject B2n(Joint joint, bool collideConnected = true)
        {
            JObject jointValue = new JObject();

            String jointName = GetJointName(joint);
            if (null != jointName)
                jointValue["name"] = jointName;

            int bodyIndexA = lookupBodyIndex(joint.GetBodyA());
            int bodyIndexB = lookupBodyIndex(joint.GetBodyB());
            jointValue["bodyA"] = bodyIndexA;
            jointValue["bodyB"] = bodyIndexB;
            //  if (joint.GetCollideConnected())
            if (collideConnected)
                jointValue["collideConnected"] = true;

            Body bodyA = joint.GetBodyA();
            Body bodyB = joint.GetBodyB();

            // why do Joint.getAnchor methods need to take an argOut style
            // parameter!?
            Vector2 tmpAnchor = new Vector2();
            if (joint.GetType() == typeof(RevoluteJoint))
            {
                jointValue["type"] = "revolute";

                RevoluteJoint revoluteJoint = (RevoluteJoint)joint;
                tmpAnchor = revoluteJoint.GetAnchorA;
                VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);

                tmpAnchor = revoluteJoint.GetAnchorB;
                VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);


                FloatToJson("refAngle", bodyB.GetAngle() - bodyA.GetAngle() - revoluteJoint.JointAngle, jointValue);
                FloatToJson("jointSpeed", revoluteJoint.JointSpeed, jointValue);
                jointValue["enableLimit"] = revoluteJoint.IsLimitEnabled;
                FloatToJson("lowerLimit", revoluteJoint.LowerLimit, jointValue);
                FloatToJson("upperLimit", revoluteJoint.UpperLimit, jointValue);
                jointValue["enableMotor"] = revoluteJoint.IsMotorEnabled;
                FloatToJson("motorSpeed", revoluteJoint.GetMotorSpeed(), jointValue);
                FloatToJson("maxMotorTorque", revoluteJoint.MotorTorque, jointValue); //todo, check if motortorque equals max motor torque

            }
            else if (joint.GetType() == typeof(PrismaticJoint))
            {
                jointValue["type"] = "prismatic";

                PrismaticJoint prismaticJoint = (PrismaticJoint)joint;
                tmpAnchor = prismaticJoint.GetAnchorA;
                VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);

                tmpAnchor = prismaticJoint.GetAnchorB;
                VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);

        //        VecToJson("localAxisA", prismaticJoint.GetLocalXAxisA, jointValue);
             //   FloatToJson("refAngle", prismaticJoint.GetReferenceAngle, jointValue);
                jointValue["enableLimit"] = prismaticJoint.IsLimitEnabled;
           //     FloatToJson("lowerLimit", prismaticJoint.GetLowerLimit, jointValue);
           //     FloatToJson("upperLimit", prismaticJoint.GetUpperLimit, jointValue);
                jointValue["enableMotor"] = prismaticJoint.IsMotorEnabled;
             //   FloatToJson("maxMotorForce", prismaticJoint.GetMaxMotorForce), jointValue);
                FloatToJson("motorSpeed", prismaticJoint.GetMotorSpeed(), jointValue);
            }
            else if (joint.GetType() == typeof(DistanceJoint))
            {
                jointValue["type"] = "distance";

                DistanceJoint distanceJoint = (DistanceJoint)joint;
                tmpAnchor = distanceJoint.GetAnchorA;
                VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
                tmpAnchor = distanceJoint.GetAnchorB;
                VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);

                //TODO: UNKOWS METHODS
                //FloatToJson("length", distanceJoint.GetLenght(), jointValue);
                //FloatToJson("frequency", distanceJoint.GetFrequency(), jointValue);
                //FloatToJson("dampingRatio", distanceJoint.GetDampingRatio(), jointValue);
            }
            else if (joint.GetType() == typeof(PulleyJoint))
            {
                jointValue["type"] = "pulley";

                PulleyJoint pulleyJoint = (PulleyJoint)joint;
                VecToJson("groundAnchorA", pulleyJoint.GroundAnchorA, jointValue);
                VecToJson("groundAnchorB", pulleyJoint.GroundAnchorB, jointValue);
                tmpAnchor = pulleyJoint.GetAnchorA;
                VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
            //    FloatToJson("lengthA", (pulleyJoint.GroundAnchorA - tmpAnchor).Length, jointValue);
                tmpAnchor = pulleyJoint.GetAnchorB;
                VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
          //      FloatToJson("lengthB", (pulleyJoint.GroundAnchorB - tmpAnchor).Length, jointValue);
                FloatToJson("ratio", pulleyJoint.Ratio, jointValue);
            }
            else if (joint.GetType() == typeof(MouseJoint))
            {
                jointValue["type"] = "mouse";

                MouseJoint mouseJoint = (MouseJoint)joint;

             //   VecToJson("target", mouseJoint.GetTarget(), jointValue);
                tmpAnchor = mouseJoint.GetAnchorB;
                VecToJson("anchorB", tmpAnchor, jointValue);
              //  FloatToJson("maxForce", mouseJoint.GetMaxForce(), jointValue);
             //   FloatToJson("frequency", mouseJoint.GetFrequency(), jointValue);
            //    FloatToJson("dampingRatio", mouseJoint.GetDampingRatio(), jointValue);
            }
            else if (joint.GetType() == typeof(GearJoint))
            {
                jointValue["type"] = "gear";
                GearJoint gearJoint = (GearJoint)joint;
       //         int jointIndex1 = lookupJointIndex(gearJoint.GetJoint1());
      //          int jointIndex2 = lookupJointIndex(gearJoint.GetJoint2());
              //  jointValue["joint1"] = jointIndex1;
         //       jointValue["joint2"] = jointIndex2;
                jointValue["ratio"] = gearJoint.Ratio;
            }
            else if (joint.GetType() == typeof(WheelJoint))
            {
                jointValue["type"] = "wheel";
                WheelJoint wheelJoint = (WheelJoint)joint;
                tmpAnchor = wheelJoint.GetAnchorA;
                VecToJson("anchorA", tmpAnchor, jointValue);
                tmpAnchor = wheelJoint.GetAnchorB;
                VecToJson("anchorB", tmpAnchor, jointValue);
                //TODO: UNKNOWN METHOD
                //VecToJson("localAxisA", wheelJoint.GetLocalAxisA(), jointValue);
                // TODO: jointValue["enableMotor"] = wheelJoint.IsMotorEnabled();
                FloatToJson("motorSpeed", wheelJoint.GetMotorSpeed(), jointValue);
                //TODO  FloatToJson("maxMotorTorque", wheelJoint.GetMaxMotorTorque(), jointValue);
                //TODO FloatToJson("springFrequency", wheelJoint.GetSpringFrequencyHz(), jointValue);
                //TODO FloatToJson("springDampingRatio", wheelJoint.GetSpringDampingRatio(), jointValue);
            }
            else if (joint.GetType() == typeof(WeldJoint))
            {
                jointValue["type"] = "weld";

                WeldJoint weldJoint = (WeldJoint)joint;
                tmpAnchor = weldJoint.GetAnchorA;
                VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
                tmpAnchor = weldJoint.GetAnchorB;
                VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
                //       FloatToJson("refAngle", weldJoint.ReferenceAngle, jointValue);
                //    FloatToJson("frequency", weldJoint.Frequency, jointValue);
                FloatToJson("dampingRatio", weldJoint.Damping, jointValue);
            }
            else if (joint.GetType() == typeof(FrictionJoint))
            {
                jointValue["type"] = "friction";

                FrictionJoint frictionJoint = (FrictionJoint)joint;
                tmpAnchor = frictionJoint.GetAnchorA;
                VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
                tmpAnchor = frictionJoint.GetAnchorB;
                VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
                FloatToJson("maxForce", frictionJoint.GetMaxForce(), jointValue);
                FloatToJson("maxTorque", frictionJoint.GetMaxTorque(), jointValue);
            }
            //TODO:  else if (joint.GetType() == typeof(RopeJoint))
            {
                // Rope joints are apparently not implemented in JBox2D yet, but
                // when they are, commenting out the following section should work.

                //         jointValue["type"] = "rope";

                //         RopeJoint ropeJoint = (RopeJoint)joint;
                //         tmpAnchor = ropeJoint.GetAnchorA();
                //         VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
                //        tmpAnchor = ropeJoint.GetAnchorB();
                //        VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
                //           FloatToJson("maxLength", ropeJoint.GetMaxLength(), jointValue);
            }

            JArray customPropertyValue = WriteCustomPropertiesToJson(joint);
            if (customPropertyValue.Count > 0)
                jointValue["customProperties"] = customPropertyValue;

            return jointValue;
        }

        public JObject B2n(Body body)
        {
            JObject bodyValue = new JObject();

            string bodyName = GetBodyName(body);
            if (null != bodyName)
                bodyValue["name"] = bodyName;
            if (body.Type() == BodyType.Static)
                bodyValue["type"] = 0;
            else
            if (body.Type() == BodyType.Kinematic)
                bodyValue["type"] = 1;
            else
            if (body.Type() == BodyType.Dynamic)
                bodyValue["type"] = 2;

            VecToJson("position", body.Position, bodyValue);
            FloatToJson("angle", body.GetAngle(), bodyValue);

            VecToJson("linearVelocity", body.GetLinearVelocity(), bodyValue);
            FloatToJson("angularVelocity", body.GetAngularVelocity(), bodyValue);

            if (body.GetLinearDamping() != 0)
                FloatToJson("linearDamping", body.GetLinearDamping(), bodyValue);
            if (body.GetAngularDamping() != 0)
                FloatToJson("angularDamping", body.GetAngularDamping(), bodyValue);
            if (body.GetGravityScale() != 1)
                FloatToJson("gravityScale", body.GetGravityScale(), bodyValue);

            if (body.IsBullet())
                bodyValue["bullet"] = true;
            if (!body.IsSleepingAllowed())
                bodyValue["allowSleep"] = false;
            if (body.IsAwake())
                bodyValue["awake"] = true;
            if (!body.IsAwake())
                bodyValue["active"] = false;
            if (body.IsFixedRotation())
                bodyValue["fixedRotation"] = true;

            MassData massData = new MassData();
            body.GetMassData(out massData);

            if (massData.mass != 0)
                FloatToJson("massData-mass", massData.mass, bodyValue);
            if (massData.center.X != 0 || massData.center.Y != 0)
                VecToJson("massData-center", body.GetLocalCenter(), bodyValue);
            if (massData.I != 0)
            {
                FloatToJson("massData-I", massData.I, bodyValue);
            }

            //int i = 0;
            JArray arr = new JArray();
            Body tmp = body;
            while (tmp != null)
            {
                bodyValue.Add("fixture", B2n(tmp));
                tmp = body.GetNext();
            }

            bodyValue["fixture"] = arr;

            JArray customPropertyValue = WriteCustomPropertiesToJson(body);
            if (customPropertyValue.Count > 0)
                bodyValue["customProperties"] = customPropertyValue;

            return bodyValue;
        }

        public JObject B2n(Fixture fixture)
        {
            JObject fixtureValue = new JObject();



            String fixtureName = GetFixtureName(fixture);
            if (null != fixtureName)
                fixtureValue["name"] = fixtureName;

            if (fixture.Restitution != 0)
                FloatToJson("restitution", fixture.Restitution, fixtureValue);
            if (fixture.m_friction != 0)
                FloatToJson("friction", fixture.m_friction, fixtureValue);
            if (fixture.Density != 0)
                FloatToJson("density", fixture.Density, fixtureValue);
            //     if (fixture.IsSensor)
            //        fixtureValue["sensor"] = true;

            if ((int)fixture.FilterData.categoryBits != 0x0001)
                fixtureValue["filter-categoryBits"] = (int)fixture.FilterData.categoryBits;
            if ((int)fixture.FilterData.maskBits != 0xffff)
                fixtureValue["filter-maskBits"] = (int)fixture.FilterData.maskBits;
            if (fixture.FilterData.groupIndex != 0)
                fixtureValue["filter-groupIndex"] = fixture.FilterData.groupIndex;

            Shape shape = fixture.Shape;
            switch (shape)
            {
                case CircleShape:
                    {
                        CircleShape circle = (CircleShape)shape;
                        JObject shapeValue = new JObject();
                        FloatToJson("radius", circle.Radius, shapeValue);
                        VecToJson("center", circle.Center, shapeValue);
                        fixtureValue["circle"] = shapeValue;
                    }
                    break;
                case EdgeShape:
                    {
                        /*
                        EdgeShape edge = (EdgeShape)shape;
                        JObject shapeValue = new JObject();
                        VecToJson("vertex1", edge.Vertex1, shapeValue);
                        VecToJson("vertex2", edge.Vertex2, shapeValue);
                        if (edge.HasVertex0)
                            shapeValue["hasVertex0"] = true;
                        if (edge.HasVertex3)
                            shapeValue["hasVertex3"] = true;
                        if (edge.HasVertex0)
                            VecToJson("vertex0", edge.Vertex0, shapeValue);
                        if (edge.HasVertex3)
                            VecToJson("vertex3", edge.Vertex3, shapeValue);
                        fixtureValue["edge"] = shapeValue;
                        */
                    }
                    break;
                case ChainShape:
                    {
                        /*
                        ChainShape chain = (ChainShape)shape;
                        JObject shapeValue = new JObject();
                        int count = chain.Vertices.Length;
                        for (int i = 0; i < count; ++i)
                            VecToJson("vertices", chain.Vertices[i], shapeValue, i);
                        if (chain.PrevVertex != null && chain.PrevVertex != Vector2.Zero)
                        {
                            shapeValue["hasPrevVertex"] = true;
                            VecToJson("prevVertex", chain.PrevVertex, shapeValue);
                        }
                        if (chain.NextVertex != null && chain.NextVertex != Vector2.Zero)
                        {
                            shapeValue["hasNextVertex"] = true;
                            VecToJson("nextVertex", chain.NextVertex, shapeValue);
                        }

                        fixtureValue["chain"] = shapeValue;
                        */
                    }
                    break;
                case PolygonShape:
                    {
                        PolygonShape poly = (PolygonShape)shape;
                        JObject shapeValue = new JObject();
                        int vertexCount = poly.GetVertices().Length;
                        for (int i = 0; i < vertexCount; ++i)
                            VecToJson("vertices", poly.GetVertices()[i], shapeValue, i);
                        fixtureValue["polygon"] = shapeValue;
                    }
                    break;
                default:
                    Console.WriteLine("Unknown shape type : " + shape.GetType().ToString());
                    break;
            }

            JArray customPropertyValue = WriteCustomPropertiesToJson(fixture);
            if (customPropertyValue.Count > 0)
                fixtureValue["customProperties"] = customPropertyValue;

            return fixtureValue;
        }

        public void VecToJson(String name, float v, JObject value, int index)
        {
            if (index > -1)
            {
                if (m_useHumanReadableFloats)
                {
                    JArray array = (JArray)value[name];
                    array[index] = v;
                }
                else
                {
                    JArray array = (JArray)value[name];
                    if (v == 0)
                        array[index] = 0;
                    else if (v == 1)
                        array[index] = 1;
                    else
                        array[index] = FloatToHex(v);
                }
            }
            else
                FloatToJson(name, v, value);
        }

        public void VecToJson(string name, Vector2 vec, JObject value)
        {
            VecToJson(name, vec, value, -1);
        }

        public void VecToJson(string name, Vector2 vec, JObject value, int index)
        {

            if (index > -1)
            {
                if (m_useHumanReadableFloats)
                {
                    bool alreadyHadArray = value[name] != null;
                    JArray arrayX = alreadyHadArray ? (JArray)value[name]["x"] : new JArray();
                    JArray arrayY = alreadyHadArray ? (JArray)value[name]["y"] : new JArray();
                    arrayX.Add(vec.X);
                    arrayY.Add(vec.Y);
                    if (!alreadyHadArray)
                    {
                        JObject subValue = new JObject();
                        subValue["x"] = arrayX;
                        subValue["y"] = arrayY;
                        value[name] = subValue;
                    }
                }
                else
                {
                    bool alreadyHadArray = value[name] != null;
                    JArray arrayX = alreadyHadArray ? (JArray)value[name]["x"] : new JArray();
                    JArray arrayY = alreadyHadArray ? (JArray)value[name]["y"] : new JArray();
                    if (vec.X == 0)
                        arrayX.Add(0);
                    else if (vec.Y== 1)
                        arrayX.Add(1);
                    else
                        arrayX.Add(FloatToHex(vec.X));
                    if (vec.Y == 0)
                        arrayY.Add(0);
                    else if (vec.Y == 1)
                        arrayY.Add(1);
                    else
                        arrayY.Add(FloatToHex(vec.Y));
                    if (!alreadyHadArray)
                    {
                        JObject subValue = new JObject();
                        subValue["x"] = arrayX;
                        subValue["y"] = arrayY;
                        value[name] = subValue;
                    }
                }
            }
            else
            {
                if (vec.X == 0 && vec.Y == 0)
                    value[name] = 0;// cut down on file space for common values
                else
                {
                    JObject vecValue = new JObject();
                    FloatToJson("x", vec.X, vecValue);
                    FloatToJson("y", vec.Y, vecValue);
                    value[name] = vecValue;
                }
            }

        }

        protected JArray WriteCustomPropertiesToJson(Object item)
        {
            JArray customPropertiesValue = new JArray();

            if (null == item)
                return customPropertiesValue;

            Nb2dJsonCustomProperties props = GetCustomPropertiesForItem(item, false);

            if (props == null)
                return customPropertiesValue;


            foreach (var customProp in props.m_customPropertyMap_int)
            {
                KeyValuePair<string, int> pair = (KeyValuePair<string, int>)customProp;
                JObject proValue = new JObject();
                proValue["name"] = pair.Key;
                proValue["int"] = pair.Value;
                customPropertiesValue.Add(proValue);
            }



            foreach (var customProp in props.m_customPropertyMap_float)
            {
                KeyValuePair<string, Double> pair = (KeyValuePair<string, Double>)customProp;
                JObject proValue = new JObject();
                proValue["name"] = pair.Key;
                proValue["float"] = pair.Value;
                customPropertiesValue.Add(proValue);
            }




            foreach (var customProp in props.m_customPropertyMap_string)
            {
                KeyValuePair<string, string> pair = (KeyValuePair<string, string>)customProp;
                JObject proValue = new JObject();
                proValue["name"] = pair.Key;
                proValue["string"] = pair.Value;
                customPropertiesValue.Add(proValue);
            }



            foreach (var customProp in props.m_customPropertyMap_vec2)
            {
                KeyValuePair<string, Vector2> pair = (KeyValuePair<string, Vector2>)customProp;
                JObject proValue = new JObject();
                proValue["name"] = pair.Key;
                VecToJson("vec2", pair.Value, proValue);
                customPropertiesValue.Add(proValue);
            }



            foreach (var customProp in props.m_customPropertyMap_bool)
            {
                KeyValuePair<string, bool> pair = (KeyValuePair<string, bool>)customProp;
                JObject proValue = new JObject();
                proValue["name"] = pair.Key;
                proValue["bool"] = pair.Value;
                customPropertiesValue.Add(proValue);
            }

            return customPropertiesValue;
        }

        public Nb2dJsonCustomProperties GetCustomPropertiesForItem(Object item, bool createIfNotExisting)
        {

            if (m_customPropertiesMap.ContainsKey(item))
                return m_customPropertiesMap[item];

            if (!createIfNotExisting)
                return null;

            Nb2dJsonCustomProperties props = new Nb2dJsonCustomProperties();
            m_customPropertiesMap[item] = props;

            return props;
        }

        Body lookupBodyFromIndex(int index)
        {
            if (m_indexToBodyMap.ContainsKey(index))
                return m_indexToBodyMap[index];
            else
                return null;
        }

        protected int lookupBodyIndex(Body body)
        {
            int? val = m_bodyToIndexMap[body];
            if (null != val)
                return val.Value;
            else
                return -1;
        }

        protected int lookupJointIndex(Joint joint)
        {
            int? val = m_jointToIndexMap[joint];
            if (null != val)
                return val.Value;
            else
                return -1;
        }

        public String GetImageName(Nb2dJsonImage image)
        {
            if (m_imageToNameMap.ContainsKey(image))
                return m_imageToNameMap[image];
            return null;
        }

        public String GetJointName(Joint joint)
        {

            if (m_jointToNameMap.ContainsKey(joint))
                return m_jointToNameMap[joint];
            return null;
        }

        public string GetBodyName(Body body)
        {
            if (m_bodyToNameMap.ContainsKey(body))
                return m_bodyToNameMap[body];
            return null;
        }

        public String GetFixtureName(Fixture fixture)
        {
            if (m_fixtureToNameMap.ContainsKey(fixture))
                return m_fixtureToNameMap[fixture];
            return null;
        }

        public void FloatToJson(String name, float f, JObject value)
        {
            // cut down on file space for common values
            if (f == 0)
                value.Add(name, 0);
            else if (f == 1)
                value.Add(name, 1);
            else
            {
                if (m_useHumanReadableFloats)
                    value.Add(name, f);
                else
                    value.Add(name, FloatToHex(f));
            }
        }

        // Convert a float argument to a byte array and display it. 
        public string FloatToHex(float argument)
        {
            //no usar todavia
            byte[] byteArray = BitConverter.GetBytes(argument);
            string formatter = "{0,16:E7}{1,20}";
            var res = string.Format(formatter, argument,
                                    BitConverter.ToString(byteArray));
            return res;
        }

        public Body[] GetBodiesByName(string name)
        {
            List<Body> bodies = new List<Body>();
            foreach (var item in m_bodyToNameMap.Keys)
            {
                if (m_bodyToNameMap[item] == name)
                    bodies.Add(item);
            }
            return bodies.ToArray();
        }


        public Fixture[] GetFixturesByName(String name)
        {
            List<Fixture> fixtures = new List<Fixture>();

            foreach (var item in m_fixtureToNameMap.Keys)
            {
                if (m_fixtureToNameMap[item] == name)
                    fixtures.Add(item);
            }
            return fixtures.ToArray();
        }

        public Joint[] GetJointsByName(String name)
        {
            List<Joint> joints = new List<Joint>();
            //
            foreach (var item in m_jointToNameMap.Keys)
            {
                if (m_jointToNameMap[item] == name)
                    joints.Add(item);
            }
            return joints.ToArray();
        }


        public Nb2dJsonImage[] GetImagesByName(string name)
        {
            List<Nb2dJsonImage> images = new List<Nb2dJsonImage>();
            foreach (var item in m_imageToNameMap.Keys)
            {
                if (m_imageToNameMap[item] == name)
                    images.Add(item);
            }
            return images.ToArray();
        }


        public List<Body> GetAllBodies()
        {
            return m_bodies;
        }

        public Nb2dJsonImage[] GetAllImages()
        {
            return (Nb2dJsonImage[])m_images.ToArray();
        }

        public Body GetBodyByName(string name)
        {
            foreach (var item in m_bodyToNameMap.Keys)
            {
                if (m_bodyToNameMap[item] == name)
                    return item;
            }
            return null;

        }

        public Fixture GetFixtureByName(string name)
        {
            foreach (var item in m_fixtureToNameMap.Keys)
            {
                if (m_fixtureToNameMap[item] == name)
                    return item;
            }
            return null;
        }

        public Joint GetJointByName(String name)
        {
            foreach (var item in m_jointToNameMap.Keys)
            {
                if (m_jointToNameMap[item] == name)
                    return item;
            }
            return null;
        }

        public Nb2dJsonImage GetImageByName(String name)
        {
            foreach (var item in m_imageToNameMap.Keys)
            {
                if (m_imageToNameMap[item] == name)
                    return item;
            }
            return null;
        }

        #region salida

        public World ReadFromFile(string filename, StringBuilder errorMsg)
        {
            if (null == filename)
                return null;

            string str = "";
            try
            {
                System.IO.TextReader readFile = new StreamReader(filename);
                str = readFile.ReadToEnd();
                readFile.Close();
                readFile = null;
                JObject worldValue = JObject.Parse(str);
                return N2b2World(worldValue);
            }
            catch (IOException ex)
            {
                errorMsg.Append("Error reading file: " + filename + ex.Message);
                return null;
            }
        }

        public World N2b2World(JObject worldValue)
        {
            World world = new World(jsonToVec("gravity", worldValue));

            world.SetAllowSleeping((bool)worldValue.GetValue("allowSleep"));
            world.ClearForces();
            //world.Set(worldValue.getBoolean("autoClearForces"));
            // world.SetWarmStarting((bool)worldValue.GetValue("warmStarting"));
            // world.SetContinuousPhysics((bool)worldValue.GetValue("continuousPhysics"));
            //  world.SetSubStepping((bool)worldValue.GetValue("subStepping"));

            readCustomPropertiesFromJson(world, worldValue);

            int i = 0;
            JArray bodyValues = (JArray)worldValue["body"];
            if (null != bodyValues)
            {
                int numBodyValues = bodyValues.Count;
                for (i = 0; i < numBodyValues; i++)
                {
                    JObject bodyValue = (JObject)bodyValues[i];
                    Body body = N2Body(world, bodyValue);
                    readCustomPropertiesFromJson(body, bodyValue);
                    m_bodies.Add(body);
                    m_indexToBodyMap.Add(i, body);
                }
            }

            // need two passes for joints because gear joints reference other joints
            JArray jointValues = (JArray)worldValue["joint"];
            if (null != jointValues)
            {
                int numJointValues = jointValues.Count;
                for (i = 0; i < numJointValues; i++)
                {
                    JObject jointValue = (JObject)jointValues[i];
                    if (jointValue["type"].ToString() != "gear")
                    {
                        Joint joint = j2Joint(world, jointValue);
                        readCustomPropertiesFromJson(joint, jointValue);
                        m_joints.Add(joint);
                    }
                }
                for (i = 0; i < numJointValues; i++)
                {
                    JObject jointValue = (JObject)jointValues[i];
                    if (jointValue["type"].ToString() == "gear")
                    {
                        Joint joint = j2Joint(world, jointValue);
                        readCustomPropertiesFromJson(joint, jointValue);
                        m_joints.Add(joint);
                    }
                }
            }
            i = 0;
            JArray imageValues = (JArray)worldValue["image"];
            if (null != imageValues)
            {
                int numImageValues = imageValues.Count;
                for (i = 0; i < numImageValues; i++)
                {
                    JObject imageValue = (JObject)imageValues[i];
                    Nb2dJsonImage image = j2b2dJsonImage(imageValue);
                    readCustomPropertiesFromJson(image, imageValue);
                    m_images.Add(image);
                }
            }
            return world;
        }

        public Body N2Body(World world, JObject bodyValue)
        {
            BodyDef bodyDef = new BodyDef();

            switch ((int)bodyValue.GetValue("type"))
            {
                case 0:
                    bodyDef.type = BodyType.Static;
                    break;
                case 1:
                    bodyDef.type = BodyType.Kinematic;
                    break;
                case 2:
                    bodyDef.type = BodyType.Dynamic;
                    break;
            }

            bodyDef.position = jsonToVec("position", bodyValue);
            bodyDef.angle = jsonToFloat("angle", bodyValue);
            bodyDef.linearVelocity = jsonToVec("linearVelocity", bodyValue);
            bodyDef.angularVelocity = jsonToFloat("angularVelocity", bodyValue);
            bodyDef.linearDamping = jsonToFloat("linearDamping", bodyValue, -1, 0);
            bodyDef.angularDamping = jsonToFloat("angularDamping", bodyValue, -1, 0);
            bodyDef.gravityScale = jsonToFloat("gravityScale", bodyValue, -1, 1);

            bodyDef.allowSleep = bodyValue["allowSleep"] == null ? false : (bool)bodyValue["allowSleep"];
            bodyDef.awake = bodyValue["awake"] == null ? false : (bool)bodyValue["awake"];
            bodyDef.fixedRotation = bodyValue["fixedRotation"] == null ? false : (bool)bodyValue["fixedRotation"];
            bodyDef.bullet = bodyValue["bullet"] == null ? false : (bool)bodyValue["bullet"];

            //bodyDef.active = bodyValue["active"] == null ? false : (bool)bodyValue["active"];

            bodyDef.awake = true;

            Body body = world.CreateBody(bodyDef);
            String bodyName = bodyValue["name"] == null ? "" : (string)bodyValue["active"];

            if ("" != bodyName)
                SetBodyName(body, bodyName);

            int i = 0;

            JArray fixtureValues = (JArray)bodyValue["fixture"];
            if (null != fixtureValues)
            {
                int numFixtureValues = fixtureValues.Count;
                for (i = 0; i < numFixtureValues; i++)
                {
                    JObject fixtureValue = (JObject)fixtureValues[i];
                    Fixture fixture = j2Fixture(body, fixtureValue);
                    readCustomPropertiesFromJson(fixture, fixtureValue);
                }
            }

            // may be necessary if user has overridden mass characteristics
            MassData massData = new MassData();
            massData.mass = jsonToFloat("massData-mass", bodyValue);
            massData.center = jsonToVec("massData-center", bodyValue);
            massData.I = jsonToFloat("massData-I", bodyValue);
            body.SetMassData(massData);

            return body;
        }


        // REVISADO =====================================================================

        Fixture j2Fixture(Body body, JObject fixtureValue)
        {
            if (null == fixtureValue)
                return null;

            FixtureDef fixtureDef = new FixtureDef();

            //Fixture fixtureDef = new Fixture();
            fixtureDef.restitution = jsonToFloat("restitution", fixtureValue);
            fixtureDef.friction = jsonToFloat("friction", fixtureValue);
            fixtureDef.density = jsonToFloat("density", fixtureValue);
            fixtureDef.isSensor = fixtureValue["sensor"] == null ? false : (bool)fixtureValue["sensor"];

            fixtureDef.filter.categoryBits = (fixtureValue["filter-categoryBits"] == null) ? (ushort)0x0001 : (ushort)fixtureValue["filter-categoryBits"];

            fixtureDef.filter.maskBits = fixtureValue["filter-maskBits"] == null ? (ushort)0xffff : (ushort)fixtureValue["filter-maskBits"];

            fixtureDef.filter.groupIndex = fixtureValue["filter-groupIndex"] == null ? (short)0 : (short)fixtureValue["filter-groupIndex"];

            Fixture fixture = null;

            if (null != fixtureValue["circle"])
            {
                JObject circleValue = (JObject)fixtureValue["circle"];

                CircleShape circleShape = new CircleShape();

                circleShape.Radius = jsonToFloat("radius", circleValue);
                circleShape.Center = jsonToVec("center", circleValue);

                fixtureDef.shape = circleShape;
                fixture = body.CreateFixture(fixtureDef);
            }
            else if (null != fixtureValue["edge"])
            {

                JObject edgeValue = (JObject)fixtureValue["edge"];
                EdgeShape edgeShape = new EdgeShape();

                edgeShape.SetOneSided(jsonToVec("vertex0", edgeValue), jsonToVec("vertex1", edgeValue), jsonToVec("vertex2", edgeValue), jsonToVec("vertex3", edgeValue));
                fixtureDef.shape = edgeShape;

                fixture = body.CreateFixture(fixtureDef);
            }
            else if (null != fixtureValue["loop"])
            {// support old
                // format (r197)

                JObject chainValue = (JObject)fixtureValue["loop"];
                ChainShape chainShape = new ChainShape();

                int numVertices = ((JArray)chainValue["x"]).Count;

                Vector2[] vertices = new Vector2[numVertices];
                for (int i = 0; i < numVertices; i++)
                    vertices[i] = jsonToVec("vertices", chainValue, i);

                chainShape.CreateLoop(vertices);

                fixtureDef.shape = chainShape;
                fixture = body.CreateFixture(fixtureDef);

            }
            else if (null != fixtureValue["chain"])
            {

                // FPE. See http://www.box2d.org/forum/viewtopic.php?f=4&t=7973&p=35363

                JObject chainValue = (JObject)fixtureValue["chain"];
                ChainShape chainShape = new ChainShape();
                int numVertices = ((JArray)chainValue["vertices"]["x"]).Count;
                var vertices = new Vector2[numVertices];

                for (int i = 0; i < numVertices; i++)
                    vertices[i] = jsonToVec("vertices", chainValue, i);

                //          chainShape.CreateChain(vertices, numVertices);
                //     chainShape.HasPrevVertex = chainValue["hasPrevVertex"] == null ? false : (bool)chainValue["hasPrevVertex"];
                //        chainShape.HasNextVertex = chainValue["hasNextVertex"] == null ? false : (bool)chainValue["hasNextVertex"];
                //
                //              if (chainShape.HasPrevVertex)
                //             chainShape.PrevVertex = (jsonToVec("prevVertex", chainValue));
                //        if (chainShape.HasNextVertex)
                //           chainShape.NextVertex = (jsonToVec("nextVertex", chainValue));

                fixtureDef.shape = chainShape;
                fixture = body.CreateFixture(fixtureDef);

            }
            else if (null != fixtureValue["polygon"])
            {

                JObject polygonValue = (JObject)fixtureValue["polygon"];
                Vector2[] vertices = new Vector2[Settings.MaxPolygonVertices];

                int numVertices = ((JArray)polygonValue["vertices"]["x"]).Count;
                if (numVertices > Settings.MaxPolygonVertices)
                {
                    Console.WriteLine("Ignoring polygon fixture with too many vertices.");
                }
                else if (numVertices < 2)
                {
                    Console.WriteLine("Ignoring polygon fixture less than two vertices.");
                }
                else if (numVertices == 2)
                {
                    Console.WriteLine("Creating edge shape instead of polygon with two vertices.");

                    EdgeShape edgeShape = new EdgeShape(jsonToVec("vertices", polygonValue, 0), jsonToVec("vertices", polygonValue, 1));
                    //edgeShape.Vertex1 = jsonToVec("vertices", polygonValue, 0);
                    //edgeShape.Vertex2 = jsonToVec("vertices", polygonValue, 1);
                    fixtureDef.shape = edgeShape;
                    fixture = body.CreateFixture(fixtureDef);

                }
                else
                {

                    PolygonShape polygonShape = new PolygonShape();
                    for (int i = 0; i < numVertices; i++)
                        vertices[i] = jsonToVec("vertices", polygonValue, i);
                    polygonShape.Set(vertices);
                    fixtureDef.shape = polygonShape;
                    fixture = body.CreateFixture(fixtureDef);
                }
            }

            String fixtureName = fixtureValue["name"] == null ? "" : fixtureValue["name"].ToString();
            if (fixtureName != "")
            {
                SetFixtureName(fixture, fixtureName);
            }

            return fixture;
        }

        // SIN REVISAR

        Joint j2Joint(World world, JObject jointValue)
        {
            Joint joint = null;

            int bodyIndexA = (int)jointValue["bodyA"];
            int bodyIndexB = (int)jointValue["bodyB"];
            if (bodyIndexA >= m_bodies.Count || bodyIndexB >= m_bodies.Count)
                return null;

            // set features common to all joints
            //var bodyA = m_bodies[bodyIndexA];
            //var bodyB = m_bodies[bodyIndexB];
            //var collideConnected = jointValue["collideConnected"] == null ? false : (bool)jointValue["collideConnected"];

            // keep these in scope after the if/else below
            RevoluteJointDef revoluteDef;
            PrismaticJointDef prismaticDef;
            DistanceJointDef distanceDef;
            PulleyJointDef pulleyDef;
            MouseJointDef mouseDef;
            GearJointDef gearDef;
            //b2WheelJoint wheelDef;
            WeldJointDef weldDef;
            FrictionJointDef frictionDef;
          //  b2RopeJointDef ropeDef;
            //MotorJoint motorDef;

            JointDef jointDef = null;

            Vector2 mouseJointTarget = new Vector2(0, 0);


            string type = jointValue["type"].ToString() == null ? "" : jointValue["type"].ToString();

            if (type == "revolute")
            {

                jointDef = revoluteDef = new RevoluteJointDef(); // JointFactory.CreateRevoluteJoint(world, bodyA, bodyB, jsonToVec("anchorB", jointValue));
                revoluteDef.localAnchorA = jsonToVec("anchorA", jointValue);
                revoluteDef.localAnchorB = jsonToVec("anchorB", jointValue);
                revoluteDef.referenceAngle = jsonToFloat("refAngle", jointValue);
                revoluteDef.enableLimit = jointValue["enableLimit"] == null ? false : (bool)jointValue["enableLimit"];
                revoluteDef.lowerAngle = jsonToFloat("lowerLimit", jointValue);
                revoluteDef.upperAngle = jsonToFloat("upperLimit", jointValue);
                revoluteDef.enableMotor = jointValue["enableMotor"] == null ? false : (bool)jointValue["enableMotor"];
                revoluteDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
                revoluteDef.maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
            }
            else if (type == "prismatic")
            {
                jointDef = prismaticDef = new PrismaticJointDef(); //JointFactory.CreatePrismaticJoint(world, bodyA, bodyB, localAnchorB, localAxis);

                prismaticDef.localAnchorA = jsonToVec("anchorA", jointValue);
                prismaticDef.localAnchorB = jsonToVec("anchorB", jointValue);


                if (jointValue["localAxisA"] != null)
                    prismaticDef.localAxisA = jsonToVec("localAxisA", jointValue);
                else
                    prismaticDef.localAxisA = jsonToVec("localAxis1", jointValue);

                prismaticDef.referenceAngle = jsonToFloat("refAngle", jointValue);

                prismaticDef.enableLimit = jointValue["enableLimit"] == null ? false : (bool)jointValue["enableLimit"];

                prismaticDef.lowerTranslation = jsonToFloat("lowerLimit", jointValue);
                prismaticDef.upperTranslation = jsonToFloat("upperLimit", jointValue);

                prismaticDef.enableMotor = jointValue["enableMotor"] == null ? false : (bool)jointValue["enableMotor"];

                prismaticDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
                prismaticDef.maxMotorForce = jsonToFloat("maxMotorForce", jointValue);

            }
            else if (type == "distance")
            {

                jointDef = distanceDef = new DistanceJointDef();
                distanceDef.localAnchorA = jsonToVec("anchorA", jointValue);
                distanceDef.localAnchorB = jsonToVec("anchorB", jointValue);
                distanceDef.length = jsonToFloat("length", jointValue);
                distanceDef.frequencyHz = jsonToFloat("frequency", jointValue);
                distanceDef.dampingRatio = jsonToFloat("dampingRatio", jointValue);

            }
            else if (type == "pulley")
            {

                jointDef = pulleyDef = new PulleyJointDef();
                pulleyDef.GroundAnchorA = jsonToVec("groundAnchorA", jointValue);
                pulleyDef.GroundAnchorB = jsonToVec("groundAnchorB", jointValue);
                pulleyDef.LocalAnchorB = jsonToVec("anchorA", jointValue);
                pulleyDef.LocalAnchorB = jsonToVec("anchorB", jointValue);
                pulleyDef.LengthA = jsonToFloat("lengthA", jointValue);
                pulleyDef.LengthB = jsonToFloat("lengthB", jointValue);
                pulleyDef.Ratio = jsonToFloat("ratio", jointValue);

            }
            else if (type == "mouse")
            {
                jointDef = mouseDef = new MouseJointDef();
                mouseJointTarget = jsonToVec("target", jointValue);
                mouseDef.Target = jsonToVec("anchorB", jointValue);// alter after creating joint
                mouseDef.MaxForce = jsonToFloat("maxForce", jointValue);
                mouseDef.FrequencyHz = jsonToFloat("frequency", jointValue);
                mouseDef.DampingRatio = jsonToFloat("dampingRatio", jointValue);
            }
            // Gear joints are apparently not implemented in JBox2D yet, but
            // when they are, commenting out the following section should work.

            else if (type == "gear")
            {

                jointDef = gearDef = new GearJointDef();  //JointFactory.CreateGearJoint(world, joint1, joint2, ratio);
                int jointIndex1 = (int)jointValue["joint1"];
                int jointIndex2 = (int)jointValue["joint2"];
                var joint1 = m_joints[jointIndex1];
                var joint2 = m_joints[jointIndex2];
                var ratio = jsonToFloat("ratio", jointValue);

                //joint = gearDef = JointFactory.CreateGearJoint(world, joint1, joint2, ratio);

            }

            // Wheel joints are apparently not implemented in JBox2D yet, but
            // when they are, commenting out the following section should work.

            else if (type == "wheel")
            {

                jointDef = revoluteDef = new RevoluteJointDef();
                revoluteDef.localAnchorA = jsonToVec("anchorA", jointValue);
                revoluteDef.localAnchorB = jsonToVec("anchorB", jointValue);

                revoluteDef.enableMotor = jointValue["enableMotor"] == null ? false : (bool)jointValue["enableMotor"];

                revoluteDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
                revoluteDef.maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);

                //jointDef = wheelDef = new b2WheelJointDef(); //JointFactory.CreateWheelJoint(world, bodyA, bodyB, localAnchorB, localAxisA);


                //var localAnchorA = jsonToVec("anchorA", jointValue);
                //var localAnchorB = (jsonToVec("anchorB", jointValue));
                //var localAxisA = (jsonToVec("localAxisA", jointValue));
                //var enableMotor = jointValue["enableMotor"] == null ? false : (bool)jointValue["enableMotor"];
                //var motorSpeed = jsonToFloat("motorSpeed", jointValue);
                //var maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
                //var frequencyHz = jsonToFloat("springFrequency", jointValue);
                //var dampingRatio = jsonToFloat("springDampingRatio", jointValue);

                //wheelDef.LocalAnchorA = localAnchorA;
                //wheelDef.LocalAnchorB = localAnchorB;
                //wheelDef.MotorEnabled = enableMotor;
                //wheelDef.MotorSpeed = motorSpeed;
                //wheelDef.SpringFrequencyHz = frequencyHz;
                //wheelDef.MaxMotorTorque = maxMotorTorque;
                //wheelDef.SpringDampingRatio = dampingRatio;
            }
            else if (type == "weld")
            {
                jointDef = weldDef = new WeldJointDef();
                weldDef.localAnchorA = jsonToVec("anchorA", jointValue);
                weldDef.localAnchorB = jsonToVec("anchorB", jointValue);
                weldDef.referenceAngle = 0;

            }
            else if (type == "friction")
            {
                jointDef = frictionDef = new FrictionJointDef();
                frictionDef.localAnchorA = jsonToVec("anchorA", jointValue);
                frictionDef.localAnchorB = jsonToVec("anchorB", jointValue);
                frictionDef.maxForce = jsonToFloat("maxForce", jointValue);
                frictionDef.maxTorque = jsonToFloat("maxTorque", jointValue);
            }
            else if (type == "rope")
            {
        //        jointDef = ropeDef = new b2RopeJointDef();
       //         ropeDef.localAnchorA = jsonToVec("anchorA", jointValue);
        //        ropeDef.localAnchorB = jsonToVec("anchorB", jointValue);
       //         ropeDef.maxLength = jsonToFloat("maxLength", jointValue);
            }

            //else if (type == "motor")
            //{
            //    var maxForce = jsonToFloat("maxForce", jointValue);
            //    var maxMotorTorque = jsonToFloat("maxTorque", jointValue);
            //    var angularOffset = jsonToFloat("refAngle", jointValue);

            //    joint = motorDef = new MotorJoint(bodyA, bodyB);
            //    world.AddJoint(joint);
            //    motorDef.LinearOffset = jsonToVec("anchorA", jointValue);
            //    motorDef.MaxForce = maxForce;
            //    motorDef.MaxTorque = maxMotorTorque;
            //    motorDef.AngularOffset = angularOffset;
            //}


            if (null != jointDef)
            {
                // set features common to all joints
                jointDef.bodyA = m_bodies[bodyIndexA];
                jointDef.bodyB = m_bodies[bodyIndexB];

                jointDef.collideConnected = jointValue["collideConnected"] == null ? false : (bool)jointValue["collideConnected"];

                joint = world.CreateJoint(jointDef);

                if (type.Equals("mouse"))
                    ((MouseJoint)joint).SetTarget(mouseJointTarget);

                String jointName = jointValue["name"] == null ? "" : (string)jointValue["name"];

                if (!jointName.Equals(""))
                {
                    SetJointName(joint, jointName);
                }
            }


            return joint;
        }

        Nb2dJsonImage j2b2dJsonImage(JObject imageValue)
        {
            Nb2dJsonImage img = new Nb2dJsonImage();

            int bodyIndex = imageValue["body"] == null ? -1 : (int)imageValue["body"];
            if (-1 != bodyIndex)
                img.Body = lookupBodyFromIndex(bodyIndex);

            String imageName = imageValue["name"] == null ? "" : imageValue["name"].ToString();
            if (imageName != "")
            {
                img.Name = imageName;
                SetImageName(img, imageName);
            }

            String fileName = imageValue["file"] == null ? "" : imageValue["file"].ToString();
            //TODO: Renombramos el fichero a una ruta relativa
            if (fileName != "")
                img.File = Path.GetFileName(fileName);

            img.Center = jsonToVec("center", imageValue);
            img.Angle = jsonToFloat("angle", imageValue);
            img.Scale = jsonToFloat("scale", imageValue);
            img.Opacity = jsonToFloat("opacity", imageValue);
            img.RenderOrder = jsonToFloat("renderOrder", imageValue);

            JArray colorTintArray = (JArray)imageValue["colorTint"];
            if (null != colorTintArray)
            {
                for (int i = 0; i < 4; i++)
                {
                    img.ColorTint[i] = (int)colorTintArray[i];
                }
            }

            img.Flip = imageValue["flip"] == null ? false : (bool)imageValue["flip"];

            img.Filter = imageValue["filter"] == null ? 1 : (int)imageValue["filter"];

            img.Corners = new Vector2[4];
            for (int i = 0; i < 4; i++)
                img.Corners[i] = jsonToVec("corners", imageValue, i);

            JArray vertexPointerArray = (JArray)imageValue["glVertexPointer"];
            JArray texCoordArray = (JArray)imageValue["glVertexPointer"];
            if (null != vertexPointerArray && null != texCoordArray && vertexPointerArray.Count == texCoordArray.Count)
            {
                int numFloats = vertexPointerArray.Count;
                img.NumPoints = numFloats / 2;
                img.Points = new float[numFloats];
                img.UvCoords = new float[numFloats];
                for (int i = 0; i < numFloats; i++)
                {
                    img.Points[i] = jsonToFloat("glVertexPointer", imageValue, i);
                    img.UvCoords[i] = jsonToFloat("glTexCoordPointer", imageValue, i);
                }
            }

            JArray drawElementsArray = (JArray)imageValue["glDrawElements"];
            if (null != drawElementsArray)
            {
                img.NumIndices = drawElementsArray.Count;
                img.Indices = new short[img.NumIndices];
                for (int i = 0; i < img.NumIndices; i++)
                    img.Indices[i] = (short)drawElementsArray[i];
            }

            return img;
        }


        public void SetBodyName(Body body, String name)
        {
            m_bodyToNameMap.Add(body, name);
        }

        public void SetFixtureName(Fixture fixture, String name)
        {
            m_fixtureToNameMap.Add(fixture, name);
        }

        public void SetJointName(Joint joint, String name)
        {
            m_jointToNameMap.Add(joint, name);
        }

        public void SetImageName(Nb2dJsonImage image, String name)
        {
            m_imageToNameMap.Add(image, name);
        }



        protected void readCustomPropertiesFromJson(Body item, JObject value)
        {
            if (null == item)
                return;

            if (value["customProperties"] != null)
                return;

            int i = 0;
            JArray propValues = (JArray)value["customProperties"];
            if (null != propValues)
            {
                int numPropValues = propValues.Count;
                for (i = 0; i < numPropValues; i++)
                {
                    JObject propValue = (JObject)propValues[i];
                    string propertyName = propValue["name"].ToString();
                    if (propValue["int"] != null)
                        SetCustomInt(item, propertyName, (int)propValue["int"]);
                    if (propValue["float"] != null)
                        SetCustomFloat(item, propertyName, (float)propValue["float"]);
                    if (propValue["string"] != null)
                        SetCustomString(item, propertyName, propValue["string"].ToString());
                    if (propValue["vec2"] != null)
                        SetCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
                    if (propValue["bool"] != null)
                        SetCustomBool(item, propertyName, (bool)propValue["bool"]);
                }
            }
        }


        protected void readCustomPropertiesFromJson(Fixture item, JObject value)
        {
            if (null == item)
                return;

            if (value["customProperties"] != null)
                return;

            int i = 0;
            JArray propValues = (JArray)value["customProperties"];
            if (null != propValues)
            {
                int numPropValues = propValues.Count;
                for (i = 0; i < numPropValues; i++)
                {
                    JObject propValue = (JObject)propValues[i];
                    String propertyName = propValue["name"].ToString();
                    if (propValue["int"] != null)
                        SetCustomInt(item, propertyName, (int)propValue["int"]);
                    if (propValue["float"] != null)
                        SetCustomFloat(item, propertyName, (float)propValue["float"]);
                    if (propValue["string"] != null)
                        SetCustomString(item, propertyName, propValue["string"].ToString());
                    if (propValue["vec2"] != null)
                        SetCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
                    if (propValue["bool"] != null)
                        SetCustomBool(item, propertyName, (bool)propValue["bool"]);
                }
            }
        }

        protected void readCustomPropertiesFromJson(Joint item, JObject value)
        {
            if (null == item)
                return;

            if (value["customProperties"] != null)
                return;

            int i = 0;
            JArray propValues = (JArray)value["customProperties"];
            if (null != propValues)
            {
                int numPropValues = propValues.Count;
                for (i = 0; i < numPropValues; i++)
                {
                    JObject propValue = (JObject)propValues[i];
                    String propertyName = propValue["name"].ToString();
                    if (propValue["int"] != null)
                        SetCustomInt(item, propertyName, (int)propValue["int"]);
                    if (propValue["float"] != null)
                        SetCustomFloat(item, propertyName, (float)propValue["float"]);
                    if (propValue["string"] != null)
                        SetCustomString(item, propertyName, propValue["string"].ToString());
                    if (propValue["vec2"] != null)
                        SetCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
                    if (propValue["bool"] != null)
                        SetCustomBool(item, propertyName, (bool)propValue["bool"]);
                }
            }
        }

        protected void readCustomPropertiesFromJson(Nb2dJsonImage item, JObject value)
        {
            if (null == item)
                return;

            if (value["customProperties"] != null)
                return;

            int i = 0;
            JArray propValues = (JArray)value["customProperties"];
            if (null != propValues)
            {
                int numPropValues = propValues.Count;
                for (i = 0; i < numPropValues; i++)
                {
                    JObject propValue = (JObject)propValues[i];
                    string propertyName = propValue["name"].ToString();
                    if (propValue["int"] != null)
                        SetCustomInt(item, propertyName, (int)propValue["int"]);
                    if (propValue["float"] != null)
                        SetCustomFloat(item, propertyName, (float)propValue["float"]);
                    if (propValue["string"] != null)
                        SetCustomString(item, propertyName, propValue["string"].ToString());
                    if (propValue["vec2"] != null)
                        SetCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
                    if (propValue["bool"] != null)
                        SetCustomBool(item, propertyName, (bool)propValue["bool"]);
                }
            }
        }

        protected void readCustomPropertiesFromJson(World item, JObject value)
        {
            if (null == item)
                return;

            if (value["customProperties"] != null)
                return;

            int i = 0;
            JArray propValues = (JArray)value["customProperties"];
            if (null != propValues)
            {
                int numPropValues = propValues.Count;
                for (i = 0; i < numPropValues; i++)
                {
                    JObject propValue = (JObject)propValues[i];
                    String propertyName = propValue["name"].ToString();
                    if (propValue["int"] != null)
                        SetCustomInt(item, propertyName, (int)propValue["int"]);
                    if (propValue["float"] != null)
                        SetCustomFloat(item, propertyName, (float)propValue["float"]);
                    if (propValue["string"] != null)
                        SetCustomString(item, propertyName, propValue["string"].ToString());
                    if (propValue["vec2"] != null)
                        SetCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
                    if (propValue["bool"] != null)
                        SetCustomBool(item, propertyName, (bool)propValue["bool"]);
                }
            }
        }

        protected void SetCustomInt(Object item, String propertyName, int val)
        {
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_int.Add(propertyName, val);
        }

        protected void SetCustomFloat(Object item, String propertyName, float val)
        {
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_float.Add(propertyName, (float)val);
        }

        protected void SetCustomString(Object item, String propertyName, String val)
        {
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_string.Add(propertyName, val);
        }

        protected void SetCustomVector(Object item, String propertyName, Vector2 val)
        {
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.Add(propertyName, val);
        }

        protected void SetCustomBool(Object item, String propertyName, bool val)
        {
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_bool.Add(propertyName, val);
        }


        public void SetCustomInt(Body item, String propertyName, int val)
        {
            m_bodiesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_int.Add(propertyName, val);
        }

        public void SetCustomFloat(Body item, String propertyName, float val)
        {
            m_bodiesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_float.Add(propertyName, (float)val);
        }

        public void SetCustomString(Body item, String propertyName, String val)
        {
            m_bodiesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_string.Add(propertyName, val);
        }

        public void SetCustomVector(Body item, String propertyName, Vector2 val)
        {
            m_bodiesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.Add(propertyName, val);
        }

        public void SetCustomBool(Body item, String propertyName, bool val)
        {
            m_bodiesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_bool.Add(propertyName, val);
        }


        public void SetCustomInt(Fixture item, String propertyName, int val)
        {
            m_fixturesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_int.Add(propertyName, val);
        }

        public void SetCustomFloat(Fixture item, String propertyName, float val)
        {
            m_fixturesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_float.Add(propertyName, (float)val);
        }

        public void SetCustomString(Fixture item, String propertyName, String val)
        {
            m_fixturesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_string.Add(propertyName, val);
        }

        public void SetCustomVector(Fixture item, String propertyName, Vector2 val)
        {
            m_fixturesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.Add(propertyName, val);
        }

        public void SetCustomBool(Fixture item, String propertyName, bool val)
        {
            m_fixturesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_bool.Add(propertyName, val);
        }


        public void SetCustomInt(Joint item, String propertyName, int val)
        {
            m_jointsWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_int.Add(propertyName, val);
        }

        public void SetCustomFloat(Joint item, String propertyName, float val)
        {
            m_jointsWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_float.Add(propertyName, (float)val);
        }

        public void SetCustomString(Joint item, String propertyName, String val)
        {
            m_jointsWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_string.Add(propertyName, val);
        }

        public void SetCustomVector(Joint item, String propertyName, Vector2 val)
        {
            m_jointsWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.Add(propertyName, val);
        }

        public void SetCustomBool(Joint item, String propertyName, bool val)
        {
            m_jointsWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_bool.Add(propertyName, val);
        }


        public void SetCustomInt(Nb2dJsonImage item, String propertyName, int val)
        {
            m_imagesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_int.Add(propertyName, val);
        }

        public void SetCustomFloat(Nb2dJsonImage item, String propertyName, float val)
        {
            m_imagesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_float.Add(propertyName, (float)val);
        }

        public void SetCustomString(Nb2dJsonImage item, String propertyName, String val)
        {
            m_imagesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_string.Add(propertyName, val);
        }

        public void SetCustomVector(Nb2dJsonImage item, String propertyName, Vector2 val)
        {
            m_imagesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.Add(propertyName, val);
        }

        public void SetCustomBool(Nb2dJsonImage item, String propertyName, bool val)
        {
            m_imagesWithCustomProperties.Add(item);
            GetCustomPropertiesForItem(item, true).m_customPropertyMap_bool.Add(propertyName, val);
        }

        Vector2 jsonToVec(String name, JObject value)
        {
            return jsonToVec(name, value, -1, new Vector2(0, 0));
        }

        Vector2 jsonToVec(String name, JObject value, int index)
        {
            return jsonToVec(name, value, index, new Vector2(0, 0));
        }

        Vector2 jsonToVec(String name, JObject value, int index, Vector2 defaultValue)
        {
            Vector2 vec = defaultValue;

            if (value[name] == null || value[name] is JValue)
                return defaultValue;

            if (index > -1)
            {
                JObject vecValue = (JObject)value[name];
                JArray arrayX = (JArray)vecValue["x"];
                JArray arrayY = (JArray)vecValue["y"];

                vec.X = (float)arrayX[index];

                vec.Y = (float)arrayY[index];
            }
            else
            {
                JObject vecValue = (JObject)value[name];
                if (null == vecValue)
                    return defaultValue;
                else if (vecValue["x"] == null) // should be zero vector
                    vec.X = vec.Y = 0;
                else
                {
                    vec.X = jsonToFloat("x", vecValue);
                    vec.Y = jsonToFloat("y", vecValue);
                }
            }

            return vec;
        }

        float jsonToFloat(String name, JObject value)
        {
            return jsonToFloat(name, value, -1, 0);
        }

        float jsonToFloat(String name, JObject value, int index)
        {
            return jsonToFloat(name, value, index, 0);
        }

        float jsonToFloat(String name, JObject value, int index, float defaultValue)
        {
            if (value[name] == null)
                return defaultValue;

            if (index > -1)
            {
                JArray array = null;
                try
                {
                    array = (JArray)value[name];
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                }
                if (null == array)
                    return defaultValue;
                Object obj = array[index];
                if (null == obj)
                    return defaultValue;
                // else if ( value[name].isString() )
                // return hexToFloat( value[name].asString() );
                else
                    return float.Parse(obj.ToString());
            }
            else
            {
                Object obj = value[name];
                if (null == obj)
                    return defaultValue;
                // else if ( value[name].isString() )
                // return hexToFloat( value[name].asString() );
                else
                    return float.Parse(obj.ToString());
            }
        }

        #endregion

    }

}


