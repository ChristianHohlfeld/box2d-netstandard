
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using Box2D.NetStandard.Dynamics.World.Callbacks;
using OpenTK;
using System;
using System.IO;
using System.Text;
using Vector2 = System.Numerics.Vector2;

namespace CocosSharp.RubeLoader
{

    public class RubeBasicLayer 
    {

        public string DEFAULT_FONT = "fonts/MarkerFelt-22";
        public string JSON_FILE = "";

        public float DEFAULT_MOVE_MODIFICATOR = 4.5f;
        public float DEFAULT_ZOOM_SCALE = 0.3f;

        protected Vector2 AnchorPoint { get; set; }

        //public float ACTUAL_SCROLL_VALUE = 0;
        public bool HasWheel { get; set; }

        private Vector2 Position;

        public float Scale { get; private set; }
        public float ScaleX { get; private set; }
        public float ScaleY { get; private set; }
        public float PositionX { get; private set; }
        public float PositionY { get; private set; }

        public World m_world;
        //public bool IsInverted { get; set; }

        DebugDraw m_debugDraw;

        public int m_textLine;
        public int m_stepCount;

        public Box2D.NetStandard.Common.Settings settings = new Box2D.NetStandard.Common.Settings();


        public virtual string GetFilename()
        {
            return Path.GetFileName(JSON_FILE);
        }

        public RubeBasicLayer(string jsonfile, Vector2 size) 
        {

            JSON_FILE = jsonfile;
          
        }

        protected void AddedToScene()
        {
            AnchorPoint = new Vector2(0, 0);

            HasWheel = true;

            //     TouchPanel.EnabledGestures = GestureType.Pinch | GestureType.PinchComplete;

            //     var touchListener = new CCEventListenerTouchAllAtOnce();
            //     touchListener.OnTouchesBegan = OnTouchesBegan;
            //      touchListener.OnTouchesMoved = OnTouchesMoved;
            //     touchListener.OnTouchesEnded = OnTouchesEnded;
            //      touchListener.OnTouchesCancelled = OnTouchesCancelled;
            //     AddEventListener(touchListener, this);

            //   var mouseListener = new CCEventListenerMouse();
            //     mouseListener.OnMouseScroll = OnMouseScroll;
       //     AddEventListener(mouseListener, this);

            // set the starting scale and offset values from the subclass
            Position = InitialWorldOffset();
            Scale = InitialWorldScale();

            // load the world from RUBE .json file (this will also call afterLoadProcessing)
            LoadWorld(out m_world);

        }


        /// <summary>
        /// Attempts to load the world from the .json file given by getFilename.
        /// If successful, the method afterLoadProcessing will also be called,
        /// to allow subclasses to do something extra while the b2dJson information
        /// is still available.
        /// </summary>
        public void LoadWorld(out World world)
        {

            Clear();

            //TODO: 
            //m_debugDraw = new CCBox2dDraw(DEFAULT_FONT);

          //  m_debugDraw.AppendFlags(b2DrawFlags.e_shapeBit | b2DrawFlags.e_aabbBit | b2DrawFlags.e_centerOfMassBit | b2DrawFlags.e_jointBit | b2DrawFlags.e_pairBit);

            string fullpath = GetFilename();

            Console.WriteLine("Full path is: %s", fullpath);

            Nb2dJson json = new Nb2dJson();

            StringBuilder tmp = new StringBuilder();

            m_world = json.ReadFromFile(fullpath, tmp);

            if (m_world != null)
            {
                Console.WriteLine("Loaded JSON ok");
                m_world.SetDebugDraw(m_debugDraw);
                BodyDef bodyDef = new BodyDef();
                AfterLoadProcessing(json);
            }
            else
                Console.WriteLine(tmp); //if this warning bothers you, turn off "Typecheck calls to printf/scanf" in the project build settings
            world = m_world;
        }

        /// <summary>
        /// Override this in subclasses to do some extra processing (eg. acquire references
        /// to named bodies, joints etc) after the world has been loaded, and while the b2dJson
        /// information is still available.
        /// </summary>
        /// <param name="json"></param>
        public virtual void AfterLoadProcessing(Nb2dJson json)
        {

        }

        /// <summary>
        /// Override this in subclasses to set the inital view position
        /// </summary>
        /// <returns></returns>
        public virtual Vector2 InitialWorldOffset()
        {
            // This function should return the location in pixels to place
            // the (0,0) point of the physics world. The screen position
            // will be relative to the bottom left corner of the screen.

            //place (0,0) of physics world at center of bottom edge of screen
            var width = DisplayDevice.Default.Width;
            return new Vector2(width / 2, 0);
        }

        /// <summary>
        /// Override this in subclasses to set the inital view scale
        /// </summary>
        /// <returns></returns>
        public virtual float InitialWorldScale()
        {
            // This method should return the number of pixels for one physics unit.
            // When creating the scene in RUBE I can see that the jointTypes scene
            // is about 8 units high, so I want the height of the view to be about
            // 10 units, which for iPhone in landscape (480x320) we would return 32.
            // But for an iPad in landscape (1024x768) we would return 76.8, so to
            // handle the general case, we can make the return value depend on the
            // current screen height.

            var height = DisplayDevice.Default.Height;
            return height / 10; //screen will be 10 physics units high
        }

        // This method should undo anything that was done by the loadWorld and afterLoadProcessing
        // methods, and return to a state where loadWorld can safely be called again.
        public virtual void Clear()
        {

            if (m_world != null)
            {
                Console.WriteLine("Deleting Box2D world");
                m_world = null;
            }

            if (m_debugDraw != null)
                m_debugDraw = null;

        }

        /// <summary>
        /// Standard Cocos2d method
        /// </summary>
    /*    protected override void Draw()
        {
            base.Draw();

            m_debugDraw.Begin();
            //m_debugDraw.DrawString(50, 15, "bodies");
            m_world.DrawDebugData();

            if (m_touch.m_mouseJoint != null)
            {
                Vector2 p1 = m_touch.m_mouseJoint.GetAnchorB();
                Vector2 p2 = m_touch.m_mouseJoint.GetTarget();

                b2Color c = new b2Color(0.0f, 1.0f, 0.0f);
                m_debugDraw.DrawPoint(p1, 4.0f, c);
                m_debugDraw.DrawPoint(p2, 4.0f, c);

                c.Set(0.8f, 0.8f, 0.8f);
                m_debugDraw.DrawSegment(p1, p2, c);
            }

            m_debugDraw.End();
        }
    */
        /// <summary>
        /// Standard Cocos2d method, just step the physics world with fixed time step length
        /// </summary>
        /// <param name="dt"></param>
   //     public override void Update(float dt)
  //      {
    //        base.Update(dt);

   //         CheckUserGesture();

   //         m_world.Dump();
  //          m_world.Step(1 / 20.0f, 8, 3);
   //     }

        #region Zoom scale

        public void ZoomIn()
        {
            Zoom(1 + DEFAULT_ZOOM_SCALE);
        }

        public void ZoomOut()
        {
            Zoom(1 - DEFAULT_ZOOM_SCALE);
        }

        public void Zoom(float scale)
        {
            this.ScaleX *= scale;
            this.ScaleY *= scale;
        }

        #endregion


        

        #region World movement

        public void MoveLeft()
        {
            MoveLeft(1);
        }

        public void MoveLeft(int Steps)
        {
            PositionX -= Steps * DEFAULT_MOVE_MODIFICATOR;
        }

        public void MoveRight()
        {
            MoveRight(1);
        }

        public void MoveRight(int Steps)
        {

            PositionX += Steps * DEFAULT_MOVE_MODIFICATOR;
        }

        public void MoveUp()
        {
            MoveUp(1);
        }

        public void MoveUp(int Steps)
        {
            PositionY -= Steps * DEFAULT_MOVE_MODIFICATOR;
        }

        public void MoveDown()
        {
            MoveDown(1);
        }

        public void MoveDown(int Steps)
        {
            PositionY += DEFAULT_MOVE_MODIFICATOR;

        }

        #endregion

        public void DrawTitle(int x, int y, string title)
        {
           // m_debugDraw.DrawString(x, y, title);
        }

   /*     public virtual void Step()
        {
            float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

            if (settings.pause)
            {
                if (settings.singleStep)
                {
                    settings.singleStep = false;
                }
                else
                {
                    timeStep = 0.0f;
                }

                m_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
                m_textLine += 15;
            }

            b2DrawFlags flags = 0;
            if (settings.drawShapes) flags |= b2DrawFlags.e_shapeBit;
            if (settings.drawJoints) flags |= b2DrawFlags.e_jointBit;
            if (settings.drawAABBs) flags |= b2DrawFlags.e_aabbBit;
            if (settings.drawPairs) flags |= b2DrawFlags.e_pairBit;
            if (settings.drawCOMs) flags |= b2DrawFlags.e_centerOfMassBit;
            m_debugDraw.SetFlags(flags);

            m_world.SetWarmStarting(settings.enableWarmStarting > 0);
            m_world.SetContinuousPhysics(settings.enableContinuous > 0);
            m_world.SetSubStepping(settings.enableSubStepping > 0);

            m_world.Step(timeStep, settings.velocityIterations, settings.positionIterations);

            if (timeStep > 0.0f)
            {
                ++m_stepCount;
            }
        }

        */
        public Vector2 WorldToScreen(Vector2 worldPos)
        {
            //worldPos *= ScaleX;
            //Vector2 layerOffset = Position;
            //Vector2 p = new Vector2(worldPos.x + layerOffset.X, worldPos.y + layerOffset.Y);
            //p.Y = Director.WindowSizeInPixels.Height - p.Y;
            //return p;

            return WorldToScreen(worldPos, Position, ScaleX, new Vector2(DisplayDevice.Default.Width, DisplayDevice.Default.Height));
        }

        public Vector2 ScreenToWorld(Vector2 screenPos)
        {
            return ScreenToWorld(screenPos, Position, ScaleX,new Vector2(DisplayDevice.Default.Width,DisplayDevice.Default.Height));
        }

        /// <summary>
        /// Converts screen to world position : Same than ¿ CCDirector.ScreenToWorld ?
        /// </summary>
        /// <param name="screenPos"></param>
        /// <param name="actualPosition"></param>
        /// <param name="actualScale"></param>
        /// <param name="wSize"></param>
        /// <returns></returns>
        public static Vector2 ScreenToWorld(Vector2 screenPos, Vector2 actualPosition, float actualScale, Vector2 wSize)
        {
            //CCDrawManager.ScreenToWorld()
            screenPos.Y = wSize.Y - screenPos.Y;
            Vector2 layerOffset = actualPosition;
            screenPos.X -= layerOffset.X;
            screenPos.Y -= layerOffset.Y;

            float layerScale = actualScale;

            return new Vector2(screenPos.X / layerScale, screenPos.Y / layerScale);
        }


        public static Vector2 WorldToScreen(Vector2 worldPos, Vector2 actualPosition, float actualScale, Vector2 wSize)
        {
            worldPos *= actualScale;
            Vector2 layerOffset = actualPosition;
            Vector2 p = new Vector2(worldPos.X + layerOffset.X, worldPos.Y + layerOffset.Y);
            p.Y = wSize.Y - p.Y;
            return p;
        }


        //public class QueryCallback : b2QueryCallback
        //{
        //    public QueryCallback(Vector2 point)
        //    {
        //        m_point = point;
        //        m_fixture = null;
        //    }

        //    public override bool ReportFixture(GetNext() fixture)
        //    {
        //        Body body = fixture.Body;
        //        if (body.BodyType == b2BodyType.b2_dynamicBody)
        //        {
        //            bool inside = fixture.TestPoint(m_point);
        //            if (inside)
        //            {
        //                m_fixture = fixture;

        //                // We are done, terminate the query.
        //                return false;
        //            }
        //        }

        //        // Continue the query.
        //        return true;
        //    }

        //    public Vector2 m_point;
        //    public GetNext() m_fixture;
        //}





    }
}
