using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using OpenTK;
using System.Collections.Generic;
using System.Linq;
using Vector2 = System.Numerics.Vector2;

namespace CocosSharp.RubeLoader
{
    // public delegate float OnFinishedLoadedHandler();
    public class RubeLayer : RubeBasicLayer
    {

        public string PLAYER_LAYER_IMAGE_NAME = "player";
        public float PLAYER_LAYER_VELOCITY = 0.7f;

        public Nb2dJsonImage Player { get; set; }
        public List<Nb2dJsonImage> m_imageInfos { get; set; }

        public RubeLayer(string jsonurl, Vector2 size)
            : base(jsonurl, size)
        {
            AnchorPoint = new Vector2(0, 0);
        }

        public override Vector2 InitialWorldOffset()
        {
            // This function should return the location in pixels to place
            // the (0,0) point of the physics world. The screen position
            // will be relative to the bottom left corner of the screen.


            //634,5,130,5 / 3,400001
            //return new CCPoint(562.5f, 175.5f);

            //place (0,0) of physics world at center of bottom edge of screen
            var width = DisplayDevice.Default.Width;
            var height = DisplayDevice.Default.Height;
            return new Vector2(width * (475 / 1024.0f), height * (397 / 768.0f));
        }

        public override float InitialWorldScale()
        {
            // This method should return the number of pixels for one physics unit.
            // When creating the scene in RUBE I can see that the jointTypes scene
            // is about 8 units high, so I want the height of the view to be about
            // 10 units, which for iPhone in landscape (480x320) we would return 32.
            // But for an iPad in landscape (1024x768) we would return 76.8, so to
            // handle the general case, we can make the return value depend on the
            // current screen height.
            //return 3.40000f;
            //CCSize s = Window.WindowSizeInPixels;
            var height = DisplayDevice.Default.Height;
            return height / 10; /// 35; //screen will be 10 physics units high
        }

        /// <summary>
        /// Reposiciona todos los cuerpos cargados del XML con un offset
        /// </summary>
        /// <returns></returns>
        public Vector2 GetOffset()
        {
            return new Vector2(6.5f, 3f);
        }

        /// <summary>
        /// Post-carga de los objetos cargados del xml
        /// </summary>
        /// <param name="json"></param>
        public override void AfterLoadProcessing(Nb2dJson json)
        {

            //Movemos todos los objetos al offset que deseemos
            var delta = GetOffset(); // move all bodies by this offset

            if (delta.X != 0 && delta.Y != 0)
                foreach (var body in json.GetAllBodies())
                    body.SetTransform(Vector2.Add(body.Position, delta), body.GetAngle());

            //Obtenemos las imageInfos con todos los objetos
            m_imageInfos = json.GetAllImages().ToList();

          /* CCSprite tmpSprite; //TODO

            foreach (var img in m_imageInfos)
            {
                //Generamos el sprite en la posición
                tmpSprite = new CCSprite(img.File);
                tmpSprite.Position = new CCPoint(0, 0);
                AddChild(tmpSprite, (int)img.RenderOrder);

                //Guardamos el sprite
                img.Sprite = tmpSprite;

                // Asignamos el volteo y la escala del sprite
                img.Sprite.FlipX = img.Flip;
                img.Sprite.Scale = img.Scale / img.Sprite.ContentSize.Height;

                //Si es el bicho
                if (img.Name.Equals(PLAYER_LAYER_IMAGE_NAME))
                {
                    //img.fixture = json.GetFixturesByName("ball");
                    Player = img;
                }

            }*/


            //OnSetImagePositionsFromPhysicsBodies();

            OnFinishedLoading();
        }

     /*   public override void Update(float dt)
        {
            base.Update(dt);

           // OnSetImagePositionsFromPhysicsBodies();
        }
     */
        #region Gravity

        public void AddGravity()
        {
            SetGravity(0, m_world.Gravity.Y + 1f);
        }

        public void RemoveGravity()
        {
            SetGravity(0, m_world.Gravity.Y - 1f);
        }

        public void SetGravity(float x, float y)
        {
            m_world.Gravity = new Vector2(x, y);
        }

        public void SetPlayerVelocity(float velocity)
        {
            PLAYER_LAYER_VELOCITY = velocity;
        }

        public void SetPlayerLayerName(string imageLayerName)
        {

            PLAYER_LAYER_IMAGE_NAME = imageLayerName;
            foreach (var img in m_imageInfos)
            {

                if (img.Name.Equals(PLAYER_LAYER_IMAGE_NAME))
                {
                    Player = img;
                }

            }

        }


        #endregion

        /// <summary>
        /// Actualizamos las imagenes de los cuerpos con las fisicas
        /// </summary>
      /*  public virtual void OnSetImagePositionsFromPhysicsBodies()
        {

            foreach (var imgInfo in m_imageInfos)
            {
                if (imgInfo != null)
                {
                    var pos = imgInfo.Center;
                    float angle = -imgInfo.Angle;
                    if (imgInfo.Body != null)
                    {
                        //need to rotate image local center by body angle
                        Vector2 localPos = new Vector2(pos.x, pos.y);
                        var rot = new Rot(imgInfo.Body.Angle);


                        localPos = CCPointExHelper.b2Mul(rot, localPos) + imgInfo.Body.Position;

                        pos.x = localPos.x;
                        pos.y = localPos.y;
                        angle += -imgInfo.Body.Angle;

                    }

                    imgInfo.Sprite.Rotation = CCMathHelper.ToDegrees(angle);
                    imgInfo.Sprite.Position = new CCPoint(pos.x, pos.y);

                }
            }

        }
      */
        public virtual void OnFinishedLoading()
        {
        }


        // Remove one body and any images is had attached to it from the layer
        public void RemoveBodyFromWorld(Body body)
        {
            m_world.DestroyBody(body);

            List<Nb2dJsonImage> imagesToRemove = new List<Nb2dJsonImage>();

            foreach (var imgInfo in m_imageInfos)
            {
                if (imgInfo.Body == body)
                {
                    //RemoveChild(imgInfo.Sprite, true);
                    imagesToRemove.Add(imgInfo);
                }
            }

            //also remove the infos for those images from the image info array
            for (int i = 0; i < imagesToRemove.Count; i++)
                imagesToRemove.Remove(imagesToRemove[i]);

        }

       /* public CCSprite GetAnySpriteOnBody(Body body)
        {
            foreach (var item in m_imageInfos)
            {
                if (item.Body == body)
                    return item.Sprite;
            }


            return null;
        }

        public CCSprite GetSpriteWithImageName(string name)
        {

            foreach (var imgInfo in m_imageInfos)
            {
                if (imgInfo.Name == name)
                    return imgInfo.Sprite;
            }

            return null;
        }

        // Remove one image from the layer
        public void RemoveImageFromWorld(Nb2dJsonImage imgInfo)
        {
            RemoveChild(imgInfo.Sprite, true);
            m_imageInfos.Remove(imgInfo);
        }

        public override void Cleanup()
        {
            base.Cleanup();

            foreach (var item in m_imageInfos)
            {
                RemoveChild(item.Sprite, true);
            }
        }
       */

    }
}
