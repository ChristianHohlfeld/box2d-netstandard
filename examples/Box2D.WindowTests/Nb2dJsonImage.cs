using Box2D.NetStandard.Dynamics.Bodies;
using Vector2 = System.Numerics.Vector2;

namespace CocosSharp.RubeLoader
{
    public class Nb2dJsonImage
    {
        public string Name { get; set; }
        public string File { get; set; }

       // public CCSprite Sprite { get; set; }

        public Body Body { get; set; }
        public Vector2 Center { get; set; }
        public float Angle { get; set; }
        public float Scale { get; set; }
        public bool Flip { get; set; }
        public float Opacity { get; set; }
        public int Filter { get; set; }
        public float RenderOrder { get; set; }
        public int[] ColorTint { get; set; }
        public Vector2[] Corners { get; set; }
        public int NumPoints { get; set; }
        public float[] Points { get; set; }
        public float[] UvCoords { get; set; }
        public int NumIndices { get; set; }
        public short[] Indices { get; set; }

        //internal Body body;

        public Nb2dJsonImage()
        {
            ColorTint = new int[4];
        }

    }
}

