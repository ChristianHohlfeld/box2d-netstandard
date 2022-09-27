/*
  Box2D.NetStandard Copyright © 2020 Ben Ukhanov & Hugh Phoenix-Hulme https://github.com/benzuk/box2d-netstandard
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  
// MIT License
// Copyright (c) 2019 Erin Catto
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
*/

using Box2D.NetStandard.Common;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;

namespace Box2D.WindowTest
{
    public static class Math
    {
        public const ushort USHRT_MAX = ushort.MaxValue;
        public const byte UCHAR_MAX = byte.MaxValue;
        public const int RAND_LIMIT = 32767;

        private static readonly Random s_rnd = new Random();

        /// <summary>
        ///  This function is used to ensure that a floating point number is
        ///  not a NaN or infinity.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsValid(float x) =>
            !(float.IsNaN(x) || float.IsNegativeInfinity(x) || float.IsPositiveInfinity(x));

        /// <summary>
        ///  This is a approximate yet fast inverse square-root.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float InvSqrt(float x)
        {
            Convert convert = default;
            convert.x = x;
            float xhalf = 0.5f * x;
            convert.i = 0x5f3759df - (convert.i >> 1);
            x = convert.x;
            x = x * (1.5f - xhalf * x * x);
            return x;
        }

        [Obsolete("Use MathF.Sqrt", true)]
        public static float Sqrt(float x) => (float)System.Math.Sqrt(x);

        /// <summary>
        ///  Random number in range [-1,1]
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Random()
        {
            float r = s_rnd.Next() & RAND_LIMIT;
            r /= RAND_LIMIT;
            r = 2.0f * r - 1.0f;
            return r;
        }

        /// <summary>
        ///  Random floating point number in range [lo, hi]
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Random(float lo, float hi)
        {
            float r = s_rnd.Next() & RAND_LIMIT;
            r /= RAND_LIMIT;
            r = (hi - lo) * r + lo;
            return r;
        }

        /// <summary>
        ///  "Next Largest Power of 2
        ///  Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
        ///  that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
        ///  the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
        ///  largest power of 2. For a 32-bit value:"
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint NextPowerOfTwo(uint x)
        {
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsPowerOfTwo(uint x)
        {
            bool result = x > 0 && (x & (x - 1)) == 0;
            return result;
        }

        /// <summary>
        ///  Multiply a matrix transpose times a vector. If a rotation matrix is provided,
        ///  then this transforms the vector from one frame to another (inverse transform).
        /// </summary>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public static Vector2 MulT(Mat22 A, Vector2 v) => new Vector2(Vector2.Dot(v, A.ex), Vector2.Dot(v, A.ey));
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 MulT(Matrix3x2 A, Vector2 v)
        {
            /*Matrix3x2*/
            Matrex.Invert(A, out Matrix3x2 AT);
            return Vector2.Transform(v, AT);
        }


        /// <summary>
        ///  A * B
        /// </summary>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public static Mat22 Mul(Mat22 A, Mat22 B) {
        //   Mat22 C = new Mat22();
        //   C.Set(Mul(A, B.ex), Mul(A, B.ey));
        //   return C;
        // }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3x2 Mul(Matrix3x2 A, Matrix3x2 B) => A * B;

        /// <summary>
        ///  A^T * B
        /// </summary>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public static Mat22 MulT(Mat22 A, Mat22 B) {
        //   Vector2 c1 = new Vector2(Vector2.Dot(A.ex, B.ex), Vector2.Dot(A.ey, B.ex));
        //   Vector2 c2 = new Vector2(Vector2.Dot(A.ex, B.ey), Vector2.Dot(A.ey, B.ey));
        //   Mat22   C  = new Mat22();
        //   C.Set(c1, c2);
        //   return C;
        // }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3x2 MulT(Matrix3x2 A, Matrix3x2 B)
        {
            /*Matrix3x2*/
            Matrex.Invert(A, out Matrix3x2 AT);
            return AT * B;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 Mul(Transform T, Vector2 v) => T.p + Vector2.Transform(v, T.q); //Mul(T.q, v);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 MulT(Transform T, Vector2 v) => MulT(T.q, v - T.p);

        /// <summary>
        ///  Multiply a matrix times a vector.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Mul(Mat33 A, Vector3 v) => v.X * A.Ex + v.Y * A.Ey + v.Z * A.Ez;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 Mul(Rot q, Vector2 v) => new Vector2(q.c * v.X - q.s * v.Y, q.s * v.X + q.c * v.Y);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 MulT(Rot q, Vector2 v) => new Vector2(q.c * v.X + q.s * v.Y, -q.s * v.X + q.c * v.Y);


        // v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
        //    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Transform Mul(in Transform A, in Transform B)
        {
            Transform C;
            C.q = Mul(A.q, B.q);
            C.p = A.p + Vector2.Transform(B.p, A.q); //Mul(A.q, B.p);
            return C;
        }

        // v2 = A.q' * (B.q * v1 + B.p - A.p)
        //    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Transform MulT(in Transform A, in Transform B)
        {
            Transform C;
            C.q = MulT(A.q, B.q);
            C.p = MulT(A.q, B.p - A.p);
            return C;
        }


        [StructLayout(LayoutKind.Explicit)]
        public struct Convert
        {
            [FieldOffset(0)]
            public float x;

            [FieldOffset(0)]
            public int i;
        }
    }

    /// <summary>
    ///  A 3-by-3 matrix. Stored in column-major order.
    /// </summary>
    public struct Mat33
    {
        public Vector3 Ex, Ey, Ez;

        /// <summary>
        ///  Construct this matrix using columns.
        /// </summary>
        private Mat33(Vector3 c1, Vector3 c2, Vector3 c3)
        {
            Ex = c1;
            Ey = c2;
            Ez = c3;
        }

        /// <summary>
        ///  Set this matrix to all zeros.
        /// </summary>
        private void SetZero()
        {
            Ex = Vector3.Zero;
            Ey = Vector3.Zero;
            Ez = Vector3.Zero;
        }

        /// <summary>
        ///  Solve A * x = b, where b is a column vector. This is more efficient
        ///  than computing the inverse in one-shot cases.
        /// </summary>
        private Vector3 Solve33(Vector3 b)
        {
            float det = Vector3.Dot(Ex, Vector3.Cross(Ey, Ez));
            //Debug.Assert(det != 0.0f);
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            var x = new Vector3();
            x.X = det * Vector3.Dot(b, Vector3.Cross(Ey, Ez));
            x.Y = det * Vector3.Dot(Ex, Vector3.Cross(b, Ez));
            x.Z = det * Vector3.Dot(Ex, Vector3.Cross(Ey, b));
            return x;
        }

        /// <summary>
        ///  Solve A * x = b, where b is a column vector. This is more efficient
        ///  than computing the inverse in one-shot cases. Solve only the upper
        ///  2-by-2 matrix equation.
        /// </summary>
        private Vector2 Solve22(Vector2 b)
        {
            float a11 = Ex.X, a12 = Ey.X, a21 = Ex.Y, a22 = Ey.Y;
            float det = a11 * a22 - a12 * a21;
            //Debug.Assert(det != 0.0f);
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            var x = new Vector2();
            x.X = det * (a22 * b.X - a12 * b.Y);
            x.Y = det * (a11 * b.Y - a21 * b.X);
            return x;
        }

        private Mat33 GetInverse22(Mat33 M)
        {
            float a = Ex.X, b = Ey.X, c = Ex.Y, d = Ey.Y;
            float det = a * d - b * c;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            M.Ex.X = det * d;
            M.Ey.X = -det * b;
            M.Ex.Z = 0.0f;
            M.Ex.Y = -det * c;
            M.Ey.Y = det * a;
            M.Ey.Z = 0.0f;
            M.Ez.X = 0.0f;
            M.Ez.Y = 0.0f;
            M.Ez.Z = 0.0f;

            return M;
        }

        private Mat33 GetSymInverse33(Mat33 M)
        {
            float det = Vector3.Dot(Ex, Vector3.Cross(Ey, Ez));
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            float a11 = Ex.X, a12 = Ey.X, a13 = Ez.X;
            float a22 = Ey.Y, a23 = Ez.Y;
            float a33 = Ez.Z;

            M.Ex.X = det * (a22 * a33 - a23 * a23);
            M.Ex.Y = det * (a13 * a23 - a12 * a33);
            M.Ex.Z = det * (a12 * a23 - a13 * a22);

            M.Ey.X = M.Ex.Y;
            M.Ey.Y = det * (a11 * a33 - a13 * a13);
            M.Ey.Z = det * (a13 * a12 - a11 * a23);

            M.Ez.X = M.Ex.Z;
            M.Ez.Y = M.Ey.Z;
            M.Ez.Z = det * (a11 * a22 - a12 * a12);

            return M;
        }
    }
    public struct Rot
    {
        /// Sine and cosine
        public float s;

        /// Sine and cosine
        public float c;

        /// Initialize from an angle in radians
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Rot(float angle)
        {
            s = MathF.Sin(angle);
            c = MathF.Cos(angle);
        }

        /// Set using an angle in radians.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Set(float angle)
        {
            s = MathF.Sin(angle);
            c = MathF.Cos(angle);
        }

        /// Set to the identity rotation
        private void SetIdentity()
        {
            s = 0.0f;
            c = 1.0f;
        }

        /// Get the angle in radians
        private float GetAngle() => MathF.Atan2(s, c);

        /// Get the x-axis
        private Vector2 GetXAxis() => new Vector2(c, s);

        /// Get the u-axis
        private Vector2 GetYAxis() => new Vector2(-s, c);
    }
}