/*
 * Window Simulation Copyright © Ben Ukhanov 2021
 */

using System;
using System.Threading;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Joints;
using Box2D.NetStandard.Dynamics.World;
using Box2D.NetStandard.Dynamics.World.Callbacks;
using Box2D.Window;
using Box2D.WorldTests;
using CocosSharp.RubeLoader;
using OpenTK;

namespace Box2D.WindowTests
{
    public static class Program
    {
        private const bool StepByStep = false;

        private static World world;
        private static Body focusBody;

        static Program()
        {
            CreateWorld();
        }

        private static void CreateWorld()
        {
            RubeLayer rlayer = new RubeLayer("bike.json", new System.Numerics.Vector2(0, 0));

            rlayer.LoadWorld(out world);
        }

        private static void Main()
        {
            var windowThread = new Thread(new ThreadStart(() =>
            {
                int width = DisplayDevice.Default.Width;
                int height = DisplayDevice.Default.Height;
                int scaleX = 2;
                int scaleY = 2;
                //   var game = new SimulationWindow("Physics Simulation", width, height, 1, 1, GameWindowFlags.Fullscreen, focusBody);
                var game = new SimulationWindow("Physics Simulation", width, height, scaleX, scaleY, GameWindowFlags.FixedWindow, focusBody);
                game.UpdateFrame += OnUpdateFrame;
                game.Disposed += OnDisposed;
                game.SetView(new CameraView(position: new Vector2(0, 0), 0.05f));
                var physicsDrawer = new DrawPhysics(game);
                // physicsDrawer.AppendFlags(DrawFlags.Aabb);
                physicsDrawer.AppendFlags(DrawFlags.Shape);
                // physicsDrawer.AppendFlags(DrawFlags.Pair);
                physicsDrawer.AppendFlags(DrawFlags.Joint);
                world.SetDebugDraw(physicsDrawer);

                game.VSync = OpenTK.VSyncMode.Off;
                game.Run(60.0, 60.0);
            }));

            windowThread.Start();
        }

        private static void OnUpdateFrame(object sender, EventArgs eventArgs)
        {
            // Prepare for simulation. Typically we use a time step of 1/60 of a
            // second (60Hz) and 10 iterations. This provides a high quality simulation
            // in most game scenarios.
            const float TimeStep = 1.0f / 60.0f;
            const int VelocityIterations = 8;
            const int PositionIterations = 8;

            // Instruct the world to perform a single step of simulation. It is
            // generally best to keep the time step and iterations fixed.
            if ((SimulationWindow.StepNext || !StepByStep) && !SimulationWindow.Paused)
            {
                world?.Step(TimeStep, VelocityIterations, PositionIterations);

                SimulationWindow.StepNext = false;
            }

            world?.DrawDebugData();
        }

        private static void OnDisposed(object sender, EventArgs eventArgs)
        {
            world?.SetDebugDraw(null);
        }
    }
}