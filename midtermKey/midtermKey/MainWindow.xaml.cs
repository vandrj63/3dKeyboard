using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Media.Media3D;
using System.Windows.Threading;

namespace midtermKey
{
    public partial class MainWindow : Window
    {
        private GeometryModel3D mGeometry;
        private bool mDown;
        private Point mLastPos;

        public MainWindow()
        {
            InitializeComponent();

            BuildSolid();
        }

        private void BuildSolid()
        {
            int[] Key = new int[88];

            Key[0] = 0;
            Key[1] = 1;
            Key[2] = 3;
            Key[87] = 4;

            int k;

            for (int i = 3; i < 87; i++)
            {
                int n = i % 12;

                if (n == 3 || n == 8)
                    k = 0;
                else if (n == 5 || n == 10 || n == 0)
                    k = 2;
                else if (n == 7 || n == 2)
                    k = 3;
                else
                    k = 1;

                Key[i] = k;

            }

            MeshGeometry3D mesh = new MeshGeometry3D();

            mGeometry = new GeometryModel3D(mesh, new DiffuseMaterial(Brushes.PaleVioletRed));
            Transform3DGroup t3d = new Transform3DGroup();
            mGeometry.Transform = t3d;

            double x = 0;

            for (int j = 0; j < 88; j++)
            {
                int type = Key[j];

                if (type == 0)
                {
                    mesh = createC(x);
                    x = x + .9;
                    mGeometry = new GeometryModel3D(mesh, new DiffuseMaterial(Brushes.White));
                }
                else if (type == 1)
                {
                    mesh = createSharp(x);
                    x = x + .85;
                    mGeometry = new GeometryModel3D(mesh, new DiffuseMaterial(Brushes.Black));
                }
                else if (type == 2)
                {
                    mesh = createD(x);
                    x = x + .7;
                    mGeometry = new GeometryModel3D(mesh, new DiffuseMaterial(Brushes.White));
                }
                else if (type == 3)
                {
                    mesh = createE(x);
                    x = x + .6;
                    mGeometry = new GeometryModel3D(mesh, new DiffuseMaterial(Brushes.White));
                }
                else if (type == 4)
                {
                    mesh = createLast(x);
                    x = x + .9;
                    mGeometry = new GeometryModel3D(mesh, new DiffuseMaterial(Brushes.White));
                }

                mGeometry.Transform = t3d;
                group.Children.Add(mGeometry);
            }
        }

        private MeshGeometry3D createC(double x)
        {
            MeshGeometry3D mesh = new MeshGeometry3D();

            double l1 = 5.5, l3 = 7;
            double w1 = .8, w3 = 1.2;
            double h = 1;

            mesh.Positions.Add(new Point3D(x, 0, 0));
            mesh.Positions.Add(new Point3D(x,  h, 0));
            mesh.Positions.Add(new Point3D(x + w1, 0, 0));
            mesh.Positions.Add(new Point3D(x + w1,  h, 0));
            mesh.Positions.Add(new Point3D(x, 0,  l1));
            mesh.Positions.Add(new Point3D(x,  h,  l1));
            mesh.Positions.Add(new Point3D(x + w1, 0,  l1));
            mesh.Positions.Add(new Point3D(x + w1,  h,  l1));
            mesh.Positions.Add(new Point3D(x + w3, 0,  l1));
            mesh.Positions.Add(new Point3D(x + w3,  h,  l1));

            mesh.Positions.Add(new Point3D(x, 0,  l3));
            mesh.Positions.Add(new Point3D(x,  h,  l3));
            mesh.Positions.Add(new Point3D(x + w3, 0,  l3));
            mesh.Positions.Add(new Point3D(x + w3,  h,  l3));

            //front side triangles
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(13);
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(13);
            mesh.TriangleIndices.Add(11);

            // right side triangles
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(8);
            mesh.TriangleIndices.Add(9);
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(9);
            mesh.TriangleIndices.Add(13);

            // Right side triangles
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(7);

            // front side triangles
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(5);


            // Back side triangles
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(3);

            // Right side triangles
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(1);

            // top side triangles
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(1);

            // bottom side triangles
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(2);

            // top side triangles
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(13);
            mesh.TriangleIndices.Add(9);
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(9);
            mesh.TriangleIndices.Add(5);

            // back side triangles
            mesh.TriangleIndices.Add(8);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(8);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(9);

            // bottom side triangles
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(8);

            return mesh;
        }
        
        private MeshGeometry3D createSharp(double x)
        {
            MeshGeometry3D mesh = new MeshGeometry3D();

            double l2 = 5.4;
            double w2 = .8;
            double h = 1;

            mesh.Positions.Add(new Point3D(x, .5, 0));
            mesh.Positions.Add(new Point3D(x, .5 + h, 0));
            mesh.Positions.Add(new Point3D(x + w2, .5, 0));
            mesh.Positions.Add(new Point3D(x + w2, .5 + h, 0));
            mesh.Positions.Add(new Point3D(x, .5, l2));
            mesh.Positions.Add(new Point3D(x, .5 + h, l2));
            mesh.Positions.Add(new Point3D(x + w2, .5, l2));
            mesh.Positions.Add(new Point3D(x + w2, .5 + h, l2));

            // Front side triangles
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(5);

            // right side triangles
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(7);

            // back side triangles
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(3);

            // left side triangles
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(1);


            // top side triangles
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(1);

            // bottom side triangles
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(2);

            return mesh;
        }

        private MeshGeometry3D createD(double x)
        {
            MeshGeometry3D mesh = new MeshGeometry3D();

            double l1 = 5.5, l3 = 7;
            double w5 = .7, w6 = .55, w7 = 1.05;
            double h = 1;

            mesh.Positions.Add(new Point3D(x, 0, 0));
            mesh.Positions.Add(new Point3D(x,  h, 0));
            mesh.Positions.Add(new Point3D(x + w5, 0, 0));
            mesh.Positions.Add(new Point3D(x + w5,  h, 0));
            mesh.Positions.Add(new Point3D(x, 0,  l1));
            mesh.Positions.Add(new Point3D(x,  h,  l1));
            mesh.Positions.Add(new Point3D(x + w5, 0,  l1));
            mesh.Positions.Add(new Point3D(x + w5,  h,  l1));
            mesh.Positions.Add(new Point3D(x - w6, 0,  l1));
            mesh.Positions.Add(new Point3D(x - w6,  h,  l1));

            mesh.Positions.Add(new Point3D(x + w7, 0,  l1));
            mesh.Positions.Add(new Point3D(x + w7,  h,  l1));
            mesh.Positions.Add(new Point3D(x - w6, 0,  l3));
            mesh.Positions.Add(new Point3D(x - w6,  h,  l3));
            mesh.Positions.Add(new Point3D(x + w7, 0,  l3));
            mesh.Positions.Add(new Point3D(x + w7,  h,  l3));


            // Front side triangles
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(14);
            mesh.TriangleIndices.Add(15);
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(15);
            mesh.TriangleIndices.Add(13);

            // right side triangles
            mesh.TriangleIndices.Add(14);
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(14);
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(15);

            // back side triangles
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(11);

            // right side triangles
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(7);


            // Back side triangles
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(3);

            // left side triangles
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(1);

            // back side triangles
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(8);
            mesh.TriangleIndices.Add(9);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(9);
            mesh.TriangleIndices.Add(5);

            // left side triangles
            mesh.TriangleIndices.Add(8);
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(13);
            mesh.TriangleIndices.Add(8);
            mesh.TriangleIndices.Add(13);
            mesh.TriangleIndices.Add(9);

            // top side triangles
            mesh.TriangleIndices.Add(13);
            mesh.TriangleIndices.Add(15);
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(13);
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(9);

            // top side triangles
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(1);

            // bottom side triangles
            mesh.TriangleIndices.Add(14);
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(8);
            mesh.TriangleIndices.Add(14);
            mesh.TriangleIndices.Add(8);
            mesh.TriangleIndices.Add(10);

            // bottom side triangles
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(2);

            return mesh;
        }

        private MeshGeometry3D createE(double x)
        {
            MeshGeometry3D mesh = new MeshGeometry3D();

            double l1 = 5.5, l3 = 7;
            double w1 = .6, w4 = .6;
            double h = 1;

            mesh.Positions.Add(new Point3D(x, 0, 0));
            mesh.Positions.Add(new Point3D(x, h, 0));
            mesh.Positions.Add(new Point3D(x + w1, 0, 0));
            mesh.Positions.Add(new Point3D(x + w1,  h, 0));
            mesh.Positions.Add(new Point3D(x - w4, 0,  l1));
            mesh.Positions.Add(new Point3D(x - w4,  h,  l1));
            mesh.Positions.Add(new Point3D(x, 0,  l1));
            mesh.Positions.Add(new Point3D(x,  h,  l1));
            mesh.Positions.Add(new Point3D(x + w1, 0,  l1));
            mesh.Positions.Add(new Point3D(x + w1,  h,  l1));

            mesh.Positions.Add(new Point3D(x - w4, 0,  l3));
            mesh.Positions.Add(new Point3D(x - w4,  h,  l3));
            mesh.Positions.Add(new Point3D(x + w1, 0,  l3));
            mesh.Positions.Add(new Point3D(x + w1,  h,  l3));

            // Front side triangles
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(13);
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(13);
            mesh.TriangleIndices.Add(11);

            // right side triangles
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(13);

            // left side triangles
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(1);

            // left side triangles
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(5);


            // Back side triangles
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(3);

            // back side triangles
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(7);

            // top side triangles
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(9);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(1);

            // bottom side triangles
            mesh.TriangleIndices.Add(8);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(8);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(2);

            // top side triangles
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(13);
            mesh.TriangleIndices.Add(9);
            mesh.TriangleIndices.Add(11);
            mesh.TriangleIndices.Add(9);
            mesh.TriangleIndices.Add(5);

            // back side triangles
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(7);

            // bottom side triangles
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(10);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(12);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(8);

            return mesh;
        }

        private MeshGeometry3D createLast(double x)
        {
            MeshGeometry3D mesh = new MeshGeometry3D();
            double l2 = 7;
            double w2 = 1.1;
            double h = 1;

            mesh.Positions.Add(new Point3D(x, 0, 0));
            mesh.Positions.Add(new Point3D(x,  h, 0));
            mesh.Positions.Add(new Point3D(x + w2, 0, 0));
            mesh.Positions.Add(new Point3D(x + w2,  h, 0));
            mesh.Positions.Add(new Point3D(x, 0,  l2));
            mesh.Positions.Add(new Point3D(x,  h,  l2));
            mesh.Positions.Add(new Point3D(x + w2, 0,  l2));
            mesh.Positions.Add(new Point3D(x + w2,  h,  l2));

            // Front side triangles
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(5);

            // right side triangles
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(7);

            // back side triangles
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(2);
            mesh.TriangleIndices.Add(1);
            mesh.TriangleIndices.Add(3);

            // left side triangles
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(1);

            // top side triangles
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(7);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(5);
            mesh.TriangleIndices.Add(3);
            mesh.TriangleIndices.Add(1);

            // bottom side triangles
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(4);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(6);
            mesh.TriangleIndices.Add(0);
            mesh.TriangleIndices.Add(2);
            return mesh;
        }

        static Point3D GetPosition(double theta, double phi, double radius, double offsetx = -4, double offsety = -4, double offsetz = -4)
        {
            double x = radius * Math.Sin(theta) * Math.Sin(phi) + offsetx;
            double y = radius * Math.Cos(phi) + offsety;
            double z = radius * Math.Cos(theta) * Math.Sin(phi) + offsetz;

            return new Point3D(x, y, z);
        }

        private static Point GetTextureCoordinate(double theta, double phi)
        {
            Point p = new Point(theta / (2 * Math.PI),
                                phi / (Math.PI));

            return p;
        }

        private static double DegToRad(double degrees)
        {
            return (degrees / 180.0) * Math.PI;
        }

        private void Grid_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            camera.Position = new Point3D(camera.Position.X, camera.Position.Y, camera.Position.Z - e.Delta / 250D);
        }

        private void Grid_MouseMove(object sender, MouseEventArgs e)
        {
            if (mDown)
            {
                Point pos = Mouse.GetPosition(viewport);
                Point actualPos = new Point(pos.X - viewport.ActualWidth / 2, viewport.ActualHeight / 2 - pos.Y);
                double dx = actualPos.X - mLastPos.X, dy = actualPos.Y - mLastPos.Y;

                double mouseAngle = 0;
                if (dx != 0 && dy != 0)
                {
                    mouseAngle = Math.Asin(Math.Abs(dy) / Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2)));
                    if (dx < 0 && dy > 0) mouseAngle += Math.PI / 2;
                    else if (dx < 0 && dy < 0) mouseAngle += Math.PI;
                    else if (dx > 0 && dy < 0) mouseAngle += Math.PI * 1.5;
                }
                else if (dx == 0 && dy != 0) mouseAngle = Math.Sign(dy) > 0 ? Math.PI / 2 : Math.PI * 1.5;
                else if (dx != 0 && dy == 0) mouseAngle = Math.Sign(dx) > 0 ? 0 : Math.PI;

                double axisAngle = mouseAngle + Math.PI / 2;

                Vector3D axis = new Vector3D(Math.Cos(axisAngle) * 4, Math.Sin(axisAngle) * 4, 0);

                double rotation = 0.01 * Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2));

                Transform3DGroup group = mGeometry.Transform as Transform3DGroup;
                QuaternionRotation3D r = new QuaternionRotation3D(new Quaternion(axis, rotation * 180 / Math.PI));
                group.Children.Add(new RotateTransform3D(r));

                mLastPos = actualPos;
            }
        }

        private void Grid_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.LeftButton != MouseButtonState.Pressed) return;
            mDown = true;
            Point pos = Mouse.GetPosition(viewport);
            mLastPos = new Point(pos.X - viewport.ActualWidth / 2, viewport.ActualHeight / 2 - pos.Y);
        }

        private void Grid_MouseUp(object sender, MouseButtonEventArgs e)
        {
            mDown = false;
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            DispatcherTimer dispatcherTimer = new DispatcherTimer();

            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);

            dispatcherTimer.Interval = new TimeSpan(0, 0, 1);

            // dispatcherTimer.Start();

            testangel = 0;
        }
        double testangel = 0;
        double rotation = 0;

        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            double axisAngle = testangel + Math.PI / 2;

            Vector3D axis = new Vector3D(Math.Cos(axisAngle) * 4, Math.Sin(axisAngle) * 4, 0);

            //double rotation = 0.01 * Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2));

            rotation += 5;

            Transform3DGroup group = mGeometry.Transform as Transform3DGroup;

            QuaternionRotation3D r = new QuaternionRotation3D(new Quaternion(axis, rotation * 180 / Math.PI));

            group.Children.Add(new RotateTransform3D(r));

            //mLastPos = actualPos;

        }
    }

}
