using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Aperiodic
{
    public class DeflationRuleK30 : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the DeflationRuleK30 class.
        /// </summary>
        public DeflationRuleK30()
          : base("DeflationRuleK30", "DefK30",
              "Output planes corresponding the the deflation rules for the K30 tile (additionally output meshes and points)",
              "Aperiodic", "Aperiodic")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("referenceMeshes", "ref", "Reference four golden zonohedra meshes", GH_ParamAccess.tree);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("outputMeshes", "outMeshes", "Output meshes after applying deflation rule K30", GH_ParamAccess.tree);
            pManager.AddPointParameter("outputPoints", "outPts", "Output points after applying deflation rule K30", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("outputPlanes", "outPlanes", "Output planes after applying deflation rule K30", GH_ParamAccess.tree);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Declare variables
            GH_Structure<GH_Mesh> outputmeshes = new GH_Structure<GH_Mesh>();
            GH_Structure<GH_Point> outputpts = new GH_Structure<GH_Point>();
            GH_Structure<GH_Plane> outputplns = new GH_Structure<GH_Plane>();

            GH_Structure<GH_Mesh> reference = new GH_Structure<GH_Mesh>();
            if (!DA.GetDataTree<GH_Mesh>(0, out reference)) { return; }

            GH_Path pth0 = new GH_Path(0);
            GH_Path pth1 = new GH_Path(1);
            GH_Path pth2 = new GH_Path(2);
            GH_Path pth3 = new GH_Path(3);

            // Get height references from original input meshes
            double a6HeightRef = GetMeshHeight(reference.Branches[0][0].Value);
            double b12HeightRef = GetMeshHeight(reference.Branches[1][0].Value);
            double f20HeightRef = GetMeshHeight(reference.Branches[2][0].Value);
            double k30HeightRef = GetMeshHeight(reference.Branches[3][0].Value);

            // Note: always duplicate these before modifying
            Mesh refA6 = reference.Branches[0][0].Value.DuplicateMesh();
            Mesh refB12 = reference.Branches[1][0].Value.DuplicateMesh();
            Mesh refF20 = reference.Branches[2][0].Value.DuplicateMesh();
            Mesh refK30 = reference.Branches[3][0].Value.DuplicateMesh();

            // Translate reference meshes so their base sits on WorldXY plane
            TranslateToWorldXY(refA6);
            TranslateToWorldXY(refB12);
            TranslateToWorldXY(refF20);
            TranslateToWorldXY(refK30);

            Mesh mesh = refK30.DuplicateMesh();
            Point3d basept = Point3d.Origin;
            Plane basepln = Plane.WorldXY;

            // Scale center point of geometry by a factor of golden ratio^3
            double goldenRatio = (1 + Math.Sqrt(5)) / 2;
            double deflationScaleFactor = Math.Pow(goldenRatio, 3);

            // Get the centroid of the geometry
            AreaMassProperties ampMesh = AreaMassProperties.Compute(mesh);
            Point3d centroidMesh = ampMesh.Centroid;
            // Create a vector using the centroid location
            Vector3d centroidVec = new Vector3d(centroidMesh);
            // Scale the vector by the deflationScalefactor to get the new centroid location
            // Subtract the original centroidVec to get the translation vector
            Vector3d inflateVec = centroidVec * deflationScaleFactor - centroidVec;
            // Translate the brep to the new scaled location (but without scaling the brep itself, so the unit size remains the same)
            mesh.Translate(inflateVec);
            // Translate the basept to the new scaled location - consistent with the brep itself, so further operations on the base pts can recurse well
            basept += inflateVec;
            Transform scaleInflate = Transform.Scale(Point3d.Origin, deflationScaleFactor);
            basepln.Transform(scaleInflate); // TODO: check if only translation is enough here for baseplns
            basepln.Translate(inflateVec);

            // Set up smaller output lists
            List<Mesh> listA6 = new List<Mesh>();
            List<Mesh> listB12 = new List<Mesh>();
            List<Mesh> listF20 = new List<Mesh>();
            List<Mesh> listK30 = new List<Mesh>();
            List<Point3d> ptsA6 = new List<Point3d>();
            List<Point3d> ptsB12 = new List<Point3d>();
            List<Point3d> ptsF20 = new List<Point3d>();
            List<Point3d> ptsK30 = new List<Point3d>();
            List<Plane> plnsA6 = new List<Plane>();
            List<Plane> plnsB12 = new List<Plane>();
            List<Plane> plnsF20 = new List<Plane>();
            List<Plane> plnsK30 = new List<Plane>();

            // TODO: move fixed reference value calculations outside of the recursion (pass to inner methods in a dictionary?)
            // Set up deflation scale factor

            // Get angle reference
            double rhombAcuteAngle = 2 * Math.Atan(1 / goldenRatio);

            // Get length reference (edge length of original triacontahedron)
            //double edgeLengthRef = refK30.Edges[0].PointAtEnd.DistanceTo(refK30.Edges[0].PointAtStart); //brep code
            double edgeLengthRef = refK30.TopologyEdges.EdgeLine(0).Length;

            // TODO: can we hardcode the location of these points / planes / hardcode the data for the mesh references? these would be fixed inside the component instead of as inputs (the inputs would be geometry to transform according to the 4 types)

            #region calculate base planes for later transformations of the cells

            #region calculate a6 base plane for later transformation
            // Set up base orientation for A6 transformation later in step (h)
            // (Alternatively we could change the base position of the refA6 geometry but this might mean rewriting everything)
            // Get base plane for orientation transform using refA6 leftmost vertex as center and adjacent edges below it
            Point3d leftmostVertex = new Point3d();
            int leftmostVertexIndex = 0;
            double minX = 0;
            for (int i = 0; i < refA6.TopologyVertices.Count; i++)
            {
                double currentX = refA6.TopologyVertices[i].X;
                if (currentX < minX)
                {
                    leftmostVertex = refA6.TopologyVertices[i];
                    leftmostVertexIndex = i;
                    minX = currentX;
                }
            }
            Point3d baseCenter = leftmostVertex;

            // Get adjacent vertex points
            int[] adjacentVertexIndices = refA6.Vertices.GetConnectedVertices(leftmostVertexIndex);
            List<Point3d> basea6pts = new List<Point3d>();
            for (int i = 0; i < 3; i++)
            {
                Point3d possibleEdgePt = refA6.TopologyVertices[adjacentVertexIndices[i]];
                if (possibleEdgePt.X < -0.0001) // We only want two of the vertices, the ones not at x = 0, y = 0
                {
                    basea6pts.Add(possibleEdgePt);
                }
            }

            // Order edgepts to set up the plane a6base
            Plane a6base;
            if (basea6pts[0].Y < basea6pts[1].Y)
            {
                a6base = new Plane(baseCenter, basea6pts[0], basea6pts[1]);
            }
            else
            {
                a6base = new Plane(baseCenter, basea6pts[1], basea6pts[0]);
            }
            #endregion

            // Set up base orientation for F20 transformation later in step (k30-i)
            Plane f20base = new Plane(Point3d.Origin, -Vector3d.XAxis, -Vector3d.YAxis);

            // Set up base orientation for K30 transformation in step (a6-000)
            Point3d k30basecenter = mesh.TopologyVertices[GetClosestVertex(mesh, new Point3d(0, -1, 0))];
            Point3d k30basexaxis = mesh.TopologyVertices[GetClosestVertex(mesh, new Point3d(1, 0, 0))];
            Point3d k30baseyaxis = new Point3d(-k30basexaxis.X, 0, 0);
            Plane k30base = new Plane(k30basecenter, k30basexaxis, k30baseyaxis);

            // Set up base orientation for B12 transformation in step (a6-000)
            Point3d b12basecenter = refB12.TopologyVertices[GetClosestVertex(refB12, new Point3d(-0.5, 0, 0))];
            Point3d b12baseyaxis = refB12.TopologyVertices[GetClosestVertex(refB12, new Point3d(-0.5, -0.5, 1))];
            Point3d b12basexaxis = new Point3d(b12baseyaxis.X, -b12baseyaxis.Y, b12baseyaxis.Z);
            Plane b12base = new Plane(b12basecenter, b12basexaxis, b12baseyaxis);

            #endregion

            //
            // This is K30 rhombic triacontahedron
            // Begin deflation for K30

            // Add the central triacontahedron
            listK30.Add(mesh);

            // Get centroid
            AreaMassProperties ampK30 = AreaMassProperties.Compute(mesh);
            Point3d centroidK30 = ampK30.Centroid;

            // Add to basepts list
            ptsK30.Add(centroidK30);

            // Add to baseplns list
            mesh.FaceNormals.ComputeFaceNormals();
            Plane plnK30 = GetOrientedPlaneFromRhombicFace(mesh, 0, -mesh.FaceNormals[0]);
            plnsK30.Add(plnK30);

            // Get 2-fold rotational axes on faces of triacontahedron
            for (int i = 0; i < mesh.Faces.Count; i++)
            {
                // Get center and normal vector of the current face
                Point3d centerFace = mesh.Faces.GetFaceCenter(i);
                Vector3d normalFace = centerFace - centroidK30;
                Plane facePlane = GetOrientedPlaneFromRhombicFace(mesh, i, normalFace);

                // Transform B12 to all 30 faces (step (b) of the deflation)
                Mesh copyb = refB12.DuplicateMesh();
                Transform xform = Transform.PlaneToPlane(Plane.WorldXY, facePlane);
                copyb.Transform(xform);

                // Add to mesh list
                listB12.Add(copyb);

                // Transform the basepoint from the origin
                Point3d bbasept = Point3d.Origin;
                bbasept.Transform(xform);

                // Add to basept list
                ptsB12.Add(bbasept);

                // Add to basepln list
                plnsB12.Add(facePlane);

                // Transform K30 further along the same axes (step (e) of the deflation)
                // First push the plane further out
                Vector3d pushPlane = new Vector3d(normalFace);
                pushPlane.Unitize();
                Transform xscale = Transform.Scale(Point3d.Origin, b12HeightRef);
                pushPlane.Transform(xscale);
                facePlane.Translate(pushPlane);
                Mesh e = mesh.DuplicateMesh();
                Transform xforme = Transform.PlaneToPlane(plnK30, facePlane);
                e.Transform(xforme);

                // Add to mesh list
                listK30.Add(e);

                // Transform the basepoint from the origin
                Point3d ebasept = Point3d.Origin;
                ebasept.Transform(xforme);

                // Add to basepts list
                ptsK30.Add(ebasept);

                // Add to baseplns list
                plnsK30.Add(facePlane);
            }

            // Get 3-fold rotational axes from vertices of triacontahedron with 3 neighbors
            List<Point3d> threeFoldAxesPoints = new List<Point3d>();
            List<Point3d> threeFoldAxesPointsOrientations = new List<Point3d>();
            // Get 5-fold rotational axes from vertices of triacontahedron with 5 neighbors
            List<Point3d> fiveFoldAxesPoints = new List<Point3d>();
            List<Point3d> fiveFoldAxesPointsOrientations = new List<Point3d>();

            // Loop through mesh vertices
            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {

                // Vertices connected to 3 edges
                if (mesh.TopologyVertices.ConnectedEdgesCount(i) == 3)
                {
                    threeFoldAxesPoints.Add(mesh.TopologyVertices[i]);
                    // Get one of the adjacent edges and the end point (that is not the same vertex)
                    int orientPtIndex = mesh.TopologyVertices.ConnectedTopologyVertices(i)[0];
                    Point3d orientPt = mesh.TopologyVertices[orientPtIndex];

                    // Save this point for orientation
                    threeFoldAxesPointsOrientations.Add(orientPt);
                }

                // Vertices connected to 5 edges
                if (mesh.TopologyVertices.ConnectedEdgesCount(i) == 5)
                {
                    fiveFoldAxesPoints.Add(mesh.TopologyVertices[i]);
                    // Get one of the adjacent edges and the end point (that is not the same vertex)
                    int orientPtIndex5 = mesh.Vertices.GetConnectedVertices(i)[0];
                    Point3d orientPt5 = mesh.Vertices[orientPtIndex5];

                    // Save this point for orientation
                    fiveFoldAxesPointsOrientations.Add(orientPt5);
                }
            }

            // Loop through 3-fold axes ends
            for (int i = 0; i < threeFoldAxesPoints.Count; i++)
            {
                // Use the plane center and the orientation point
                Point3d planeCenter = (Point3d)threeFoldAxesPoints[i];
                Point3d orientX = (Point3d)threeFoldAxesPointsOrientations[i];

                // First create a plane using center and normal vector
                Vector3d normal = planeCenter - centroidK30;
                Plane threeFoldPlaneUnoriented = new Plane(planeCenter, normal);

                // Project orientX point onto that plane
                Transform projectX = Transform.PlanarProjection(threeFoldPlaneUnoriented);
                orientX.Transform(projectX);

                // Rotate orientX point by 90 degress (angle is arbitrary) on that plane to get orientY point
                Transform rotate90 = Transform.Rotation(Math.PI / 2, normal, planeCenter);
                Point3d orientY = new Point3d(orientX);
                orientY.Transform(rotate90);

                // Get the final oriented plane for transformation
                Plane threeFoldPlaneOriented = new Plane(planeCenter, orientX, orientY);

                // Transform A6 to all 20 3-fold rotational axes sides (step (c) of the deflation)
                Transform xformThree = Transform.PlaneToPlane(Plane.WorldXY, threeFoldPlaneOriented);
                Mesh copyc = refA6.DuplicateMesh();
                copyc.Transform(xformThree);

                // Add to mesh list
                listA6.Add(copyc);

                // Tranform basept
                Point3d cbasept = Point3d.Origin;
                cbasept.Transform(xformThree);

                // Add to basepts list
                ptsA6.Add(cbasept);

                // Add to baseplns list
                plnsA6.Add(threeFoldPlaneOriented);

                // Copy A6 6 more times around this one... (step (d) but in a different way from article)
                // Get furthest vertex of the copy
                int furthestVertexIndex = GetFurthestVertex(copyc, (Point3d)centroidK30);
                Point3d furthestVertexPt = (Point3d)copyc.TopologyVertices[furthestVertexIndex];

                // Get adjacent vertex points
                adjacentVertexIndices = copyc.Vertices.GetConnectedVertices(furthestVertexIndex);
                List<Point3d> edgePoints = new List<Point3d>();
                for (int j = 0; j < 3; j++)
                {
                    edgePoints.Add((Point3d)copyc.Vertices[adjacentVertexIndices[j]]);
                }

                // Get mirror planes
                Plane plane0 = new Plane(furthestVertexPt, edgePoints[1], edgePoints[2]);
                Plane plane1 = new Plane(furthestVertexPt, edgePoints[2], edgePoints[0]);
                Plane plane2 = new Plane(furthestVertexPt, edgePoints[0], edgePoints[1]);
                Transform xform0 = Transform.Mirror(plane0);
                Transform xform1 = Transform.Mirror(plane1);
                Transform xform2 = Transform.Mirror(plane2);

                // Make copies
                Mesh copyc0 = copyc.DuplicateMesh();
                copyc0.Transform(xform0);
                listA6.Add(copyc0);
                Mesh copyc1 = copyc.DuplicateMesh();
                copyc1.Transform(xform1);
                listA6.Add(copyc1);
                Mesh copyc2 = copyc.DuplicateMesh();
                copyc2.Transform(xform2);
                listA6.Add(copyc2);

                // Copy base pts
                Point3d c0basept = new Point3d(cbasept);
                c0basept.Transform(xform0);
                ptsA6.Add(c0basept);
                Point3d c1basept = new Point3d(cbasept);
                c1basept.Transform(xform1);
                ptsA6.Add(c1basept);
                Point3d c2basept = new Point3d(cbasept);
                c2basept.Transform(xform2);
                ptsA6.Add(c2basept);

                // Copy base plns
                Plane c0basepln = new Plane(threeFoldPlaneOriented);
                Plane c1basepln = new Plane(threeFoldPlaneOriented);
                Plane c2basepln = new Plane(threeFoldPlaneOriented);
                c0basepln.Transform(xform0);
                c1basepln.Transform(xform1);
                c2basepln.Transform(xform2);
                c0basepln.Flip();
                c1basepln.Flip();
                c2basepln.Flip();
                c0basepln.Rotate(Math.PI / 2, c0basepln.Normal);
                c1basepln.Rotate(Math.PI / 2, c1basepln.Normal);
                c2basepln.Rotate(Math.PI / 2, c2basepln.Normal);
                plnsA6.Add(c0basepln);
                plnsA6.Add(c1basepln);
                plnsA6.Add(c2basepln);

                // Get further set of mirror planes
                Vector3d shift0 = furthestVertexPt - edgePoints[0];
                Vector3d shift1 = furthestVertexPt - edgePoints[1];
                Vector3d shift2 = furthestVertexPt - edgePoints[2];
                plane0.Translate(shift0);
                plane1.Translate(shift1);
                plane2.Translate(shift2);
                Transform xform0d = Transform.Mirror(plane0);
                Transform xform1d = Transform.Mirror(plane1);
                Transform xform2d = Transform.Mirror(plane2);

                // Make copies (last part of step (d))
                Mesh copyd0 = copyc0.DuplicateMesh();
                copyd0.Transform(xform0d);
                listA6.Add(copyd0);
                Mesh copyd1 = copyc1.DuplicateMesh();
                copyd1.Transform(xform1d);
                listA6.Add(copyd1);
                Mesh copyd2 = copyc2.DuplicateMesh();
                copyd2.Transform(xform2d);
                listA6.Add(copyd2);

                // Copy base pts (note - these base pts actually don't move because they are on the mirror plane)
                Point3d d0basept = new Point3d(c0basept);
                d0basept.Transform(xform0d);
                ptsA6.Add(d0basept);
                Point3d d1basept = new Point3d(c1basept);
                d1basept.Transform(xform1d);
                ptsA6.Add(d1basept);
                Point3d d2basept = new Point3d(c2basept);
                d2basept.Transform(xform2d);
                ptsA6.Add(d2basept);

                // Copy and add to baseplns list
                Plane d0basepln = new Plane(c0basepln);
                Plane d1basepln = new Plane(c1basepln);
                Plane d2basepln = new Plane(c2basepln);
                d0basepln.Transform(xform0d);
                d1basepln.Transform(xform1d);
                d2basepln.Transform(xform2d);
                d0basepln.Flip();
                d1basepln.Flip();
                d2basepln.Flip();
                d0basepln.Rotate(Math.PI / 2, d0basepln.Normal);
                d1basepln.Rotate(Math.PI / 2, d1basepln.Normal);
                d2basepln.Rotate(Math.PI / 2, d2basepln.Normal);
                plnsA6.Add(d0basepln);
                plnsA6.Add(d1basepln);
                plnsA6.Add(d2basepln);

                // Beginning of step (g) - mirror the rhombohedra on these axes even further out, and rotate 180
                // Get mirror plane from normal plane
                Plane planeg = new Plane(furthestVertexPt, normal);
                Transform xformg = Transform.Mirror(planeg);
                Mesh copyg = copyc.DuplicateMesh();
                copyg.Transform(xformg);
                Transform xrot180 = Transform.Rotation(Math.PI, normal, furthestVertexPt);
                copyg.Transform(xrot180);

                // Add to mesh list
                listA6.Add(copyg);

                // Copy base pt
                Point3d gbasept = new Point3d(cbasept);
                gbasept.Transform(xformg);

                // Add to basepts list
                ptsA6.Add(gbasept);

                // Add to baseplns list
                Plane gbasepln = new Plane(threeFoldPlaneOriented);
                gbasepln.Transform(xformg);
                gbasepln.Rotate(Math.PI / 2, gbasepln.Normal);
                gbasepln.Flip();
                plnsA6.Add(gbasepln);

                // Next we'll mirror these ones 3 more times on the outer faces using the method above
                // Get furthest vertex of the copy
                furthestVertexIndex = GetFurthestVertex(copyg, (Point3d)centroidK30);
                furthestVertexPt = copyg.TopologyVertices[furthestVertexIndex];

                // Get adjacent vertex points
                adjacentVertexIndices = copyg.Vertices.GetConnectedVertices(furthestVertexIndex);
                edgePoints = new List<Point3d>();
                for (int j = 0; j < 3; j++)
                {
                    edgePoints.Add((Point3d)copyg.Vertices[adjacentVertexIndices[j]]);
                }

                // Get mirror planes
                plane0 = new Plane(furthestVertexPt, edgePoints[1], edgePoints[2]);
                plane1 = new Plane(furthestVertexPt, edgePoints[2], edgePoints[0]);
                plane2 = new Plane(furthestVertexPt, edgePoints[0], edgePoints[1]);
                xform0 = Transform.Mirror(plane0);
                xform1 = Transform.Mirror(plane1);
                xform2 = Transform.Mirror(plane2);

                // Make copies
                Mesh copyg0 = copyg.DuplicateMesh();
                copyg0.Transform(xform0);
                listA6.Add(copyg0);
                Mesh copyg1 = copyg.DuplicateMesh();
                copyg1.Transform(xform1);
                listA6.Add(copyg1);
                Mesh copyg2 = copyg.DuplicateMesh();
                copyg2.Transform(xform2);
                listA6.Add(copyg2);

                // Copy base pts (note - these base pts actually don't move because they are on the mirror plane)
                Point3d g0basept = new Point3d(gbasept);
                g0basept.Transform(xform0);
                ptsA6.Add(g0basept);
                Point3d g1basept = new Point3d(gbasept);
                g1basept.Transform(xform1);
                ptsA6.Add(g1basept);
                Point3d g2basept = new Point3d(gbasept);
                g2basept.Transform(xform2);
                ptsA6.Add(g2basept);

                // Copy baseplns and add to baseplns list
                Plane g0basepln = new Plane(gbasepln);
                Plane g1basepln = new Plane(gbasepln);
                Plane g2basepln = new Plane(gbasepln);
                g0basepln.Transform(xform0);
                g1basepln.Transform(xform1);
                g2basepln.Transform(xform2);
                g0basepln.Flip();
                g1basepln.Flip();
                g2basepln.Flip();
                g0basepln.Rotate(Math.PI / 2, g0basepln.Normal);
                g1basepln.Rotate(Math.PI / 2, g1basepln.Normal);
                g2basepln.Rotate(Math.PI / 2, g2basepln.Normal);
                plnsA6.Add(g0basepln);
                plnsA6.Add(g1basepln);
                plnsA6.Add(g2basepln);

                // Finally for each of these 3 copies we want to mirror again 2 more times (using the 2 most outer faces of each)
                // Or we just mirror the 2 and then rotate around the 3-fold axis
                Mesh copyg00 = copyg0.DuplicateMesh();
                Mesh copyg01 = copyg0.DuplicateMesh();

                // Again, get furthest vertex/edges
                furthestVertexIndex = GetFurthestVertex(copyg0, (Point3d)centroidK30);

                // Get adjacent vertex points
                adjacentVertexIndices = copyg0.Vertices.GetConnectedVertices(furthestVertexIndex);
                edgePoints = new List<Point3d>();
                for (int j = 0; j < 3; j++)
                {
                    edgePoints.Add((Point3d)copyg0.Vertices[adjacentVertexIndices[j]]);
                }

                // Identify furthest edgepoint - we need the two mirror planes adjacent to it
                List<Point3d> sortedEdgePts = new List<Point3d>();
                double test0 = edgePoints[0].DistanceTo(centroidK30);
                double test1 = edgePoints[1].DistanceTo(centroidK30);
                if (test0 > test1)
                {
                    sortedEdgePts.Add(edgePoints[0]);
                    sortedEdgePts.Add(edgePoints[1]);
                    sortedEdgePts.Add(edgePoints[2]);
                }
                else if (test0 < test1)
                {
                    sortedEdgePts.Add(edgePoints[1]);
                    sortedEdgePts.Add(edgePoints[0]);
                    sortedEdgePts.Add(edgePoints[2]);
                }
                else if (test0 == test1)
                {
                    sortedEdgePts.Add(edgePoints[2]);
                    sortedEdgePts.Add(edgePoints[0]);
                    sortedEdgePts.Add(edgePoints[1]);
                }

                // Get mirror planes
                plane0 = new Plane(furthestVertexPt, sortedEdgePts[0], sortedEdgePts[1]);
                plane1 = new Plane(furthestVertexPt, sortedEdgePts[0], sortedEdgePts[2]);
                xform0 = Transform.Mirror(plane0);
                xform1 = Transform.Mirror(plane1);

                // Transform the copies and add to mesh list
                copyg00.Transform(xform0);
                listA6.Add(copyg00);
                copyg01.Transform(xform1);
                listA6.Add(copyg01);

                // Transform base pts (note - these base pts actually don't move because they are on the mirror plane)
                Point3d g00basept = new Point3d(g0basept);
                g00basept.Transform(xform0);
                ptsA6.Add(g00basept);
                Point3d g01basept = new Point3d(g0basept);
                g00basept.Transform(xform1);
                ptsA6.Add(g01basept);

                // Transform the baseplns and add to baseplns list
                Plane g00basepln = new Plane(g0basepln);
                Plane g01basepln = new Plane(g0basepln);
                g00basepln.Transform(xform0);
                g01basepln.Transform(xform1);
                g00basepln.Flip();
                g01basepln.Flip();
                g00basepln.Rotate(Math.PI / 2, g00basepln.Normal);
                g01basepln.Rotate(Math.PI / 2, g01basepln.Normal);
                plnsA6.Add(g00basepln);
                plnsA6.Add(g01basepln);

                // Finally copy and rotate to make 4 more (this completes step (g) of the deflation)
                Mesh copyg10 = copyg00.DuplicateMesh();
                Mesh copyg11 = copyg01.DuplicateMesh();
                Mesh copyg20 = copyg00.DuplicateMesh();
                Mesh copyg21 = copyg01.DuplicateMesh();

                Transform rotate120 = Transform.Rotation(2 * Math.PI / 3, normal, planeCenter);
                Transform rotate240 = Transform.Rotation(4 * Math.PI / 3, normal, planeCenter);
                copyg10.Transform(rotate120);
                copyg11.Transform(rotate120);
                copyg20.Transform(rotate240);
                copyg21.Transform(rotate240);
                listA6.Add(copyg10);
                listA6.Add(copyg11);
                listA6.Add(copyg20);
                listA6.Add(copyg21);

                // Copy base pts for this last step (note - these base pts actually don't move because they are on the mirror plane)
                Point3d g10basept = new Point3d(g00basept);
                Point3d g11basept = new Point3d(g01basept);
                Point3d g20basept = new Point3d(g00basept);
                Point3d g21basept = new Point3d(g01basept);
                g10basept.Transform(rotate120);
                g11basept.Transform(rotate120);
                g20basept.Transform(rotate240);
                g21basept.Transform(rotate240);
                ptsA6.Add(g10basept);
                ptsA6.Add(g11basept);
                ptsA6.Add(g20basept);
                ptsA6.Add(g21basept);

                // Copy baseplns
                Plane g10basepln = new Plane(g00basepln);
                Plane g11basepln = new Plane(g01basepln);
                Plane g20basepln = new Plane(g00basepln);
                Plane g21basepln = new Plane(g01basepln);
                g10basepln.Transform(rotate120);
                g11basepln.Transform(rotate120);
                g20basepln.Transform(rotate240);
                g21basepln.Transform(rotate240);
                plnsA6.Add(g10basepln);
                plnsA6.Add(g11basepln);
                plnsA6.Add(g20basepln);
                plnsA6.Add(g21basepln);
            }


            // Loop through 5-fold axes ends
            for (int i = 0; i < fiveFoldAxesPoints.Count; i++)
            {
                // Use the plane center and the orientation point
                Point3d planeCenter5 = fiveFoldAxesPoints[i];
                Point3d orientX5 = fiveFoldAxesPointsOrientations[i];

                // First create a plane using center and normal vector
                Vector3d normal5 = planeCenter5 - centroidK30;
                Plane fiveFoldPlaneUnoriented = new Plane(planeCenter5, normal5);

                // Project orientX point onto that plane
                Transform projectX5 = Transform.PlanarProjection(fiveFoldPlaneUnoriented);
                orientX5.Transform(projectX5);

                // Rotate orientX point by 90 degress (angle is arbitrary) on that plane to get orientY point
                Transform rotate90 = Transform.Rotation(Math.PI / 2, normal5, planeCenter5);
                Point3d orientY5 = new Point3d(orientX5);
                orientY5.Transform(rotate90);

                // Get the oriented plane for transformation
                Plane fiveFoldPlaneOriented = new Plane(planeCenter5, orientX5, orientY5);

                // Push the plane outwards by a multiple of the edge length
                Vector3d pushPlane5 = new Vector3d(normal5);
                pushPlane5.Unitize();
                Transform xscale5 = Transform.Scale(Point3d.Origin, edgeLengthRef);
                pushPlane5.Transform(xscale5);
                fiveFoldPlaneOriented.Translate(pushPlane5 * 2);

                // Transform F20 to all 12 5-fold rotational axes sides (step (f) of the deflation)
                Transform xformFive = Transform.PlaneToPlane(Plane.WorldXY, fiveFoldPlaneOriented);
                Mesh copyf = refF20.DuplicateMesh();
                copyf.Transform(xformFive);

                // Add to mesh list
                listF20.Add(copyf);

                // Get furthest vertex
                int furthestVertexIndex = GetFurthestVertex(copyf, (Point3d)centroidK30);
                Point3d furthestVertexPt = (Point3d)copyf.TopologyVertices[furthestVertexIndex];
                Point3d baseptf = furthestVertexPt;

                // Add to basepts list
                ptsF20.Add(baseptf);

                // Add to baseplns list
                Plane fiveFoldPlaneOrientedflip = new Plane(fiveFoldPlaneOriented);
                Vector3d pushPlane5b = new Vector3d(pushPlane5);
                pushPlane5b.Unitize();
                pushPlane5b *= f20HeightRef;
                fiveFoldPlaneOrientedflip.Flip();
                fiveFoldPlaneOrientedflip.Rotate(-Math.PI / 2, fiveFoldPlaneOrientedflip.Normal);
                fiveFoldPlaneOrientedflip.Translate(pushPlane5b);
                plnsF20.Add(fiveFoldPlaneOrientedflip);

                // Step (h) of the inflation requires capping each of the F20 by clusters of 5 rhombohedra
                // Using copyf, we get the 5 outermost faces and transform the rhombohedra to each one

                // Get one adjacent vertex from furthest vertex
                adjacentVertexIndices = copyf.Vertices.GetConnectedVertices(furthestVertexIndex);
                Point3d edgept = (Point3d)copyf.Vertices[adjacentVertexIndices[0]];

                // Start with one edge, then rotate around the plane to get the others in order
                List<Point3d> edgePoints = new List<Point3d>();
                edgePoints.Add(edgept);
                for (int j = 1; j < 5; j++)
                {
                    // Rotate edgept around normal 2pi/5 degrees
                    Transform xrot5 = Transform.Rotation(2 * Math.PI / 5, normal5, planeCenter5);
                    edgept.Transform(xrot5);
                    edgePoints.Add(edgept);
                }

                // Get planes
                Plane planeh0 = new Plane(furthestVertexPt, edgePoints[0], edgePoints[1]);
                Plane planeh1 = new Plane(furthestVertexPt, edgePoints[1], edgePoints[2]);
                Plane planeh2 = new Plane(furthestVertexPt, edgePoints[2], edgePoints[3]);
                Plane planeh3 = new Plane(furthestVertexPt, edgePoints[3], edgePoints[4]);
                Plane planeh4 = new Plane(furthestVertexPt, edgePoints[4], edgePoints[0]);

                // Make tranformations
                Transform xh0 = Transform.PlaneToPlane(a6base, planeh0);
                Transform xh1 = Transform.PlaneToPlane(a6base, planeh1);
                Transform xh2 = Transform.PlaneToPlane(a6base, planeh2);
                Transform xh3 = Transform.PlaneToPlane(a6base, planeh3);
                Transform xh4 = Transform.PlaneToPlane(a6base, planeh4);

                // Make copies and transform
                Mesh copyh0 = refA6.DuplicateMesh();
                copyh0.Transform(xh0);
                Mesh copyh1 = refA6.DuplicateMesh();
                copyh1.Transform(xh1);
                Mesh copyh2 = refA6.DuplicateMesh();
                copyh2.Transform(xh2);
                Mesh copyh3 = refA6.DuplicateMesh();
                copyh3.Transform(xh3);
                Mesh copyh4 = refA6.DuplicateMesh();
                copyh4.Transform(xh4);

                // Add to output (end of step (h))
                listA6.Add(copyh0);
                listA6.Add(copyh1);
                listA6.Add(copyh2);
                listA6.Add(copyh3);
                listA6.Add(copyh4);

                // Copy base pts and add
                Point3d basepth0 = Point3d.Origin;
                basepth0.Transform(xh0);
                ptsA6.Add(basepth0);
                Point3d basepth1 = Point3d.Origin;
                basepth1.Transform(xh1);
                ptsA6.Add(basepth1);
                Point3d basepth2 = Point3d.Origin;
                basepth2.Transform(xh2);
                ptsA6.Add(basepth2);
                Point3d basepth3 = Point3d.Origin;
                basepth3.Transform(xh3);
                ptsA6.Add(basepth3);
                Point3d basepth4 = Point3d.Origin;
                basepth4.Transform(xh4);
                ptsA6.Add(basepth4);

                // Copy base plns and add
                Plane baseplnh0 = Plane.WorldXY;
                Plane baseplnh1 = Plane.WorldXY;
                Plane baseplnh2 = Plane.WorldXY;
                Plane baseplnh3 = Plane.WorldXY;
                Plane baseplnh4 = Plane.WorldXY;
                baseplnh0.Transform(xh0);
                baseplnh1.Transform(xh1);
                baseplnh2.Transform(xh2);
                baseplnh3.Transform(xh3);
                baseplnh4.Transform(xh4);
                plnsA6.Add(baseplnh0);
                plnsA6.Add(baseplnh1);
                plnsA6.Add(baseplnh2);
                plnsA6.Add(baseplnh3);
                plnsA6.Add(baseplnh4);

                // Get planes for step (i)
                // We can use the points in step (h) for generating planes
                // The normal vector of the (i) planes corresponds to the x vector for the step (h) planes
                Vector3d normali0 = edgePoints[0] - furthestVertexPt;
                Vector3d normali1 = edgePoints[1] - furthestVertexPt;
                Vector3d normali2 = edgePoints[2] - furthestVertexPt;
                Vector3d normali3 = edgePoints[3] - furthestVertexPt;
                Vector3d normali4 = edgePoints[4] - furthestVertexPt;

                // The center point of the (i) planes correponds to those same end points but pushed out by one edgelength
                Point3d centeri0 = edgePoints[0];
                centeri0 = Point3d.Add(centeri0, pushPlane5);
                Point3d centeri1 = edgePoints[1];
                centeri1 = Point3d.Add(centeri1, pushPlane5);
                Point3d centeri2 = edgePoints[2];
                centeri2 = Point3d.Add(centeri2, pushPlane5);
                Point3d centeri3 = edgePoints[3];
                centeri3 = Point3d.Add(centeri3, pushPlane5);
                Point3d centeri4 = edgePoints[4];
                centeri4 = Point3d.Add(centeri4, pushPlane5);

                // Set up the unrotated (i) planes using center point and normal
                Plane planei0u = new Plane(centeri0, normali0);
                Plane planei1u = new Plane(centeri1, normali1);
                Plane planei2u = new Plane(centeri2, normali2);
                Plane planei3u = new Plane(centeri3, normali3);
                Plane planei4u = new Plane(centeri4, normali4);

                // Determine x-axis of the (i) planes by projecting original edgepoint onto it
                Point3d xpti0 = edgePoints[0];
                Transform projectXi0 = Transform.PlanarProjection(planei0u);
                xpti0.Transform(projectXi0);
                Point3d xpti1 = edgePoints[1];
                Transform projectXi1 = Transform.PlanarProjection(planei1u);
                xpti1.Transform(projectXi1);
                Point3d xpti2 = edgePoints[2];
                Transform projectXi2 = Transform.PlanarProjection(planei2u);
                xpti2.Transform(projectXi2);
                Point3d xpti3 = edgePoints[3];
                Transform projectXi3 = Transform.PlanarProjection(planei3u);
                xpti3.Transform(projectXi3);
                Point3d xpti4 = edgePoints[4];
                Transform projectXi4 = Transform.PlanarProjection(planei4u);
                xpti4.Transform(projectXi4);

                // Rotate xpts by 90 degress (angle is arbitrary) on that plane to get ypts
                Transform rotate90i0 = Transform.Rotation(Math.PI / 2, normali0, centeri0);
                Point3d ypti0 = new Point3d(xpti0);
                ypti0.Transform(rotate90i0);
                Transform rotate90i1 = Transform.Rotation(Math.PI / 2, normali1, centeri1);
                Point3d ypti1 = new Point3d(xpti1);
                ypti1.Transform(rotate90i1);
                Transform rotate90i2 = Transform.Rotation(Math.PI / 2, normali2, centeri2);
                Point3d ypti2 = new Point3d(xpti2);
                ypti2.Transform(rotate90i2);
                Transform rotate90i3 = Transform.Rotation(Math.PI / 2, normali3, centeri3);
                Point3d ypti3 = new Point3d(xpti3);
                ypti3.Transform(rotate90i3);
                Transform rotate90i4 = Transform.Rotation(Math.PI / 2, normali4, centeri4);
                Point3d ypti4 = new Point3d(xpti4);
                ypti4.Transform(rotate90i4);

                // Get the final oriented plane for step (i) transformation using the 3 points
                Plane planei0 = new Plane(centeri0, xpti0, ypti0);
                Plane planei1 = new Plane(centeri1, xpti1, ypti1);
                Plane planei2 = new Plane(centeri2, xpti2, ypti2);
                Plane planei3 = new Plane(centeri3, xpti3, ypti3);
                Plane planei4 = new Plane(centeri4, xpti4, ypti4);

                // Make tranformations
                Transform xi0 = Transform.PlaneToPlane(f20base, planei0);
                Transform xi1 = Transform.PlaneToPlane(f20base, planei1);
                Transform xi2 = Transform.PlaneToPlane(f20base, planei2);
                Transform xi3 = Transform.PlaneToPlane(f20base, planei3);
                Transform xi4 = Transform.PlaneToPlane(f20base, planei4);

                // Make copies and transform
                Mesh copyi0 = refF20.DuplicateMesh();
                copyi0.Transform(xi0);
                Mesh copyi1 = refF20.DuplicateMesh();
                copyi1.Transform(xi1);
                Mesh copyi2 = refF20.DuplicateMesh();
                copyi2.Transform(xi2);
                Mesh copyi3 = refF20.DuplicateMesh();
                copyi3.Transform(xi3);
                Mesh copyi4 = refF20.DuplicateMesh();
                copyi4.Transform(xi4);

                // Add to output (end of step (i))
                listF20.Add(copyi0);
                listF20.Add(copyi1);
                listF20.Add(copyi2);
                listF20.Add(copyi3);
                listF20.Add(copyi4);

                // Copy and add base pts
                Point3d basepti0 = Point3d.Origin;
                basepti0.Transform(xi0);
                ptsF20.Add(basepti0);
                Point3d basepti1 = Point3d.Origin;
                basepti1.Transform(xi1);
                ptsF20.Add(basepti1);
                Point3d basepti2 = Point3d.Origin;
                basepti2.Transform(xi2);
                ptsF20.Add(basepti2);
                Point3d basepti3 = Point3d.Origin;
                basepti3.Transform(xi3);
                ptsF20.Add(basepti3);
                Point3d basepti4 = Point3d.Origin;
                basepti4.Transform(xi4);
                ptsF20.Add(basepti4);

                // Copy and add baseplns
                Plane baseplni0 = Plane.WorldXY;
                Plane baseplni1 = Plane.WorldXY;
                Plane baseplni2 = Plane.WorldXY;
                Plane baseplni3 = Plane.WorldXY;
                Plane baseplni4 = Plane.WorldXY;
                baseplni0.Transform(xi0);
                baseplni1.Transform(xi1);
                baseplni2.Transform(xi2);
                baseplni3.Transform(xi3);
                baseplni4.Transform(xi4);
                plnsF20.Add(baseplni0);
                plnsF20.Add(baseplni1);
                plnsF20.Add(baseplni2);
                plnsF20.Add(baseplni3);
                plnsF20.Add(baseplni4);
            }
            //

            // Output meshes
            foreach (var m in listA6)
            {
                if (m == null) continue;
                outputmeshes.Append(new GH_Mesh(m), pth0);
            }
            foreach (var m in listB12)
            {
                if (m == null) continue;
                outputmeshes.Append(new GH_Mesh(m), pth1);
            }
            foreach (var m in listF20)
            {
                if (m == null) continue;
                outputmeshes.Append(new GH_Mesh(m), pth2);
            }
            foreach (var m in listK30)
            {
                if (m == null) continue;
                outputmeshes.Append(new GH_Mesh(m), pth3);
            }
            DA.SetDataTree(0, outputmeshes);

            // Output points
            foreach (var p in ptsA6)
            {
                outputpts.Append(new GH_Point(p), pth0);
            }
            foreach (var p in ptsB12)
            {
                outputpts.Append(new GH_Point(p), pth1);
            }
            foreach (var p in ptsF20)
            {
                outputpts.Append(new GH_Point(p), pth2);
            }
            foreach (var p in ptsK30)
            {
                outputpts.Append(new GH_Point(p), pth3);
            }
            DA.SetDataTree(1, outputpts);

            // Translate planes along their normals by half the tile height
            double a6HalfHeight = a6HeightRef / 2;
            double b12HalfHeight = b12HeightRef / 2;
            double f20HalfHeight = f20HeightRef / 2;
            double k30HalfHeight = k30HeightRef / 2;

            TranslatePlanesAlongNormals(plnsA6, a6HalfHeight);
            TranslatePlanesAlongNormals(plnsB12, b12HalfHeight);
            TranslatePlanesAlongNormals(plnsF20, f20HalfHeight);
            TranslatePlanesAlongNormals(plnsK30, k30HalfHeight);

            // Final Z-offset for all planes based on the deflated K30 half-height
            double zTranslation = -k30HalfHeight * deflationScaleFactor;
            Vector3d zOffset = new Vector3d(0, 0, zTranslation);
            TranslatePlanesInDirection(plnsA6, zOffset);
            TranslatePlanesInDirection(plnsB12, zOffset);
            TranslatePlanesInDirection(plnsF20, zOffset);
            TranslatePlanesInDirection(plnsK30, zOffset);

            // Output plns
            foreach (Plane pl in plnsA6)
            {
                outputplns.Append(new GH_Plane(pl), pth0);
            }
            foreach (Plane pl in plnsB12)
            {
                outputplns.Append(new GH_Plane(pl), pth1);
            }
            foreach (Plane pl in plnsF20)
            {
                outputplns.Append(new GH_Plane(pl), pth2);
            }
            foreach (Plane pl in plnsK30)
            {
                outputplns.Append(new GH_Plane(pl), pth3);
            }
            DA.SetDataTree(2, outputplns);
        }

        public static int GetFurthestVertex(Mesh mesh, Point3d reference)
        {
            int furthestVertexIndex = 0;
            Point3d currentVertex = new Point3d();
            double maxDistance = 0;
            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                currentVertex = mesh.TopologyVertices[i];
                double distance = currentVertex.DistanceTo(reference);
                if (distance > maxDistance)
                {
                    furthestVertexIndex = i;
                    maxDistance = distance;
                }
            }
            return furthestVertexIndex;
        }

        public static int GetClosestVertex(Mesh mesh, Point3d reference)
        {
            int closestVertexIndex = 0;
            Point3d currentVertex = new Point3d();
            double minDistance = 1000000000;
            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                currentVertex = mesh.TopologyVertices[i];
                double distance = currentVertex.DistanceTo(reference);
                if (distance < minDistance)
                {
                    closestVertexIndex = i;
                    minDistance = distance;
                }
            }
            return closestVertexIndex;
        }

        public static Plane GetOrientedPlaneFromRhombicFace(Mesh mesh, int faceIndex, Vector3d normalRef)
        {
            // Get center and normal vector of the current face
            Point3d centerFace = mesh.Faces.GetFaceCenter(faceIndex);

            // Get two points on the face (one at acute and one at obtuse vertex)
            Point3f a;
            Point3f b;
            Point3f c;
            Point3f d;
            mesh.Faces.GetFaceVertices(faceIndex, out a, out b, out c, out d);

            // Get oriented plane
            Point3d xPt = new Point3d();
            Point3d yPt = new Point3d();

            // Set xPt to be the closer of the two points
            if (centerFace.DistanceTo(a) < centerFace.DistanceTo(b))
            {
                xPt = a;
                yPt = b;
            }
            else
            {
                xPt = b;
                yPt = a;
            }

            Plane plane = new Plane(centerFace, xPt, yPt);
            // If dot product is negative we need to flip/rotate to point outwards
            if (Vector3d.Multiply(plane.Normal, normalRef) < 0)
            {
                plane.Flip();
                plane.Rotate(Math.PI / 2, plane.Normal);
            }
            return plane;
        }

        // TODO: Refactor code to stop using the x-projection method and checking cases for the angle
        // Below function is simpler and allows comparing angles on a plane without needing to project
        // This function is extremely useful since Vector3d.VectorAngle will only ever return positive values
        // Having a signed vector angle ensures we always rotate in the correct direction
        public static double GetSignedVectorAngle(Vector3d v1, Vector3d v2, Plane plane)
        {
            return Math.Atan2(Vector3d.CrossProduct(v1, v2) * plane.ZAxis, v1 * v2);
        }

        public static void TranslateToWorldXY(Mesh mesh)
        {
            // Find the minimum Z value among all topology vertices
            double minZ = double.MaxValue;
            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                double z = mesh.TopologyVertices[i].Z;
                if (z < minZ)
                    minZ = z;
            }

            // Translate the mesh so its base sits on WorldXY (Z = 0)
            if (Math.Abs(minZ) > 0.0001) // Only translate if not already at Z = 0
            {
                mesh.Translate(new Vector3d(0, 0, -minZ));
            }
        }

        public static double GetMeshHeight(Mesh mesh)
        {
            BoundingBox bbox = mesh.GetBoundingBox(true);
            return bbox.Max.Z - bbox.Min.Z;
        }

        public static void TranslatePlanesAlongNormals(List<Plane> planes, double distance)
        {
            for (int i = 0; i < planes.Count; i++)
            {
                Plane plane = planes[i];
                Vector3d translation = plane.Normal * distance;
                plane.Translate(translation);
                planes[i] = plane;  // ← Must reassign back to the list
            }
        }

        public static void TranslatePlanesInDirection(List<Plane> planes, Vector3d translation)
        {
            for (int i = 0; i < planes.Count; i++)
            {
                Plane plane = planes[i];
                plane.Translate(translation);
                planes[i] = plane;  // ← Must reassign back to the list
            }
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("3675AE7D-DB51-4939-86E1-0F2A422B7D8E"); }
        }
    }
}