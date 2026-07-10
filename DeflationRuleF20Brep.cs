using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Aperiodic
{
    public class DeflationRuleF20Brep : GH_Component
    {
        // Cache commonly used constants
        private static readonly double GoldenRatio = (1 + Math.Sqrt(5)) / 2;
        private static readonly double DeflationScaleFactor = Math.Pow(GoldenRatio, 3);
        private static readonly double InverseDeflationScaleFactor = 1.0 / DeflationScaleFactor;

        /// <summary>
        /// Initializes a new instance of the DeflationRuleF20Brep class.
        /// </summary>
        public DeflationRuleF20Brep()
          : base("DeflationRuleF20Brep", "DefF20Brep",
              "Output planes corresponding the the deflation rules for the F20 tile (additionally output breps and points)",
              "Aperiodic", "Aperiodic")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("referenceBreps", "refBreps", "Reference four golden zonohedra breps", GH_ParamAccess.tree);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("outputBreps", "outBreps", "Output breps after applying deflation rule F20", GH_ParamAccess.tree);
            pManager.AddPointParameter("outputPoints", "outPts", "Output points after applying deflation rule F20", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("outputPlanes", "outPlanes", "Output planes after applying deflation rule F20", GH_ParamAccess.tree);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Declare variables
            GH_Structure<GH_Brep> outputbreps = new GH_Structure<GH_Brep>();
            GH_Structure<GH_Point> outputpts = new GH_Structure<GH_Point>();
            GH_Structure<GH_Plane> outputplns = new GH_Structure<GH_Plane>();

            GH_Structure<GH_Brep> reference = new GH_Structure<GH_Brep>();
            if (!DA.GetDataTree<GH_Brep>(0, out reference)) { return; }

            GH_Path pth0 = new GH_Path(0);
            GH_Path pth1 = new GH_Path(1);
            GH_Path pth2 = new GH_Path(2);
            GH_Path pth3 = new GH_Path(3);

            // Get height references from original input breps
            double a6HeightRef = GetBrepHeight(reference.Branches[0][0].Value);
            double b12HeightRef = GetBrepHeight(reference.Branches[1][0].Value);
            double f20HeightRef = GetBrepHeight(reference.Branches[2][0].Value);
            double k30HeightRef = GetBrepHeight(reference.Branches[3][0].Value);

            // Note: always duplicate these before modifying
            Brep refA6 = reference.Branches[0][0].Value.DuplicateBrep();
            Brep refB12 = reference.Branches[1][0].Value.DuplicateBrep();
            Brep refF20 = reference.Branches[2][0].Value.DuplicateBrep();
            Brep refK30 = reference.Branches[3][0].Value.DuplicateBrep();

            // Translate reference breps so their base sits on WorldXY plane
            TranslateToWorldXY(refA6);
            TranslateToWorldXY(refB12);
            TranslateToWorldXY(refF20);
            TranslateToWorldXY(refK30);

            // Set up smaller output lists
            List<Brep> listA6 = new List<Brep>();
            List<Brep> listB12 = new List<Brep>();
            List<Brep> listF20 = new List<Brep>();
            List<Brep> listK30 = new List<Brep>();
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
            double rhombAcuteAngle = 2 * Math.Atan(1 / GoldenRatio);

            // Get length reference (edge length of original triacontahedron)
            double scale = refK30.Edges[0].PointAtEnd.DistanceTo(refK30.Edges[0].PointAtStart);

            Plane a6base = SetUpA6BasePlane(refA6);

            // Set up base orientation for K30 transformation in step (a6-000)
            Point3d k30basecenter = GetClosestVertex(refK30, new Point3d(0, -1*scale, 0)).Location;
            Point3d k30basexaxis = GetClosestVertex(refK30, new Point3d(1*scale, 0, 0)).Location;
            Point3d k30baseyaxis = new Point3d(-k30basexaxis.X, 0, 0);
            Plane k30base = new Plane(k30basecenter, k30basexaxis, k30baseyaxis);

            // Begin deflation for F20

            // Scale up F20 unit to get general boundaries of the inflated shapes
            // Scale center point of geometry by a factor of golden ratio^3
            Transform xformScaleF20 = Transform.Scale(Point3d.Origin, DeflationScaleFactor);
            Brep f20boundary = refF20.DuplicateBrep();
            f20boundary.Transform(xformScaleF20);
            Point3d f20boundarybase = Point3d.Origin;
            f20boundarybase.Transform(xformScaleF20);

            // Get far pt vertex
            Point3d f20farvertexPt = GetFurthestVertex(f20boundary, f20boundarybase).Location;

            // Array 5 of the A6 around base pt
            // Get base pt vertex
            BrepVertex f20basevertex = GetClosestVertex(f20boundary, f20boundarybase);
            Point3d f20basevertexPt = f20basevertex.Location;
            Vector3d normalf20 = f20farvertexPt - f20basevertexPt;
            Point3d baseptf20a60 = f20basevertexPt;

            // Get one adjacent vertex from vertex
            Point3d boundaryedgept = GetOneAdjacentVertexPoint(f20basevertex);

            Transform xrot5 = Transform.Rotation(2 * Math.PI / 5, normalf20, f20basevertexPt);
            Point3d edgept2 = boundaryedgept;
            edgept2.Transform(xrot5);

            // Get plane and transform
            Plane planef20a60 = new Plane(f20basevertexPt, edgept2, boundaryedgept);
            Transform xformf20a60 = Transform.PlaneToPlane(a6base, planef20a60);
            Vector3d moveInEdge = normalf20;
            moveInEdge.Unitize();
            Transform xscaleEdgeLength = Transform.Scale(f20basevertexPt, scale);
            moveInEdge.Transform(xscaleEdgeLength);
            Transform xMoveInEdge = Transform.Translation(moveInEdge);

            // Add the first A6 tile
            Brep f20a60 = refA6.DuplicateBrep();
            f20a60.Transform(xformf20a60);
            f20a60.Transform(xMoveInEdge);

            // Add to brep list
            listA6.Add(f20a60);

            // Add to basepts list
            ptsA6.Add(baseptf20a60);

            // Add to baseplns list
            Plane planef20a60s = Plane.WorldXY;
            planef20a60s.Transform(xformf20a60);
            planef20a60s.Flip();
            planef20a60s.Rotate(-Math.PI / 2, planef20a60s.Normal);
            planef20a60s.Origin = baseptf20a60;
            plnsA6.Add(planef20a60s);

            // Find adjacent vertex on the A6 to boundaryedgept that is not the f20basevertexpt
            BrepVertex edgept = GetClosestVertex(f20a60, boundaryedgept);
            int[] adjacentIndices = edgept.EdgeIndices();
            Point3d f20f200xpt = new Point3d();
            for (int j = 0; j < adjacentIndices.Length; j++)
            {
                BrepEdge edge = f20a60.Edges[adjacentIndices[j]];
                Point3d f20a60adjacentPt = edge.EdgeCurve.PointAtEnd;
                if (f20a60adjacentPt == boundaryedgept)
                {
                    f20a60adjacentPt = edge.EdgeCurve.PointAtStart;
                }
                if (f20a60adjacentPt.DistanceTo(f20basevertexPt) > 0.0001)  // Use distance tolerance instead of !=
                {
                    f20f200xpt = f20a60adjacentPt;
                    break;  // Take the FIRST valid point and stop
                }
            }

            // Also prepare to add F20 tile here
            Brep f20f200 = refF20.DuplicateBrep();
            Vector3d f20f200normal = boundaryedgept - f20basevertexPt;
            f20f200normal.Unitize();
            f20f200normal.Transform(xscaleEdgeLength);
            Point3d innerptf20f200 = f20basevertexPt + f20f200normal;
            Plane planef20f200 = new Plane(innerptf20f200, f20f200normal);

            // Orient plane
            Vector3d f20f200xaxis = f20f200xpt - innerptf20f200;
            double anglef20f200 = GetSignedVectorAngle(planef20f200.XAxis, f20f200xaxis, planef20f200);
            planef20f200.Rotate(anglef20f200, f20f200normal, innerptf20f200); // Possibly need to change the Math.PI, check

            // Copy and transform the F20 mesh
            Transform xformf20f200 = Transform.PlaneToPlane(Plane.WorldXY, planef20f200);
            f20f200.Transform(xformf20f200);

            // Add to brep list
            listF20.Add(f20f200);

            Vector3d basef20f200 = f20f200normal;
            basef20f200.Unitize();
            Transform xscalef20f200 = Transform.Scale(Point3d.Origin, f20HeightRef);
            basef20f200.Transform(xscalef20f200);
            Point3d baseptf20f200 = innerptf20f200;
            baseptf20f200 += basef20f200;

            // Add to basepts list
            ptsF20.Add(baseptf20f200);

            // Add to baseplns list
            planef20f200.Translate(basef20f200);
            planef20f200.Flip();
            planef20f200.Rotate(-Math.PI / 2, planef20f200.Normal);
            plnsF20.Add(planef20f200);

            Brep f20a60copy = f20a60.DuplicateBrep();
            Brep f20f200copy = f20f200.DuplicateBrep();
            Point3d baseptf20f200copy = baseptf20f200;
            Plane planef20a60scopy = new Plane(planef20a60s);
            Plane planef20f200copy = new Plane(planef20f200);
            // Rotate to get 4 more copies of the A6 and the F20
            for (int j = 1; j < 5; j++)
            {
                // Rotate edgept around normal 2pi/5 degrees
                f20a60copy.Transform(xrot5);

                // Add to brep list
                listA6.Add(f20a60copy);

                // Add to basepts list
                ptsA6.Add(baseptf20a60);

                // Add to baseplns list
                planef20a60scopy.Transform(xrot5);
                plnsA6.Add(planef20a60scopy);

                f20a60copy = f20a60copy.DuplicateBrep();
                f20f200copy.Transform(xrot5);

                // Add to brep list
                listF20.Add(f20f200copy);

                baseptf20f200copy.Transform(xrot5);
                planef20f200copy.Transform(xrot5);

                // Add to basepts list
                ptsF20.Add(baseptf20f200copy);

                // Add to baseplns list
                plnsF20.Add(planef20f200copy);

                // Not sure if this is necessary
                f20a60copy = f20a60copy.DuplicateBrep();
                f20f200copy = f20f200copy.DuplicateBrep();
            }

            // Add the central triacontahedron
            Point3d f20k300base = f20basevertexPt;
            f20k300base += moveInEdge;
            Plane planef20k300 = planef20a60;
            Plane k30baseflip = k30base;
            k30baseflip.Flip();
            k30baseflip.Rotate(Math.PI / 2 - rhombAcuteAngle, k30baseflip.Normal);
            planef20k300.Translate(moveInEdge);
            Transform xf20k300 = Transform.PlaneToPlane(k30baseflip, planef20k300);
            Brep f20k300 = refK30.DuplicateBrep();
            f20k300.Transform(xf20k300);

            // Add to brep list
            listK30.Add(f20k300);

            // Add to basepts list
            ptsK30.Add(f20k300base);

            // Add to baseplns list
            Plane planef20k300s = Plane.WorldXY;
            planef20k300s.Transform(xf20k300);
            plnsK30.Add(planef20k300s);

            // Unify normals does not seem to work properly
            // But we can use the centroid to calculate them in a consistent way
            AreaMassProperties ampf20k30 = AreaMassProperties.Compute(f20k300);
            Point3d centroidf20k30 = ampf20k30.Centroid;

            // Place B12 around the K30. Loop through faces of the K30
            for (int i = 0; i < f20k300.Faces.Count; i++)
            {
                // Get normal of face and see if it has positive dot product with normal
                BrepFace brepFace = f20k300.Faces[i];
                AreaMassProperties amp = AreaMassProperties.Compute(brepFace);
                Point3d f20b12base = amp.Centroid;
                Vector3d f20b12normal = f20b12base - centroidf20k30;
                double compareFaceNormal = Vector3d.Multiply(f20b12normal, normalf20);
                if (compareFaceNormal >= -0.5)
                {
                    // Add a b12 to the list using this face as a base
                    Plane planef20b12 = GetOrientedPlaneFromRhombicFace(f20k300, i, f20b12normal);
                    Transform xformf20b12 = Transform.PlaneToPlane(Plane.WorldXY, planef20b12);
                    Brep f20b12 = refB12.DuplicateBrep();
                    f20b12.Transform(xformf20b12);

                    // Add to mesh list
                    listB12.Add(f20b12);

                    // Add to basepts list
                    ptsB12.Add(f20b12base);

                    // Add to baseplns list
                    plnsB12.Add(planef20b12);

                    // Transform K30 further along the same axes
                    // First push the plane further out
                    Vector3d pushPlane = new Vector3d(f20b12normal);
                    pushPlane.Unitize();
                    Transform xscale = Transform.Scale(Point3d.Origin, b12HeightRef);
                    pushPlane.Transform(xscale);
                    planef20b12.Translate(pushPlane);
                    Brep e = refK30.DuplicateBrep();
                    Transform xforme = Transform.PlaneToPlane(Plane.WorldXY, planef20b12);
                    e.Transform(xforme);

                    // Add to mesh list
                    listK30.Add(e);

                    // Transform the basepoint from the origin
                    Point3d ebasept = Point3d.Origin;
                    ebasept.Transform(xforme);

                    // Add to basepts list
                    ptsK30.Add(ebasept);

                    // Add to baseplns list
                    plnsK30.Add(planef20b12);
                }
            }

            // Loop through vertices of the K30. Check for the ones with 3 neighbors that are also facing down
            for (int i = 0; i < f20k300.Vertices.Count; i++)
            {
                if (f20k300.Vertices[i].EdgeIndices().Length == 3)
                {
                    Vector3d normal = GetVertexNormal(f20k300, i);
                    if (Vector3d.Multiply(normal, normalf20) > 0)
                    {
                        // Get plane center
                        Point3d planeCenter = f20k300.Vertices[i].Location;

                        // Get orient point using adjacent vertex
                        Point3d orientX = GetOneAdjacentVertexPoint(f20k300.Vertices[i]);

                        // First create a plane using center and normal vector
                        Plane planeUnoriented = new Plane(planeCenter, normal);

                        // Project orientX point onto that plane
                        Transform xformprojectf20a6 = Transform.PlanarProjection(planeUnoriented);
                        orientX.Transform(xformprojectf20a6);

                        // Rotate orientX point by 90 degress (angle is arbitrary) on that plane to get orientY point
                        Transform rotate90 = Transform.Rotation(Math.PI / 2, normal, planeCenter);
                        Point3d orientY = new Point3d(orientX);
                        orientY.Transform(rotate90);

                        // Get the final oriented plane for transformation
                        Plane planeOriented = new Plane(planeCenter, orientX, orientY);

                        // Transform A6
                        Transform xformf20a61 = Transform.PlaneToPlane(Plane.WorldXY, planeOriented);
                        Brep f20a61 = refA6.DuplicateBrep();
                        f20a61.Transform(xformf20a61);

                        // Add to brep list
                        listA6.Add(f20a61);

                        // Tranform basept
                        Point3d f20a61base = Point3d.Origin;
                        f20a61base.Transform(xformf20a61);

                        // Add to basepts list
                        ptsA6.Add(f20a61base);

                        // Add to baseplns list
                        plnsA6.Add(planeOriented);

                        // Copy A6 3 more times...
                        BrepVertex furthestVertex = GetFurthestVertex(f20a61, f20a61base);
                        Point3d furthestVertexPt = furthestVertex.Location;

                        // Get adjacent vertex points
                        List<Point3d> edgePoints = GetAdjacentVertexPoints(furthestVertex);

                        // Get mirror planes
                        Plane plane0 = new Plane(furthestVertexPt, edgePoints[1], edgePoints[2]);
                        Plane plane1 = new Plane(furthestVertexPt, edgePoints[2], edgePoints[0]);
                        Plane plane2 = new Plane(furthestVertexPt, edgePoints[0], edgePoints[1]);
                        Transform xform0 = Transform.Mirror(plane0);
                        Transform xform1 = Transform.Mirror(plane1);
                        Transform xform2 = Transform.Mirror(plane2);

                        // Make copies and add to brep list
                        Brep copyc0 = f20a61.DuplicateBrep();
                        copyc0.Transform(xform0);
                        listA6.Add(copyc0);
                        Brep copyc1 = f20a61.DuplicateBrep();
                        copyc1.Transform(xform1);
                        listA6.Add(copyc1);
                        Brep copyc2 = f20a61.DuplicateBrep();
                        copyc2.Transform(xform2);
                        listA6.Add(copyc2);

                        // Copy base pts and add to basepts list
                        Point3d c0basept = new Point3d(f20a61base);
                        c0basept.Transform(xform0);
                        ptsA6.Add(c0basept);
                        Point3d c1basept = new Point3d(f20a61base);
                        c1basept.Transform(xform1);
                        ptsA6.Add(c1basept);
                        Point3d c2basept = new Point3d(f20a61base);
                        c2basept.Transform(xform2);
                        ptsA6.Add(c2basept);

                        // Add to baseplns list
                        Plane planeOrientedc = new Plane(planeOriented);
                        planeOrientedc.Flip();
                        planeOrientedc.Rotate(Math.PI / 2, planeOrientedc.Normal);
                        Plane c0baseplane = new Plane(planeOrientedc);
                        Plane c1baseplane = new Plane(planeOrientedc);
                        Plane c2baseplane = new Plane(planeOrientedc);
                        c0baseplane.Transform(xform0);
                        c1baseplane.Transform(xform1);
                        c2baseplane.Transform(xform2);
                        plnsA6.Add(c0baseplane);
                        plnsA6.Add(c1baseplane);
                        plnsA6.Add(c2baseplane);

                        // Also push out f20a61 again
                        // Beginning of step (g) - mirror the rhombohedra on these axes even further out, and rotate 180
                        // Get mirror plane from normal plane
                        Plane planeg = new Plane(furthestVertexPt, normal);
                        Transform xformg = Transform.Mirror(planeg);
                        Brep copyg = f20a61.DuplicateBrep();
                        copyg.Transform(xformg);
                        Transform xrot180 = Transform.Rotation(Math.PI, normal, furthestVertexPt);
                        copyg.Transform(xrot180);

                        // Add to brep list
                        listA6.Add(copyg);

                        // Copy base pt
                        Point3d gbasept = new Point3d(f20a61base);
                        gbasept.Transform(xformg);

                        // Add to basepts list
                        ptsA6.Add(gbasept);

                        // Add to baseplns list
                        Plane copygplane = new Plane(planeOrientedc);
                        copygplane.Transform(xformg);
                        copygplane.Transform(xrot180);
                        plnsA6.Add(copygplane);

                        if (Vector3d.Multiply(normal, normalf20) > 2)
                        {
                            // Next we'll mirror these ones 3 more times on the outer faces using the method above
                            // Get furthest vertex of the copy
                            furthestVertex = GetFurthestVertex(copyg, centroidf20k30);
                            furthestVertexPt = furthestVertex.Location;

                            // Get adjacent vertex points
                            edgePoints = GetAdjacentVertexPoints(furthestVertex);

                            // Get mirror planes
                            plane0 = new Plane(furthestVertexPt, edgePoints[1], edgePoints[2]);
                            plane1 = new Plane(furthestVertexPt, edgePoints[2], edgePoints[0]);
                            plane2 = new Plane(furthestVertexPt, edgePoints[0], edgePoints[1]);
                            xform0 = Transform.Mirror(plane0);
                            xform1 = Transform.Mirror(plane1);
                            xform2 = Transform.Mirror(plane2);

                            // Make copies
                            Brep copyg0 = copyg.DuplicateBrep();
                            copyg0.Transform(xform0);
                            listA6.Add(copyg0);
                            Brep copyg1 = copyg.DuplicateBrep();
                            copyg1.Transform(xform1);
                            listA6.Add(copyg1);
                            Brep copyg2 = copyg.DuplicateBrep();
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

                            // Copy and add to baseplns list
                            copygplane.Flip();
                            copygplane.Rotate(Math.PI / 2, copygplane.Normal);
                            Plane copyg0plane = new Plane(copygplane);
                            Plane copyg1plane = new Plane(copygplane);
                            Plane copyg2plane = new Plane(copygplane);
                            copyg0plane.Transform(xform0);
                            copyg1plane.Transform(xform1);
                            copyg2plane.Transform(xform2);
                            plnsA6.Add(copyg0plane);
                            plnsA6.Add(copyg1plane);
                            plnsA6.Add(copyg2plane);

                            // Finally for each of these 3 copies we want to mirror again 2 more times (using the 2 most outer faces of each)
                            // Or we just mirror the 2 and then rotate around the 3-fold axis
                            Brep copyg00 = copyg0.DuplicateBrep();
                            Brep copyg01 = copyg0.DuplicateBrep();

                            // Again, get furthest vertex/edges
                            furthestVertex = GetFurthestVertex(copyg0, centroidf20k30);
                            furthestVertexPt = furthestVertex.Location;

                            // Get adjacent vertex points
                            edgePoints = GetAdjacentVertexPoints(furthestVertex);

                            // Identify furthest edgepoint - we need the two mirror planes adjacent to it
                            List<Point3d> sortedEdgePts = new List<Point3d>();
                            double test0 = edgePoints[0].DistanceTo(centroidf20k30);
                            double test1 = edgePoints[1].DistanceTo(centroidf20k30);
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

                            // Transform and add baseplns
                            copyg0plane.Flip();
                            copyg0plane.Rotate(Math.PI / 2, copyg0plane.Normal);
                            Plane g00baseplane = new Plane(copyg0plane);
                            Plane g01baseplane = new Plane(copyg0plane);
                            g00baseplane.Transform(xform0);
                            g01baseplane.Transform(xform1);
                            plnsA6.Add(g00baseplane);
                            plnsA6.Add(g01baseplane);

                            // Finally copy and rotate to make 4 more (this completes step (g) of the deflation)
                            Brep copyg10 = copyg00.DuplicateBrep();
                            Brep copyg11 = copyg01.DuplicateBrep();
                            Brep copyg20 = copyg00.DuplicateBrep();
                            Brep copyg21 = copyg01.DuplicateBrep();

                            Transform rotate120 = Transform.Rotation(2 * Math.PI / 3, normal, planeCenter);
                            Transform rotate240 = Transform.Rotation(4 * Math.PI / 3, normal, planeCenter);
                            copyg10.Transform(rotate120);
                            copyg11.Transform(rotate120);
                            copyg20.Transform(rotate240);
                            copyg21.Transform(rotate240);

                            // Add to brep list
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

                            // Copy baseplns and add to baseplns list
                            Plane g10baseplane = new Plane(g00baseplane);
                            Plane g11baseplane = new Plane(g01baseplane);
                            Plane g20baseplane = new Plane(g00baseplane);
                            Plane g21baseplane = new Plane(g01baseplane);
                            g10baseplane.Transform(rotate120);
                            g11baseplane.Transform(rotate120);
                            g20baseplane.Transform(rotate240);
                            g21baseplane.Transform(rotate240);
                            plnsA6.Add(g10baseplane);
                            plnsA6.Add(g11baseplane);
                            plnsA6.Add(g20baseplane);
                            plnsA6.Add(g21baseplane);
                        }
                        else // For the ones closer to the edge, we don't copy it as many times
                        {
                            // Instead of mirroring, we rotate around an axis to get 5 copies total
                            // Get edge pts
                            furthestVertex = GetFurthestVertex(copyg, centroidf20k30);
                            furthestVertexPt = furthestVertex.Location;

                            // Get rotation axis by finding edge with positive dot product to f20basevector
                            adjacentIndices = furthestVertex.EdgeIndices();
                            Point3d axisPoint = new Point3d();
                            Vector3d axis = new Vector3d();
                            for (int j = 0; j < 3; j++)
                            {
                                BrepEdge edge = copyg.Edges[adjacentIndices[j]];
                                Point3d adjacentPt = edge.EdgeCurve.PointAtEnd;
                                if (adjacentPt == furthestVertexPt)
                                {
                                    adjacentPt = edge.EdgeCurve.PointAtStart;
                                }
                                axis = adjacentPt - furthestVertexPt;
                                if (Vector3d.Multiply(axis, normalf20) > 0)
                                {
                                    axisPoint = adjacentPt;
                                }
                            }
                            axis = axisPoint - furthestVertexPt;

                            // Rotate copyg about the axis
                            Transform xformrotatef20a6g = Transform.Rotation(2 * Math.PI / 5, axis, axisPoint);
                            Brep copyf20a6g = copyg.DuplicateBrep();
                            Plane copyf20a6gplane = new Plane(copygplane);
                            for (int j = 0; j < 4; j++)
                            {
                                copyf20a6g.Transform(xformrotatef20a6g);

                                // Add to brep list
                                listA6.Add(copyf20a6g);
                                copyf20a6g = copyf20a6g.DuplicateBrep();

                                // Add to basepts list
                                ptsA6.Add(gbasept);

                                // Add to baseplns list
                                copyf20a6gplane.Transform(xformrotatef20a6g);
                                plnsA6.Add(copyf20a6gplane);
                            }
                        }
                    }
                }

                // Five-fold vertices
                else if (f20k300.Vertices[i].EdgeIndices().Length == 5)
                {
                    Vector3d normal = GetVertexNormal(f20k300, i);
                    double vectoreComparef20k30 = Vector3d.Multiply(normal, normalf20);

                    if (vectoreComparef20k30 > 0.8)
                    {
                        // Use the plane center and the orientation point
                        Point3d planeCenter5 = f20k300.Vertices[i].Location;

                        // Get orient point using adjacent vertex
                        Point3d orientX5 = GetOneAdjacentVertexPoint(f20k300.Vertices[i]);

                        // First create a plane using center and normal vector
                        Vector3d normal5 = normal;
                        Plane fiveFoldPlaneUnoriented = new Plane(planeCenter5, normal5);

                        // UPDATED CODE
                        double fiveFoldPlaneAngle = GetSignedVectorAngle(fiveFoldPlaneUnoriented.XAxis, orientX5 - planeCenter5, fiveFoldPlaneUnoriented);
                        fiveFoldPlaneUnoriented.Rotate(fiveFoldPlaneAngle, fiveFoldPlaneUnoriented.Normal);

                        // Push the plane outwards by a multiple of the edge length
                        Vector3d pushPlane5 = new Vector3d(normal5);
                        pushPlane5.Unitize();
                        Transform xscale5 = Transform.Scale(Point3d.Origin, scale);
                        pushPlane5.Transform(xscale5);
                        fiveFoldPlaneUnoriented.Translate(pushPlane5 * 2);

                        // Transform 6 F20 to these 5-fold rotational axes
                        Transform xformFive = Transform.PlaneToPlane(Plane.WorldXY, fiveFoldPlaneUnoriented);
                        Brep copyf = refF20.DuplicateBrep();
                        copyf.Transform(xformFive);

                        // Add to mesh list
                        listF20.Add(copyf);

                        // Get base pts - these should be on the outside - get furthest vertex
                        BrepVertex furthestVertex = GetFurthestVertex(copyf, planeCenter5);
                        Point3d furthestVertexPt = furthestVertex.Location;
                        Point3d baseptf = furthestVertexPt;

                        // Add to basepts list
                        ptsF20.Add(baseptf);

                        // Add to baseplns list
                        Vector3d pushPlane5f20 = new Vector3d(pushPlane5);
                        pushPlane5f20.Unitize();
                        pushPlane5f20 *= f20HeightRef;
                        Plane baseplnf = Plane.WorldXY;
                        baseplnf.Transform(xformFive);
                        baseplnf.Translate(pushPlane5f20);
                        baseplnf.Flip();
                        baseplnf.Rotate(-Math.PI / 2, baseplnf.Normal);
                        plnsF20.Add(baseplnf);

                        // Using this F20, we find the two poles and array five A6 around both...
                        // Array 5 of the A6 around base pt
                        Vector3d normalf20a6h = baseptf - planeCenter5;

                        // Get one adjacent vertex from vertex
                        boundaryedgept = GetOneAdjacentVertexPoint(furthestVertex);

                        xrot5 = Transform.Rotation(2 * Math.PI / 5, normalf20a6h, baseptf);
                        edgept2 = boundaryedgept;
                        edgept2.Transform(xrot5);

                        // Get plane and transform
                        Plane planef20a6h = new Plane(furthestVertexPt, boundaryedgept, edgept2);
                        Transform xformf20a6h = Transform.PlaneToPlane(a6base, planef20a6h);

                        // Add the first A6 tile
                        Brep f20a6h = refA6.DuplicateBrep();
                        f20a6h.Transform(xformf20a6h);

                        // Add to brep list
                        listA6.Add(f20a6h);

                        Point3d baseptf20a6h = Point3d.Origin;
                        baseptf20a6h.Transform(xformf20a6h);

                        // Add to basepts list
                        ptsA6.Add(baseptf20a6h);

                        // Add to baseplns list
                        Plane baseplnf20a6h = Plane.WorldXY;
                        baseplnf20a6h.Transform(xformf20a6h);
                        plnsA6.Add(baseplnf20a6h);

                        // Get xpt for next F20 tile using the A6 tile
                        // Find adjacent vertex to edgept that is not the furthestVertexPt
                        Point3d baseptf20f20h = boundaryedgept + pushPlane5;
                        Point3d f20f20hxpt = boundaryedgept;

                        // Also prepare to add F20 tile here
                        Brep f20f20h = refF20.DuplicateBrep();
                        Vector3d f20f20hnormal = boundaryedgept - baseptf;
                        Plane planef20f20h = new Plane(baseptf20f20h, f20f20hnormal);
                        Vector3d f20f20hxaxis = f20f20hxpt - baseptf20f20h;
                        double anglef20f20h = GetSignedVectorAngle(planef20f20h.XAxis, f20f20hxaxis, planef20f20h);
                        planef20f20h.Rotate(anglef20f20h + Math.PI, f20f20hnormal, baseptf20f20h);

                        // Copy and transform the F20 mesh
                        Transform xformf20f20h = Transform.PlaneToPlane(Plane.WorldXY, planef20f20h);
                        f20f20h.Transform(xformf20f20h);

                        // Check if F20 basept is inside inflated F20 shape
                        if (Vector3d.Multiply(f20f20hnormal, normalf20) > -5.0)
                        {
                            // Add to lists
                            listF20.Add(f20f20h);
                            ptsF20.Add(baseptf20f20h);
                            plnsF20.Add(planef20f20h);
                        }

                        Brep f20a6hcopy = f20a6h.DuplicateBrep();
                        Point3d baseptf20a6hcopy = baseptf20a6h;
                        Brep f20f20hcopy = f20f20h.DuplicateBrep();
                        Point3d baseptf20f20hcopy = baseptf20f20h;
                        Vector3d f20f20hnormalcopy = f20f20hnormal;
                        Plane baseplnf20a6hcopy = baseplnf20a6h;
                        Plane baseplnf20f20hcopy = planef20f20h;

                        // Rotate to get 4 more copies of the A6 and the F20
                        for (int j = 1; j < 5; j++)
                        {
                            // Rotate around 2pi/5 degrees
                            f20f20hnormalcopy.Transform(xrot5);

                            // Add A6
                            f20a6hcopy.Transform(xrot5);
                            baseptf20a6hcopy.Transform(xrot5);
                            baseplnf20a6hcopy.Transform(xrot5);

                            // Add to mesh list
                            listA6.Add(f20a6hcopy);

                            // Add to basepts list
                            ptsA6.Add(baseptf20a6hcopy);

                            // Add to baseplns list
                            plnsA6.Add(baseplnf20a6hcopy);

                            // Add F20
                            f20f20hcopy.Transform(xrot5);
                            baseptf20f20hcopy.Transform(xrot5);
                            baseplnf20f20hcopy.Transform(xrot5);

                            // Check if F20 basept is inside inflated F20 shape
                            if (Vector3d.Multiply(f20f20hnormalcopy, normalf20) > -5.0)
                            {
                                listF20.Add(f20f20hcopy);
                                ptsF20.Add(baseptf20f20hcopy);
                                plnsF20.Add(baseplnf20f20hcopy);
                            }

                            f20a6hcopy = f20a6hcopy.DuplicateBrep();
                            f20f20hcopy = f20f20hcopy.DuplicateBrep();
                        }

                        // A6 array of 5 on the inner side the F20
                        // Get closest vertex
                        BrepVertex closestVertex = GetClosestVertex(copyf, planeCenter5);
                        Point3d closestVertexPt = closestVertex.Location;

                        // Get one adjacent vertex
                        boundaryedgept = GetOneAdjacentVertexPoint(closestVertex);

                        // Start with one edge, then rotate around the plane to get the others in order
                        List<Point3d> edgePoints = new List<Point3d>();
                        edgePoints.Add(boundaryedgept);
                        for (int j = 1; j < 5; j++)
                        {
                            // Rotate edgept around normal 2pi/5 degrees
                            xrot5 = Transform.Rotation(-2 * Math.PI / 5, normal5, planeCenter5);
                            boundaryedgept.Transform(xrot5);
                            edgePoints.Add(boundaryedgept);
                        }

                        // Get planes
                        Plane planeh0 = new Plane(closestVertexPt, edgePoints[0], edgePoints[1]);
                        Plane planeh1 = new Plane(closestVertexPt, edgePoints[1], edgePoints[2]);
                        Plane planeh2 = new Plane(closestVertexPt, edgePoints[2], edgePoints[3]);
                        Plane planeh3 = new Plane(closestVertexPt, edgePoints[3], edgePoints[4]);
                        Plane planeh4 = new Plane(closestVertexPt, edgePoints[4], edgePoints[0]);

                        // Make tranformations
                        Transform xh0 = Transform.PlaneToPlane(a6base, planeh0);
                        Transform xh1 = Transform.PlaneToPlane(a6base, planeh1);
                        Transform xh2 = Transform.PlaneToPlane(a6base, planeh2);
                        Transform xh3 = Transform.PlaneToPlane(a6base, planeh3);
                        Transform xh4 = Transform.PlaneToPlane(a6base, planeh4);

                        // Make copies and transform
                        Brep copyh0 = refA6.DuplicateBrep();
                        copyh0.Transform(xh0);

                        Brep copyh1 = refA6.DuplicateBrep();
                        copyh1.Transform(xh1);

                        Brep copyh2 = refA6.DuplicateBrep();
                        copyh2.Transform(xh2);

                        Brep copyh3 = refA6.DuplicateBrep();
                        copyh3.Transform(xh3);

                        Brep copyh4 = refA6.DuplicateBrep();
                        copyh4.Transform(xh4);

                        // Add to output
                        listA6.Add(copyh0);
                        listA6.Add(copyh1);
                        listA6.Add(copyh2);
                        listA6.Add(copyh3);
                        listA6.Add(copyh4);

                        // Simultaneously find A6 for next step
                        Brep f20a62 = copyh0;
                        Point3d baseptf20a62 = baseptf20a6h;
                        Plane baseplnf20a62 = baseplnf20a6h;

                        // Copy base pln and add
                        Plane baseplnh0 = Plane.WorldXY;
                        baseplnh0.Transform(xh0);
                        // Additional transformation to flip
                        Transform xh02 = Transform.PlaneToPlane(baseplnh0, GetFlippedA6Plane(baseplnh0, a6HeightRef));
                        baseplnh0.Transform(xh02);
                        plnsA6.Add(baseplnh0);

                        // Copy base pts and add
                        Point3d basepth0 = Point3d.Origin;
                        basepth0.Transform(xh0);
                        basepth0.Transform(xh02);
                        ptsA6.Add(basepth0);

                        if (!CheckIfA6BaseVectorIsAcute(copyh0, basepth0, normalf20))
                        {
                            f20a62 = copyh0;
                            baseptf20a62 = basepth0;
                            baseplnf20a62 = baseplnh0;
                        }

                        Plane baseplnh1 = Plane.WorldXY;
                        baseplnh1.Transform(xh1);
                        Transform xh12 = Transform.PlaneToPlane(baseplnh1, GetFlippedA6Plane(baseplnh1, a6HeightRef));
                        baseplnh1.Transform(xh12);
                        plnsA6.Add(baseplnh1);

                        Point3d basepth1 = Point3d.Origin;
                        basepth1.Transform(xh1);
                        basepth1.Transform(xh12);
                        ptsA6.Add(basepth1);

                        if (!CheckIfA6BaseVectorIsAcute(copyh1, basepth1, normalf20))
                        {
                            f20a62 = copyh1;
                            baseptf20a62 = basepth1;
                            baseplnf20a62 = baseplnh1;
                        }

                        Plane baseplnh2 = Plane.WorldXY;
                        baseplnh2.Transform(xh2);
                        Transform xh22 = Transform.PlaneToPlane(baseplnh2, GetFlippedA6Plane(baseplnh2, a6HeightRef));
                        baseplnh2.Transform(xh22);
                        plnsA6.Add(baseplnh2);

                        Point3d basepth2 = Point3d.Origin;
                        basepth2.Transform(xh2);
                        basepth2.Transform(xh22);
                        ptsA6.Add(basepth2);

                        if (!CheckIfA6BaseVectorIsAcute(copyh2, basepth2, normalf20))
                        {
                            f20a62 = copyh2;
                            baseptf20a62 = basepth2;
                            baseplnf20a62 = baseplnh2;
                        }

                        Plane baseplnh3 = Plane.WorldXY;
                        baseplnh3.Transform(xh3);
                        Transform xh32 = Transform.PlaneToPlane(baseplnh3, GetFlippedA6Plane(baseplnh3, a6HeightRef));
                        baseplnh3.Transform(xh32);
                        plnsA6.Add(baseplnh3);

                        Point3d basepth3 = Point3d.Origin;
                        basepth3.Transform(xh3);
                        basepth3.Transform(xh32);
                        ptsA6.Add(basepth3);

                        if (!CheckIfA6BaseVectorIsAcute(copyh3, basepth3, normalf20))
                        {
                            f20a62 = copyh3;
                            baseptf20a62 = basepth3;
                            baseplnf20a62 = baseplnh3;
                        }

                        Plane baseplnh4 = Plane.WorldXY;
                        baseplnh4.Transform(xh4);
                        Transform xh42 = Transform.PlaneToPlane(baseplnh4, GetFlippedA6Plane(baseplnh4, a6HeightRef));
                        baseplnh4.Transform(xh42);
                        plnsA6.Add(baseplnh4);

                        Point3d basepth4 = Point3d.Origin;
                        basepth4.Transform(xh4);
                        basepth4.Transform(xh42);
                        ptsA6.Add(basepth4);

                        if (!CheckIfA6BaseVectorIsAcute(copyh4, basepth4, normalf20))
                        {
                            f20a62 = copyh4;
                            baseptf20a62 = basepth4;
                            baseplnf20a62 = baseplnh4;
                        }

                        // Now continue to mirrorf20a62
                        // But first check it is not the top vertex...
                        if (vectoreComparef20k30 < 5) // Sketchy but works for now
                        {
                            int closeFaceIndex = GetClosestFace(f20a62, f20boundarybase);

                            // Get center and normal vector of the current face
                            BrepFace brepFace = f20a62.Faces[closeFaceIndex];
                            AreaMassProperties ampFace = AreaMassProperties.Compute(brepFace);
                            Point3d f20a62facecenter = ampFace.Centroid;
                            Vector3d f20a62facenormal = brepFace.NormalAt(0.5, 0.5);
                            Plane planef20a63 = new Plane(f20a62facecenter, f20a62facenormal);
                            Transform xmirrorf20a63 = Transform.Mirror(planef20a63);
                            Brep f20a63 = f20a62.DuplicateBrep();
                            f20a63.Transform(xmirrorf20a63);

                            // Add to brep list
                            listA6.Add(f20a63);

                            // Add to basepts list
                            Point3d baseptf20a63 = baseptf20a62;
                            baseptf20a63.Transform(xmirrorf20a63);
                            ptsA6.Add(baseptf20a63);

                            // Add to baseplns list
                            Plane baseplnf20a63 = baseplnf20a62;
                            baseplnf20a63.Transform(xmirrorf20a63);
                            baseplnf20a63.Flip();
                            baseplnf20a63.Rotate(Math.PI / 2, baseplnf20a63.Normal);
                            plnsA6.Add(baseplnf20a63);

                            // Get furthest face of f20a63 to get new mirror
                            int farFaceIndex = GetFurthestFace(f20a63, f20a62facecenter);
                            brepFace = f20a63.Faces[farFaceIndex];
                            ampFace = AreaMassProperties.Compute(brepFace);
                            Point3d f20a63facecenter = ampFace.Centroid;
                            Plane planef20a64 = new Plane(f20a63facecenter, f20a62facenormal);
                            Transform xmirrorf20a64 = Transform.Mirror(planef20a64);
                            Brep f20a64 = f20a63.DuplicateBrep();
                            f20a64.Transform(xmirrorf20a64);

                            // Add to brep list
                            listA6.Add(f20a64);

                            // Add to basepts list
                            Point3d baseptf20a64 = baseptf20a63;
                            baseptf20a64.Transform(xmirrorf20a64);
                            ptsA6.Add(baseptf20a64);

                            // Add to baseplns list
                            Plane baseplnf20a64 = new Plane(baseplnf20a63);
                            baseplnf20a64.Transform(xmirrorf20a64);
                            baseplnf20a64.Flip();
                            baseplnf20a64.Rotate(Math.PI / 2, baseplnf20a64.Normal);
                            plnsA6.Add(baseplnf20a64);

                            // Prep planes for next mirroring
                            baseplnf20a64.Flip();
                            baseplnf20a64.Rotate(Math.PI / 2, baseplnf20a64.Normal);

                            // From this, get vertex base pt
                            // and find two adjacent faces that are not planef20a64 to use as mirrors
                            BrepVertex baseVertex = GetFurthestVertex(f20a64, baseptf20a64);
                            List<int> adjacentFaceIndices = new List<int>();
                            foreach (int edgeIndex in baseVertex.EdgeIndices())
                            {
                                BrepEdge edge = f20a64.Edges[edgeIndex];
                                int[] faces = edge.AdjacentFaces();

                                if (faces != null)
                                {
                                    foreach (int faceIndex in faces)
                                    {
                                        // Add unique face indices
                                        if (!adjacentFaceIndices.Contains(faceIndex))
                                        {
                                            adjacentFaceIndices.Add(faceIndex);
                                        }
                                    }
                                }
                            }

                            for (int j = 0; j < 3; j++)
                            {
                                int faceIndex = adjacentFaceIndices[j];
                                brepFace = f20a64.Faces[faceIndex];
                                ampFace = AreaMassProperties.Compute(brepFace);
                                Point3d f20a64facecenter = ampFace.Centroid;
                                if (planef20a64.DistanceTo(f20a64facecenter) > 0.00001) // tolerance issue - causing one additional A6 to be created if only using > 0
                                {
                                    Vector3d f20a64facenormal = brepFace.NormalAt(0.5, 0.5);
                                    Plane planef20a65 = new Plane(f20a64facecenter, f20a64facenormal);
                                    Transform xmirrorf20a65 = Transform.Mirror(planef20a65);
                                    Brep f20a65 = f20a64.DuplicateBrep();
                                    f20a65.Transform(xmirrorf20a65);

                                    // Add to brep list
                                    listA6.Add(f20a65);

                                    // Add to basepts list
                                    //ptsA6.Add(f20a64facecenter); // NOTE: This was an error - the basept is wrong
                                    ptsA6.Add(baseptf20a64);

                                    // Add to baseplns list
                                    Plane baseplnf20a65 = new Plane(baseplnf20a64);
                                    baseplnf20a65.Transform(xmirrorf20a65);
                                    plnsA6.Add(baseplnf20a65);
                                }
                            }
                        }
                    }
                }
            }

            // Output breps
            foreach (var b in listA6)
            {
                if (b == null) continue;
                outputbreps.Append(new GH_Brep(b), pth0);
            }
            foreach (var b in listB12)
            {
                if (b == null) continue;
                outputbreps.Append(new GH_Brep(b), pth1);
            }
            foreach (var b in listF20)
            {
                if (b == null) continue;
                outputbreps.Append(new GH_Brep(b), pth2);
            }
            foreach (var b in listK30)
            {
                if (b == null) continue;
                outputbreps.Append(new GH_Brep(b), pth3);
            }
            DA.SetDataTree(0, outputbreps);

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

            // Final Z-offset for all planes based on the deflated A6 half-height
            double zTranslation = -f20HalfHeight * DeflationScaleFactor;
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

        public static Plane SetUpA6BasePlane(Brep refA6)
        {
            BrepVertex leftmostVertex = refA6.Vertices[0];
            double minX = 0;
            foreach (BrepVertex a6v in refA6.Vertices)
            {
                double currentX = a6v.Location.X;
                if (currentX < minX)
                {
                    leftmostVertex = a6v;
                    minX = currentX;
                }
            }

            Point3d baseCenter = leftmostVertex.Location;

            int[] edgeIndices = leftmostVertex.EdgeIndices();
            List<Point3d> edgePoints = new List<Point3d>();
            for (int j = 0; j < 3; j++)
            {
                BrepEdge edgeh = refA6.Edges[edgeIndices[j]];
                Point3d edgepth = edgeh.EdgeCurve.PointAtEnd;
                if (edgepth == baseCenter)
                {
                    edgepth = edgeh.EdgeCurve.PointAtStart;
                }
                // We only want two of the vertices, the ones not at x = 0, y = 0
                if (edgepth.X < -0.0001) edgePoints.Add(edgepth);
            }

            // Order edgepts to set up the plane a6base
            Plane a6base;
            if (edgePoints[0].Y < edgePoints[1].Y)
            {
                a6base = new Plane(baseCenter, edgePoints[0], edgePoints[1]);
            }
            else
            {
                a6base = new Plane(baseCenter, edgePoints[1], edgePoints[0]);
            }
            return a6base;
        }

        public static List<Point3d> GetAdjacentVertexPoints(BrepVertex vertex)
        {
            List<Point3d> adjacentVertexPoints = new List<Point3d>();
            int[] edgeIndices = vertex.EdgeIndices();
            for (int i = 0; i < edgeIndices.Length; i++)
            {
                BrepEdge edge = vertex.Brep.Edges[edgeIndices[i]];
                Point3d start = edge.EdgeCurve.PointAtStart;
                Point3d end = edge.EdgeCurve.PointAtEnd;
                Point3d edgept = (start.DistanceTo(vertex.Location) > end.DistanceTo(vertex.Location)) ? start : end;
                adjacentVertexPoints.Add(edgept);
            }
            return adjacentVertexPoints;
        }

        public static Point3d GetOneAdjacentVertexPoint(BrepVertex vertex)
        {
            int firstEdgeIndex = vertex.EdgeIndices()[0];
            BrepEdge edge = vertex.Brep.Edges[firstEdgeIndex];
            Point3d start = edge.EdgeCurve.PointAtStart;
            Point3d end = edge.EdgeCurve.PointAtEnd;
            Point3d edgept = (start.DistanceTo(vertex.Location) > end.DistanceTo(vertex.Location)) ? start : end;
            return edgept;
        }

        public static BrepVertex GetFurthestVertex(Brep brep, Point3d reference)
        {
            BrepVertex furthestVertex = brep.Vertices[0];
            Point3d currentVertex = new Point3d();
            double maxDistance = 0;
            foreach (BrepVertex v in brep.Vertices)
            {
                currentVertex = v.Location;
                double distance = currentVertex.DistanceTo(reference);
                if (distance > maxDistance)
                {
                    furthestVertex = v;
                    maxDistance = distance;
                }
            }
            return furthestVertex;
        }

        public static BrepVertex GetClosestVertex(Brep brep, Point3d reference)
        {
            BrepVertex closestVertex = brep.Vertices[0];
            Point3d currentVertex = new Point3d();
            double minDistance = double.MaxValue;
            foreach (BrepVertex v in brep.Vertices)
            {
                currentVertex = v.Location;
                double distance = currentVertex.DistanceTo(reference);
                if (distance < minDistance)
                {
                    closestVertex = v;
                    minDistance = distance;
                }
            }
            return closestVertex;
        }

        /// <summary>
        /// Calculate vertex normal of a brep
        /// In this case because all breps are zonohedra, the calculation is simplified
        /// </summary>
        /// <param name="brep"></param>
        /// <param name="vertexIndex"></param>
        /// <returns></returns>
        public static Vector3d GetVertexNormal(Brep brep, int vertexIndex)
        {
            BrepVertex vertex = brep.Vertices[vertexIndex];
            AreaMassProperties amp = AreaMassProperties.Compute(brep);
            Point3d centroid = amp.Centroid;
            Vector3d normal = (vertex.Location - centroid);
            normal.Unitize();
            return normal;
        }

        public static int GetFurthestFace(Brep brep, Point3d reference)
        {
            int furthestFaceIndex = 0;
            Point3d currentFaceCenter = new Point3d();
            double maxDistance = 0;
            for (int i = 0; i < brep.Faces.Count; i++)
            {
                currentFaceCenter = GetBrepFaceCenter(brep, i);
                double distance = currentFaceCenter.DistanceTo(reference);
                if (distance > maxDistance)
                {
                    furthestFaceIndex = i;
                    maxDistance = distance;
                }
            }
            return furthestFaceIndex;
        }

        public static int GetClosestFace(Brep brep, Point3d reference)
        {
            int closestFaceIndex = 0;
            Point3d currentFaceCenter = new Point3d();
            double minDistance = 1000000;
            for (int i = 0; i < brep.Faces.Count; i++)
            {
                currentFaceCenter = GetBrepFaceCenter(brep, i);
                double distance = currentFaceCenter.DistanceTo(reference);
                if (distance < minDistance)
                {
                    closestFaceIndex = i;
                    minDistance = distance;
                }
            }
            return closestFaceIndex;
        }

        public static Point3d GetBrepFaceCenter(Brep brep, int faceIndex)
        {
            BrepFace face = brep.Faces[faceIndex];
            int[] adjacentEdgeIndices = face.AdjacentEdges();

            Point3d center = Point3d.Origin;
            foreach (int edgeIdx in adjacentEdgeIndices)
            {
                BrepEdge edge = brep.Edges[edgeIdx];
                center += (edge.PointAtStart + edge.PointAtEnd) * 0.5;
            }

            center /= adjacentEdgeIndices.Length;
            return center;
        }

        // Note that this only works for placement of B12 and K30, not A6 or F20, since those are oriented based on the face center
        // X-axis is aligned with closer of the two vertices
        public static Plane GetOrientedPlaneFromRhombicFace(Brep brep, int faceIndex, Vector3d normalRef)
        {
            // Get center and normal vector of the current face
            BrepFace brepFace = brep.Faces[faceIndex];
            AreaMassProperties amp = AreaMassProperties.Compute(brepFace);
            Point3d centerFace = amp.Centroid;

            // Get face vertex indices and convert to Point3d for better precision
            int edgeIndex = brepFace.AdjacentEdges()[0];
            BrepEdge edge = brep.Edges[edgeIndex];
            Curve edgeCurve = edge.EdgeCurve;
            Point3d a = edgeCurve.PointAtStart;
            Point3d b = edgeCurve.PointAtEnd;

            // Get oriented plane
            Point3d xPt;
            Point3d yPt;

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

        public static bool CheckIfA6BaseVectorIsAcute(Brep a6tocheck, Point3d basept, Vector3d vec)
        {
            BrepVertex farVertex = GetFurthestVertex(a6tocheck, basept);
            Point3d farPt = farVertex.Location;
            Vector3d a6basevector = farPt - basept;
            if (Vector3d.VectorAngle(vec, a6basevector) < Math.PI / 2)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        // TODO: Refactor code to stop using the x-projection method and checking cases for the angle
        // Below function is simpler and allows comparing angles on a plane without needing to project
        // This function is extremely useful since Vector3d.VectorAngle will only ever return positive values
        // Having a signed vector angle ensures we always rotate in the correct direction
        public static double GetSignedVectorAngle(Vector3d v1, Vector3d v2, Plane plane)
        {
            return Math.Atan2(Vector3d.CrossProduct(v1, v2) * plane.ZAxis, v1 * v2);
        }

        public static Plane GetFlippedA6Plane(Plane plntoflip, double a6HeightRef)
        {
            Vector3d normal = plntoflip.Normal;
            Vector3d movePlane = new Vector3d(normal);
            movePlane.Unitize();
            movePlane *= a6HeightRef;
            Plane newpln = new Plane(plntoflip);
            newpln.Translate(movePlane);
            newpln.Flip();
            newpln.Rotate(-Math.PI / 2, newpln.Normal);
            return newpln;
        }

        public static void TranslateToWorldXY(Brep brep)
        {
            // Find the minimum Z value among all topology vertices
            double minZ = double.MaxValue;
            for (int i = 0; i < brep.Vertices.Count; i++)
            {
                double z = brep.Vertices[i].Location.Z;
                if (z < minZ)
                    minZ = z;
            }

            // Translate the brep so its base sits on WorldXY (Z = 0)
            brep.Translate(new Vector3d(0, 0, -minZ));
        }

        public static double GetBrepHeight(Brep brep)
        {
            BoundingBox bbox = brep.GetBoundingBox(true);
            return bbox.Max.Z - bbox.Min.Z;
        }

        public static void TranslatePlanesAlongNormals(List<Plane> planes, double distance)
        {
            for (int i = 0; i < planes.Count; i++)
            {
                Plane plane = planes[i];
                Vector3d translation = plane.Normal * distance;
                plane.Translate(translation);
                planes[i] = plane;
            }
        }

        public static void TranslatePlanesInDirection(List<Plane> planes, Vector3d translation)
        {
            for (int i = 0; i < planes.Count; i++)
            {
                Plane plane = planes[i];
                plane.Translate(translation);
                planes[i] = plane;
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

        public override GH_Exposure Exposure => GH_Exposure.quinary;

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("DC32D7A4-7D65-4946-87E1-55F3D5288F8E"); }
        }
    }
}