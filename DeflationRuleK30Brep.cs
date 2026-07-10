using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Aperiodic
{
    public class DeflationRuleK30Brep : GH_Component
    {
        // Cache commonly used constants
        private static readonly double GoldenRatio = (1 + Math.Sqrt(5)) / 2;
        private static readonly double DeflationScaleFactor = Math.Pow(GoldenRatio, 3);
        private static readonly double InverseDeflationScaleFactor = 1.0 / DeflationScaleFactor;

        /// <summary>
        /// Initializes a new instance of the DeflationRuleK30Brep class.
        /// </summary>
        public DeflationRuleK30Brep()
          : base("DeflationRuleK30Brep", "DefK30Brep",
              "Output planes corresponding the the deflation rules for the K30 tile (additionally output breps and points)",
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
            pManager.AddBrepParameter("outputBreps", "outBreps", "Output breps after applying deflation rule K30", GH_ParamAccess.tree);
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

            Brep brep = refK30.DuplicateBrep();

            // Get the centroid of the geometry
            AreaMassProperties ampBrep = AreaMassProperties.Compute(brep);
            Point3d centroidBrep = ampBrep.Centroid;
            // Create a vector using the centroid location
            Vector3d centroidVec = new Vector3d(centroidBrep);
            // Scale the vector by the deflationScalefactor to get the new centroid location
            // Subtract the original centroidVec to get the translation vector
            Vector3d inflateVec = centroidVec * DeflationScaleFactor - centroidVec;
            // Translate the brep to the new scaled location (but without scaling the brep itself, so the unit size remains the same)
            brep.Translate(inflateVec);

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

            // Get length reference (edge length of original triacontahedron)
           double scale = refK30.Edges[0].PointAtEnd.DistanceTo(refK30.Edges[0].PointAtStart);

            // Set up base orientation for A6 transformation later in step (h)
            Plane a6base = SetUpA6BasePlane(refA6);

            // Set up base orientation for F20 transformation later in step (k30-i)
            Plane f20base = new Plane(Point3d.Origin, -Vector3d.XAxis, -Vector3d.YAxis);

            // Begin deflation for K30
            // Add the central triacontahedron
            listK30.Add(brep);

            // Get centroid
            AreaMassProperties ampK30 = AreaMassProperties.Compute(brep);
            Point3d centroidK30 = ampK30.Centroid;

            // Add to basepts list
            ptsK30.Add(centroidK30);

            // Add to baseplns list
            //brep.FaceNormals.ComputeFaceNormals();
            // TODO: confirm ZAxis is ok here (maybe it should be negative?)
            Plane plnK30 = GetOrientedPlaneFromRhombicFace(brep, 0, Vector3d.ZAxis);
            plnsK30.Add(plnK30);

            // Get 2-fold rotational axes on faces of triacontahedron
            foreach (BrepFace f in brep.Faces)
            {
                // Get center and normal vector of the current face
                AreaMassProperties ampFace = AreaMassProperties.Compute(f);
                Point3d centerFace = ampFace.Centroid;
                Vector3d normalFace = centerFace - centroidK30;

                Plane facePlane = GetOrientedPlaneFromRhombicFace(brep, f.FaceIndex, normalFace);

                // Transform B12 to all 30 faces (step (b) of the deflation)
                Brep copyb = refB12.DuplicateBrep();
                Transform xform = Transform.PlaneToPlane(Plane.WorldXY, facePlane);
                copyb.Transform(xform);

                // Add to brep list
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
                Brep e = brep.DuplicateBrep();
                Transform xforme = Transform.PlaneToPlane(plnK30, facePlane);
                e.Transform(xforme);

                // Add to brep list
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

            // Loop through brep vertices
            foreach (BrepVertex v in brep.Vertices)
            {

                // Vertices connected to 3 edges
                if (v.EdgeIndices().Length == 3)
                {
                    threeFoldAxesPoints.Add(v.Location);
                    // Get one of the adjacent edges and the end point (that is not the same vertex)
                    Point3d orientPt = GetOneAdjacentVertexPoint(v);

                    // Save this point for orientation
                    threeFoldAxesPointsOrientations.Add(orientPt);
                }

                // Vertices connected to 5 edges
                if (v.EdgeIndices().Length == 5)
                {
                    fiveFoldAxesPoints.Add(v.Location);
                    // Get one of the adjacent edges and the end point (that is not the same vertex)
                    Point3d orientPt5 = GetOneAdjacentVertexPoint(v);
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
                Brep copyc = refA6.DuplicateBrep();
                copyc.Transform(xformThree);

                // Add to brep list
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
                BrepVertex furthestVertex = GetFurthestVertex(copyc, centroidK30);
                Point3d furthestVertexPt = furthestVertex.Location;

                // Now get the planes adjacent to this vertex - first by getting the 3 adjacent edges
                List<Point3d> edgePoints = GetAdjacentVertexPoints(furthestVertex);

                // Get mirror planes
                Plane plane0 = new Plane(furthestVertexPt, edgePoints[1], edgePoints[2]);
                Plane plane1 = new Plane(furthestVertexPt, edgePoints[2], edgePoints[0]);
                Plane plane2 = new Plane(furthestVertexPt, edgePoints[0], edgePoints[1]);
                Transform xform0 = Transform.Mirror(plane0);
                Transform xform1 = Transform.Mirror(plane1);
                Transform xform2 = Transform.Mirror(plane2);

                // Make copies
                Brep copyc0 = copyc.DuplicateBrep();
                copyc0.Transform(xform0);
                listA6.Add(copyc0);
                Brep copyc1 = copyc.DuplicateBrep();
                copyc1.Transform(xform1);
                listA6.Add(copyc1);
                Brep copyc2 = copyc.DuplicateBrep();
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
                Brep copyd0 = copyc0.DuplicateBrep();
                copyd0.Transform(xform0d);
                listA6.Add(copyd0);
                Brep copyd1 = copyc1.DuplicateBrep();
                copyd1.Transform(xform1d);
                listA6.Add(copyd1);
                Brep copyd2 = copyc2.DuplicateBrep();
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
                Brep copyg = copyc.DuplicateBrep();
                copyg.Transform(xformg);
                Transform xrot180 = Transform.Rotation(Math.PI, normal, furthestVertexPt);
                copyg.Transform(xrot180);

                // Add to brep list
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
                furthestVertex = GetFurthestVertex(copyg, centroidK30);
                furthestVertexPt = furthestVertex.Location;

                // Now get the planes adjacent to this vertex - first by getting the 3 adjacent edges
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
                Brep copyg00 = copyg0.DuplicateBrep();
                Brep copyg01 = copyg0.DuplicateBrep();

                // Again, get furthest vertex/edges
                furthestVertex = GetFurthestVertex(copyg0, centroidK30);

                // Now get the planes adjacent to this vertex - first by getting the 3 adjacent edges
                edgePoints = GetAdjacentVertexPoints(furthestVertex);

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

                // Transform the copies and add to brep list
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
                Transform xscale5 = Transform.Scale(Point3d.Origin, scale);
                pushPlane5.Transform(xscale5);
                fiveFoldPlaneOriented.Translate(pushPlane5 * 2);

                // Transform F20 to all 12 5-fold rotational axes sides (step (f) of the deflation)
                Transform xformFive = Transform.PlaneToPlane(Plane.WorldXY, fiveFoldPlaneOriented);
                Brep copyf = refF20.DuplicateBrep();
                copyf.Transform(xformFive);

                // Add to brep list
                listF20.Add(copyf);

                // Get furthest vertex
                Point3d baseptf = Point3d.Origin;
                baseptf.Transform(xformFive);

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

                // Get furthest vertex
                BrepVertex furthestVertex = GetFurthestVertex(copyf, centroidK30);
                Point3d furthestVertexPt = furthestVertex.Location;

                // Now get the planes adjacent to this vertex - first by getting the 5 adjacent edges
                // Start with one edge, then rotate around the plane to get the others in order
                Point3d edgept = GetOneAdjacentVertexPoint(furthestVertex);

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
                Brep copyi0 = refF20.DuplicateBrep();
                copyi0.Transform(xi0);
                Brep copyi1 = refF20.DuplicateBrep();
                copyi1.Transform(xi1);
                Brep copyi2 = refF20.DuplicateBrep();
                copyi2.Transform(xi2);
                Brep copyi3 = refF20.DuplicateBrep();
                copyi3.Transform(xi3);
                Brep copyi4 = refF20.DuplicateBrep();
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

            // Final Z-offset for all planes based on the deflated K30 half-height
            double zTranslation = -k30HalfHeight * DeflationScaleFactor;
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
                Point3d edgept = edge.EdgeCurve.PointAtEnd;
                if (edgept == vertex.Location)
                {
                    edgept = edge.EdgeCurve.PointAtStart;
                }
                adjacentVertexPoints.Add(edgept);
            }
            return adjacentVertexPoints;
        }

        public static Point3d GetOneAdjacentVertexPoint(BrepVertex vertex)
        {
            int firstEdgeIndex = vertex.EdgeIndices()[0];
            BrepEdge edge = vertex.Brep.Edges[firstEdgeIndex];
            Point3d edgept = edge.EdgeCurve.PointAtEnd;
            if (edgept == vertex.Location)
            {
                edgept = edge.EdgeCurve.PointAtStart;
            }
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
            double minDistance = 1000000000;
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

        // Note that this only works for placement of B12 and K30, not A6 or F20, since those are oriented based on the face center
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

        // TODO: Refactor code to stop using the x-projection method and checking cases for the angle
        // Below function is simpler and allows comparing angles on a plane without needing to project
        // This function is extremely useful since Vector3d.VectorAngle will only ever return positive values
        // Having a signed vector angle ensures we always rotate in the correct direction
        public static double GetSignedVectorAngle(Vector3d v1, Vector3d v2, Plane plane)
        {
            return Math.Atan2(Vector3d.CrossProduct(v1, v2) * plane.ZAxis, v1 * v2);
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
            if (Math.Abs(minZ) > 0.0001) // Only translate if not already at Z = 0
            {
                brep.Translate(new Vector3d(0, 0, -minZ));
            }
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

        public override GH_Exposure Exposure => GH_Exposure.quinary;

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("643F3A88-D9BE-4606-ABED-4EBF204BCAAE"); }
        }
    }
}