using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Aperiodic
{
    public class DeflationRuleB12 : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the DeflationRuleB12 class.
        /// </summary>
        public DeflationRuleB12()
          : base("DeflationRuleB12", "DefB12",
              "Output planes corresponding the the deflation rules for the B12 tile (additionally output meshes and points)",
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
            pManager.AddMeshParameter("outputMeshes", "outMeshes", "Output meshes after applying deflation rule B12", GH_ParamAccess.tree);
            pManager.AddPointParameter("outputPoints", "outPts", "Output points after applying deflation rule B12", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("outputPlanes", "outPlanes", "Output planes after applying deflation rule B12", GH_ParamAccess.tree);
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

            // Note: always duplicate these before modifying
            Mesh refA6 = reference.Branches[0][0].Value;
            Mesh refB12 = reference.Branches[1][0].Value;
            Mesh refF20 = reference.Branches[2][0].Value;
            Mesh refK30 = reference.Branches[3][0].Value;

            Mesh mesh = refB12.DuplicateMesh();
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
            //mesh.Translate(inflateVec);
            // Translate the basept to the new scaled location - consistent with the brep itself, so further operations on the base pts can recurse well
            //basept += inflateVec;
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

            // Get lenght reference (height of refA6)
            double a6HeightRef = refA6.GetBoundingBox(true).Max.Z;

            // Get length reference (height of refB12)
            double rhombLengthRef = refB12.GetBoundingBox(true).Max.Z;

            // Get length reference (height of refF20)
            double f20HeightRef = refF20.GetBoundingBox(true).Max.Z;

            // Get length reference (height of refK30)
            double k30HeightRef = refK30.GetBoundingBox(true).Max.Z;

            // Get length reference (edge length of original triacontahedron)
            //double edgeLengthRef = refK30.Edges[0].PointAtEnd.DistanceTo(refK30.Edges[0].PointAtStart); //brep code
            double edgeLengthRef = refK30.TopologyEdges.EdgeLine(0).Length;

            // TODO: can we hardcode the location of these points / planes / hardcode the data for the mesh references? these would be fixed inside the component instead of as inputs (the inputs would be geometry to transform according to the 4 types)

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

            // Set up base orientation for F20 transformation later in step (k30-i)
            Plane f20base = new Plane(Point3d.Origin, -Vector3d.XAxis, -Vector3d.YAxis);

            // Set up base orientation for K30 transformation in step (a6-000)
            Point3d k30basecenter = refK30.TopologyVertices[GetClosestVertex(refK30, new Point3d(0, -1, 0))];
            Point3d k30basexaxis = refK30.TopologyVertices[GetClosestVertex(refK30, new Point3d(1, 0, 0))];
            Point3d k30baseyaxis = new Point3d(-k30basexaxis.X, 0, 0);
            Plane k30base = new Plane(k30basecenter, k30basexaxis, k30baseyaxis);

            // Set up base orientation for B12 transformation in step (a6-000)
            Point3d b12basecenter = refB12.TopologyVertices[GetClosestVertex(refB12, new Point3d(-0.5f, 0, 0))];
            Point3d b12baseyaxis = refB12.TopologyVertices[GetClosestVertex(refB12, new Point3d(-0.5f, -0.5f, 1))];
            Point3d b12basexaxis = new Point3d(b12baseyaxis.X, -b12baseyaxis.Y, b12baseyaxis.Z);
            Plane b12base = new Plane(b12basecenter, b12basexaxis, b12baseyaxis);

            //
            // Get centroid
            AreaMassProperties ampB12 = AreaMassProperties.Compute(mesh);
            Point3d centroidB12 = ampB12.Centroid;

            // Scale up B12 unit to get general boundaries of the inflated shapes
            // Scale center point of geometry by a factor of golden ratio^3
            //Transform xformScaleB12 = Transform.Scale(centroidB12, deflationScaleFactor);
            Transform xformScaleB12 = Transform.Scale(Point3d.Origin, deflationScaleFactor);
            Mesh b12boundary = mesh.DuplicateMesh();
            b12boundary.Transform(xformScaleB12);
            Point3d boundarybasept = new Point3d(basept);
            boundarybasept.Transform(xformScaleB12);

            // Test/visualize
            //listB12.Add(b12boundary);
            //ptsB12.Add(boundarybasept);

            // Find top face
            int farFaceIndex = GetFurthestFace(b12boundary, boundarybasept);
            Point3d centerTopPoint = b12boundary.Faces.GetFaceCenter(farFaceIndex);

            // Get face normal vector (pointing inwards)
            Vector3d orientb12k300 = boundarybasept - centerTopPoint;

            // Get oriented plane
            Plane planeb12k300 = GetOrientedPlaneFromRhombicFace(b12boundary, farFaceIndex, orientb12k300);
            Transform xformb12k300 = Transform.PlaneToPlane(Plane.WorldXY, planeb12k300);

            // Add K30
            Mesh b12k300 = refK30.DuplicateMesh();
            b12k300.Transform(xformb12k300);

            // Add to mesh list
            listK30.Add(b12k300);

            // Add to basepts list
            ptsK30.Add(centerTopPoint);

            // Add to baseplns list
            plnsK30.Add(planeb12k300);

            // Must call this before referencing mesh face normals
            b12k300.FaceNormals.ComputeFaceNormals();
            b12k300.UnifyNormals();

            // Place more B12 around the K30. Loop through faces of the K30
            for (int i = 0; i < b12k300.Faces.Count; i++)
            {
                // Get normal of face and see if it has positive dot product with normal
                Point3d b12b12base = b12k300.Faces.GetFaceCenter(i);
                Vector3d b12b12normal = b12k300.FaceNormals[i];
                double compareFaceNormal = Vector3d.Multiply(b12b12normal, planeb12k300.Normal);
                if (compareFaceNormal > 0.35)
                {
                    // Add a b12 to the list using this face as a base
                    Plane planeb12b12 = GetOrientedPlaneFromRhombicFace(b12k300, i, b12b12normal);
                    Transform xformb12b12 = Transform.PlaneToPlane(Plane.WorldXY, planeb12b12);
                    Mesh b12b12 = refB12.DuplicateMesh();
                    b12b12.Transform(xformb12b12);

                    // Add to mesh list
                    listB12.Add(b12b12);

                    // Add to basepts list
                    ptsB12.Add(b12b12base);

                    // Add to baseplns list
                    plnsB12.Add(planeb12b12);

                    // Transform K30 further along the same axes
                    // First push the plane further out
                    Vector3d pushPlane = new Vector3d(b12b12normal);
                    pushPlane.Unitize();
                    Transform xscale = Transform.Scale(Point3d.Origin, rhombLengthRef);
                    pushPlane.Transform(xscale);
                    planeb12b12.Translate(pushPlane);
                    Mesh e = refK30.DuplicateMesh();
                    Transform xforme = Transform.PlaneToPlane(Plane.WorldXY, planeb12b12);
                    e.Transform(xforme);

                    // Add to mesh list
                    listK30.Add(e);

                    // Transform the basepoint from the origin
                    Point3d ebasept = Point3d.Origin;
                    ebasept.Transform(xforme);

                    // Add to basepts list
                    ptsK30.Add(ebasept);

                    // Add to baseplns list
                    plnsK30.Add(planeb12b12);
                }
                // Add 2 more b12 on the sides
                // Note: needed to add tolerances since != 0 and == 0 were not giving correct results
                else if ((compareFaceNormal < 0.0001) && (compareFaceNormal > -0.0001))
                {
                    Plane planeb12b12 = GetOrientedPlaneFromRhombicFace(b12k300, i, b12b12normal);
                    double compareFaceOrientation = Vector3d.Multiply(planeb12b12.XAxis, planeb12k300.Normal);
                    if ((compareFaceOrientation < -0.0001) || (compareFaceOrientation > 0.0001))
                    {
                        Transform xformb12b12 = Transform.PlaneToPlane(Plane.WorldXY, planeb12b12);
                        Mesh b12b12 = refB12.DuplicateMesh();
                        b12b12.Transform(xformb12b12);

                        // Add to mesh list
                        listB12.Add(b12b12);

                        // Add to basepts list
                        ptsB12.Add(b12b12base);

                        // Add to baseplns list
                        plnsB12.Add(planeb12b12);

                        // Transform K30 further along the same axes
                        // First push the plane further out
                        Vector3d pushPlane = new Vector3d(b12b12normal);
                        pushPlane.Unitize();
                        Transform xscale = Transform.Scale(Point3d.Origin, rhombLengthRef);
                        pushPlane.Transform(xscale);
                        planeb12b12.Translate(pushPlane);
                        Mesh e = refK30.DuplicateMesh();
                        Transform xforme = Transform.PlaneToPlane(Plane.WorldXY, planeb12b12);
                        e.Transform(xforme);

                        // Add to mesh list
                        listK30.Add(e);

                        // Transform the basepoint from the origin
                        Point3d ebasept = Point3d.Origin;
                        ebasept.Transform(xforme);

                        // Add to basepts list
                        ptsK30.Add(ebasept);

                        // Add to baseplns list
                        plnsK30.Add(planeb12b12);

                        // Also place 3 A6 tiles on underside
                        // Start with face facing down on these b12
                        b12b12.FaceNormals.ComputeFaceNormals();
                        for (int j = 0; j < b12b12.FaceNormals.Count; j++)
                        {
                            Vector3d b12b12facenormal = (Vector3d)b12b12.FaceNormals[j];
                            if (b12b12facenormal.IsParallelTo(orientb12k300) == -1)
                            {
                                // Place at A6 tile
                                Plane planeb12a61 = GetOrientedPlaneFromRhombicFace(b12b12, j, b12b12facenormal);
                                // Move plane
                                Vector3d shiftb12a61 = -planeb12a61.YAxis;
                                shiftb12a61.Unitize();
                                planeb12a61.Translate(shiftb12a61 * rhombLengthRef / 2);
                                // Rotate plane (not super neat)
                                double acuteRhombusAngle = Math.Atan(2);
                                planeb12a61.Rotate((Math.PI - acuteRhombusAngle) / 2, planeb12a61.Normal);
                                Transform xformb12a61 = Transform.PlaneToPlane(a6base, planeb12a61);
                                Mesh b12a61 = refA6.DuplicateMesh();
                                b12a61.Transform(xformb12a61);

                                // Add to mesh list
                                listA6.Add(b12a61);

                                // Tranform basept
                                Point3d b12a61base = Point3d.Origin;
                                b12a61base.Transform(xformb12a61);

                                // Add to basepts list
                                ptsA6.Add(b12a61base);

                                // Add to baseplns list
                                Plane planeb12a61adjust = Plane.WorldXY;
                                planeb12a61adjust.Transform(xformb12a61);
                                plnsA6.Add(planeb12a61adjust);

                                // Mirror the A6 on two sides
                                // Get furthest vertex of the copy
                                int furthestVertexIndex = GetFurthestVertex(b12a61, (Point3d)b12a61base);
                                Point3d furthestVertexPt = (Point3d)b12a61.TopologyVertices[furthestVertexIndex];

                                // Get adjacent vertex points
                                adjacentVertexIndices = b12a61.Vertices.GetConnectedVertices(furthestVertexIndex);
                                List<Point3d> edgePoints = new List<Point3d>();
                                for (int k = 0; k < 3; k++)
                                {
                                    edgePoints.Add((Point3d)b12a61.Vertices[adjacentVertexIndices[k]]);
                                }

                                // Get mirror planes - but we don't want the one parallel to original base plane
                                Plane plane0 = new Plane(furthestVertexPt, edgePoints[1], edgePoints[2]);
                                Plane plane1 = new Plane(furthestVertexPt, edgePoints[2], edgePoints[0]);
                                Plane plane2 = new Plane(furthestVertexPt, edgePoints[0], edgePoints[1]);
                                if (plane0.Normal.IsParallelTo(planeb12a61.Normal) == 0)
                                {
                                    Transform xform0 = Transform.Mirror(plane0);
                                    Mesh copyc0 = b12a61.DuplicateMesh();
                                    copyc0.Transform(xform0);

                                    // Add to mesh list
                                    listA6.Add(copyc0);

                                    Point3d c0basept = new Point3d(b12a61base);
                                    c0basept.Transform(xform0);

                                    // Add to basepts list
                                    ptsA6.Add(c0basept);

                                    // Add to baseplns list
                                    Plane planecopyc0 = new Plane(planeb12a61adjust);
                                    planecopyc0.Transform(xform0);
                                    planecopyc0.Flip();
                                    planecopyc0.Rotate(Math.PI / 2, planecopyc0.Normal);
                                    plnsA6.Add(planecopyc0);
                                }
                                if (plane1.Normal.IsParallelTo(planeb12a61.Normal) == 0)
                                {
                                    Transform xform1 = Transform.Mirror(plane1);
                                    Mesh copyc1 = b12a61.DuplicateMesh();
                                    copyc1.Transform(xform1);

                                    // Add to mesh list
                                    listA6.Add(copyc1);

                                    Point3d c1basept = new Point3d(b12a61base);
                                    c1basept.Transform(xform1);

                                    // Add to basepts list
                                    ptsA6.Add(c1basept);

                                    // Add to baseplns list
                                    Plane planecopyc1 = new Plane(planeb12a61adjust);
                                    planecopyc1.Transform(xform1);
                                    planecopyc1.Flip();
                                    planecopyc1.Rotate(Math.PI / 2, planecopyc1.Normal);
                                    plnsA6.Add(planecopyc1);
                                }
                                if (plane2.Normal.IsParallelTo(planeb12a61.Normal) == 0)
                                {
                                    Transform xform2 = Transform.Mirror(plane2);
                                    Mesh copyc2 = b12a61.DuplicateMesh();
                                    copyc2.Transform(xform2);

                                    // Add to mesh list
                                    listA6.Add(copyc2);

                                    Point3d c2basept = new Point3d(b12a61base);
                                    c2basept.Transform(xform2);

                                    // Add to basepts list
                                    ptsA6.Add(c2basept);

                                    // Add to baseplns list
                                    Plane planecopyc2 = new Plane(planeb12a61adjust);
                                    planecopyc2.Transform(xform2);
                                    planecopyc2.Flip();
                                    planecopyc2.Rotate(Math.PI / 2, planecopyc2.Normal);
                                    plnsA6.Add(planecopyc2);
                                }
                            }
                        }
                    }
                }
            }

            // Must call this before referencing mesh vertex normals
            b12k300.Normals.ComputeNormals();
            b12k300.UnifyNormals();

            // Loop through vertices of the K30. Check for the ones with 3 neighbors that are also facing down
            for (int i = 0; i < b12k300.TopologyVertices.Count; i++)
            {
                if (b12k300.TopologyVertices.ConnectedEdgesCount(i) == 3)
                {
                    Vector3d normal = b12k300.Normals[i];
                    if (Vector3d.Multiply(normal, planeb12k300.Normal) > 0.0001)
                    {
                        // Get plane center
                        Point3d planeCenter = b12k300.TopologyVertices[i];

                        // Get orient point using adjacent  vertex
                        int orientPtIndex = b12k300.TopologyVertices.ConnectedTopologyVertices(i)[0];
                        Point3d orientX = (Point3d)b12k300.TopologyVertices[orientPtIndex];

                        // First create a plane using center and normal vector
                        Plane planeUnoriented = new Plane(planeCenter, normal);

                        // Project orientX point onto that plane
                        Transform xformprojectb12a6 = Transform.PlanarProjection(planeUnoriented);
                        orientX.Transform(xformprojectb12a6);

                        // Rotate orientX point by 90 degress (angle is arbitrary) on that plane to get orientY point
                        Transform rotate90 = Transform.Rotation(Math.PI / 2, normal, planeCenter);
                        Point3d orientY = new Point3d(orientX);
                        orientY.Transform(rotate90);

                        // Get the final oriented plane for transformation
                        Plane planeOriented = new Plane(planeCenter, orientX, orientY);

                        // Transform A6
                        Transform xformb12a60 = Transform.PlaneToPlane(Plane.WorldXY, planeOriented);
                        Mesh b12a60 = refA6.DuplicateMesh();
                        b12a60.Transform(xformb12a60);

                        // Add to mesh list
                        listA6.Add(b12a60);

                        // Transform basept
                        Point3d b12a60base = Point3d.Origin;
                        b12a60base.Transform(xformb12a60);

                        // Add to basepts list
                        ptsA6.Add(b12a60base);

                        // Add to baseplns list
                        plnsA6.Add(planeOriented);

                        // Also check to copy further out on 2 ends
                        if ((Vector3d.Multiply(normal, planeb12k300.Normal) < 0.4) && (Vector3d.Multiply(normal, planeb12k300.Normal) > 0.3))
                        {
                            // Reflect out once more - get mirror plane from outer vertex
                            int outerVertexFlip = GetFurthestVertex(b12a60, (Point3d)b12a60base);
                            b12a60.Normals.ComputeNormals();
                            Point3d b12a63origin = b12a60.TopologyVertices[outerVertexFlip];
                            Vector3d b12a63normal = b12a60.Normals[outerVertexFlip];
                            Plane planeb12a63 = new Plane(b12a63origin, b12a63normal);
                            Transform mirrorb12a63 = Transform.Mirror(planeb12a63);
                            // Perform mirror transformation, then rotate 60 degrees
                            Mesh b12a63 = b12a60.DuplicateMesh();
                            b12a63.Transform(mirrorb12a63);
                            b12a63.Rotate(Math.PI / 3, b12a63normal, b12a63origin);

                            // Add to mesh list
                            listA6.Add(b12a63);

                            Point3d b12a63base = new Point3d(b12a60base);
                            b12a63base.Transform(mirrorb12a63);

                            // Add to basepts list
                            ptsA6.Add(b12a63base);

                            // Add to baseplns list
                            Plane planeb12a63base = new Plane(planeOriented);
                            planeb12a63base.Transform(mirrorb12a63);
                            planeb12a63base.Flip();
                            planeb12a63base.Rotate(-Math.PI / 2, planeb12a63base.Normal);
                            plnsA6.Add(planeb12a63base);

                            // Mirror this once more
                            int faceFlip = GetFurthestFace(b12a63, (Point3d)b12a63origin - orientb12k300);
                            Point3d centerFaceFlip = b12a63.Faces.GetFaceCenter(faceFlip);
                            Vector3d faceFlipNormal = b12a63.FaceNormals[faceFlip];
                            Plane planeb12a64 = new Plane(centerFaceFlip, faceFlipNormal);
                            Transform mirrorb12a64 = Transform.Mirror(planeb12a64);
                            Mesh b12a64 = b12a63.DuplicateMesh();
                            b12a64.Transform(mirrorb12a64);

                            // Add to mesh list
                            listA6.Add(b12a64);

                            Point3d b12a64base = new Point3d(b12a63base);
                            b12a64base.Transform(mirrorb12a64);

                            // Add to basepts list
                            ptsA6.Add(b12a64base);

                            // Add to baseplns list
                            Plane planeb12a64base = new Plane(planeb12a63base);
                            planeb12a64base.Transform(mirrorb12a64);
                            planeb12a64base.Flip();
                            planeb12a64base.Rotate(Math.PI / 2, planeb12a64base.Normal);
                            plnsA6.Add(planeb12a64base);

                            // Finally use edges from this last A6 as axis for placing F20 on the end
                            int outerVertexIndex = GetClosestVertex(b12a64, (Point3d)b12a64base);
                            Point3d outerVertexPt = (Point3d)b12a64.TopologyVertices[outerVertexIndex];

                            // Get adjacent vertex points and find top adjacent vertex
                            adjacentVertexIndices = b12a64.Vertices.GetConnectedVertices(outerVertexIndex);
                            int topAdjacentVertexIndex = 0;
                            Point3d topAdjacentVertex = new Point3d();
                            for (int k = 0; k < 3; k++)
                            {
                                Point3d possiblePt = (Point3d)b12a64.Vertices[adjacentVertexIndices[k]];
                                if (Math.Abs(planeb12a64.DistanceTo(possiblePt)) > 0.0000001)
                                {
                                    topAdjacentVertex = possiblePt;
                                    topAdjacentVertexIndex = adjacentVertexIndices[k];
                                }
                            }

                            // Get normal vector for F20 placement, unrotated plane
                            Vector3d normalb12f20 = topAdjacentVertex - b12a64base;
                            Point3d centerb12f20 = topAdjacentVertex;
                            Plane unrotatedb12f20 = new Plane(centerb12f20, normalb12f20);

                            // Get one more adjacent vertex to this (that is not base pt)
                            adjacentVertexIndices = b12a64.Vertices.GetConnectedVertices(topAdjacentVertexIndex);
                            Point3d sideAdjacentVertex = new Point3d();
                            for (int k = 0; k < 3; k++)
                            {
                                Point3d possiblePt = (Point3d)b12a64.Vertices[adjacentVertexIndices[k]];
                                if (Math.Abs(planeb12a64.DistanceTo(possiblePt)) > 0.0000001)
                                {
                                    sideAdjacentVertex = possiblePt;
                                }
                            }
                            Point3d xPt = new Point3d(sideAdjacentVertex);
                            Transform xformProjectb12f20 = Transform.PlanarProjection(unrotatedb12f20);
                            xPt.Transform(xformProjectb12f20);
                            Point3d yPt = new Point3d(xPt);
                            Transform xrot = Transform.Rotation(Math.PI / 2, normalb12f20, centerb12f20);
                            yPt.Transform(xrot);

                            // Construct the plane for F20 placement
                            Plane b12f20 = new Plane(centerb12f20, xPt, yPt);
                            b12f20.Rotate(Math.PI, normalb12f20);

                            // TEST flipping these incorrect planes the other way
                            Plane flippedPln = new Plane(GetFlippedF20Plane(b12f20, f20HeightRef));
                            b12f20 = flippedPln;
                            Point3d b12f205base = flippedPln.Origin;

                            Transform xformorientb12f20 = Transform.PlaneToPlane(Plane.WorldXY, b12f20);
                            Mesh b12f205 = refF20.DuplicateMesh();
                            b12f205.Transform(xformorientb12f20);

                            // Add to mesh list
                            listF20.Add(b12f205);

                            //Point3d b12f205base = new Point3d(b12a64base);

                            // Add to basepts list
                            ptsF20.Add(b12f205base);

                            // Add to baseplns list
                            plnsF20.Add(b12f20);
                        }

                        // Copy A6 3 more times...
                        int furthestVertexIndex = GetFurthestVertex(b12a60, (Point3d)b12a60base);
                        Point3d furthestVertexPt = (Point3d)b12a60.TopologyVertices[furthestVertexIndex];

                        // Get adjacent vertex points
                        adjacentVertexIndices = b12a60.Vertices.GetConnectedVertices(furthestVertexIndex);
                        List<Point3d> edgePoints = new List<Point3d>();
                        for (int j = 0; j < 3; j++)
                        {
                            edgePoints.Add((Point3d)b12a60.Vertices[adjacentVertexIndices[j]]);
                        }

                        // Get mirror planes
                        Plane plane0 = new Plane(furthestVertexPt, edgePoints[1], edgePoints[2]);
                        Plane plane1 = new Plane(furthestVertexPt, edgePoints[2], edgePoints[0]);
                        Plane plane2 = new Plane(furthestVertexPt, edgePoints[0], edgePoints[1]);
                        Transform xform0 = Transform.Mirror(plane0);
                        Transform xform1 = Transform.Mirror(plane1);
                        Transform xform2 = Transform.Mirror(plane2);

                        // Make copies
                        Mesh copyc0 = b12a60.DuplicateMesh();
                        copyc0.Transform(xform0);

                        // Add to mesh list
                        listA6.Add(copyc0);
                        Mesh copyc1 = b12a60.DuplicateMesh();
                        copyc1.Transform(xform1);
                        listA6.Add(copyc1);
                        Mesh copyc2 = b12a60.DuplicateMesh();
                        copyc2.Transform(xform2);
                        listA6.Add(copyc2);

                        // Copy base pts
                        Point3d c0basept = new Point3d(b12a60base);
                        c0basept.Transform(xform0);
                        Point3d c1basept = new Point3d(b12a60base);
                        c1basept.Transform(xform1);
                        Point3d c2basept = new Point3d(b12a60base);
                        c2basept.Transform(xform2);

                        // Add to basepts list
                        ptsA6.Add(c0basept);
                        ptsA6.Add(c1basept);
                        ptsA6.Add(c2basept);

                        // Copy baseplns
                        planeOriented.Flip();
                        planeOriented.Rotate(Math.PI / 2, planeOriented.Normal);
                        Plane c0baseplane = new Plane(planeOriented);
                        Plane c1baseplane = new Plane(planeOriented);
                        Plane c2baseplane = new Plane(planeOriented);
                        c0baseplane.Transform(xform0);
                        c1baseplane.Transform(xform1);
                        c2baseplane.Transform(xform2);
                        plnsA6.Add(c0baseplane);
                        plnsA6.Add(c1baseplane);
                        plnsA6.Add(c2baseplane);
                    }
                }
                // Five-fold vertices
                else if (b12k300.TopologyVertices.ConnectedEdgesCount(i) == 5)
                {
                    Vector3d normal = b12k300.Normals[i];
                    double vectoreCompareb12k30 = Vector3d.Multiply(normal, planeb12k300.Normal);
                    if (vectoreCompareb12k30 > 0.8)
                    {
                        // Use the plane center and the orientation point
                        Point3d planeCenter5 = (Point3d)b12k300.TopologyVertices[i];
                        // Get orient point using adjacent  vertex
                        int orientPtIndex5 = b12k300.TopologyVertices.ConnectedTopologyVertices(i)[0];
                        Point3d orientX5 = (Point3d)b12k300.TopologyVertices[orientPtIndex5];

                        // First create a plane using center and normal vector
                        Vector3d normal5 = b12k300.Normals[i];
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

                        // TEST flipping these incorrect planes the other way
                        Plane flippedPln2 = new Plane(GetFlippedF20Plane(fiveFoldPlaneOriented, f20HeightRef));
                        //b12f20 = flippedPln;
                        //Point3d b12f205base = flippedPln.Origin;

                        // Transform two F20 to these 2 5-fold rotational axes
                        //Transform xformFive = Transform.PlaneToPlane(Plane.WorldXY, fiveFoldPlaneOriented);
                        Transform xformFive = Transform.PlaneToPlane(Plane.WorldXY, flippedPln2);
                        Mesh copyf = refF20.DuplicateMesh();
                        copyf.Transform(xformFive);

                        // Add to mesh list
                        listF20.Add(copyf);

                        // Transform base pts
                        Point3d baseptf = Point3d.Origin;
                        baseptf.Transform(xformFive);

                        // Add to basepts list
                        ptsF20.Add(baseptf);

                        // Add to baseplns list
                        //plnsF20.Add(fiveFoldPlaneOriented);
                        plnsF20.Add(flippedPln2);

                        // Using this F20, we find the two poles and array five A6 around both...
                        // Get furthest vertex
                        int furthestVertexIndex = GetFurthestVertex(copyf, (Point3d)planeCenter5);
                        Point3d furthestVertexPt = (Point3d)copyf.TopologyVertices[furthestVertexIndex];

                        // Get one adjacent vertex
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

                        // Copy baseplns and add
                        Plane basepln0 = new Plane(Plane.WorldXY);
                        Plane basepln1 = new Plane(Plane.WorldXY);
                        Plane basepln2 = new Plane(Plane.WorldXY);
                        Plane basepln3 = new Plane(Plane.WorldXY);
                        Plane basepln4 = new Plane(Plane.WorldXY);
                        basepln0.Transform(xh0);
                        basepln1.Transform(xh1);
                        basepln2.Transform(xh2);
                        basepln3.Transform(xh3);
                        basepln4.Transform(xh4);
                        plnsA6.Add(basepln0);
                        plnsA6.Add(basepln1);
                        plnsA6.Add(basepln2);
                        plnsA6.Add(basepln3);
                        plnsA6.Add(basepln4);

                        // A6 array of 5 on the inner side the F20
                        // Get closest vertex
                        int closestVertexIndex = GetClosestVertex(copyf, (Point3d)planeCenter5);
                        Point3d closestVertexPt = (Point3d)copyf.TopologyVertices[closestVertexIndex];

                        // Get one adjacent vertex
                        adjacentVertexIndices = copyf.Vertices.GetConnectedVertices(closestVertexIndex);
                        edgept = (Point3d)copyf.Vertices[adjacentVertexIndices[0]];

                        // Start with one edge, then rotate around the plane to get the others in order
                        edgePoints = new List<Point3d>();
                        edgePoints.Add(edgept);
                        for (int j = 1; j < 5; j++)
                        {
                            // Rotate edgept around normal -2pi/5 degrees
                            Transform xrot5 = Transform.Rotation(-2 * Math.PI / 5, normal5, planeCenter5);
                            edgept.Transform(xrot5);
                            edgePoints.Add(edgept);
                        }

                        // Get planes
                        planeh0 = new Plane(closestVertexPt, edgePoints[0], edgePoints[1]);
                        planeh1 = new Plane(closestVertexPt, edgePoints[1], edgePoints[2]);
                        planeh2 = new Plane(closestVertexPt, edgePoints[2], edgePoints[3]);
                        planeh3 = new Plane(closestVertexPt, edgePoints[3], edgePoints[4]);
                        planeh4 = new Plane(closestVertexPt, edgePoints[4], edgePoints[0]);

                        // Make tranformations
                        xh0 = Transform.PlaneToPlane(a6base, planeh0);
                        xh1 = Transform.PlaneToPlane(a6base, planeh1);
                        xh2 = Transform.PlaneToPlane(a6base, planeh2);
                        xh3 = Transform.PlaneToPlane(a6base, planeh3);
                        xh4 = Transform.PlaneToPlane(a6base, planeh4);

                        // Make copies and transform
                        copyh0 = refA6.DuplicateMesh();
                        copyh0.Transform(xh0);
                        copyh1 = refA6.DuplicateMesh();
                        copyh1.Transform(xh1);
                        copyh2 = refA6.DuplicateMesh();
                        copyh2.Transform(xh2);
                        copyh3 = refA6.DuplicateMesh();
                        copyh3.Transform(xh3);
                        copyh4 = refA6.DuplicateMesh();
                        copyh4.Transform(xh4);

                        // Add to output
                        listA6.Add(copyh0);
                        listA6.Add(copyh1);
                        listA6.Add(copyh2);
                        listA6.Add(copyh3);
                        listA6.Add(copyh4);

                        // Copy baseplns and add
                        basepln0 = new Plane(Plane.WorldXY);
                        basepln1 = new Plane(Plane.WorldXY);
                        basepln2 = new Plane(Plane.WorldXY);
                        basepln3 = new Plane(Plane.WorldXY);
                        basepln4 = new Plane(Plane.WorldXY);
                        basepln0.Transform(xh0);
                        basepln1.Transform(xh1);
                        basepln2.Transform(xh2);
                        basepln3.Transform(xh3);
                        basepln4.Transform(xh4);

                        // Additional transformation to flip
                        Transform xh02 = Transform.PlaneToPlane(basepln0, GetFlippedA6Plane(basepln0, a6HeightRef));
                        Transform xh12 = Transform.PlaneToPlane(basepln1, GetFlippedA6Plane(basepln1, a6HeightRef));
                        Transform xh22 = Transform.PlaneToPlane(basepln2, GetFlippedA6Plane(basepln2, a6HeightRef));
                        Transform xh32 = Transform.PlaneToPlane(basepln3, GetFlippedA6Plane(basepln3, a6HeightRef));
                        Transform xh42 = Transform.PlaneToPlane(basepln4, GetFlippedA6Plane(basepln4, a6HeightRef));
                        basepln0.Transform(xh02);
                        basepln1.Transform(xh12);
                        basepln2.Transform(xh22);
                        basepln3.Transform(xh32);
                        basepln4.Transform(xh42);

                        plnsA6.Add(basepln0);
                        plnsA6.Add(basepln1);
                        plnsA6.Add(basepln2);
                        plnsA6.Add(basepln3);
                        plnsA6.Add(basepln4);

                        // Copy base pts and add
                        basepth0 = Point3d.Origin;
                        basepth0.Transform(xh0);
                        basepth0.Transform(xh02);
                        ptsA6.Add(basepth0);
                        basepth1 = Point3d.Origin;
                        basepth1.Transform(xh1);
                        basepth1.Transform(xh12);
                        ptsA6.Add(basepth1);
                        basepth2 = Point3d.Origin;
                        basepth2.Transform(xh2);
                        basepth2.Transform(xh22);
                        ptsA6.Add(basepth2);
                        basepth3 = Point3d.Origin;
                        basepth3.Transform(xh3);
                        basepth3.Transform(xh32);
                        ptsA6.Add(basepth3);
                        basepth4 = Point3d.Origin;
                        basepth4.Transform(xh4);
                        basepth4.Transform(xh42);
                        ptsA6.Add(basepth4);
                    }

                    // For two other 5-fold vertices of the K30 we array A6 tiles around
                    else if ((vectoreCompareb12k30 > -0.8) && (vectoreCompareb12k30 < -0.2))
                    {
                        // Get one adjacent vertex
                        Point3d centerpt = (Point3d)b12k300.TopologyVertices[i];
                        adjacentVertexIndices = b12k300.Vertices.GetConnectedVertices(i);
                        Point3d edgept = (Point3d)b12k300.Vertices[adjacentVertexIndices[0]];
                        Vector3d b12a62normal = b12k300.Normals[i];

                        // Start with one edge, then rotate around the plane to get the others in order
                        List<Point3d> edgePoints = new List<Point3d>();
                        edgePoints.Add(edgept);
                        for (int j = 1; j < 5; j++)
                        {
                            // Rotate edgept around normal 2pi/5 degrees
                            Transform xrot5 = Transform.Rotation(2 * Math.PI / 5, b12a62normal, centerpt);
                            edgept.Transform(xrot5);
                            edgePoints.Add(edgept);
                        }

                        // Get planes
                        Plane planeh0 = new Plane(centerpt, edgePoints[0], edgePoints[1]);
                        Plane planeh1 = new Plane(centerpt, edgePoints[1], edgePoints[2]);
                        Plane planeh2 = new Plane(centerpt, edgePoints[2], edgePoints[3]);
                        Plane planeh3 = new Plane(centerpt, edgePoints[3], edgePoints[4]);
                        Plane planeh4 = new Plane(centerpt, edgePoints[4], edgePoints[0]);

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

                        // Copy baseplns and add
                        Plane basepln0 = new Plane(Plane.WorldXY);
                        Plane basepln1 = new Plane(Plane.WorldXY);
                        Plane basepln2 = new Plane(Plane.WorldXY);
                        Plane basepln3 = new Plane(Plane.WorldXY);
                        Plane basepln4 = new Plane(Plane.WorldXY);
                        basepln0.Transform(xh0);
                        basepln1.Transform(xh1);
                        basepln2.Transform(xh2);
                        basepln3.Transform(xh3);
                        basepln4.Transform(xh4);

                        // Additional transformation to flip
                        Transform xh02 = Transform.PlaneToPlane(basepln0, GetFlippedA6Plane(basepln0, a6HeightRef));
                        Transform xh12 = Transform.PlaneToPlane(basepln1, GetFlippedA6Plane(basepln1, a6HeightRef));
                        Transform xh22 = Transform.PlaneToPlane(basepln2, GetFlippedA6Plane(basepln2, a6HeightRef));
                        Transform xh32 = Transform.PlaneToPlane(basepln3, GetFlippedA6Plane(basepln3, a6HeightRef));
                        Transform xh42 = Transform.PlaneToPlane(basepln4, GetFlippedA6Plane(basepln4, a6HeightRef));
                        basepln0.Transform(xh02);
                        basepln1.Transform(xh12);
                        basepln2.Transform(xh22);
                        basepln3.Transform(xh32);
                        basepln4.Transform(xh42);

                        plnsA6.Add(basepln0);
                        plnsA6.Add(basepln1);
                        plnsA6.Add(basepln2);
                        plnsA6.Add(basepln3);
                        plnsA6.Add(basepln4);

                        // Copy base pts and add
                        Point3d basepth0 = Point3d.Origin;
                        Point3d basepth1 = Point3d.Origin;
                        Point3d basepth2 = Point3d.Origin;
                        Point3d basepth3 = Point3d.Origin;
                        Point3d basepth4 = Point3d.Origin;
                        basepth0.Transform(xh0);
                        basepth1.Transform(xh1);
                        basepth2.Transform(xh2);
                        basepth3.Transform(xh3);
                        basepth4.Transform(xh4);
                        basepth0.Transform(xh02);
                        basepth1.Transform(xh12);
                        basepth2.Transform(xh22);
                        basepth3.Transform(xh32);
                        basepth4.Transform(xh42);
                        ptsA6.Add(basepth0);
                        ptsA6.Add(basepth1);
                        ptsA6.Add(basepth2);
                        ptsA6.Add(basepth3);
                        ptsA6.Add(basepth4);

                        // We also want to mirror these to the opposite side of the configuration of the overall deflation
                        Plane planeb12a62 = new Plane(planeb12k300);
                        Vector3d pushb12a62 = planeb12k300.Normal;
                        pushb12a62.Unitize();
                        planeb12a62.Translate(pushb12a62 * (k30HeightRef + rhombLengthRef / 2));
                        Transform xformMirrorb12a62 = Transform.Mirror(planeb12a62);

                        // Add to mesh list, add to basepts list
                        Mesh copyh0b = copyh0.DuplicateMesh();
                        copyh0b.Transform(xformMirrorb12a62);
                        listA6.Add(copyh0b);
                        basepth0.Transform(xformMirrorb12a62);
                        ptsA6.Add(basepth0);

                        Mesh copyh1b = copyh1.DuplicateMesh();
                        copyh1b.Transform(xformMirrorb12a62);
                        listA6.Add(copyh1b);
                        basepth1.Transform(xformMirrorb12a62);
                        ptsA6.Add(basepth1);

                        Mesh copyh2b = copyh2.DuplicateMesh();
                        copyh2b.Transform(xformMirrorb12a62);
                        listA6.Add(copyh2b);
                        basepth2.Transform(xformMirrorb12a62);
                        ptsA6.Add(basepth2);

                        Mesh copyh3b = copyh3.DuplicateMesh();
                        copyh3b.Transform(xformMirrorb12a62);
                        listA6.Add(copyh3b);
                        basepth3.Transform(xformMirrorb12a62);
                        ptsA6.Add(basepth3);

                        Mesh copyh4b = copyh4.DuplicateMesh();
                        copyh4b.Transform(xformMirrorb12a62);
                        listA6.Add(copyh4b);
                        basepth4.Transform(xformMirrorb12a62);
                        ptsA6.Add(basepth4);

                        // Copy baseplns and add
                        basepln0.Transform(xformMirrorb12a62);
                        basepln1.Transform(xformMirrorb12a62);
                        basepln2.Transform(xformMirrorb12a62);
                        basepln3.Transform(xformMirrorb12a62);
                        basepln4.Transform(xformMirrorb12a62);
                        basepln0.Flip();
                        basepln1.Flip();
                        basepln2.Flip();
                        basepln3.Flip();
                        basepln4.Flip();
                        basepln0.Rotate(Math.PI / 2, basepln0.Normal);
                        basepln1.Rotate(Math.PI / 2, basepln1.Normal);
                        basepln2.Rotate(Math.PI / 2, basepln2.Normal);
                        basepln3.Rotate(Math.PI / 2, basepln3.Normal);
                        basepln4.Rotate(Math.PI / 2, basepln4.Normal);
                        plnsA6.Add(basepln0);
                        plnsA6.Add(basepln1);
                        plnsA6.Add(basepln2);
                        plnsA6.Add(basepln3);
                        plnsA6.Add(basepln4);

                        // We also want to use the edgepoints as axes for transforming/arranging F20 tiles
                        Vector3d pushb12f20j = b12a62normal;
                        pushb12f20j.Unitize();
                        Plane planeb12f20j0 = new Plane(edgePoints[0] + pushb12f20j * edgeLengthRef, edgePoints[0] - centerpt);
                        Plane planeb12f20j1 = new Plane(edgePoints[1] + pushb12f20j * edgeLengthRef, edgePoints[1] - centerpt);
                        Plane planeb12f20j2 = new Plane(edgePoints[2] + pushb12f20j * edgeLengthRef, edgePoints[2] - centerpt);
                        Plane planeb12f20j3 = new Plane(edgePoints[3] + pushb12f20j * edgeLengthRef, edgePoints[3] - centerpt);
                        Plane planeb12f20j4 = new Plane(edgePoints[4] + pushb12f20j * edgeLengthRef, edgePoints[4] - centerpt);
                        if (Vector3d.Multiply(planeb12f20j0.Normal, planeb12k300.Normal) > -0.5)
                        {
                            Transform projectPt = Transform.PlanarProjection(planeb12f20j0);
                            Point3d xaxisRef = edgePoints[0];
                            xaxisRef.Transform(projectPt);
                            double angleRot = Vector3d.VectorAngle(xaxisRef - planeb12f20j0.Origin, planeb12f20j0.XAxis);
                            planeb12f20j0.Rotate(angleRot + Math.PI, planeb12f20j0.Normal);

                            Transform xformb12f20j0 = Transform.PlaneToPlane(Plane.WorldXY, planeb12f20j0);
                            Mesh b12f20j0 = refF20.DuplicateMesh();
                            b12f20j0.Transform(xformb12f20j0);

                            // Add to mesh list
                            listF20.Add(b12f20j0);

                            Point3d baseb12f20j0 = Point3d.Origin;
                            baseb12f20j0.Transform(xformb12f20j0);

                            // Add to basepts list
                            ptsF20.Add(baseb12f20j0);

                            // Add to baseplns list
                            plnsF20.Add(planeb12f20j0);

                            // Also mirror it if normal vector is orthogonal to planeb12k300.Normal
                            if (Math.Abs(Vector3d.Multiply(planeb12f20j0.Normal, planeb12k300.Normal)) < 0.0001)
                            {
                                Mesh b12f20mirror = b12f20j0.DuplicateMesh();
                                b12f20mirror.Transform(xformMirrorb12a62);

                                // Add to mesh list
                                listF20.Add(b12f20mirror);

                                Point3d b12f20mirrorbase = new Point3d(baseb12f20j0);
                                b12f20mirrorbase.Transform(xformMirrorb12a62);

                                // Add to basepts list
                                //ptsF20.Add(b12f20mirrorbase);

                                // Add to baseplns list
                                Plane planeb12f20mirror = new Plane(planeb12f20j0);
                                planeb12f20mirror.Transform(xformMirrorb12a62);
                                planeb12f20mirror.Flip();
                                planeb12f20mirror.Rotate(Math.PI / 2, planeb12f20mirror.Normal);

                                // EDIT FLIP
                                Plane flippedPln3 = new Plane(GetFlippedF20Plane(planeb12f20mirror, f20HeightRef));
                                ptsF20.Add(flippedPln3.Origin);

                                //plnsF20.Add(planeb12f20mirror);
                                plnsF20.Add(flippedPln3);
                            }
                        }
                        if (Vector3d.Multiply(planeb12f20j1.Normal, planeb12k300.Normal) > -0.5)
                        {
                            Transform projectPt = Transform.PlanarProjection(planeb12f20j1);
                            Point3d xaxisRef = edgePoints[1];
                            xaxisRef.Transform(projectPt);
                            double angleRot = Vector3d.VectorAngle(xaxisRef - planeb12f20j1.Origin, planeb12f20j1.XAxis);
                            planeb12f20j1.Rotate(angleRot + Math.PI, planeb12f20j1.Normal);

                            Transform xformb12f20j1 = Transform.PlaneToPlane(Plane.WorldXY, planeb12f20j1);
                            Mesh b12f20j1 = refF20.DuplicateMesh();
                            b12f20j1.Transform(xformb12f20j1);

                            // Add to mesh list
                            listF20.Add(b12f20j1);

                            Point3d baseb12f20j1 = Point3d.Origin;
                            baseb12f20j1.Transform(xformb12f20j1);

                            // Add to basepts list
                            ptsF20.Add(baseb12f20j1);

                            // Add to baseplns list
                            plnsF20.Add(planeb12f20j1);

                            // Also mirror it if normal vector is orthogonal to planeb12k300.Normal
                            if (Math.Abs(Vector3d.Multiply(planeb12f20j1.Normal, planeb12k300.Normal)) < 0.0001)
                            {
                                Mesh b12f20mirror = b12f20j1.DuplicateMesh();
                                b12f20mirror.Transform(xformMirrorb12a62);

                                // Add to mesh list
                                listF20.Add(b12f20mirror);

                                Point3d b12f20mirrorbase = new Point3d(baseb12f20j1);
                                b12f20mirrorbase.Transform(xformMirrorb12a62);

                                // Add to basepts list
                                //ptsF20.Add(b12f20mirrorbase);

                                // Add to baseplns list
                                Plane planeb12f20mirror = new Plane(planeb12f20j1);
                                planeb12f20mirror.Transform(xformMirrorb12a62);
                                planeb12f20mirror.Flip();
                                planeb12f20mirror.Rotate(Math.PI / 2, planeb12f20mirror.Normal);

                                // EDIT FLIP
                                Plane flippedPln3 = new Plane(GetFlippedF20Plane(planeb12f20mirror, f20HeightRef));
                                ptsF20.Add(flippedPln3.Origin);

                                //plnsF20.Add(planeb12f20mirror);
                                plnsF20.Add(flippedPln3);
                            }
                        }
                        if (Vector3d.Multiply(planeb12f20j2.Normal, planeb12k300.Normal) > -0.5)
                        {
                            Transform projectPt = Transform.PlanarProjection(planeb12f20j2);
                            Point3d xaxisRef = edgePoints[2];
                            xaxisRef.Transform(projectPt);
                            double angleRot = Vector3d.VectorAngle(xaxisRef - planeb12f20j2.Origin, planeb12f20j2.XAxis);
                            planeb12f20j2.Rotate(angleRot + Math.PI, planeb12f20j2.Normal);

                            Transform xformb12f20j2 = Transform.PlaneToPlane(Plane.WorldXY, planeb12f20j2);
                            Mesh b12f20j2 = refF20.DuplicateMesh();
                            b12f20j2.Transform(xformb12f20j2);

                            // Add to mesh list
                            listF20.Add(b12f20j2);

                            Point3d baseb12f20j2 = Point3d.Origin;
                            baseb12f20j2.Transform(xformb12f20j2);

                            // Add to basepts list
                            ptsF20.Add(baseb12f20j2);

                            // Add to baseplns list
                            plnsF20.Add(planeb12f20j2);

                            // Also mirror it if normal vector is orthogonal to planeb12k300.Normal
                            if (Math.Abs(Vector3d.Multiply(planeb12f20j2.Normal, planeb12k300.Normal)) < 0.0001)
                            {
                                Mesh b12f20mirror = b12f20j2.DuplicateMesh();
                                b12f20mirror.Transform(xformMirrorb12a62);

                                // Add to mesh list
                                listF20.Add(b12f20mirror);

                                Point3d b12f20mirrorbase = new Point3d(baseb12f20j2);
                                b12f20mirrorbase.Transform(xformMirrorb12a62);

                                // Add to basepts list
                                //ptsF20.Add(b12f20mirrorbase);

                                // Add to baseplns list
                                Plane planeb12f20mirror = new Plane(planeb12f20j2);
                                planeb12f20mirror.Transform(xformMirrorb12a62);
                                planeb12f20mirror.Flip();
                                planeb12f20mirror.Rotate(Math.PI / 2, planeb12f20mirror.Normal);

                                // EDIT FLIP
                                Plane flippedPln3 = new Plane(GetFlippedF20Plane(planeb12f20mirror, f20HeightRef));
                                ptsF20.Add(flippedPln3.Origin);

                                //plnsF20.Add(planeb12f20mirror);
                                plnsF20.Add(flippedPln3);
                            }
                        }
                        if (Vector3d.Multiply(planeb12f20j3.Normal, planeb12k300.Normal) > -0.5)
                        {
                            Transform projectPt = Transform.PlanarProjection(planeb12f20j3);
                            Point3d xaxisRef = edgePoints[3];
                            xaxisRef.Transform(projectPt);
                            double angleRot = Vector3d.VectorAngle(xaxisRef - planeb12f20j3.Origin, planeb12f20j3.XAxis);
                            planeb12f20j3.Rotate(angleRot + Math.PI, planeb12f20j3.Normal);

                            Transform xformb12f20j3 = Transform.PlaneToPlane(Plane.WorldXY, planeb12f20j3);
                            Mesh b12f20j3 = refF20.DuplicateMesh();
                            b12f20j3.Transform(xformb12f20j3);

                            // Add to mesh list
                            listF20.Add(b12f20j3);

                            Point3d baseb12f20j3 = Point3d.Origin;
                            baseb12f20j3.Transform(xformb12f20j3);

                            // Add to basepts list
                            ptsF20.Add(baseb12f20j3);

                            // Add to baseplns list
                            plnsF20.Add(planeb12f20j3);

                            // Also mirror it if normal vector is orthogonal to planeb12k300.Normal
                            if (Math.Abs(Vector3d.Multiply(planeb12f20j3.Normal, planeb12k300.Normal)) < 0.0001)
                            {
                                Mesh b12f20mirror = b12f20j3.DuplicateMesh();
                                b12f20mirror.Transform(xformMirrorb12a62);

                                // Add to mesh list
                                listF20.Add(b12f20mirror);

                                Point3d b12f20mirrorbase = new Point3d(baseb12f20j3);
                                b12f20mirrorbase.Transform(xformMirrorb12a62);

                                // Add to basepts list
                                //ptsF20.Add(b12f20mirrorbase);

                                // Add to baseplns list
                                Plane planeb12f20mirror = new Plane(planeb12f20j3);
                                planeb12f20mirror.Transform(xformMirrorb12a62);
                                planeb12f20mirror.Flip();
                                planeb12f20mirror.Rotate(Math.PI / 2, planeb12f20mirror.Normal);

                                // EDIT FLIP
                                Plane flippedPln3 = new Plane(GetFlippedF20Plane(planeb12f20mirror, f20HeightRef));
                                ptsF20.Add(flippedPln3.Origin);

                                //plnsF20.Add(planeb12f20mirror);
                                plnsF20.Add(flippedPln3);
                            }
                        }
                        if (Vector3d.Multiply(planeb12f20j4.Normal, planeb12k300.Normal) > -0.5)
                        {
                            Transform projectPt = Transform.PlanarProjection(planeb12f20j4);
                            Point3d xaxisRef = edgePoints[4];
                            xaxisRef.Transform(projectPt);
                            double angleRot = Vector3d.VectorAngle(xaxisRef - planeb12f20j4.Origin, planeb12f20j4.XAxis);
                            planeb12f20j4.Rotate(angleRot + Math.PI, planeb12f20j4.Normal);

                            Transform xformb12f20j4 = Transform.PlaneToPlane(Plane.WorldXY, planeb12f20j4);
                            Mesh b12f20j4 = refF20.DuplicateMesh();
                            b12f20j4.Transform(xformb12f20j4);

                            // Add to mesh list
                            listF20.Add(b12f20j4);

                            Point3d baseb12f20j4 = Point3d.Origin;
                            baseb12f20j4.Transform(xformb12f20j4);

                            // Add to basepts list
                            ptsF20.Add(baseb12f20j4);

                            // Add to baseplns list
                            plnsF20.Add(planeb12f20j4);

                            // Also mirror it if normal vector is orthogonal to planeb12k300.Normal
                            if (Math.Abs(Vector3d.Multiply(planeb12f20j4.Normal, planeb12k300.Normal)) < 0.0001)
                            {
                                Mesh b12f20mirror = b12f20j4.DuplicateMesh();
                                b12f20mirror.Transform(xformMirrorb12a62);

                                // Add to mesh list
                                listF20.Add(b12f20mirror);

                                Point3d b12f20mirrorbase = new Point3d(baseb12f20j4);
                                b12f20mirrorbase.Transform(xformMirrorb12a62);

                                // Add to basepts list
                                //ptsF20.Add(b12f20mirrorbase);

                                // Add to baseplns list
                                Plane planeb12f20mirror = new Plane(planeb12f20j4);
                                planeb12f20mirror.Transform(xformMirrorb12a62);
                                planeb12f20mirror.Flip();
                                planeb12f20mirror.Rotate(Math.PI / 2, planeb12f20mirror.Normal);

                                // EDIT FLIP
                                Plane flippedPln3 = new Plane(GetFlippedF20Plane(planeb12f20mirror, f20HeightRef));
                                ptsF20.Add(flippedPln3.Origin);

                                //plnsF20.Add(planeb12f20mirror);
                                plnsF20.Add(flippedPln3);
                            }
                        }
                    }
                }
            }

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
                if (p == null) continue;
                outputpts.Append(new GH_Point(p), pth0);
            }
            foreach (var p in ptsB12)
            {
                if (p == null) continue;
                outputpts.Append(new GH_Point(p), pth1);
            }
            foreach (var p in ptsF20)
            {
                if (p == null) continue;
                outputpts.Append(new GH_Point(p), pth2);
            }
            foreach (var p in ptsK30)
            {
                if (p == null) continue;
                outputpts.Append(new GH_Point(p), pth3);
            }
            DA.SetDataTree(1, outputpts);

            // Output plns
            foreach (Plane pl in plnsA6)
            {
                if (pl == null) continue;
                outputplns.Append(new GH_Plane(pl), pth0);
            }
            foreach (Plane pl in plnsB12)
            {
                if (pl == null) continue;
                outputplns.Append(new GH_Plane(pl), pth1);
            }
            foreach (Plane pl in plnsF20)
            {
                if (pl == null) continue;
                outputplns.Append(new GH_Plane(pl), pth2);
            }
            foreach (Plane pl in plnsK30)
            {
                if (pl == null) continue;
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

        public static int GetFurthestFace(Mesh mesh, Point3d reference)
        {
            int furthestFaceIndex = 0;
            Point3d currentFaceCenter = new Point3d();
            double maxDistance = 0;
            for (int i = 0; i < mesh.Faces.Count; i++)
            {
                currentFaceCenter = mesh.Faces.GetFaceCenter(i);
                double distance = currentFaceCenter.DistanceTo(reference);
                if (distance > maxDistance)
                {
                    furthestFaceIndex = i;
                    maxDistance = distance;
                }
            }
            return furthestFaceIndex;
        }

        public static int GetClosestFace(Mesh mesh, Point3d reference)
        {
            int closestFaceIndex = 0;
            Point3d currentFaceCenter = new Point3d();
            double minDistance = 1000000;
            for (int i = 0; i < mesh.Faces.Count; i++)
            {
                currentFaceCenter = mesh.Faces.GetFaceCenter(i);
                double distance = currentFaceCenter.DistanceTo(reference);
                if (distance < minDistance)
                {
                    closestFaceIndex = i;
                    minDistance = distance;
                }
            }
            return closestFaceIndex;
        }

        public static Plane GetOrientedPlaneFromRhombicFace(Mesh mesh, int faceIndex, Vector3d normalRef)
        {
            // Get center and normal vector of the current face
            Point3d centerFace = mesh.Faces.GetFaceCenter(faceIndex);

            // Get face vertex indices and convert to Point3d for better precision
            MeshFace face = mesh.Faces[faceIndex];
            Point3d a = new Point3d(mesh.Vertices[face.A]);
            Point3d b = new Point3d(mesh.Vertices[face.B]);

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

        public static Plane GetFlippedF20Plane(Plane plntoflip, double f20HeightRef)
        {
            Vector3d normal = plntoflip.Normal;
            Vector3d movePlane = new Vector3d(normal);
            movePlane.Unitize();
            movePlane *= f20HeightRef;
            Plane newpln = new Plane(plntoflip);
            newpln.Translate(movePlane);
            newpln.Flip();
            newpln.Rotate(-Math.PI / 2, newpln.Normal);
            return newpln;
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
            get { return new Guid("E5C90275-109A-435E-95CD-661676536242"); }
        }
    }
}