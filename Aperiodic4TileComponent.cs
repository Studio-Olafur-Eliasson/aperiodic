using Ed.Eto;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Aperiodic
{
    public class Aperiodic4TileComponent : GH_Component
    {
        // Cache commonly used constants
        private static readonly double GoldenRatio = (1 + Math.Sqrt(5)) / 2;
        private static readonly double DeflationScaleFactor = Math.Pow(GoldenRatio, 3);
        private static readonly double InverseDeflationScaleFactor = 1.0 / DeflationScaleFactor;

        // Store geometry to preview
        private List<Curve> _previewCurves = new List<Curve>();

        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public Aperiodic4TileComponent()
          : base("Aperiodic 4-Tile", "4-Tile",
            "Generate aperiodic 4-tile transformations (v1.0)",
            "Aperiodic", "Aperiodic")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGeometryParameter("geometryFilter", "geoFilter", "(Optional) Input a geometry filter (Brep or Curve) to define the output shape of the tiling and reduce computation time.", GH_ParamAccess.item);
            pManager.AddNumberParameter("filterDistance", "fDist", "Distance from the geometryFilter within which tiles should be included in the output.", GH_ParamAccess.item, 1.0);
            pManager.AddBooleanParameter("includeInterior", "incInt", "Boolean for whether to include tiles on the interior of the filter geometry (if it is a closed Brep). Default false.", GH_ParamAccess.item, false);
            pManager.AddPlaneParameter("center_pln", "center", "Plane input for the center of the recursive tile-generation process. Default: World XY.", GH_ParamAccess.item, Plane.WorldXY);
            pManager.AddIntegerParameter("iterations", "i", "Number of iterations of the recursive process. If iterations > 2, must use geometryFilter to avoid crashing. Set iterations = 0 to view the starting \"seed\" tiles of the recusive process. Default: 1", GH_ParamAccess.item, 1);
            pManager.AddIntegerParameter("seedOption", "seedOption", "Enter an integer option, 0, 1, or 2. According to Socolar and Steinhardt, who published the discovery of this 4-tile configuration in 1986, there exist exactly three packings with a single center of icosahedral point symmetry in 3D Euclidean space. These three options are each generated with one of the following \"seed\" tile configurations: 0 = a single rhombic triacontahedron tile (Default); 1 = a star of twenty rhombohedra, which, after deflation/inflation, are surrounded by rhombic triacontahedra; 2 = a star of twenty rhombohedra, with flipped orientations with respect to the previous option, so that they are surrounded by rhombic icosahedra on the next layer after deflation/inflation.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGeometryParameter("geometryFilterArray", "geoFilterArr", "Array of geometry (Brep or Curve) scaled according to the deflation scale factor and iterations.", GH_ParamAccess.list);
            pManager.AddPlaneParameter("recursionResultplns", "resultPlns", "Tree of planes representing the positions and orientations of the generated tiles.", GH_ParamAccess.tree);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Declare a variables for the input
            GeometryBase geometryFilter = null;
            double filterDistance = 1.0;
            bool includeInterior = false;
            Plane centerpln = Plane.WorldXY;
            int seed = 0;
            int iterations = 1;
            double scale = 1.0;

            // Retrieve data from input parameters
            if (!DA.GetData(0, ref geometryFilter)) { return; }
            if (!DA.GetData(1, ref filterDistance)) { return; }
            if (!DA.GetData(2, ref includeInterior)) { return; }
            if (!DA.GetData(3, ref centerpln)) { return; }
            if (!DA.GetData(4, ref iterations)) { return; }
            if (!DA.GetData(5, ref seed)) { return; }

            List<GeometryBase> gfa = null;
            List<GeometryBase> gfaCopy = null;

            if (geometryFilter == null)
            {
                if (iterations > 2)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Missing geometry filter input (necessary for > 2 iterations).");
                    return;
                }
                gfa = null;
                gfaCopy = null;
            }
            else
            {
                gfa = GetGeoFilterArray(geometryFilter, iterations, centerpln);
                gfaCopy = gfa.GetRange(0, gfa.Count);
            }

            DA.SetDataList(0, gfaCopy);

            // Generate meshes for each tile type
            Mesh refA6 = GenerateMeshA6(scale);
            Mesh refB12 = GenerateMeshB12(scale);
            Mesh refF20 = GenerateMeshF20(scale);
            Mesh refK30 = GenerateMeshK30(scale);

            // Generate the base planes according to chosen seed option and center plane
            double a6HeightRef = GetMeshHeight(refA6);
            DataTree<Plane> baseplns = GenerateBasePlnsFromSeed(seed, centerpln, a6HeightRef);

            // Generate wireframe preview (only need to check branches 0 and 3 since seed options only include those two types of tiles)
            _previewCurves.Clear();
            foreach (var pln in baseplns.Branch(0))
            {
                Mesh meshCopy = refA6.DuplicateMesh();
                meshCopy.Transform(Transform.PlaneToPlane(Plane.WorldXY, pln));
                meshCopy.Scale(Math.Pow(DeflationScaleFactor, iterations));
                _previewCurves.AddRange(GetWireframeEdges(meshCopy));
            }
            foreach (var pln in baseplns.Branch(3))
            {
                Mesh meshCopy = refK30.DuplicateMesh();
                meshCopy.Transform(Transform.PlaneToPlane(Plane.WorldXY, pln));
                meshCopy.Scale(Math.Pow(DeflationScaleFactor, iterations));
                _previewCurves.AddRange(GetWireframeEdges(meshCopy));
            }

            // Translate reference meshes so their base sits on WorldXY plane (preparing to apply deflation rules)
            TranslateToWorldXY(refA6);
            TranslateToWorldXY(refB12);
            TranslateToWorldXY(refF20);
            TranslateToWorldXY(refK30);

            // Generate deflation rules
            DataTree<Plane> generatedA6plns = GenerateDeflationPlanesA6(refA6, refB12, refF20, refK30);
            DataTree<Plane> generatedB12plns = GenerateDeflationPlanesB12(refA6, refB12, refF20, refK30);
            DataTree<Plane> generatedF20plns = GenerateDeflationPlanesF20(refA6, refB12, refF20, refK30);
            DataTree<Plane> generatedK30plns = GenerateDeflationPlanesK30(refA6, refB12, refF20, refK30);

            // Pre-extract deflation planes to native Plane arrays for faster access
            // deflationRules[tileType] = Plane[branchIndex][planeIndex]
            Plane[][][] deflationRules = new Plane[4][][];
            deflationRules[0] = ExtractPlaneArrays(generatedA6plns);
            deflationRules[1] = ExtractPlaneArrays(generatedB12plns);
            deflationRules[2] = ExtractPlaneArrays(generatedF20plns);
            deflationRules[3] = ExtractPlaneArrays(generatedK30plns);

            DataTree<Plane> outputplns = RecurseInflateGeometry(gfa, filterDistance, includeInterior, centerpln, baseplns, iterations, scale, deflationRules);

            DA.SetDataTree(1, outputplns);
        }

        #region Generate Deflation Planes / Rules
        private DataTree<Plane> GenerateDeflationPlanesA6(Mesh refA6, Mesh refB12, Mesh refF20, Mesh refK30)
        {
            // Note: always duplicate reference meshes before modifying

            // Declare variables
            DataTree<Plane> deflationRulesA6 = new DataTree<Plane>();

            GH_Path pth0 = new GH_Path(0);
            GH_Path pth1 = new GH_Path(1);
            GH_Path pth2 = new GH_Path(2);
            GH_Path pth3 = new GH_Path(3);

            // Get height references from original input meshes
            double a6HeightRef = GetMeshHeight(refA6);
            double b12HeightRef = GetMeshHeight(refB12);
            double f20HeightRef = GetMeshHeight(refF20);
            double k30HeightRef = GetMeshHeight(refK30);

            Mesh mesh = refA6.DuplicateMesh();
            Point3d basept = Point3d.Origin;
            Plane basepln = Plane.WorldXY;

            // TODO: Clean up this area - possibly remove
            // Get the centroid of the geometry
            AreaMassProperties ampMesh = AreaMassProperties.Compute(mesh);
            Point3d centroidMesh = ampMesh.Centroid;
            // Create a vector using the centroid location
            Vector3d centroidVec = new Vector3d(centroidMesh);
            // Scale the vector by the deflationScalefactor to get the new centroid location
            // Subtract the original centroidVec to get the translation vector
            Vector3d inflateVec = centroidVec * DeflationScaleFactor - centroidVec;
            // Translate the brep to the new scaled location (but without scaling the brep itself, so the unit size remains the same)
            mesh.Translate(inflateVec);
            // Translate the basept to the new scaled location - consistent with the brep itself, so further operations on the base pts can recurse well
            basept += inflateVec;
            Transform scaleInflate = Transform.Scale(Point3d.Origin, DeflationScaleFactor);
            basepln.Transform(scaleInflate);

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

            // This is rhombohedron (long tile)
            // Begin deflation for A6

            // TODO: Eventual order of operations for each new mesh/(pt)/pln - find new base plane first, then transform basemesh from worldXY to new base plane

            // Get centroid
            AreaMassProperties ampA6 = AreaMassProperties.Compute(mesh);
            Point3d centroidA6 = ampA6.Centroid;

            // Scale up A6 unit to get general boundaries of the inflated shapes
            // Scale center point of geometry by a factor of golden ratio^3
            Transform xformScaleA6 = Transform.Scale(centroidA6, DeflationScaleFactor);
            Mesh a6boundary = mesh.DuplicateMesh();
            a6boundary.Transform(xformScaleA6);

            // Find vertex/point furthest from the base point
            Point3d furthestVertexpt = mesh.TopologyVertices[GetFurthestVertex(mesh, (Point3d)basept)];
            Point3d scaledbasept = new Point3d(basept);
            scaledbasept.Transform(xformScaleA6);
            Point3d scaledFurthestVertexpt = new Point3d(furthestVertexpt);
            scaledFurthestVertexpt.Transform(xformScaleA6);
            Vector3d moveCloseCopy = scaledbasept - basept;
            Vector3d moveFarCopy = scaledFurthestVertexpt - furthestVertexpt;

            // Move one copy of A6 to the base point, and one to the far end of the inflated shape
            Mesh a6a600 = mesh.DuplicateMesh();
            Mesh a6a601 = mesh.DuplicateMesh();
            a6a600.Translate(moveCloseCopy);
            a6a601.Translate(moveFarCopy);

            // Add to mesh list
            listA6.Add(a6a600);
            listA6.Add(a6a601);

            // Add to basepts list
            Point3d basepta6a600 = basept + moveCloseCopy;
            Point3d basepta6a601 = basept + moveFarCopy;
            ptsA6.Add(basepta6a600);
            ptsA6.Add(basepta6a601);

            // Add to baseplns list
            Plane baseplna6a600 = new Plane(basepln);
            Plane baseplna6a601 = new Plane(basepln);
            //baseplna6a600.Translate(moveCloseCopy); // TODOL understand why
            baseplna6a601.Translate(moveFarCopy);
            baseplna6a601.Translate(moveFarCopy); // TODO: understand why
            plnsA6.Add(baseplna6a600);
            plnsA6.Add(baseplna6a601);

            // For testing/visual purposes
            //listA6.Add(brep);
            //ptsA6.Add(basept);

            // TODO: refactor to take advantage of the 3-fold symmetry (only build 1/3 sides and rotate at the end or after each construction step)

            // For the close copy, move three K30 tiles to each of the top 3 faces
            int closeCopyInnerIndex = GetFurthestVertex(a6a600, (Point3d)basepta6a600);
            Point3d closeCopyInnerPt = (Point3d)a6a600.Vertices[closeCopyInnerIndex];

            // Get orientation planes for the K30

            // Get adjacent vertex points
            adjacentVertexIndices = a6a600.Vertices.GetConnectedVertices(closeCopyInnerIndex);
            List<Point3d> edgePoints = new List<Point3d>();
            for (int j = 0; j < 3; j++)
            {
                edgePoints.Add((Point3d)a6a600.Vertices[adjacentVertexIndices[j]]);
            }

            Plane planea6k300 = new Plane(closeCopyInnerPt, edgePoints[0], edgePoints[1]);
            Plane planea6k301 = new Plane(closeCopyInnerPt, edgePoints[1], edgePoints[2]);
            Plane planea6k302 = new Plane(closeCopyInnerPt, edgePoints[2], edgePoints[0]);

            // Check normals for each plane to make sure they are flipped the right way
            // TODO: incorporate signed vector angle
            double anglea6k300 = Vector3d.VectorAngle(planea6k300.Normal, edgePoints[2] - closeCopyInnerPt);
            if (anglea6k300 < Math.PI / 2)
            {
                planea6k300 = new Plane(closeCopyInnerPt, edgePoints[1], edgePoints[0]);
            }
            double anglea6k301 = Vector3d.VectorAngle(planea6k301.Normal, edgePoints[0] - closeCopyInnerPt);
            if (anglea6k301 < Math.PI / 2)
            {
                planea6k301 = new Plane(closeCopyInnerPt, edgePoints[2], edgePoints[1]);
            }
            double anglea6k302 = Vector3d.VectorAngle(planea6k302.Normal, edgePoints[1] - closeCopyInnerPt);
            if (anglea6k302 < Math.PI / 2)
            {
                planea6k302 = new Plane(closeCopyInnerPt, edgePoints[0], edgePoints[2]);
            }
            Transform xforma6k300 = Transform.PlaneToPlane(k30base, planea6k300);
            Transform xforma6k301 = Transform.PlaneToPlane(k30base, planea6k301);
            Transform xforma6k302 = Transform.PlaneToPlane(k30base, planea6k302);
            Mesh a6k300 = refK30.DuplicateMesh();
            Mesh a6k301 = refK30.DuplicateMesh();
            Mesh a6k302 = refK30.DuplicateMesh();
            a6k300.Transform(xforma6k300);
            a6k301.Transform(xforma6k301);
            a6k302.Transform(xforma6k302);

            // Add to mesh list
            listK30.Add(a6k300);
            listK30.Add(a6k301);
            listK30.Add(a6k302);

            // Transform the basepoints from the origin for the K30
            Point3d basepta6k300 = Point3d.Origin;
            Point3d basepta6k301 = Point3d.Origin;
            Point3d basepta6k302 = Point3d.Origin;
            basepta6k300.Transform(xforma6k300);
            basepta6k301.Transform(xforma6k301);
            basepta6k302.Transform(xforma6k302);

            // Add to basepts list
            ptsK30.Add(basepta6k300);
            ptsK30.Add(basepta6k301);
            ptsK30.Add(basepta6k302);

            // Add to baseplns list TODO: reorient planes correctly
            Plane baseplna6k300 = Plane.WorldXY;
            Plane baseplna6k301 = Plane.WorldXY;
            Plane baseplna6k302 = Plane.WorldXY;
            baseplna6k300.Transform(xforma6k300);
            baseplna6k301.Transform(xforma6k301);
            baseplna6k302.Transform(xforma6k302);
            plnsK30.Add(baseplna6k300);
            plnsK30.Add(baseplna6k301);
            plnsK30.Add(baseplna6k302);

            // Reflect out another copy of A6 - get mirror plane
            Vector3d normala6a602 = furthestVertexpt - basept;
            Plane planea6a602 = new Plane(closeCopyInnerPt, normala6a602);
            Transform xforma6a602 = Transform.Mirror(planea6a602);
            Transform xforma6a602rot = Transform.Rotation(Math.PI, normala6a602, closeCopyInnerPt);
            Mesh a6a602 = a6a600.DuplicateMesh();
            a6a602.Transform(xforma6a602);
            a6a602.Transform(xforma6a602rot);

            // Add to mesh list
            listA6.Add(a6a602);

            // Transform basept
            Point3d basepta6a602 = new Point3d(basepta6a600);
            basepta6a602.Transform(xforma6a602);

            // Add to basepts list
            ptsA6.Add(basepta6a602);

            // Transform basepln
            Plane baseplna6a602 = new Plane(baseplna6a600);
            baseplna6a602.Transform(xforma6a602);
            baseplna6a602.Rotate(Math.PI, baseplna6a602.Normal);

            // Add to baseplns list
            Plane baseplna6a602final = new Plane(baseplna6a602);
            baseplna6a602final.Flip();
            baseplna6a602final.Rotate(Math.PI / 2, baseplna6a602final.Normal);
            plnsA6.Add(baseplna6a602final);

            // Transform planes used for K30 for use in further reflection of A6 tiles
            Plane planea6a603 = new Plane(planea6k300);
            Plane planea6a604 = new Plane(planea6k301);
            Plane planea6a605 = new Plane(planea6k302);
            planea6a603.Transform(xforma6a602);
            planea6a603.Transform(xforma6a602rot);
            planea6a604.Transform(xforma6a602);
            planea6a604.Transform(xforma6a602rot);
            planea6a605.Transform(xforma6a602);
            planea6a605.Transform(xforma6a602rot);
            Transform xforma6a603 = Transform.Mirror(planea6a603);
            Transform xforma6a604 = Transform.Mirror(planea6a604);
            Transform xforma6a605 = Transform.Mirror(planea6a605);

            // Make 3 more copies of A6 using the mirrors
            Mesh a6a603 = a6a602.DuplicateMesh();
            Mesh a6a604 = a6a602.DuplicateMesh();
            Mesh a6a605 = a6a602.DuplicateMesh();
            a6a603.Transform(xforma6a603);
            a6a604.Transform(xforma6a604);
            a6a605.Transform(xforma6a605);

            // Add to mesh list
            listA6.Add(a6a603);
            listA6.Add(a6a604);
            listA6.Add(a6a605);

            // Transform basepts
            Point3d basepta6a603 = new Point3d(basepta6a602);
            Point3d basepta6a604 = new Point3d(basepta6a602);
            Point3d basepta6a605 = new Point3d(basepta6a602);
            basepta6a603.Transform(xforma6a603);
            basepta6a604.Transform(xforma6a604);
            basepta6a605.Transform(xforma6a605);

            // Add to basepts list
            ptsA6.Add(basepta6a603);
            ptsA6.Add(basepta6a604);
            ptsA6.Add(basepta6a605);

            // Transform baseplns
            Plane baseplna6a603 = new Plane(baseplna6a602);
            Plane baseplna6a604 = new Plane(baseplna6a602);
            Plane baseplna6a605 = new Plane(baseplna6a602);
            baseplna6a603.Transform(xforma6a603);
            baseplna6a604.Transform(xforma6a604);
            baseplna6a605.Transform(xforma6a605);

            // Add to baseplns list
            plnsA6.Add(baseplna6a603);
            plnsA6.Add(baseplna6a604);
            plnsA6.Add(baseplna6a605);

            // Get new set of 3 planes for orienting three B12 tiles
            // Start with same 3 planes used for K30
            Plane planea6b120 = new Plane(planea6k300);
            Plane planea6b121 = new Plane(planea6k301);
            Plane planea6b122 = new Plane(planea6k302);
            planea6b120.Translate(normala6a602);
            planea6b121.Translate(normala6a602);
            planea6b122.Translate(normala6a602);
            Transform xforma6b120 = Transform.PlaneToPlane(b12base, planea6b120);
            Transform xforma6b121 = Transform.PlaneToPlane(b12base, planea6b121);
            Transform xforma6b122 = Transform.PlaneToPlane(b12base, planea6b122);
            Mesh a6b120 = refB12.DuplicateMesh();
            Mesh a6b121 = refB12.DuplicateMesh();
            Mesh a6b122 = refB12.DuplicateMesh();
            a6b120.Transform(xforma6b120);
            a6b121.Transform(xforma6b121);
            a6b122.Transform(xforma6b122);

            // Add to mesh list
            listB12.Add(a6b120);
            listB12.Add(a6b121);
            listB12.Add(a6b122);

            // Transform basepts
            Point3d basepta6b120 = Point3d.Origin;
            Point3d basepta6b121 = Point3d.Origin;
            Point3d basepta6b122 = Point3d.Origin;
            basepta6b120.Transform(xforma6b120);
            basepta6b121.Transform(xforma6b121);
            basepta6b122.Transform(xforma6b122);

            // Add to basepts list
            ptsB12.Add(basepta6b120);
            ptsB12.Add(basepta6b121);
            ptsB12.Add(basepta6b122);

            // Transform baseplns
            Plane baseplna6b120 = Plane.WorldXY;
            Plane baseplna6b121 = Plane.WorldXY;
            Plane baseplna6b122 = Plane.WorldXY;
            baseplna6b120.Transform(xforma6b120);
            baseplna6b121.Transform(xforma6b121);
            baseplna6b122.Transform(xforma6b122);

            // Add to baseplns list
            plnsB12.Add(baseplna6b120);
            plnsB12.Add(baseplna6b121);
            plnsB12.Add(baseplna6b122);

            // Using orientation of the B12 tiles, move their planes outwards to their basepts to create mirror planes for further propogating A6 tiles
            Vector3d veca6a606 = basepta6b120 - planea6b120.Origin;
            Vector3d veca6a607 = basepta6b121 - planea6b121.Origin;
            Vector3d veca6a608 = basepta6b122 - planea6b122.Origin;
            Plane planea6a606 = new Plane(planea6b120);
            Plane planea6a607 = new Plane(planea6b121);
            Plane planea6a608 = new Plane(planea6b122);
            planea6a606.Translate(veca6a606);
            planea6a607.Translate(veca6a607);
            planea6a608.Translate(veca6a608);
            Transform xforma6a606 = Transform.Mirror(planea6a606);
            Transform xforma6a607 = Transform.Mirror(planea6a607);
            Transform xforma6a608 = Transform.Mirror(planea6a608);
            Mesh a6a606 = a6a602.DuplicateMesh();
            Mesh a6a607 = a6a602.DuplicateMesh();
            Mesh a6a608 = a6a602.DuplicateMesh();
            a6a606.Transform(xforma6a606);
            a6a607.Transform(xforma6a607);
            a6a608.Transform(xforma6a608);

            // Add to mesh list
            listA6.Add(a6a606);
            listA6.Add(a6a607);
            listA6.Add(a6a608);

            // Transform basepts
            Point3d basepta6a606 = new Point3d(basepta6a602);
            Point3d basepta6a607 = new Point3d(basepta6a602);
            Point3d basepta6a608 = new Point3d(basepta6a602);
            basepta6a606.Transform(xforma6a606);
            basepta6a607.Transform(xforma6a607);
            basepta6a608.Transform(xforma6a608);

            // Add to basepts list
            ptsA6.Add(basepta6a606);
            ptsA6.Add(basepta6a607);
            ptsA6.Add(basepta6a608);

            // Transform baseplns
            Plane baseplna6a606 = new Plane(baseplna6a602);
            Plane baseplna6a607 = new Plane(baseplna6a602);
            Plane baseplna6a608 = new Plane(baseplna6a602);
            baseplna6a606.Transform(xforma6a606);
            baseplna6a607.Transform(xforma6a607);
            baseplna6a608.Transform(xforma6a608);

            // Add to baseplns list
            plnsA6.Add(baseplna6a606);
            plnsA6.Add(baseplna6a607);
            plnsA6.Add(baseplna6a608);

            // Use the same mirrors to copy more A6 tiles
            Mesh a6a6061 = a6a604.DuplicateMesh();
            Mesh a6a6071 = a6a605.DuplicateMesh();
            Mesh a6a6081 = a6a603.DuplicateMesh();
            Mesh a6a6062 = a6a605.DuplicateMesh();
            Mesh a6a6072 = a6a603.DuplicateMesh();
            Mesh a6a6082 = a6a604.DuplicateMesh();
            a6a6061.Transform(xforma6a606);
            a6a6071.Transform(xforma6a607);
            a6a6081.Transform(xforma6a608);
            a6a6062.Transform(xforma6a606);
            a6a6072.Transform(xforma6a607);
            a6a6082.Transform(xforma6a608);

            // Add to mesh list
            listA6.Add(a6a6061);
            listA6.Add(a6a6071);
            listA6.Add(a6a6081);
            listA6.Add(a6a6062);
            listA6.Add(a6a6072);
            listA6.Add(a6a6082);

            // Transform basepts
            Point3d basepta6a6061 = new Point3d(basepta6a604);
            Point3d basepta6a6071 = new Point3d(basepta6a605);
            Point3d basepta6a6081 = new Point3d(basepta6a603);
            Point3d basepta6a6062 = new Point3d(basepta6a605);
            Point3d basepta6a6072 = new Point3d(basepta6a603);
            Point3d basepta6a6082 = new Point3d(basepta6a604);
            basepta6a6061.Transform(xforma6a606);
            basepta6a6062.Transform(xforma6a606);
            basepta6a6071.Transform(xforma6a607);
            basepta6a6072.Transform(xforma6a607);
            basepta6a6081.Transform(xforma6a608);
            basepta6a6082.Transform(xforma6a608);

            // Add to basepts list
            ptsA6.Add(basepta6a6061);
            ptsA6.Add(basepta6a6071);
            ptsA6.Add(basepta6a6081);
            ptsA6.Add(basepta6a6062);
            ptsA6.Add(basepta6a6072);
            ptsA6.Add(basepta6a6082);

            // Flip/rotate planes 3/4/5 for future steps
            baseplna6a603.Flip();
            baseplna6a604.Flip();
            baseplna6a605.Flip();
            baseplna6a603.Rotate(Math.PI / 2, baseplna6a603.Normal);
            baseplna6a604.Rotate(Math.PI / 2, baseplna6a604.Normal);
            baseplna6a605.Rotate(Math.PI / 2, baseplna6a605.Normal);

            // Transform baseplns
            Plane baseplna6a6061 = new Plane(baseplna6a604);
            Plane baseplna6a6071 = new Plane(baseplna6a605);
            Plane baseplna6a6081 = new Plane(baseplna6a603);
            Plane baseplna6a6062 = new Plane(baseplna6a605);
            Plane baseplna6a6072 = new Plane(baseplna6a603);
            Plane baseplna6a6082 = new Plane(baseplna6a604);
            baseplna6a6061.Transform(xforma6a606);
            baseplna6a6062.Transform(xforma6a606);
            baseplna6a6071.Transform(xforma6a607);
            baseplna6a6072.Transform(xforma6a607);
            baseplna6a6081.Transform(xforma6a608);
            baseplna6a6082.Transform(xforma6a608);

            // Add to baseplns list
            plnsA6.Add(baseplna6a6061);
            plnsA6.Add(baseplna6a6071);
            plnsA6.Add(baseplna6a6081);
            plnsA6.Add(baseplna6a6062);
            plnsA6.Add(baseplna6a6072);
            plnsA6.Add(baseplna6a6082);

            // Get plane to copy one more K30 into the pattern
            // Transform the WorldXY plane using the same transform as the B12, then use this plane to orient the K30
            Plane planea6k303 = new Plane(Plane.WorldXY);
            planea6k303.Transform(xforma6b120);
            planea6k303.Flip();
            planea6k303.Rotate(Math.PI / 2, planea6k303.Normal);
            Transform xforma6k303 = Transform.PlaneToPlane(Plane.WorldXY, planea6k303);
            Mesh a6k303 = refK30.DuplicateMesh();
            a6k303.Transform(xforma6k303);

            // Add to mesh list
            listK30.Add(a6k303);

            // Transform basept
            Point3d basepta6k303 = Point3d.Origin;
            basepta6k303.Transform(xforma6k303);

            // Add to basepts list
            ptsK30.Add(basepta6k303);

            // Transform basepln
            Plane baseplna6k303 = Plane.WorldXY;
            baseplna6k303.Transform(xforma6k303);

            // Add to baseplns list
            plnsK30.Add(baseplna6k303);

            // Get translation vectors for additional A6 tiles
            // Between tiles a6a600 and a6a603, a6a604, a6a605
            // Between initial base point and base points
            Vector3d veca6a609 = basepta6a603 - basepta6a600;
            Vector3d veca6a610 = basepta6a604 - basepta6a600;
            Vector3d veca6a611 = basepta6a605 - basepta6a600;
            double scalefactora6a609 = (veca6a609.Length - b12HeightRef) / veca6a609.Length;
            veca6a609 *= scalefactora6a609;
            veca6a610 *= scalefactora6a609;
            veca6a611 *= scalefactora6a609;
            Mesh a6a609 = a6a603.DuplicateMesh();
            Mesh a6a610 = a6a604.DuplicateMesh();
            Mesh a6a611 = a6a605.DuplicateMesh();
            a6a609.Translate(veca6a609);
            a6a610.Translate(veca6a610);
            a6a611.Translate(veca6a611);

            // Add to mesh list
            listA6.Add(a6a609);
            listA6.Add(a6a610);
            listA6.Add(a6a611);

            // Transform basepts
            Point3d basepta6a609 = new Point3d(basepta6a603);
            basepta6a609 += veca6a609;
            Point3d basepta6a610 = new Point3d(basepta6a604);
            basepta6a610 += veca6a610;
            Point3d basepta6a611 = new Point3d(basepta6a605);
            basepta6a611 += veca6a611;

            // Add to basepts list
            ptsA6.Add(basepta6a609);
            ptsA6.Add(basepta6a610);
            ptsA6.Add(basepta6a611);

            // Transform baseplns
            Plane baseplna6a609 = new Plane(baseplna6a603);
            baseplna6a609.Translate(veca6a609);
            baseplna6a609.Flip();
            baseplna6a609.Rotate(Math.PI / 2, baseplna6a609.Normal);
            Plane baseplna6a610 = new Plane(baseplna6a604);
            baseplna6a610.Translate(veca6a610);
            baseplna6a610.Flip();
            baseplna6a610.Rotate(Math.PI / 2, baseplna6a610.Normal);
            Plane baseplna6a611 = new Plane(baseplna6a605);
            baseplna6a611.Translate(veca6a611);
            baseplna6a611.Flip();
            baseplna6a611.Rotate(Math.PI / 2, baseplna6a611.Normal);

            // Add to baseplns list
            plnsA6.Add(baseplna6a609);
            plnsA6.Add(baseplna6a610);
            plnsA6.Add(baseplna6a611);

            // Now get axes of rotation using center of the last K30 to be added
            AreaMassProperties ampa6k303 = AreaMassProperties.Compute(a6k303);
            Point3d centroida6k303 = ampa6k303.Centroid;
            Vector3d axisa6a609 = basepta6a609 - centroida6k303;
            Vector3d axisa6a610 = basepta6a610 - centroida6k303;
            Vector3d axisa6a611 = basepta6a611 - centroida6k303;
            Transform xforma6a609rot = Transform.Rotation(2 * Math.PI / 5, axisa6a609, basepta6a609);
            Transform xforma6a610rot = Transform.Rotation(2 * Math.PI / 5, axisa6a610, basepta6a610);
            Transform xforma6a611rot = Transform.Rotation(2 * Math.PI / 5, axisa6a611, basepta6a611);

            // Rotate the a6 tiles
            Mesh a6a6091 = a6a609.DuplicateMesh();
            a6a6091.Transform(xforma6a609rot);
            Mesh a6a6092 = a6a6091.DuplicateMesh();
            a6a6092.Transform(xforma6a609rot);
            Mesh a6a6093 = a6a6092.DuplicateMesh();
            a6a6093.Transform(xforma6a609rot);
            Mesh a6a6094 = a6a6093.DuplicateMesh();
            a6a6094.Transform(xforma6a609rot);
            Mesh a6a6101 = a6a610.DuplicateMesh();
            a6a6101.Transform(xforma6a610rot);
            Mesh a6a6102 = a6a6101.DuplicateMesh();
            a6a6102.Transform(xforma6a610rot);
            Mesh a6a6103 = a6a6102.DuplicateMesh();
            a6a6103.Transform(xforma6a610rot);
            Mesh a6a6104 = a6a6103.DuplicateMesh();
            a6a6104.Transform(xforma6a610rot);
            Mesh a6a6111 = a6a611.DuplicateMesh();
            a6a6111.Transform(xforma6a611rot);
            Mesh a6a6112 = a6a6111.DuplicateMesh();
            a6a6112.Transform(xforma6a611rot);
            Mesh a6a6113 = a6a6112.DuplicateMesh();
            a6a6113.Transform(xforma6a611rot);
            Mesh a6a6114 = a6a6113.DuplicateMesh();
            a6a6114.Transform(xforma6a611rot);

            // Add to A6 list
            listA6.Add(a6a6091);
            listA6.Add(a6a6092);
            listA6.Add(a6a6093);
            listA6.Add(a6a6094);
            listA6.Add(a6a6101);
            listA6.Add(a6a6102);
            listA6.Add(a6a6103);
            listA6.Add(a6a6104);
            listA6.Add(a6a6111);
            listA6.Add(a6a6112);
            listA6.Add(a6a6113);
            listA6.Add(a6a6114);

            // Add basepts - which are just duplicates of existing basepts
            ptsA6.Add(basepta6a609);
            ptsA6.Add(basepta6a609);
            ptsA6.Add(basepta6a609);
            ptsA6.Add(basepta6a609);
            ptsA6.Add(basepta6a610);
            ptsA6.Add(basepta6a610);
            ptsA6.Add(basepta6a610);
            ptsA6.Add(basepta6a610);
            ptsA6.Add(basepta6a611);
            ptsA6.Add(basepta6a611);
            ptsA6.Add(basepta6a611);
            ptsA6.Add(basepta6a611);

            // Rotate the a6 baseplns
            Plane baseplna6a6091 = new Plane(baseplna6a609);
            baseplna6a6091.Transform(xforma6a609rot);
            Plane baseplna6a6092 = new Plane(baseplna6a6091);
            baseplna6a6092.Transform(xforma6a609rot);
            Plane baseplna6a6093 = new Plane(baseplna6a6092);
            baseplna6a6093.Transform(xforma6a609rot);
            Plane baseplna6a6094 = new Plane(baseplna6a6093);
            baseplna6a6094.Transform(xforma6a609rot);
            Plane baseplna6a6101 = new Plane(baseplna6a610);
            baseplna6a6101.Transform(xforma6a610rot);
            Plane baseplna6a6102 = new Plane(baseplna6a6101);
            baseplna6a6102.Transform(xforma6a610rot);
            Plane baseplna6a6103 = new Plane(baseplna6a6102);
            baseplna6a6103.Transform(xforma6a610rot);
            Plane baseplna6a6104 = new Plane(baseplna6a6103);
            baseplna6a6104.Transform(xforma6a610rot);
            Plane baseplna6a6111 = new Plane(baseplna6a611);
            baseplna6a6111.Transform(xforma6a611rot);
            Plane baseplna6a6112 = new Plane(baseplna6a6111);
            baseplna6a6112.Transform(xforma6a611rot);
            Plane baseplna6a6113 = new Plane(baseplna6a6112);
            baseplna6a6113.Transform(xforma6a611rot);
            Plane baseplna6a6114 = new Plane(baseplna6a6113);
            baseplna6a6114.Transform(xforma6a611rot);

            // Add baseplns
            plnsA6.Add(baseplna6a6091);
            plnsA6.Add(baseplna6a6092);
            plnsA6.Add(baseplna6a6093);
            plnsA6.Add(baseplna6a6094);
            plnsA6.Add(baseplna6a6101);
            plnsA6.Add(baseplna6a6102);
            plnsA6.Add(baseplna6a6103);
            plnsA6.Add(baseplna6a6104);
            plnsA6.Add(baseplna6a6111);
            plnsA6.Add(baseplna6a6112);
            plnsA6.Add(baseplna6a6113);
            plnsA6.Add(baseplna6a6114);

            // Add three F20 tiles near the far vertex
            // Get normal vectors using the far copy a6a601
            int farVertexIndex = GetFurthestVertex(a6a601, (Point3d)basept);
            Point3d farVertexPt = (Point3d)a6a601.TopologyVertices[farVertexIndex];

            // Get adjacent vertex points
            adjacentVertexIndices = a6a601.TopologyVertices.ConnectedTopologyVertices(farVertexIndex);
            edgePoints = new List<Point3d>();
            for (int j = 0; j < 3; j++)
            {
                edgePoints.Add((Point3d)a6a601.TopologyVertices[adjacentVertexIndices[j]]);
            }

            // Create normal vectors
            Vector3d normala6f200 = edgePoints[0] - farVertexPt;
            Vector3d normala6f201 = edgePoints[1] - farVertexPt;
            Vector3d normala6f202 = edgePoints[2] - farVertexPt;
            Plane planea6f200 = new Plane(edgePoints[0], normala6f200);
            Plane planea6f201 = new Plane(edgePoints[1], normala6f201);
            Plane planea6f202 = new Plane(edgePoints[2], normala6f202);

            // Orient the planes correctly by projecting neighboring vertices onto them to get xaxis
            Point3d a6f200xaxispt = new Point3d(edgePoints[1]);
            Point3d a6f201xaxispt = new Point3d(edgePoints[2]);
            Point3d a6f202xaxispt = new Point3d(edgePoints[0]);

            double anglea6f200 = GetSignedVectorAngle(planea6f200.XAxis, a6f200xaxispt - edgePoints[0], planea6f200);
            double anglea6f201 = GetSignedVectorAngle(planea6f201.XAxis, a6f201xaxispt - edgePoints[1], planea6f201);
            double anglea6f202 = GetSignedVectorAngle(planea6f202.XAxis, a6f202xaxispt - edgePoints[2], planea6f202);

            planea6f200.Rotate(anglea6f200 + Math.PI, normala6f200, edgePoints[0]);
            planea6f201.Rotate(anglea6f201 + Math.PI, normala6f201, edgePoints[1]);
            planea6f202.Rotate(anglea6f202 + Math.PI, normala6f202, edgePoints[2]);

            Transform xforma6f200 = Transform.PlaneToPlane(Plane.WorldXY, planea6f200);
            Transform xforma6f201 = Transform.PlaneToPlane(Plane.WorldXY, planea6f201);
            Transform xforma6f202 = Transform.PlaneToPlane(Plane.WorldXY, planea6f202);
            Mesh a6f200 = refF20.DuplicateMesh();
            Mesh a6f201 = refF20.DuplicateMesh();
            Mesh a6f202 = refF20.DuplicateMesh();
            a6f200.Transform(xforma6f200);
            a6f201.Transform(xforma6f201);
            a6f202.Transform(xforma6f202);

            // Add to mesh list
            listF20.Add(a6f200);
            listF20.Add(a6f201);
            listF20.Add(a6f202);

            // Transform basepts
            Point3d basepta6f200 = Point3d.Origin;
            Point3d basepta6f201 = Point3d.Origin;
            Point3d basepta6f202 = Point3d.Origin;
            basepta6f200.Transform(xforma6f200);
            basepta6f201.Transform(xforma6f201);
            basepta6f202.Transform(xforma6f202);

            // Add to basept list
            ptsF20.Add(basepta6f200);
            ptsF20.Add(basepta6f201);
            ptsF20.Add(basepta6f202);

            // Add to basepln list
            plnsF20.Add(planea6f200);
            plnsF20.Add(planea6f201);
            plnsF20.Add(planea6f202);

            // Add 6 more F20 tiles using orientations and normals from previous A6 tile placement
            // Use a6a609, a6a610, a6a611 and the two edges adjacent to their base pts that are not the axes axisa6a609, axisa6a610, axisa6a611
            List<Mesh> a6a6fora6f20 = new List<Mesh> { a6a609, a6a610, a6a611 };
            List<Point3d> a6a6fora6f20pts = new List<Point3d> { basepta6a609, basepta6a610, basepta6a611 };
            List<Vector3d> a6a6fora6f20axes = new List<Vector3d> { axisa6a609, axisa6a610, axisa6a611 };
            for (int i = 0; i < 3; i++)
            {
                int topVertexIndex = GetClosestVertex(a6a6fora6f20[i], (Point3d)a6a6fora6f20pts[i]);
                Point3d topVertexIndexPt = (Point3d)a6a6fora6f20[i].TopologyVertices[topVertexIndex];

                // Get adjacent vertex points
                adjacentVertexIndices = a6a6fora6f20[i].TopologyVertices.ConnectedTopologyVertices(topVertexIndex);
                edgePoints = new List<Point3d>();
                for (int j = 0; j < 3; j++)
                {
                    edgePoints.Add((Point3d)a6a6fora6f20[i].TopologyVertices[adjacentVertexIndices[j]]);
                }

                for (int j = 0; j < 3; j++)
                {
                    Vector3d normala6f20b = edgePoints[j] - topVertexIndexPt;
                    if (normala6f20b.IsParallelTo(a6a6fora6f20axes[i]) == 0)
                    {
                        Plane planea6f20b = new Plane(edgePoints[j], normala6f20b);
                        Point3d planea6f20bxaxis = new Point3d(centroida6k303);
                        double anglea6f20b = GetSignedVectorAngle(planea6f20b.XAxis, planea6f20bxaxis - edgePoints[j], planea6f20b);
                        planea6f20b.Rotate(anglea6f20b + Math.PI, normala6f20b, edgePoints[j]);
                        Transform xforma6f20b = Transform.PlaneToPlane(Plane.WorldXY, planea6f20b);
                        Mesh a6f203 = refF20.DuplicateMesh();
                        a6f203.Transform(xforma6f20b);

                        // Add to mesh list
                        listF20.Add(a6f203);

                        // Transform basept
                        Point3d basepta6f203 = Point3d.Origin;
                        //Save base point on opposite side
                        basepta6f203.Transform(xforma6f20b);
                        Vector3d basepta6f203push = new Vector3d(normala6f20b);
                        basepta6f203push.Unitize();
                        basepta6f203push *= f20HeightRef;
                        basepta6f203 += basepta6f203push;

                        // Add to basepts list
                        ptsF20.Add(basepta6f203);

                        // Transform plane
                        Plane baseplna6f203 = Plane.WorldXY;
                        baseplna6f203.Transform(xforma6f20b);
                        baseplna6f203.Translate(basepta6f203push);
                        baseplna6f203.Flip();
                        baseplna6f203.Rotate(-Math.PI / 2, baseplna6f203.Normal);

                        // Add to baseplns list
                        plnsF20.Add(baseplna6f203);
                    }
                }
            }

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
            double zTranslation = -a6HalfHeight * DeflationScaleFactor;
            Vector3d zOffset = new Vector3d(0, 0, zTranslation);
            TranslatePlanesInDirection(plnsA6, zOffset);
            TranslatePlanesInDirection(plnsB12, zOffset);
            TranslatePlanesInDirection(plnsF20, zOffset);
            TranslatePlanesInDirection(plnsK30, zOffset);

            // Output deflationRules
            deflationRulesA6.AddRange(plnsA6, pth0);
            deflationRulesA6.AddRange(plnsB12, pth1);
            deflationRulesA6.AddRange(plnsF20, pth2);
            deflationRulesA6.AddRange(plnsK30, pth3);

            return deflationRulesA6;
        }

        private DataTree<Plane> GenerateDeflationPlanesB12(Mesh refA6, Mesh refB12, Mesh refF20, Mesh refK30)
        {
            throw new NotImplementedException();
        }

        private DataTree<Plane> GenerateDeflationPlanesF20(Mesh refA6, Mesh refB12, Mesh refF20, Mesh refK30)
        {
            throw new NotImplementedException();
        }

        private DataTree<Plane> GenerateDeflationPlanesK30(Mesh refA6, Mesh refB12, Mesh refF20, Mesh refK30)
        {
            throw new NotImplementedException();
        }

        #endregion

        #region Helper Methods for Deflation Rule Generation

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

        #endregion

        // Extract planes from GH_Structure to native arrays for faster iteration
        // Returns Plane[branchIndex][planeIndex]
        private static Plane[][] ExtractPlaneArrays(DataTree<Plane> planesDataTree)
        {
            int branchCount = Math.Min(planesDataTree.Branches.Count, 4);
            var result = new Plane[4][];
            
            for (int i = 0; i < 4; i++)
            {
                if (i < branchCount)
                {
                    var branch = planesDataTree.Branches[i];
                    result[i] = new Plane[branch.Count];
                    for (int j = 0; j < branch.Count; j++)
                    {
                        result[i][j] = branch[j];
                    }
                }
                else
                {
                    result[i] = new Plane[0];
                }
            }
            return result;
        }

        public static DataTree<Plane> RecurseInflateGeometry(
            List<GeometryBase> geometryFilterArray, 
            double filterDistance, 
            bool includeInterior, 
            Plane centerpln,
            DataTree<Plane> baseplns, 
            int iterations, 
            double scale, 
            Plane[][][] deflationRules)
        {
            if (iterations == 0)
            {
                return baseplns;
            }

            Transform scaleInflate = Transform.Scale(centerpln.Origin, DeflationScaleFactor);

            // Scale all baseplns - build lists first, then set all at once
            var scaledPlanes = new List<Plane>[4];
            for (int i = 0; i < 4; i++)
            {
                var planes = baseplns.Branches[i];
                scaledPlanes[i] = new List<Plane>(planes.Count);
                for (int j = 0; j < planes.Count; j++)
                {
                    Plane scaledPlane = planes[j];
                    scaledPlane.Transform(scaleInflate);
                    scaledPlanes[i].Add(new Plane(scaledPlane));
                }
            }

            // Rebuild baseplns efficiently
            baseplns = new DataTree<Plane>();
            for (int i = 0; i < 4; i++)
            {
                GH_Path pth = new GH_Path(i);
                baseplns.AddRange(scaledPlanes[i], pth);
            }

            // Perform deflation
            DataTree<Plane> inflatedbaseplns = InflateGeometryOptimized(baseplns, deflationRules);

            // Filter
            DataTree<Plane> filteredbaseplns;
            if (geometryFilterArray == null || geometryFilterArray.Count == 0)
            {
                filteredbaseplns = inflatedbaseplns;
            }
            else
            {
                GeometryBase geometryFilter = geometryFilterArray[iterations - 1];
                double buffer = scale;
                if (geometryFilter.HasBrepForm)
                {
                    Brep brepFilter = Brep.TryConvertBrep(geometryFilter);
                    filteredbaseplns = BrepFilterPlanesOptimized(inflatedbaseplns, brepFilter, filterDistance, includeInterior, iterations, buffer);
                }
                else
                {
                    Curve crvFilter = geometryFilter as Curve;
                    filteredbaseplns = CrvFilterPlanesOptimized(inflatedbaseplns, crvFilter, filterDistance, iterations, buffer);
                }
                geometryFilterArray.RemoveAt(iterations - 1);
            }

            // Remove duplicates
            DataTree<Plane> culledbaseplns = CullDuplicatePlanesOptimized(filteredbaseplns, 0.1 * scale);

            return RecurseInflateGeometry(geometryFilterArray, filterDistance, includeInterior, centerpln, culledbaseplns, iterations - 1, scale, deflationRules);
        }

        // Optimized InflateGeometry using batch operations (sequential to maintain deterministic order)
        public static DataTree<Plane> InflateGeometryOptimized(
            DataTree<Plane> baseplns, 
            Plane[][][] deflationRules)
        {
            // Use lists for deterministic ordering (matching original behavior)
            var resultLists = new List<Plane>[4];
            for (int i = 0; i < 4; i++)
            {
                resultLists[i] = new List<Plane>();
            }

            // Process each tile type
            for (int tileType = 0; tileType < 4; tileType++)
            {
                var planes = baseplns.Branches[tileType];
                if (planes.Count == 0) continue;

                Plane[][] planesToCopy = deflationRules[tileType];

                // Sequential processing to maintain deterministic order
                for (int j = 0; j < planes.Count; j++)
                {
                    if (!planes[j].IsValid) continue;

                    Transform xcopyPlane = Transform.PlaneToPlane(Plane.WorldXY, planes[j]);

                    // Copy all planes from each branch
                    for (int branchIdx = 0; branchIdx < 4; branchIdx++)
                    {
                        var sourcePlanes = planesToCopy[branchIdx];
                        for (int p = 0; p < sourcePlanes.Length; p++)
                        {
                            Plane transformedPlane = sourcePlanes[p];
                            transformedPlane.Transform(xcopyPlane);
                            resultLists[branchIdx].Add(new Plane(transformedPlane));
                        }
                    }
                }
            }

            // Build result structure using AddRange (much faster than individual Add)
            DataTree<Plane> result = new DataTree<Plane>();
            for (int i = 0; i < 4; i++)
            {
                result.AddRange(resultLists[i], new GH_Path(i));
            }

            return result;
        }

        public static DataTree<Plane> BrepFilterPlanesOptimized(
            DataTree<Plane> inflatedbaseplns, 
            Brep brepFilter, 
            double filterDistance, 
            bool includeInterior, 
            int iterations, 
            double buffer)
        {
            if (iterations == 1) buffer = 0;

            double filterDivisionFactor = Math.Pow(InverseDeflationScaleFactor, iterations - 1);
            double maxDistance = filterDistance * filterDivisionFactor + (buffer * 1.5);

            var resultLists = new List<Plane>[4];
            
            for (int i = 0; i < 4; i++)
            {
                var planes = inflatedbaseplns.Branches[i];
                resultLists[i] = new List<Plane>(planes.Count);
                
                // Sequential processing to maintain deterministic order
                for (int j = 0; j < planes.Count; j++)
                {
                    Point3d testPoint = planes[j].Origin;
                    
                    if (includeInterior && brepFilter.IsPointInside(testPoint, RhinoMath.SqrtEpsilon, false))
                    {
                        resultLists[i].Add(planes[j]);
                        continue;
                    }
                    
                    Point3d closestPoint;
                    ComponentIndex ci;
                    double s, t;
                    Vector3d normal;
                    brepFilter.ClosestPoint(testPoint, out closestPoint, out ci, out s, out t, maxDistance, out normal);
                    
                    double dist = testPoint.DistanceTo(closestPoint);
                    if (dist > 0 && dist < maxDistance)
                    {
                        resultLists[i].Add(planes[j]);
                    }
                }
            }

            DataTree<Plane> result = new DataTree<Plane>();
            for (int i = 0; i < 4; i++)
            {
                result.AddRange(resultLists[i], new GH_Path(i));
            }
            return result;
        }

        public static DataTree<Plane> CrvFilterPlanesOptimized(
            DataTree<Plane> inflatedbaseplns, 
            Curve crvFilter, 
            double filterDistance, 
            int iterations, 
            double buffer)
        {
            if (iterations == 1) buffer = 0;

            double filterDivisionFactor = Math.Pow(InverseDeflationScaleFactor, iterations - 1);
            double maxDistance = filterDistance * filterDivisionFactor + (buffer * 1.5);

            var resultLists = new List<Plane>[4];
            
            for (int i = 0; i < 4; i++)
            {
                var planes = inflatedbaseplns.Branches[i];
                resultLists[i] = new List<Plane>(planes.Count);
                
                // Sequential processing to maintain deterministic order
                for (int j = 0; j < planes.Count; j++)
                {
                    Point3d testPoint = planes[j].Origin;
                    double t;
                    if (crvFilter.ClosestPoint(testPoint, out t, maxDistance))
                    {
                        resultLists[i].Add(planes[j]);
                    }
                }
            }

            DataTree<Plane> result = new DataTree<Plane>();
            for (int i = 0; i < 4; i++)
            {
                result.AddRange(resultLists[i], new GH_Path(i));
            }
            return result;
        }

        // Properly optimized duplicate culling with correct spatial hashing
        public static DataTree<Plane> CullDuplicatePlanesOptimized(DataTree<Plane> filteredbaseplns, double tolerance)
        {
            // Cell size should be at least tolerance to ensure all potential duplicates 
            // are in adjacent cells. Using tolerance * 1.0 means checking 27 neighbor cells
            // will cover all points within tolerance distance.
            double cellSize = tolerance;
            double toleranceSq = tolerance * tolerance; // Use squared distance to avoid sqrt
            
            var resultLists = new List<Plane>[4];
            
            for (int i = 0; i < 4; i++)
            {
                var planes = filteredbaseplns.Branches[i];
                
                if (planes.Count == 0)
                {
                    resultLists[i] = new List<Plane>();
                    continue;
                }

                // Dictionary mapping cell keys to list of points in that cell
                var spatialGrid = new Dictionary<long, List<Point3d>>();
                var uniquePlanes = new List<Plane>(planes.Count);
                
                for (int j = 0; j < planes.Count; j++)
                {
                    Point3d testPoint = planes[j].Origin;
                    bool isDup = false;
                    
                    // Calculate cell coordinates
                    int cx = (int)Math.Floor(testPoint.X / cellSize);
                    int cy = (int)Math.Floor(testPoint.Y / cellSize);
                    int cz = (int)Math.Floor(testPoint.Z / cellSize);
                    
                    // Check only neighboring cells (27 total including self)
                    for (int dx = -1; dx <= 1 && !isDup; dx++)
                    {
                        for (int dy = -1; dy <= 1 && !isDup; dy++)
                        {
                            for (int dz = -1; dz <= 1 && !isDup; dz++)
                            {
                                long key = GetCellKey(cx + dx, cy + dy, cz + dz);
                                
                                if (spatialGrid.TryGetValue(key, out List<Point3d> cellPoints))
                                {
                                    for (int k = 0; k < cellPoints.Count; k++)
                                    {
                                        // Use squared distance comparison (faster, no sqrt)
                                        double ddx = testPoint.X - cellPoints[k].X;
                                        double ddy = testPoint.Y - cellPoints[k].Y;
                                        double ddz = testPoint.Z - cellPoints[k].Z;
                                        if (ddx*ddx + ddy*ddy + ddz*ddz < toleranceSq)
                                        {
                                            isDup = true;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                    if (!isDup)
                    {
                        long myKey = GetCellKey(cx, cy, cz);
                        if (!spatialGrid.TryGetValue(myKey, out List<Point3d> myCell))
                        {
                            myCell = new List<Point3d>();
                            spatialGrid[myKey] = myCell;
                        }
                        myCell.Add(testPoint);
                        uniquePlanes.Add(planes[j]);
                    }
                }
                
                resultLists[i] = uniquePlanes;
            }

            DataTree<Plane> result = new DataTree<Plane>();
            for (int i = 0; i < 4; i++)
            {
                result.AddRange(resultLists[i], new GH_Path(i));
            }
            return result;
        }

        private static long GetCellKey(int x, int y, int z)
        {
            // Use prime number multiplication for better hash distribution
            unchecked
            {
                return ((long)x * 73856093L) ^ ((long)y * 19349663L) ^ ((long)z * 83492791L);
            }
        }

        public static List<GeometryBase> GetGeoFilterArray(GeometryBase geo, int iterations, Plane centerpln)
        {
            Transform scaleInflate = Transform.Scale(centerpln.Origin, InverseDeflationScaleFactor);

            GeometryBase geoCopy = geo.Duplicate();
            List<GeometryBase> geoFilterArray = new List<GeometryBase>(iterations);
            int count = iterations;
            while (count > 0)
            {
                geoFilterArray.Add(geoCopy);
                geoCopy = geoCopy.Duplicate();
                geoCopy.Transform(scaleInflate);
                count--;
            }
            return geoFilterArray;
        }


        /// <summary>
        /// Returns the signed angle between two vectors on a plane.
        /// Useful since Vector3d.VectorAngle only returns positive values.
        /// </summary>
        public static double GetSignedVectorAngle(Vector3d v1, Vector3d v2, Plane plane)
        {
            return Math.Atan2(Vector3d.CrossProduct(v1, v2) * plane.ZAxis, v1 * v2);
        }

        #region ---Zonohedra Generation---
        public static Mesh GenerateMeshA6(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(3, false);
            Mesh meshA6 = GenerateZonohedronMeshFromStarVectors(starVectors, scale);
            meshA6.Rotate(Math.PI / 2, Vector3d.ZAxis, Point3d.Origin);

            // Golden ratio
            double phi = (1 + Math.Sqrt(5)) / 2;
            // Rotate according to angle between long diagonal and face, so that long diagonal axis aligns with Z axis
            // Note: This rotation potentially introduces inaccuracies - maybe cleaner to generate the zonohedron already at this angle
            meshA6.Rotate(-Math.Acos(phi / Math.Sqrt(3)) - (Math.PI / 2), Vector3d.YAxis, Point3d.Origin);
            return meshA6;
        }

        public static Mesh GenerateMeshB12(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(4, false);
            return GenerateZonohedronMeshFromStarVectors(starVectors, scale);
        }

        public static Mesh GenerateMeshF20(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(5, false);
            Mesh meshF20 = GenerateZonohedronMeshFromStarVectors(starVectors, scale);
            meshF20.Rotate(Math.PI, Vector3d.ZAxis, Point3d.Origin);
            meshF20.Rotate(Math.Asin(Math.Sqrt((5 + Math.Sqrt(5)) / 10)), Vector3d.YAxis, Point3d.Origin);
            return meshF20;
        }
        public static Mesh GenerateMeshK30(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(6, false);
            return GenerateZonohedronMeshFromStarVectors(starVectors, scale);
        }

        public static Mesh GenerateZonohedronMeshFromStarVectors(List<Vector3d> starVectors, double scale)
        {
            // Scale star vectors
            List<Vector3d> scaledStarVectors = new List<Vector3d>();
            foreach (Vector3d vec in starVectors)
            {
                vec.Unitize();
                scaledStarVectors.Add(Vector3d.Multiply(vec, scale * 0.5));
            }

            // Generate normal vectors for faces
            List<Vector3d> normalVectors = new List<Vector3d>();

            // Using all pairwise combinations of generators
            for (int i = 0; i < scaledStarVectors.Count; i++)
            {
                for (int j = i + 1; j < scaledStarVectors.Count; j++)
                {
                    Vector3d a = scaledStarVectors[i];
                    Vector3d b = scaledStarVectors[j];

                    // Get normal vector via cross product
                    Vector3d n = Vector3d.CrossProduct(a, b);
                    normalVectors.Add(n);
                }
            }

            // Loop through normal vectors to create p-representations, faces
            IList<IList<int>> faces_p = new List<IList<int>>();

            foreach (Vector3d n in normalVectors)
            {
                // Set up face p-representation and its opposite
                List<int> face_p_representation = new List<int>();
                List<int> face_p_representationOpposite = new List<int>();

                // Loop through star vectors
                foreach (Vector3d v in scaledStarVectors)
                {
                    // Get dot product
                    double d = Vector3d.Multiply(n, v);
                    // Create p-representation entries
                    if (Math.Abs(d) < 0.0001)
                    {
                        face_p_representation.Add(0);
                        face_p_representationOpposite.Add(0);
                    }
                    else if (d > 0)
                    {
                        face_p_representation.Add(1);
                        face_p_representationOpposite.Add(-1);
                    }
                    else
                    {
                        face_p_representation.Add(-1);
                        face_p_representationOpposite.Add(1);
                    }
                }

                // Add both face representations to the list
                faces_p.Add(face_p_representation);
                faces_p.Add(face_p_representationOpposite);
            }

            // Build mesh from face representations
            Mesh mesh = BuildMeshFromFaceRepresentations(scaledStarVectors, faces_p);
            return mesh;
        }

        public static List<Vector3d> GenerateStarVectors(int numZones, bool isO6)
        {
            // Golden ratio
            double phi = (1 + Math.Sqrt(5)) / 2;

            // Get all icosahedral star vectors
            List<Vector3d> allStarVectors = new List<Vector3d>
            {
                new Vector3d(-1, phi, 0),
                new Vector3d(1, phi, 0),
                new Vector3d(0, 1, phi),
                new Vector3d(0, 1, -phi),
                new Vector3d(-phi, 0, 1),
                new Vector3d(-phi, 0, -1)
            };

            List<Vector3d> selectedVectors;
            // For O6, only use selection of 3 vectors
            if (isO6)
            {
                selectedVectors = new List<Vector3d>() { allStarVectors[1], allStarVectors[2], allStarVectors[3] };
            }
            else
            {
                // Select the required number of zones
                selectedVectors = allStarVectors.GetRange(0, numZones);
            }
            return selectedVectors;
        }

        public static int ToMaskFromSigns(IList<int> signs)
        {
            int mask = 0;
            for (int i = 0; i < signs.Count; i++)
            {
                if (signs[i] == +1) mask |= (1 << i); // if +1 => bit is set to 1
                                                      // if -1 => bit stays 0
            }
            return mask;
        }

        public static (int m1, int m2, int m3, int m4) FaceToQuadMasks(IList<int> face)
        {
            // Find the two zero indices
            int z0 = -1, z1 = -1;
            for (int i = 0; i < face.Count; i++)
            {
                if (face[i] != 0) continue;
                if (z0 < 0) z0 = i;
                else { z1 = i; break; }
            }

            if (z0 < 0 || z1 < 0)
                throw new ArgumentException("Face must contain exactly two zeros.");

            // Start from the fixed signs, zeros will be modified in next step
            var baseSigns = new int[face.Count];
            for (int i = 0; i < face.Count; i++) baseSigns[i] = face[i];

            // Now create the 4 combos in cyclic order
            // (+,+), (+,-), (-,-), (-,+)
            int[] a = (int[])baseSigns.Clone(); a[z0] = +1; a[z1] = +1;
            int[] b = (int[])baseSigns.Clone(); b[z0] = +1; b[z1] = -1;
            int[] c = (int[])baseSigns.Clone(); c[z0] = -1; c[z1] = -1;
            int[] d = (int[])baseSigns.Clone(); d[z0] = -1; d[z1] = +1;

            return (ToMaskFromSigns(a), ToMaskFromSigns(b), ToMaskFromSigns(c), ToMaskFromSigns(d));
        }

        // Create Point3d vertex from mask and list of generator vectors
        static Point3d VertexFromMask(IList<Vector3d> gens, int mask)
        {
            Vector3d sum = Vector3d.Zero;
            for (int i = 0; i < gens.Count; i++)
            {
                double s = ((mask & (1 << i)) != 0) ? 1 : -1;
                sum += s * gens[i];
            }
            return new Point3d(sum);
        }

        // Build mesh from face p-representations and list of generator vectors
        static Mesh BuildMeshFromFaceRepresentations(IList<Vector3d> gens, IList<IList<int>> facesP) // each is -1/0/+1 list
        {
            // Set up indexing of masks to unique vertex indices and list of mesh quad faces
            var maskToIndex = new Dictionary<int, int>();
            var uniqueVerts = new List<Point3d>();
            var quads = new List<(int a, int b, int c, int d)>();

            // Local function to get vertex index from mask, or to add new vertex entry
            int GetIndex(int mask)
            {
                // Existing vertex
                if (maskToIndex.TryGetValue(mask, out int idx))
                    return idx;

                // New vertex, creates new entry
                idx = uniqueVerts.Count;
                uniqueVerts.Add(VertexFromMask(gens, mask));
                maskToIndex.Add(mask, idx);
                return idx;
            }

            // Loop through faces to build quads
            foreach (var face in facesP)
            {
                var (m1, m2, m3, m4) = FaceToQuadMasks(face);
                int a = GetIndex(m1);
                int b = GetIndex(m2);
                int c = GetIndex(m3);
                int d = GetIndex(m4);
                quads.Add((a, b, c, d));
            }

            // Build mesh
            var mesh = new Mesh();
            mesh.Vertices.AddVertices(uniqueVerts);
            foreach (var q in quads) mesh.Faces.AddFace(q.a, q.b, q.c, q.d);

            // Cleanup
            mesh.Weld(Math.PI);
            mesh.UnifyNormals();
            mesh.Normals.ComputeNormals();
            mesh.Compact();

            return mesh;
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

        #endregion

        #region ---Seed Options---

        public static double GetMeshHeight(Mesh mesh)
        {
            BoundingBox bbox = mesh.GetBoundingBox(true);
            return bbox.Max.Z - bbox.Min.Z;
        }

        public static DataTree<Plane> GenerateBasePlnsFromSeed(int seed, Plane centerpln, double a6HeightRef)
        {
            // Create baseplns data tree and ensure paths for each tile type
            DataTree<Plane> baseplns = new DataTree<Plane>();

            GH_Path pth0 = new GH_Path(0);
            GH_Path pth1 = new GH_Path(1);
            GH_Path pth2 = new GH_Path(2);
            GH_Path pth3 = new GH_Path(3);

            baseplns.EnsurePath(pth0);
            baseplns.EnsurePath(pth1);
            baseplns.EnsurePath(pth2);
            baseplns.EnsurePath(pth3);

            if (seed == 0)
            {
                Plane basepln = new Plane(Plane.WorldXY);
                basepln.Transform(Transform.PlaneToPlane(Plane.WorldXY, centerpln));
                baseplns.Add(basepln, pth3);
            }
            else
            {
                // Set up seed 1 and 2 option (20 A6 tiles arranged as dodecahedron star)
                // Golden ratio
                double phi = (1 + Math.Sqrt(5)) / 2;
                double invPhi = 1.0 / phi;

                // 20 vertices of a regular dodecahedron using golden ratio
                List<Vector3d> dodecahedronStarVectors = new List<Vector3d>
                {
                    // 8 vertices at (±1, ±1, ±1)
                    new Vector3d(1, 1, 1),
                    new Vector3d(1, 1, -1),
                    new Vector3d(1, -1, 1),
                    new Vector3d(1, -1, -1),
                    new Vector3d(-1, 1, 1),
                    new Vector3d(-1, 1, -1),
                    new Vector3d(-1, -1, 1),
                    new Vector3d(-1, -1, -1),
                    // 4 vertices at (0, ±1/φ, ±φ)
                    new Vector3d(0, invPhi, phi),
                    new Vector3d(0, invPhi, -phi),
                    new Vector3d(0, -invPhi, phi),
                    new Vector3d(0, -invPhi, -phi),
                    // 4 vertices at (±1/φ, ±φ, 0)
                    new Vector3d(invPhi, phi, 0),
                    new Vector3d(invPhi, -phi, 0),
                    new Vector3d(-invPhi, phi, 0),
                    new Vector3d(-invPhi, -phi, 0),
                    // 4 vertices at (±φ, 0, ±1/φ)
                    new Vector3d(phi, 0, invPhi),
                    new Vector3d(phi, 0, -invPhi),
                    new Vector3d(-phi, 0, invPhi),
                    new Vector3d(-phi, 0, -invPhi)
                };

                // Adjacent dodecahedron vertices have dot product = √5 = φ + 1/φ (without unitizing)
                double adjacentDotProduct = Math.Sqrt(5);  // ≈ 2.236
                double tolerance = 0.1;  // Scaled for non-unitized vectors

                foreach (Vector3d vector in dodecahedronStarVectors)
                {
                    Vector3d unitVector = vector;
                    unitVector.Unitize();
                    Vector3d scaledVector = unitVector * a6HeightRef / 2;
                    Plane seedPlane = new Plane(Point3d.Origin + (Point3d)scaledVector, unitVector);

                    // Find an adjacent vector to align the X-axis
                    foreach (Vector3d otherVector in dodecahedronStarVectors)
                    {
                        double dot = vector * otherVector;  // No need to unitize for comparison
                        if (Math.Abs(dot - adjacentDotProduct) < tolerance)
                        {
                            // Rotate the plane to align X-axis toward the adjacent vertex
                            Vector3d edgeDirection = otherVector - vector;
                            double angle = GetSignedVectorAngle(seedPlane.XAxis, edgeDirection, seedPlane);
                            seedPlane.Rotate(angle + Math.PI, seedPlane.Normal);
                            break;
                        }
                    }
                    if (seed == 1)
                    {
                        seedPlane.Transform(Transform.PlaneToPlane(Plane.WorldXY, centerpln));
                        baseplns.Add(seedPlane, pth0);
                    }
                    else if (seed == 2)
                    {
                        Plane flippedSeedPlane = new Plane(seedPlane);
                        flippedSeedPlane.Flip();
                        flippedSeedPlane.Rotate(-Math.PI / 2, flippedSeedPlane.Normal);
                        flippedSeedPlane.Transform(Transform.PlaneToPlane(Plane.WorldXY, centerpln));
                        baseplns.Add(flippedSeedPlane, pth0);
                    }
                }
            }
            return baseplns;
        }

        public static List<Curve> GetWireframeEdges(Mesh mesh)
        {
            var curves = new List<Curve>();
            for (int i = 0; i < mesh.TopologyEdges.Count; i++)
            {
                Line line = mesh.TopologyEdges.EdgeLine(i);
                curves.Add(new LineCurve(line));
            }
            return curves;
        }
        #endregion

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            var color = Attributes.Selected
                ? args.WireColour_Selected
                : args.WireColour;

            foreach (var crv in _previewCurves)
                args.Display.DrawCurve(crv, color, 2);
        }

        public override BoundingBox ClippingBox
        {
            get
            {
                var bb = BoundingBox.Empty;

                foreach (var crv in _previewCurves)
                    bb.Union(crv.GetBoundingBox(false));

                return bb;
            }
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => Resources.aperiodic4tile24px;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("{7693d4a5-60af-4d28-a69d-739af538a058}");
    }
}