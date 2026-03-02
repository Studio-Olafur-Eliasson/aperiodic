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
            pManager.AddGeometryParameter("Geometry Filter", "geometryFilter", "(Optional) Input a geometry filter (Brep or Curve) to define the output shape of the tiling and reduce computation time.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Filter Distance", "filterDistance", "Distance from the geometryFilter within which tiles should be included in the output.", GH_ParamAccess.item, 1.0);
            pManager.AddBooleanParameter("IncludeInterior", "includeInterior", "Boolean for whether to include tiles on the interior of the filter geometry (if it is a closed Brep). Default false.", GH_ParamAccess.item, false);
            pManager.AddPlaneParameter("Center Plane", "centerPln", "Plane input for the center of the recursive tile-generation process. Default: World XY.", GH_ParamAccess.item, Plane.WorldXY);
            pManager.AddIntegerParameter("Iterations", "iterations", "Number of iterations of the recursive process. If iterations > 2, must use geometryFilter to avoid crashing. Set iterations = 0 to view the starting \"seed\" tiles of the recusive process. Default: 1", GH_ParamAccess.item, 1);
            pManager.AddIntegerParameter("SeedOption", "seedOption", "Enter an integer option, 0, 1, or 2. According to Socolar and Steinhardt, who published the discovery of this 4-tile configuration in 1986, there exist exactly three packings with a single center of icosahedral point symmetry in 3D Euclidean space. These three options are each generated with one of the following \"seed\" tile configurations: 0 = a single rhombic triacontahedron tile (Default); 1 = a star of twenty rhombohedra, which, after deflation/inflation, are surrounded by rhombic triacontahedra; 2 = a star of twenty rhombohedra, with flipped orientations with respect to the previous option, so that they are surrounded by rhombic icosahedra on the next layer after deflation/inflation.", GH_ParamAccess.item, 0);
            pManager[0].Optional = true;
            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[4].Optional = true;
            pManager[5].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Base Meshes", "baseMeshes", "Set of base mesh geometry of the four zonohedral tiles. Apply the output transformations to view tiling result.", GH_ParamAccess.tree);
            pManager.AddBrepParameter("Base Breps", "baseBreps", "Set of base brep geometry of the four zonohedral tiles. Apply the output transformations to view tiling result.", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("Transformations", "X", "Result of the recursive process. Apply these output transformations to the baseMeshes, baseBreps, or other substitute geometry. The tree structure contains a separate branch for each of the four tile types: {0} = rhombohedron; {1} = rhombic (Bilinski) dodecahedron; {2} = rhombic icosahedron; {3} = rhombic triacontahedron. The Default output values correspond to 1 iteration of the deflation using seed option 0 (beginning with a rhombic triacontahedron) and no geometryFilter.", GH_ParamAccess.tree);
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
            DA.GetData(0, ref geometryFilter);
            DA.GetData(1, ref filterDistance);
            DA.GetData(2, ref includeInterior);
            DA.GetData(3, ref centerpln);
            DA.GetData(4, ref iterations);
            DA.GetData(5, ref seed);

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

            // DA.SetDataList(0, gfaCopy);

            // Generate meshes for each tile type
            Mesh refA6 = GenerateMeshA6(scale);
            Mesh refB12 = GenerateMeshB12(scale);
            Mesh refF20 = GenerateMeshF20(scale);
            Mesh refK30 = GenerateMeshK30(scale);
            DataTree<Mesh> baseMeshes = new DataTree<Mesh>();
            baseMeshes.Add(refA6.DuplicateMesh(), new GH_Path(0));
            baseMeshes.Add(refB12.DuplicateMesh(), new GH_Path(1));
            baseMeshes.Add(refF20.DuplicateMesh(), new GH_Path(2));
            baseMeshes.Add(refK30.DuplicateMesh(), new GH_Path(3));

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

            // Perform recursive inflation/deflation process to get output planes for transformations
            DataTree<Plane> outputplns = RecurseInflateGeometry(gfa, filterDistance, includeInterior, centerpln, baseplns, iterations, scale, deflationRules);

            // Generate Base Breps
            DataTree<Brep> baseBreps = new DataTree<Brep>();
            Brep brepA6 = GenerateBrepA6(scale);
            Brep brepB12 = GenerateBrepB12(scale);
            Brep brepF20 = GenerateBrepF20(scale);
            Brep brepK30 = GenerateBrepK30(scale);
            baseBreps.Add(brepA6, new GH_Path(0));
            baseBreps.Add(brepB12, new GH_Path(1));
            baseBreps.Add(brepF20, new GH_Path(2));
            baseBreps.Add(brepK30, new GH_Path(3));

            // Set output parameter data
            DA.SetDataTree(0, baseMeshes);
            DA.SetDataTree(1, baseBreps);
            DA.SetDataTree(2, outputplns);
        }

        #region Generate Deflation Planes / Rules
        private static DataTree<Plane> GenerateDeflationPlanesA6(Mesh refA6, Mesh refB12, Mesh refF20, Mesh refK30)
        {
            // Note: always duplicate reference meshes before modifying

            // Set up DataTree
            DataTree<Plane> deflationRulesA6 = new DataTree<Plane>();

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
            deflationRulesA6.AddRange(plnsA6, new GH_Path(0));
            deflationRulesA6.AddRange(plnsB12, new GH_Path(1));
            deflationRulesA6.AddRange(plnsF20, new GH_Path(2));
            deflationRulesA6.AddRange(plnsK30, new GH_Path(3));

            return deflationRulesA6;
        }

        private static DataTree<Plane> GenerateDeflationPlanesB12(Mesh refA6, Mesh refB12, Mesh refF20, Mesh refK30)
        {
            // Set up DataTree
            DataTree<Plane> deflationRulesB12 = new DataTree<Plane>();

            // Get height references from original input meshes
            double a6HeightRef = GetMeshHeight(refA6);
            double b12HeightRef = GetMeshHeight(refB12);
            double f20HeightRef = GetMeshHeight(refF20);
            double k30HeightRef = GetMeshHeight(refK30);
            double edgeLengthRef = refK30.TopologyEdges.EdgeLine(0).Length;
            //double edgeLengthRef = refK30.Edges[0].PointAtEnd.DistanceTo(refK30.Edges[0].PointAtStart); //brep code

            Mesh mesh = refB12.DuplicateMesh();
            Point3d basept = Point3d.Origin;
            Plane basepln = Plane.WorldXY;

            // Get the centroid of the geometry
            AreaMassProperties ampMesh = AreaMassProperties.Compute(mesh);
            Point3d centroidMesh = ampMesh.Centroid;
            // Create a vector using the centroid location
            Vector3d centroidVec = new Vector3d(centroidMesh);
            // Scale the vector by the deflationScalefactor to get the new centroid location
            // Subtract the original centroidVec to get the translation vector
            Vector3d inflateVec = centroidVec * DeflationScaleFactor - centroidVec;
            // Translate the brep to the new scaled location (but without scaling the brep itself, so the unit size remains the same)
            //mesh.Translate(inflateVec);
            // Translate the basept to the new scaled location - consistent with the brep itself, so further operations on the base pts can recurse well
            //basept += inflateVec;
            Transform scaleInflate = Transform.Scale(Point3d.Origin, DeflationScaleFactor);
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

            // Scale up B12 unit to get general boundaries of the inflated shapes
            // Scale center point of geometry by a factor of golden ratio^3
            //Transform xformScaleB12 = Transform.Scale(centroidB12, deflationScaleFactor);
            Transform xformScaleB12 = Transform.Scale(Point3d.Origin, DeflationScaleFactor);
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
                    Transform xscale = Transform.Scale(Point3d.Origin, b12HeightRef);
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
                // This is narowing down to the 4 faces of the rhombic triacontahedron that are perpendicular to planeb12k30 normal
                else if ((compareFaceNormal < 0.0001) && (compareFaceNormal > -0.0001))
                {
                    Plane planeb12b12 = GetOrientedPlaneFromRhombicFace(b12k300, i, b12b12normal);
                    double compareFaceOrientation = Vector3d.Multiply(planeb12b12.XAxis, planeb12k300.Normal);
                    // Narrow down to the two faces along the long direction of the boundary B12 (x-axis is parallel to planeb12k300 normal)
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
                        Transform xscale = Transform.Scale(Point3d.Origin, b12HeightRef);
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
                            // Find face of b12 that faces opposite direction of orientb12300
                            if (b12b12facenormal.IsParallelTo(orientb12k300) == -1)
                            {
                                // Place an A6 tile, get plane centered on acute vertex
                                Plane planeb12a61 = GetOrientedPlaneFromRhombicFaceAcute(b12b12, j, b12b12facenormal);

                                Point3d projectedOrigin = planeb12a61.Origin;
                                projectedOrigin.Transform(Transform.PlanarProjection(planeb12b12));
                                // If the plane origin is not on planeb12b12, we want to rotate 180 around the face center
                                if (projectedOrigin.DistanceTo(planeb12a61.Origin) > 0.0001)
                                {
                                    Plane rotationPlane = GetOrientedPlaneFromRhombicFace(b12b12, j, b12b12facenormal);
                                    // Use Rotate with 3 parameters to rotate around another origin
                                    planeb12a61.Rotate(Math.PI, rotationPlane.Normal, rotationPlane.Origin);
                                }

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
                        planeb12a62.Translate(pushb12a62 * (k30HeightRef + b12HeightRef / 2));
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
            double zTranslation = -b12HalfHeight * DeflationScaleFactor;
            Vector3d zOffset = new Vector3d(0, 0, zTranslation);
            TranslatePlanesInDirection(plnsA6, zOffset);
            TranslatePlanesInDirection(plnsB12, zOffset);
            TranslatePlanesInDirection(plnsF20, zOffset);
            TranslatePlanesInDirection(plnsK30, zOffset);

            // Output plns
            deflationRulesB12.AddRange(plnsA6, new GH_Path(0));
            deflationRulesB12.AddRange(plnsB12, new GH_Path(1));
            deflationRulesB12.AddRange(plnsF20, new GH_Path(2));
            deflationRulesB12.AddRange(plnsK30, new GH_Path(3));

            return deflationRulesB12;
        }

        private static DataTree<Plane> GenerateDeflationPlanesF20(Mesh refA6, Mesh refB12, Mesh refF20, Mesh refK30)
        {
            // Set up DataTree
            DataTree<Plane> deflationRulesF20 = new DataTree<Plane>();

            // Get height references from original input meshes
            double a6HeightRef = GetMeshHeight(refA6);
            double b12HeightRef = GetMeshHeight(refB12);
            double f20HeightRef = GetMeshHeight(refF20);
            double k30HeightRef = GetMeshHeight(refK30);

            // Get length reference (edge length of original triacontahedron)
            //double edgeLengthRef = refK30.Edges[0].PointAtEnd.DistanceTo(refK30.Edges[0].PointAtStart); //brep code
            double edgeLengthRef = refK30.TopologyEdges.EdgeLine(0).Length;

            // Get angle reference
            double rhombAcuteAngle = 2 * Math.Atan(1 / GoldenRatio);

            Mesh mesh = refF20.DuplicateMesh();
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

            // Set up base orientation for K30 transformation in step (a6-000)
            Point3d k30basecenter = refK30.TopologyVertices[GetClosestVertex(refK30, new Point3d(0, -1, 0))];
            Point3d k30basexaxis = refK30.TopologyVertices[GetClosestVertex(refK30, new Point3d(1, 0, 0))];
            Point3d k30baseyaxis = new Point3d(-k30basexaxis.X, 0, 0);
            Plane k30base = new Plane(k30basecenter, k30basexaxis, k30baseyaxis);

            // Begin deflation for F20

            // Scale up F20 unit to get general boundaries of the inflated shapes
            // Scale center point of geometry by a factor of golden ratio^3
            Transform xformScaleF20 = Transform.Scale(Point3d.Origin, deflationScaleFactor);
            Mesh f20boundary = mesh.DuplicateMesh();
            f20boundary.Transform(xformScaleF20);
            Point3d f20boundarybase = basept;
            f20boundarybase.Transform(xformScaleF20);

            // Get far pt vertex
            int f20farvertexIndex = GetFurthestVertex(f20boundary, (Point3d)f20boundarybase);
            Point3d f20farvertexPt = (Point3d)f20boundary.TopologyVertices[f20farvertexIndex];

            // Array 5 of the A6 around base pt
            // Get base pt vertex
            int f20basevertexIndex = GetClosestVertex(f20boundary, (Point3d)f20boundarybase);
            Point3d f20basevertexPt = (Point3d)f20boundary.TopologyVertices[f20basevertexIndex];
            Vector3d normalf20 = f20farvertexPt - f20basevertexPt;
            Point3d baseptf20a60 = f20basevertexPt;

            // Get one adjacent vertex from vertex
            adjacentVertexIndices = f20boundary.Vertices.GetConnectedVertices(f20basevertexIndex);
            Point3d boundaryedgept = (Point3d)f20boundary.Vertices[adjacentVertexIndices[0]];
            Transform xrot5 = Transform.Rotation(2 * Math.PI / 5, normalf20, f20basevertexPt);
            Point3d edgept2 = boundaryedgept;
            edgept2.Transform(xrot5);

            // Get plane and transform
            Plane planef20a60 = new Plane(f20basevertexPt, edgept2, boundaryedgept);
            Transform xformf20a60 = Transform.PlaneToPlane(a6base, planef20a60);
            Vector3d moveInEdge = normalf20;
            moveInEdge.Unitize();
            Transform xscaleEdgeLength = Transform.Scale(f20basevertexPt, edgeLengthRef);
            moveInEdge.Transform(xscaleEdgeLength);
            Transform xMoveInEdge = Transform.Translation(moveInEdge);

            // Add the first A6 tile
            Mesh f20a60 = refA6.DuplicateMesh();
            f20a60.Transform(xformf20a60);
            f20a60.Transform(xMoveInEdge);

            // Add to mesh list
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
            int edgptPtIndex = GetClosestVertex(f20a60, boundaryedgept);
            Point3d f20f200xpt = new Point3d();
            // Get adjacent TOPOLOGY vertex points
            int[] adjacentTopoIndices = f20a60.TopologyVertices.ConnectedTopologyVertices(edgptPtIndex);
            for (int j = 0; j < adjacentTopoIndices.Length; j++)
            {
                Point3d f20a60adjacentPt = f20a60.TopologyVertices[adjacentTopoIndices[j]];
                if (f20a60adjacentPt.DistanceTo(f20basevertexPt) > 0.0001)  // Use distance tolerance instead of !=
                {
                    f20f200xpt = f20a60adjacentPt;
                    break;  // Take the FIRST valid point and stop
                }
            }

            // Also prepare to add F20 tile here
            Mesh f20f200 = refF20.DuplicateMesh();
            Vector3d f20f200normal = boundaryedgept - f20basevertexPt;
            f20f200normal.Unitize();
            f20f200normal.Transform(xscaleEdgeLength);
            Point3d innerptf20f200 = f20basevertexPt + f20f200normal;
            Plane planef20f200 = new Plane(innerptf20f200, f20f200normal);

            // Orient plane
            Vector3d f20f200xaxis = f20f200xpt - innerptf20f200;
            double anglef20f200 = GetSignedVectorAngle(planef20f200.XAxis, f20f200xaxis, planef20f200);
            planef20f200.Rotate(anglef20f200 - Math.PI, f20f200normal, innerptf20f200); // Possibly need to change the Math.PI, check

            // Copy and transform the F20 mesh
            Transform xformf20f200 = Transform.PlaneToPlane(Plane.WorldXY, planef20f200);
            f20f200.Transform(xformf20f200);

            // Add to mesh list
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

            Mesh f20a60copy = f20a60.DuplicateMesh();
            Mesh f20f200copy = f20f200.DuplicateMesh();
            Point3d baseptf20f200copy = baseptf20f200;
            Plane planef20a60scopy = new Plane(planef20a60s);
            Plane planef20f200copy = new Plane(planef20f200);
            // Rotate to get 4 more copies of the A6 and the F20
            for (int j = 1; j < 5; j++)
            {
                // Rotate edgept around normal 2pi/5 degrees
                f20a60copy.Transform(xrot5);

                // Add to mesh list
                listA6.Add(f20a60copy);

                // Add to basepts list
                ptsA6.Add(baseptf20a60);

                // Add to baseplns list
                planef20a60scopy.Transform(xrot5);
                plnsA6.Add(planef20a60scopy);

                f20a60copy = f20a60copy.DuplicateMesh();
                f20f200copy.Transform(xrot5);

                // Add to mesh list
                listF20.Add(f20f200copy);

                baseptf20f200copy.Transform(xrot5);
                planef20f200copy.Transform(xrot5);

                // Add to basepts list
                ptsF20.Add(baseptf20f200copy);

                // Add to baseplns list
                plnsF20.Add(planef20f200copy);

                // Not sure if this is necessary
                f20a60copy = f20a60copy.DuplicateMesh();
                f20f200copy = f20f200copy.DuplicateMesh();
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
            Mesh f20k300 = refK30.DuplicateMesh();
            f20k300.Transform(xf20k300);

            // Add to mesh list
            listK30.Add(f20k300);

            // Add to basepts list
            ptsK30.Add(f20k300base);

            // Add to baseplns list
            Plane planef20k300s = Plane.WorldXY;
            planef20k300s.Transform(xf20k300);
            plnsK30.Add(planef20k300s);

            // Place B12 tiles on the inward-facing faces
            // Must call this before referencing mesh face normals
            f20k300.FaceNormals.ComputeFaceNormals();
            f20k300.UnifyNormals();

            // Unify normals does not seem to work properly
            // But we can use the centroid to calculate them in a consistent way
            AreaMassProperties ampf20k30 = AreaMassProperties.Compute(f20k300);
            Point3d centroidf20k30 = ampf20k30.Centroid;

            // Place B12 around the K30. Loop through faces of the K30
            for (int i = 0; i < f20k300.Faces.Count; i++)
            {
                // Get normal of face and see if it has positive dot product with normal
                Point3d f20b12base = f20k300.Faces.GetFaceCenter(i);
                //Vector3d f20b12normal = f20k300.FaceNormals[i];
                Vector3d f20b12normal = f20b12base - centroidf20k30;
                double compareFaceNormal = Vector3d.Multiply(f20b12normal, normalf20);
                if (compareFaceNormal >= -0.5)
                {
                    // Add a b12 to the list using this face as a base
                    Plane planef20b12 = GetOrientedPlaneFromRhombicFace(f20k300, i, f20b12normal);
                    Transform xformf20b12 = Transform.PlaneToPlane(Plane.WorldXY, planef20b12);
                    Mesh f20b12 = refB12.DuplicateMesh();
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
                    Mesh e = refK30.DuplicateMesh();
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

            // Must call this before referencing mesh vertex normals
            f20k300.Normals.ComputeNormals();

            // Loop through vertices of the K30. Check for the ones with 3 neighbors that are also facing down
            for (int i = 0; i < f20k300.TopologyVertices.Count; i++)
            {
                if (f20k300.TopologyVertices.ConnectedEdgesCount(i) == 3)
                {
                    Vector3d normal = f20k300.Normals[i];
                    if (Vector3d.Multiply(normal, normalf20) > 0)
                    {
                        // Get plane center
                        Point3d planeCenter = f20k300.TopologyVertices[i];

                        // Get orient point using adjacent  vertex
                        int orientPtIndex = f20k300.TopologyVertices.ConnectedTopologyVertices(i)[0];
                        Point3d orientX = (Point3d)f20k300.TopologyVertices[orientPtIndex];

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
                        Mesh f20a61 = refA6.DuplicateMesh();
                        f20a61.Transform(xformf20a61);

                        // Add to mesh list
                        listA6.Add(f20a61);

                        // Tranform basept
                        Point3d f20a61base = Point3d.Origin;
                        f20a61base.Transform(xformf20a61);

                        // Add to basepts list
                        ptsA6.Add(f20a61base);

                        // Add to baseplns list
                        plnsA6.Add(planeOriented);

                        // Copy A6 3 more times...
                        int furthestVertexIndex = GetFurthestVertex(f20a61, (Point3d)f20a61base);
                        Point3d furthestVertexPt = (Point3d)f20a61.TopologyVertices[furthestVertexIndex];

                        // Get adjacent vertex points
                        adjacentVertexIndices = f20a61.Vertices.GetConnectedVertices(furthestVertexIndex);
                        List<Point3d> edgePoints = new List<Point3d>();
                        for (int j = 0; j < 3; j++)
                        {
                            edgePoints.Add((Point3d)f20a61.Vertices[adjacentVertexIndices[j]]);
                        }

                        // Get mirror planes
                        Plane plane0 = new Plane(furthestVertexPt, edgePoints[1], edgePoints[2]);
                        Plane plane1 = new Plane(furthestVertexPt, edgePoints[2], edgePoints[0]);
                        Plane plane2 = new Plane(furthestVertexPt, edgePoints[0], edgePoints[1]);
                        Transform xform0 = Transform.Mirror(plane0);
                        Transform xform1 = Transform.Mirror(plane1);
                        Transform xform2 = Transform.Mirror(plane2);

                        // Make copies and add to mesh list
                        Mesh copyc0 = f20a61.DuplicateMesh();
                        copyc0.Transform(xform0);
                        listA6.Add(copyc0);
                        Mesh copyc1 = f20a61.DuplicateMesh();
                        copyc1.Transform(xform1);
                        listA6.Add(copyc1);
                        Mesh copyc2 = f20a61.DuplicateMesh();
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
                        Mesh copyg = f20a61.DuplicateMesh();
                        copyg.Transform(xformg);
                        Transform xrot180 = Transform.Rotation(Math.PI, normal, furthestVertexPt);
                        copyg.Transform(xrot180);

                        // Add to mesh list
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
                            furthestVertexIndex = GetFurthestVertex(copyg, (Point3d)centroidf20k30);
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
                            Mesh copyg00 = copyg0.DuplicateMesh();
                            Mesh copyg01 = copyg0.DuplicateMesh();

                            // Again, get furthest vertex/edges
                            furthestVertexIndex = GetFurthestVertex(copyg0, (Point3d)centroidf20k30);

                            // Get adjacent vertex points
                            adjacentVertexIndices = copyg0.Vertices.GetConnectedVertices(furthestVertexIndex);
                            edgePoints = new List<Point3d>();
                            for (int j = 0; j < 3; j++)
                            {
                                edgePoints.Add((Point3d)copyg0.Vertices[adjacentVertexIndices[j]]);
                            }

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

                            // Add to mesh list
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
                            furthestVertexIndex = GetFurthestVertex(copyg, (Point3d)centroidf20k30);
                            furthestVertexPt = copyg.TopologyVertices[furthestVertexIndex];

                            // Get rotation axis by finding edge with positive dot product to f20basevector
                            adjacentVertexIndices = copyg.Vertices.GetConnectedVertices(furthestVertexIndex);
                            Point3d axisPoint = new Point3d();
                            Vector3d axis = new Vector3d();
                            for (int j = 0; j < 3; j++)
                            {
                                axis = copyg.Vertices[adjacentVertexIndices[j]] - furthestVertexPt;
                                if (Vector3d.Multiply(axis, normalf20) > 0)
                                {
                                    axisPoint = copyg.Vertices[adjacentVertexIndices[j]];
                                }
                            }
                            axis = axisPoint - furthestVertexPt;

                            // Rotate copyg about the axis
                            Transform xformrotatef20a6g = Transform.Rotation(2 * Math.PI / 5, axis, axisPoint);
                            Mesh copyf20a6g = copyg.DuplicateMesh();
                            Plane copyf20a6gplane = new Plane(copygplane);
                            for (int j = 0; j < 4; j++)
                            {
                                copyf20a6g.Transform(xformrotatef20a6g);

                                // Add to mesh list
                                listA6.Add(copyf20a6g);
                                copyf20a6g = copyf20a6g.DuplicateMesh();

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
                else if (f20k300.TopologyVertices.ConnectedEdgesCount(i) == 5)
                {
                    Vector3d normal = f20k300.Normals[i];
                    double vectoreComparef20k30 = Vector3d.Multiply(normal, normalf20);

                    if (vectoreComparef20k30 > 0.8)
                    {
                        // Use the plane center and the orientation point
                        Point3d planeCenter5 = (Point3d)f20k300.TopologyVertices[i];
                        // Get orient point using adjacent vertex
                        int orientPtIndex5 = f20k300.TopologyVertices.ConnectedTopologyVertices(i)[0];
                        Point3d orientX5 = (Point3d)f20k300.TopologyVertices[orientPtIndex5];

                        // First create a plane using center and normal vector
                        Vector3d normal5 = f20k300.Normals[i];
                        Plane fiveFoldPlaneUnoriented = new Plane(planeCenter5, normal5);

                        // UPDATED CODE
                        double fiveFoldPlaneAngle = GetSignedVectorAngle(fiveFoldPlaneUnoriented.XAxis, orientX5 - planeCenter5, fiveFoldPlaneUnoriented);
                        fiveFoldPlaneUnoriented.Rotate(fiveFoldPlaneAngle, fiveFoldPlaneUnoriented.Normal);

                        // Push the plane outwards by a multiple of the edge length
                        Vector3d pushPlane5 = new Vector3d(normal5);
                        pushPlane5.Unitize();
                        Transform xscale5 = Transform.Scale(Point3d.Origin, edgeLengthRef);
                        pushPlane5.Transform(xscale5);
                        fiveFoldPlaneUnoriented.Translate(pushPlane5 * 2);

                        // Transform 6 F20 to these 5-fold rotational axes
                        Transform xformFive = Transform.PlaneToPlane(Plane.WorldXY, fiveFoldPlaneUnoriented);
                        Mesh copyf = refF20.DuplicateMesh();
                        copyf.Transform(xformFive);

                        // Add to mesh list
                        listF20.Add(copyf);

                        // Get base pts - these should be on the outside - get furthest vertex
                        int furthestVertexIndex = GetFurthestVertex(copyf, (Point3d)planeCenter5);
                        Point3d furthestVertexPt = (Point3d)copyf.TopologyVertices[furthestVertexIndex];
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
                        adjacentVertexIndices = copyf.Vertices.GetConnectedVertices(furthestVertexIndex);
                        boundaryedgept = (Point3d)copyf.Vertices[adjacentVertexIndices[0]];
                        xrot5 = Transform.Rotation(2 * Math.PI / 5, normalf20a6h, baseptf);
                        edgept2 = boundaryedgept;
                        edgept2.Transform(xrot5);

                        // Get plane and transform
                        Plane planef20a6h = new Plane(furthestVertexPt, boundaryedgept, edgept2);
                        Transform xformf20a6h = Transform.PlaneToPlane(a6base, planef20a6h);

                        // Add the first A6 tile
                        Mesh f20a6h = refA6.DuplicateMesh();
                        f20a6h.Transform(xformf20a6h);

                        // Add to mesh list
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
                        int baseptf20f20hIndex = GetClosestVertex(f20a6h, (Point3d)baseptf20f20h);
                        Point3d f20f20hxpt = boundaryedgept;

                        // Also prepare to add F20 tile here
                        Mesh f20f20h = refF20.DuplicateMesh();
                        Vector3d f20f20hnormal = boundaryedgept - baseptf;
                        Plane planef20f20h = new Plane(baseptf20f20h, f20f20hnormal);
                        Vector3d f20f20hxaxis = f20f20hxpt - baseptf20f20h;
                        double anglef20f20h = GetSignedVectorAngle(planef20f20h.XAxis, f20f20hxaxis, planef20f20h);
                        planef20f20h.Rotate(anglef20f20h + Math.PI, f20f20hnormal, baseptf20f20h);

                        // Copy and transform the F20 mesh
                        Transform xformf20f20h = Transform.PlaneToPlane(Plane.WorldXY, planef20f20h);
                        f20f20h.Transform(xformf20f20h);

                        // Check if F20 basept is inside inflated F20 shape before adding
                        if (Vector3d.Multiply(f20f20hnormal, normalf20) > -5.0)
                        {
                            // Add to lists
                            listF20.Add(f20f20h);
                            ptsF20.Add(baseptf20f20h);
                            plnsF20.Add(planef20f20h);
                        }

                        Mesh f20a6hcopy = f20a6h.DuplicateMesh();
                        Point3d baseptf20a6hcopy = baseptf20a6h;
                        Mesh f20f20hcopy = f20f20h.DuplicateMesh();
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

                            f20a6hcopy = f20a6hcopy.DuplicateMesh();
                            f20f20hcopy = f20f20hcopy.DuplicateMesh();
                        }

                        // A6 array of 5 on the inner side the F20
                        // Get closest vertex
                        int closestVertexIndex = GetClosestVertex(copyf, (Point3d)planeCenter5);
                        Point3d closestVertexPt = (Point3d)copyf.TopologyVertices[closestVertexIndex];

                        // Get one adjacent vertex
                        adjacentVertexIndices = copyf.Vertices.GetConnectedVertices(closestVertexIndex);
                        boundaryedgept = (Point3d)copyf.Vertices[adjacentVertexIndices[0]];

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

                        // Add to output
                        listA6.Add(copyh0);
                        listA6.Add(copyh1);
                        listA6.Add(copyh2);
                        listA6.Add(copyh3);
                        listA6.Add(copyh4);

                        // Simultaneously find A6 for next step
                        Mesh f20a62 = copyh0;
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
                            f20a62.FaceNormals.ComputeFaceNormals();
                            int closeFaceIndex = GetClosestFace(f20a62, f20boundarybase);
                            Vector3d f20a62facenormal = f20a62.FaceNormals[closeFaceIndex];
                            Point3d f20a62facecenter = f20a62.Faces.GetFaceCenter(closeFaceIndex);
                            Plane planef20a63 = new Plane(f20a62facecenter, f20a62facenormal);
                            Transform xmirrorf20a63 = Transform.Mirror(planef20a63);
                            Mesh f20a63 = f20a62.DuplicateMesh();
                            f20a63.Transform(xmirrorf20a63);

                            // Add to mesh list
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
                            Point3d f20a63facecenter = f20a63.Faces.GetFaceCenter(farFaceIndex);
                            Plane planef20a64 = new Plane(f20a63facecenter, f20a62facenormal);
                            Transform xmirrorf20a64 = Transform.Mirror(planef20a64);
                            Mesh f20a64 = f20a63.DuplicateMesh();
                            f20a64.Transform(xmirrorf20a64);

                            // Add to mesh list
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
                            int baseVertexIndex = GetFurthestVertex(f20a64, (Point3d)baseptf20a64);
                            int[] adjacentFacesIndices = f20a64.Vertices.GetVertexFaces(baseVertexIndex);
                            f20a64.FaceNormals.ComputeFaceNormals();
                            for (int j = 0; j < 3; j++)
                            {
                                int faceIndex = adjacentFacesIndices[j];
                                Point3d f20a64facecenter = f20a64.Faces.GetFaceCenter(faceIndex);
                                if (planef20a64.DistanceTo(f20a64facecenter) > 0.00001) // tolerance issue - causing one additional A6 to be created if only using > 0
                                {
                                    Vector3d f20a64facenormal = f20a64.FaceNormals[faceIndex];
                                    Plane planef20a65 = new Plane(f20a64facecenter, f20a64facenormal);
                                    Transform xmirrorf20a65 = Transform.Mirror(planef20a65);
                                    Mesh f20a65 = f20a64.DuplicateMesh();
                                    f20a65.Transform(xmirrorf20a65);

                                    // Add to mesh list
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
            double zTranslation = -f20HalfHeight * deflationScaleFactor;
            Vector3d zOffset = new Vector3d(0, 0, zTranslation);
            TranslatePlanesInDirection(plnsA6, zOffset);
            TranslatePlanesInDirection(plnsB12, zOffset);
            TranslatePlanesInDirection(plnsF20, zOffset);
            TranslatePlanesInDirection(plnsK30, zOffset);

            // Output plns
            deflationRulesF20.AddRange(plnsA6,new GH_Path(0));
            deflationRulesF20.AddRange(plnsB12, new GH_Path(1));
            deflationRulesF20.AddRange(plnsF20, new GH_Path(2));
            deflationRulesF20.AddRange(plnsK30, new GH_Path(3));

            return deflationRulesF20;
        }

        private static DataTree<Plane> GenerateDeflationPlanesK30(Mesh refA6, Mesh refB12, Mesh refF20, Mesh refK30)
        {
            // Set up DataTree
            DataTree<Plane> deflationRulesK30 = new DataTree<Plane>();

            // Get height references from original input meshes
            double a6HeightRef = GetMeshHeight(refA6);
            double b12HeightRef = GetMeshHeight(refB12);
            double f20HeightRef = GetMeshHeight(refF20);
            double k30HeightRef = GetMeshHeight(refK30);

            // Get length reference (edge length of original triacontahedron)
            //double edgeLengthRef = refK30.Edges[0].PointAtEnd.DistanceTo(refK30.Edges[0].PointAtStart); //brep code
            double edgeLengthRef = refK30.TopologyEdges.EdgeLine(0).Length;

            Mesh mesh = refK30.DuplicateMesh();
            Point3d basept = Point3d.Origin;
            Plane basepln = Plane.WorldXY;

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
            Point3d k30basecenter = mesh.TopologyVertices[GetClosestVertex(mesh, new Point3d(0, -1, 0))];
            Point3d k30basexaxis = mesh.TopologyVertices[GetClosestVertex(mesh, new Point3d(1, 0, 0))];
            Point3d k30baseyaxis = new Point3d(-k30basexaxis.X, 0, 0);
            Plane k30base = new Plane(k30basecenter, k30basexaxis, k30baseyaxis);

            // Set up base orientation for B12 transformation in step (a6-000)
            Point3d b12basecenter = refB12.TopologyVertices[GetClosestVertex(refB12, new Point3d(-0.5, 0, 0))];
            Point3d b12baseyaxis = refB12.TopologyVertices[GetClosestVertex(refB12, new Point3d(-0.5, -0.5, 1))];
            Point3d b12basexaxis = new Point3d(b12baseyaxis.X, -b12baseyaxis.Y, b12baseyaxis.Z);
            Plane b12base = new Plane(b12basecenter, b12basexaxis, b12baseyaxis);

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

            deflationRulesK30.AddRange(plnsA6, new GH_Path(0));
            deflationRulesK30.AddRange(plnsB12, new GH_Path(1));
            deflationRulesK30.AddRange(plnsF20, new GH_Path(2));
            deflationRulesK30.AddRange(plnsK30, new GH_Path(3));

            return deflationRulesK30;
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

        // Note that this only works for placement of B12 and K30, not A6 or F20, since those are oriented based on the face center
        // X-axis is aligned with closer of the two vertices
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

        // Helper for placement of A6
        public static Plane GetOrientedPlaneFromRhombicFaceAcute(Mesh mesh, int faceIndex, Vector3d normalRef)
        {
            // Get center and normal vector of the current face
            Point3d centerFace = mesh.Faces.GetFaceCenter(faceIndex);

            // Get face vertex indices and convert to Point3d for better precision
            MeshFace face = mesh.Faces[faceIndex];
            Point3d a = new Point3d(mesh.Vertices[face.A]);
            Point3d b = new Point3d(mesh.Vertices[face.B]);
            Point3d c = new Point3d(mesh.Vertices[face.C]);

            // Get oriented plane
            Point3d originPt;
            Point3d xPt;
            Point3d yPt;

            // Set origin to be the point furthest from the center, xPt to be the closer of the two remaining points, and yPt to be the last point
            if (centerFace.DistanceTo(a) > centerFace.DistanceTo(b))
            {
                originPt = a;
                xPt = b;
                yPt = c;
            }
            else
            {
                originPt = b;
                xPt = c;
                yPt = a;
            }

            Plane plane = new Plane(originPt, xPt, yPt);
            // If dot product is negative we need to flip/rotate to point outwards
            // if (Vector3d.Multiply(plane.Normal, normalRef) < 0)
            // {
            //     plane.Flip();
            //     plane.Rotate(Math.PI / 2, plane.Normal);
            // }
            return plane;
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

        public static bool CheckIfA6BaseVectorIsAcute(Mesh a6tocheck, Point3d basept, Vector3d vec)
        {
            int farVertexIndex = GetFurthestVertex(a6tocheck, (Point3d)basept);
            Point3d farPt = (Point3d)a6tocheck.TopologyVertices[farVertexIndex];
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

        public static Brep GenerateBrepA6(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(3, false);
            Brep brepA6 = GenerateZonohedronBrepFromStarVectors(starVectors, scale);
            brepA6.Rotate(Math.PI / 2, Vector3d.ZAxis, Point3d.Origin);

            // Golden ratio
            double phi = (1 + Math.Sqrt(5)) / 2;
            // Rotate according to angle between long diagonal and face, so that long diagonal axis aligns with Z axis
            // Note: This rotation potentially introduces inaccuracies - maybe cleaner to generate the zonohedron already at this angle
            brepA6.Rotate(-Math.Acos(phi / Math.Sqrt(3)) - (Math.PI / 2), Vector3d.YAxis, Point3d.Origin);
            return brepA6;
        }

        public static Brep GenerateBrepB12(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(4, false);
            return GenerateZonohedronBrepFromStarVectors(starVectors, scale);
        }

        public static Brep GenerateBrepF20(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(5, false);
            Brep brepF20 = GenerateZonohedronBrepFromStarVectors(starVectors, scale);
            brepF20.Rotate(Math.PI, Vector3d.ZAxis, Point3d.Origin);
            brepF20.Rotate(Math.Asin(Math.Sqrt((5 + Math.Sqrt(5)) / 10)), Vector3d.YAxis, Point3d.Origin);
            return brepF20;
        }

        public static Brep GenerateBrepK30(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(6, false);
            return GenerateZonohedronBrepFromStarVectors(starVectors, scale);
        }

        public static Brep GenerateZonohedronBrepFromStarVectors(List<Vector3d> starVectors, double scale)
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

            // Brep face generation
            List<Surface> faces = new List<Surface>();

            foreach (Vector3d n in normalVectors)
            {
                // Set up face p-representation and its opposite
                List<int> face_p_representation = new List<int>();

                // Loop through star vectors
                foreach (Vector3d v in scaledStarVectors)
                {
                    // Get dot product
                    double d = Vector3d.Multiply(n, v);
                    // Create p-representation entries
                    if (Math.Abs(d) < 0.0001)
                    {
                        face_p_representation.Add(0);
                    }
                    else if (d > 0)
                    {
                        face_p_representation.Add(1);
                    }
                    else
                    {
                        face_p_representation.Add(-1);
                    }
                }

                // Create vertex p-represetnations
                List<int> v1_p_representation = new List<int>();
                List<int> v2_p_representation = new List<int>();
                List<int> v3_p_representation = new List<int>();
                List<int> v4_p_representation = new List<int>();

                bool firstZeroFound = false;
                // Loop through face_p_representation to create vertices
                for (int i = 0; i < face_p_representation.Count; i++)
                {
                    if (!firstZeroFound && face_p_representation[i] == 0)
                    {
                        firstZeroFound = true;
                        v1_p_representation.Add(1);
                        v2_p_representation.Add(1);
                        v3_p_representation.Add(-1);
                        v4_p_representation.Add(-1);
                    }
                    else if (firstZeroFound && face_p_representation[i] == 0)
                    {
                        v1_p_representation.Add(1);
                        v2_p_representation.Add(-1);
                        v3_p_representation.Add(-1);
                        v4_p_representation.Add(1);
                    }
                    else
                    {
                        v1_p_representation.Add(face_p_representation[i]);
                        v2_p_representation.Add(face_p_representation[i]);
                        v3_p_representation.Add(face_p_representation[i]);
                        v4_p_representation.Add(face_p_representation[i]);
                    }
                }

                // Compute vertex positions from p-representations
                Point3d v1 = new Point3d(0, 0, 0);
                Point3d v2 = new Point3d(0, 0, 0);
                Point3d v3 = new Point3d(0, 0, 0);
                Point3d v4 = new Point3d(0, 0, 0);
                for (int i = 0; i < scaledStarVectors.Count; i++)
                {
                    v1 += scaledStarVectors[i] * v1_p_representation[i];
                    v2 += scaledStarVectors[i] * v2_p_representation[i];
                    v3 += scaledStarVectors[i] * v3_p_representation[i];
                    v4 += scaledStarVectors[i] * v4_p_representation[i];
                }

                // Create face from vertices (v1, v2, v3, v4)
                Surface face = NurbsSurface.CreateFromCorners(v1, v2, v3, v4);
                Surface faceOpposite = NurbsSurface.CreateFromCorners(-v1, -v2, -v3, -v4);
                faces.Add((Surface)face.Duplicate());
                faces.Add((Surface)faceOpposite.Duplicate());
            }

            // Assemble face surfaces into a Brep
            Brep brep = Brep.JoinBreps(faces.ConvertAll(f => Brep.CreateFromSurface(f)), 0.01)[0];
            return brep;
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