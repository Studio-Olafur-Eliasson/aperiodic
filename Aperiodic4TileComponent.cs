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
            pManager.AddPlaneParameter("base_plns", "base", "Plane input to begin the recursive process, based on seed options.", GH_ParamAccess.tree);
            pManager.AddIntegerParameter("iterations", "i", "Number of iterations of the recursive process. If iterations > 2, must use geometryFilter to avoid crashing. Set iterations = 0 to view the starting \"seed\" tiles of the recusive process. Default: 1", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("scale", "scale", "Scale factor (edge length) of the tiles. Default: 1.0 (no scaling)", GH_ParamAccess.item, 1.0);
            pManager.AddPlaneParameter("deflationA6plns", "a6plns", "Deflation planes making up the A6 deflation rule.", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("deflationB12plns", "b12plns", "Deflation planes making up the B12 deflation rule.", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("deflationF20plns", "f20plns", "Deflation planes making up the F20 deflation rule.", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("deflationK30plns", "k30plns", "Deflation planes making up the K30 deflation rule.", GH_ParamAccess.tree);
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
            GH_Structure<GH_Plane> baseplns = new GH_Structure<GH_Plane>();
            int iterations = 1;
            double scale = 1.0;
            GH_Structure<GH_Plane> deflationA6plns = new GH_Structure<GH_Plane>();
            GH_Structure<GH_Plane> deflationB12plns = new GH_Structure<GH_Plane>();
            GH_Structure<GH_Plane> deflationF20plns = new GH_Structure<GH_Plane>();
            GH_Structure<GH_Plane> deflationK30plns = new GH_Structure<GH_Plane>();

            // Retrieve data from input parameters
            if (!DA.GetData(0, ref geometryFilter)) { return; }
            if (!DA.GetData(1, ref filterDistance)) { return; }
            if (!DA.GetData(2, ref includeInterior)) { return; }
            if (!DA.GetData(3, ref centerpln)) { return; }
            if (!DA.GetDataTree<GH_Plane>(4, out baseplns)) { return; }
            if (!DA.GetData(5, ref iterations)) { return; }
            if (!DA.GetData(6, ref scale)) { return; }
            if (!DA.GetDataTree<GH_Plane>(7, out deflationA6plns)) { return; }
            if (!DA.GetDataTree<GH_Plane>(8, out deflationB12plns)) { return; }
            if (!DA.GetDataTree<GH_Plane>(9, out deflationF20plns)) { return; }
            if (!DA.GetDataTree<GH_Plane>(10, out deflationK30plns)) { return; }

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
            Mesh meshA6 = GenerateMeshA6(scale);
            Mesh meshB12 = GenerateMeshB12(scale);
            Mesh meshF20 = GenerateMeshF20(scale);
            Mesh meshK30 = GenerateMeshK30(scale);

            // Pre-extract deflation planes to native Plane arrays for faster access
            // deflationRules[tileType] = Plane[branchIndex][planeIndex]
            Plane[][][] deflationRules = new Plane[4][][];
            deflationRules[0] = ExtractPlaneArrays(deflationA6plns);
            deflationRules[1] = ExtractPlaneArrays(deflationB12plns);
            deflationRules[2] = ExtractPlaneArrays(deflationF20plns);
            deflationRules[3] = ExtractPlaneArrays(deflationK30plns);

            GH_Structure<GH_Plane> outputplns = RecurseInflateGeometry(gfa, filterDistance, includeInterior, centerpln, baseplns, iterations, scale, deflationRules);

            DA.SetDataTree(1, outputplns);
        }

        // Extract planes from GH_Structure to native arrays for faster iteration
        // Returns Plane[branchIndex][planeIndex]
        private static Plane[][] ExtractPlaneArrays(GH_Structure<GH_Plane> ghStructure)
        {
            int branchCount = Math.Min(ghStructure.Branches.Count, 4);
            var result = new Plane[4][];
            
            for (int i = 0; i < 4; i++)
            {
                if (i < branchCount)
                {
                    var branch = ghStructure.Branches[i];
                    result[i] = new Plane[branch.Count];
                    for (int j = 0; j < branch.Count; j++)
                    {
                        result[i][j] = branch[j].Value;
                    }
                }
                else
                {
                    result[i] = new Plane[0];
                }
            }
            return result;
        }

        public static GH_Structure<GH_Plane> RecurseInflateGeometry(
            List<GeometryBase> geometryFilterArray, 
            double filterDistance, 
            bool includeInterior, 
            Plane centerpln, 
            GH_Structure<GH_Plane> baseplns, 
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
            var scaledPlanes = new List<GH_Plane>[4];
            for (int i = 0; i < 4; i++)
            {
                var planes = baseplns.Branches[i];
                scaledPlanes[i] = new List<GH_Plane>(planes.Count);
                for (int j = 0; j < planes.Count; j++)
                {
                    Plane scaledPlane = planes[j].Value;
                    scaledPlane.Transform(scaleInflate);
                    scaledPlanes[i].Add(new GH_Plane(scaledPlane));
                }
            }

            // Rebuild baseplns efficiently
            baseplns = new GH_Structure<GH_Plane>();
            for (int i = 0; i < 4; i++)
            {
                GH_Path pth = new GH_Path(i);
                baseplns.AppendRange(scaledPlanes[i], pth);
            }

            // Perform deflation
            GH_Structure<GH_Plane> inflatedbaseplns = InflateGeometryOptimized(baseplns, deflationRules);

            // Filter
            GH_Structure<GH_Plane> filteredbaseplns;
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
            GH_Structure<GH_Plane> culledbaseplns = CullDuplicatePlanesOptimized(filteredbaseplns, 0.1 * scale);

            return RecurseInflateGeometry(geometryFilterArray, filterDistance, includeInterior, centerpln, culledbaseplns, iterations - 1, scale, deflationRules);
        }

        // Optimized InflateGeometry using batch operations (sequential to maintain deterministic order)
        public static GH_Structure<GH_Plane> InflateGeometryOptimized(
            GH_Structure<GH_Plane> baseplns, 
            Plane[][][] deflationRules)
        {
            // Use lists for deterministic ordering (matching original behavior)
            var resultLists = new List<GH_Plane>[4];
            for (int i = 0; i < 4; i++)
            {
                resultLists[i] = new List<GH_Plane>();
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

                    Transform xcopyPlane = Transform.PlaneToPlane(Plane.WorldXY, planes[j].Value);

                    // Copy all planes from each branch
                    for (int branchIdx = 0; branchIdx < 4; branchIdx++)
                    {
                        var sourcePlanes = planesToCopy[branchIdx];
                        for (int p = 0; p < sourcePlanes.Length; p++)
                        {
                            Plane transformedPlane = sourcePlanes[p];
                            transformedPlane.Transform(xcopyPlane);
                            resultLists[branchIdx].Add(new GH_Plane(transformedPlane));
                        }
                    }
                }
            }

            // Build result structure using AppendRange (much faster than individual Append)
            GH_Structure<GH_Plane> result = new GH_Structure<GH_Plane>();
            for (int i = 0; i < 4; i++)
            {
                result.AppendRange(resultLists[i], new GH_Path(i));
            }

            return result;
        }

        public static GH_Structure<GH_Plane> BrepFilterPlanesOptimized(
            GH_Structure<GH_Plane> inflatedbaseplns, 
            Brep brepFilter, 
            double filterDistance, 
            bool includeInterior, 
            int iterations, 
            double buffer)
        {
            if (iterations == 1) buffer = 0;

            double filterDivisionFactor = Math.Pow(InverseDeflationScaleFactor, iterations - 1);
            double maxDistance = filterDistance * filterDivisionFactor + (buffer * 1.5);

            var resultLists = new List<GH_Plane>[4];
            
            for (int i = 0; i < 4; i++)
            {
                var planes = inflatedbaseplns.Branches[i];
                resultLists[i] = new List<GH_Plane>(planes.Count);
                
                // Sequential processing to maintain deterministic order
                for (int j = 0; j < planes.Count; j++)
                {
                    Point3d testPoint = planes[j].Value.Origin;
                    
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

            GH_Structure<GH_Plane> result = new GH_Structure<GH_Plane>();
            for (int i = 0; i < 4; i++)
            {
                result.AppendRange(resultLists[i], new GH_Path(i));
            }
            return result;
        }

        public static GH_Structure<GH_Plane> CrvFilterPlanesOptimized(
            GH_Structure<GH_Plane> inflatedbaseplns, 
            Curve crvFilter, 
            double filterDistance, 
            int iterations, 
            double buffer)
        {
            if (iterations == 1) buffer = 0;

            double filterDivisionFactor = Math.Pow(InverseDeflationScaleFactor, iterations - 1);
            double maxDistance = filterDistance * filterDivisionFactor + (buffer * 1.5);

            var resultLists = new List<GH_Plane>[4];
            
            for (int i = 0; i < 4; i++)
            {
                var planes = inflatedbaseplns.Branches[i];
                resultLists[i] = new List<GH_Plane>(planes.Count);
                
                // Sequential processing to maintain deterministic order
                for (int j = 0; j < planes.Count; j++)
                {
                    Point3d testPoint = planes[j].Value.Origin;
                    double t;
                    if (crvFilter.ClosestPoint(testPoint, out t, maxDistance))
                    {
                        resultLists[i].Add(planes[j]);
                    }
                }
            }

            GH_Structure<GH_Plane> result = new GH_Structure<GH_Plane>();
            for (int i = 0; i < 4; i++)
            {
                result.AppendRange(resultLists[i], new GH_Path(i));
            }
            return result;
        }

        // Properly optimized duplicate culling with correct spatial hashing
        public static GH_Structure<GH_Plane> CullDuplicatePlanesOptimized(GH_Structure<GH_Plane> filteredbaseplns, double tolerance)
        {
            // Cell size should be at least tolerance to ensure all potential duplicates 
            // are in adjacent cells. Using tolerance * 1.0 means checking 27 neighbor cells
            // will cover all points within tolerance distance.
            double cellSize = tolerance;
            double toleranceSq = tolerance * tolerance; // Use squared distance to avoid sqrt
            
            var resultLists = new List<GH_Plane>[4];
            
            for (int i = 0; i < 4; i++)
            {
                var planes = filteredbaseplns.Branches[i];
                
                if (planes.Count == 0)
                {
                    resultLists[i] = new List<GH_Plane>();
                    continue;
                }

                // Dictionary mapping cell keys to list of points in that cell
                var spatialGrid = new Dictionary<long, List<Point3d>>();
                var uniquePlanes = new List<GH_Plane>(planes.Count);
                
                for (int j = 0; j < planes.Count; j++)
                {
                    Point3d testPoint = planes[j].Value.Origin;
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

            GH_Structure<GH_Plane> result = new GH_Structure<GH_Plane>();
            for (int i = 0; i < 4; i++)
            {
                result.AppendRange(resultLists[i], new GH_Path(i));
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

        #region Zonohedra Generation
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

        #endregion

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