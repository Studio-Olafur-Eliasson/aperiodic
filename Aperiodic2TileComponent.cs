using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Aperiodic
{
    public class Aperiodic2TileComponent : GH_Component
    {
        // Cache commonly used constants
        private static readonly double GoldenRatio = (1 + Math.Sqrt(5)) / 2;

        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public Aperiodic2TileComponent()
          : base("Aperiodic 2-Tile", "2-Tile",
            "Generate aperiodic 2-tile transformations (v1.0)",
            "Aperiodic", "Aperiodic")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGeometryParameter("Geometry Filter", "geometryFilter", "(Optional) Input a geometry filter (Brep or Curve) to define the output shape of the tiling. This acts as a secondary filtering operation after 4-tile component filtering.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Filter Distance", "filterDistance", "Distance from the geometryFilter within which tiles should be included in the output.", GH_ParamAccess.item, 1.0);
            pManager.AddBooleanParameter("IncludeInterior", "includeInterior", "Boolean for whether to include tiles on the interior of the filter geometry (if it is a closed Brep). Default false. Note: interior may already be filtered out from the 4-tile component.", GH_ParamAccess.item, false);
            pManager.AddPlaneParameter("Transformations", "X", "(Required) The output transformations generated from the Aperiodic 4-Tile component. The tree structure contains a separate branch for each of the four tile types: {0} = rhombohedron; {1} = rhombic (Bilinski) dodecahedron; {2} = rhombic icosahedron; {3} = rhombic triacontahedron.", GH_ParamAccess.tree);
            pManager[0].Optional = true;
            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Base Meshes", "baseMeshes", "Set of base mesh geometry of the two tiles. Apply the input transformations to view tiling result.", GH_ParamAccess.tree);
            pManager.AddBrepParameter("Base Breps", "baseBreps", "Set of base brep geometry of the two tiles. Apply the input transformations to view tiling result.", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("Tranformations", "X", "Apply these output transformations to the baseMeshes, baseBreps, or other substitute geometry. The tree structure contains a separate branch for each of the two tile types: {0} = oblate rhombohedron aka flat tile; {1} = prolate rhombohedron aka long tile", GH_ParamAccess.tree);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Declare variables for the input
            GeometryBase geometryFilter = null;
            double filterDistance = 1.0;
            bool includeInterior = false;
            double scale = 1.0;
            GH_Structure<GH_Plane> transformations = new GH_Structure<GH_Plane>();

            // Retrieve data from input parameters
            DA.GetData(0, ref geometryFilter);
            DA.GetData(1, ref filterDistance);
            DA.GetData(2, ref includeInterior);
            DA.GetDataTree(3, out transformations);

            if (transformations.IsEmpty)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Missing transformation (X) input. Connect a valid transformation tree, generated from the Aperiodic 4-Tile Component.");
            }

            // Get decomposition planes
            Plane[][] decompositionPlanesA6 = GetA6DecompositionPlanes(scale);
            Plane[][] decompositionPlanesB12 = GetB12DecompositionPlanes(scale);
            //Plane[][] decompositionPlanesF20 = GetF20DecompositionPlanes(scale);
            //Plane[][] decompositionPlanesK30 = GetK30DecompositionPlanes(scale);

            // Generate data tree for output
            DataTree<Plane> outputTransformations = new DataTree<Plane>();   
            GH_Path pth0 = new GH_Path(0);
            GH_Path pth1 = new GH_Path(1);
            outputTransformations.EnsurePath(pth0);
            outputTransformations.EnsurePath(pth1); 

            // Work with 2D Array of planes
            Plane[][] inputTransformationPlanes = ExtractPlaneArrays(transformations);

            // Decompose A6 tiles and add to outputTransformations
            foreach (Plane pl in inputTransformationPlanes[0])
            {
                foreach (Plane decompO6 in decompositionPlanesA6[0])
                {
                    Plane transformedO6 = decompO6;
                    transformedO6.Transform(Transform.PlaneToPlane(Plane.WorldXY, pl));
                    if (CheckGeometryFilter(geometryFilter, transformedO6, filterDistance, includeInterior))
                    {
                        outputTransformations.Add(transformedO6, pth0);
                    }
                    
                }
                foreach (Plane decompA6 in decompositionPlanesA6[1])
                {
                    Plane transformedA6 = decompA6;
                    transformedA6.Transform(Transform.PlaneToPlane(Plane.WorldXY, pl));
                    if (CheckGeometryFilter(geometryFilter, transformedA6, filterDistance, includeInterior))
                    {
                        outputTransformations.Add(transformedA6, pth1);
                    }
                }
            }

            // Decompose B12 tiles and add to outputTransformations
            foreach (Plane pl in inputTransformationPlanes[1])
            {
                foreach (Plane decompO6 in decompositionPlanesB12[0])
                {
                    Plane transformedO6 = decompO6;
                    transformedO6.Transform(Transform.PlaneToPlane(Plane.WorldXY, pl));
                    if (CheckGeometryFilter(geometryFilter, transformedO6, filterDistance, includeInterior))
                    {
                        outputTransformations.Add(transformedO6, pth0);
                    }
                }
                foreach (Plane decompA6 in decompositionPlanesB12[1])
                {
                    Plane transformedA6 = decompA6;
                    transformedA6.Transform(Transform.PlaneToPlane(Plane.WorldXY, pl));
                    if (CheckGeometryFilter(geometryFilter, transformedA6, filterDistance, includeInterior))
                    {
                        outputTransformations.Add(transformedA6, pth1);
                    }
                }
            }

            // Set Up for base mesh and brep generation for 2-tile system
            DataTree<Mesh> baseMeshes = new DataTree<Mesh>();
            DataTree<Brep> baseBreps = new DataTree<Brep>();

            // Generate Base Meshes
            Mesh meshO6 = GenerateMeshO6(scale);
            Mesh meshA6 = GenerateMeshA6(scale);
            baseMeshes.Add(meshO6, new GH_Path(0));
            baseMeshes.Add(meshA6, new GH_Path(1));

            // Generate Base Breps
            Brep brepO6 = GenerateBrepO6(scale);
            Brep brepA6 = GenerateBrepA6(scale);
            baseBreps.Add(brepO6, new GH_Path(0));
            baseBreps.Add(brepA6, new GH_Path(1));

            DA.SetDataTree(0, baseMeshes);
            DA.SetDataTree(1, baseBreps);
            DA.SetDataTree(2, outputTransformations);
        }

        

        public static bool CheckGeometryFilter(GeometryBase geometryFilter, Plane tilePlane, double filterDistance, bool includeInterior)
        {
            if (geometryFilter == null) return true; // No filter, so all tiles are valid

            // Create a point on the tile plane to check against the filter
            Point3d tilePoint = tilePlane.Origin;

            // If filter is a Brep
            if (geometryFilter.HasBrepForm)
            {
                Brep brepFilter = Brep.TryConvertBrep(geometryFilter);
                if (includeInterior && brepFilter.IsSolid)
                {
                    if (brepFilter.IsPointInside(tilePoint, 0.01, true))
                    {
                        return true; // Tile is inside the filter
                    }
                }
                // For non-solid Breps or if not including interior, or for remaining tiles
                // Check if tile is within filter distance of the Brep surface
                double closestDist = brepFilter.ClosestPoint(tilePoint).DistanceTo(tilePoint);
                if (closestDist <= filterDistance)
                {
                    return true; // Tile is within distance of the filter
                }
                else return false; // Tile is too far from the filter
            }
            // If filter is a Curve
            else if (geometryFilter is Curve crvFilter)
            {
                // For curves, we can check if the tile point is within a certain distance
                double t;
                if (crvFilter.ClosestPoint(tilePoint, out t, filterDistance)) return true;
                else return false;
            }
            else return true; // TODO: Implement filter for other geometry types (mesh, point, etc)
        }

        #region ---Decomposition Plane Generation---

        public static Plane[][] GetA6DecompositionPlanes(double scale)
        {
            Plane[][] planes = new Plane[2][];
            planes[0] = new Plane[0];  // No O6 tiles in A6 decomposition
            planes[1] = new Plane[1];  // One A6 tile
            planes[1][0] = Plane.WorldXY;
            return planes;
        }

        public static Plane[][] GetB12DecompositionPlanes(double scale)
        {
            // Set Up jagged array - must initialize sub-arrays
            Plane[][] planes = new Plane[2][];
            planes[0] = new Plane[2];  // 2 O6 tiles
            planes[1] = new Plane[2];  // 2 A6 tiles

            Mesh refB12 = GenerateMeshB12(scale);
            BoundingBox bbox = refB12.GetBoundingBox(true);
            double Xdist = bbox.Max.X;
            double Ydist = bbox.Max.Y;
            double Zdist = bbox.Max.Z;

            // Get O6 Planes using 3 points, then mirroring across WorldYZ
            int minYpointIndex = GetClosestVertex(refB12, new Point3d(0, -Ydist, 0));
            Point3d minYpoint = refB12.TopologyVertices[minYpointIndex];
            int topPointIndex = GetClosestVertex(refB12, new Point3d(Xdist, Ydist*0.5, 0));
            Point3d topPoint = refB12.TopologyVertices[topPointIndex];
            int xPointIndex = GetClosestVertex(refB12, new Point3d(Xdist, -Ydist * 0.5, 0));
            Point3d xPoint = refB12.TopologyVertices[xPointIndex];
            Plane plane0601 = PlaneFromRhombohedronCoordinates(minYpoint, topPoint, xPoint);
            planes[0][0] = plane0601;
            plane0601.Transform(Transform.Mirror(Plane.WorldYZ));
            plane0601.Rotate(Math.PI, plane0601.Normal); // Rotate to match proper orientation
            planes[0][1] = plane0601;

            // Get A6 Planes using 3 points, then mirroring across WorldXY
            int maxYpointIndex = GetClosestVertex(refB12, new Point3d(0, Ydist, 0));
            Point3d maxYpoint = refB12.TopologyVertices[maxYpointIndex];
            int bottomPointIndex = GetClosestVertex(refB12, new Point3d(0, -Ydist, -Zdist));
            Point3d bottomPoint = refB12.TopologyVertices[bottomPointIndex];
            xPointIndex = GetClosestVertex(refB12, new Point3d(Xdist, 0, -Zdist));
            xPoint = refB12.TopologyVertices[xPointIndex];
            Plane planeA601 = PlaneFromRhombohedronCoordinates(bottomPoint, maxYpoint, xPoint);
            planes[1][0] = planeA601;
            planeA601.Transform(Transform.Mirror(Plane.WorldXY));
            planeA601.Rotate(Math.PI, planeA601.Normal); // Rotate to match proper orientation
            planes[1][1] = planeA601;

            return planes;
        }

        public static DataTree<Plane> GetF20DecompositionPlanes(double scale)
        {
            DataTree<Plane> planes = new DataTree<Plane>();
            return planes;
        }

        public static DataTree<Plane> GetK30DecompositionPlanes(double scale)
        {
            DataTree<Plane> planes = new DataTree<Plane>();
            return planes;
        }

        public static Plane PlaneFromRhombohedronCoordinates(Point3d bottom, Point3d top, Point3d xPoint)
        {
            Point3d origin = (bottom + top) * 0.5;
            Vector3d normal = top - bottom;
            Vector3d xAxis = xPoint - origin;
            return PlaneFromNormalAndXAxis(origin, normal, xAxis);
        }

        public static Plane PlaneFromNormalAndXAxis(Point3d origin, Vector3d normal, Vector3d xAxis)
        {
            normal.Unitize();

            // Compute Y-axis as cross product of normal (Z) and X
            Vector3d yAxis = Vector3d.CrossProduct(normal, xAxis);
            yAxis.Unitize();

            // Recompute X to ensure orthogonality
            xAxis = Vector3d.CrossProduct(yAxis, normal);
            xAxis.Unitize();

            return new Plane(origin, xAxis, yAxis);
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

        #endregion

        #region ---Zonohedra Generation---

        public static Mesh GenerateMeshO6(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(3, true);
            Mesh meshO6 = GenerateZonohedronMeshFromStarVectors(starVectors, scale);

            // Golden ratio
            double phi = (1 + Math.Sqrt(5)) / 2;

            // Rotate to orient according to base position in previous GH logic for transforming the tile
            // Note: This rotation potentially introduces inaccuracies - maybe cleaner to generate the zonohedron already at this angle
            meshO6.Rotate((Math.PI / 2) - Math.Asin(phi / Math.Sqrt(3)), Vector3d.ZAxis, Point3d.Origin);
            meshO6.Rotate(-Math.PI / 6, Vector3d.XAxis, Point3d.Origin);

            // Get additional rotation angle
            double theta = Math.Acos(Math.Sqrt((5 - (2 * Math.Sqrt(5))) / 15));
            meshO6.Rotate(((Math.PI / 2) - theta) / 2, Vector3d.YAxis, Point3d.Origin);
            return meshO6;
        }

        public static Brep GenerateBrepO6(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(3, true);
            Brep brepO6 = GenerateZonohedronBrepFromStarVectors(starVectors, scale);

            // Golden ratio
            double phi = (1 + Math.Sqrt(5)) / 2;

            // Rotate to orient according to base position in previous GH logic for transforming the tile
            // Note: This rotation potentially introduces inaccuracies - maybe cleaner to generate the zonohedron already at this angle
            brepO6.Rotate((Math.PI / 2) - Math.Asin(phi / Math.Sqrt(3)), Vector3d.ZAxis, Point3d.Origin);
            brepO6.Rotate(-Math.PI / 6, Vector3d.XAxis, Point3d.Origin);

            // Get additional rotation angle
            double theta = Math.Acos(Math.Sqrt((5 - (2 * Math.Sqrt(5))) / 15));
            brepO6.Rotate(((Math.PI / 2) - theta) / 2, Vector3d.YAxis, Point3d.Origin);
            return brepO6;
        }

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

        public static Mesh GenerateMeshB12(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(4, false);
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
            double phi = GoldenRatio;

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

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => Resources.aperiodic2tile24px;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("A1B2C3D4-E5F6-7890-ABCD-EF1234567890");
    }
}
