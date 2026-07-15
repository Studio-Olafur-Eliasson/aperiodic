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
    public class Aperiodic2TileComponent : GH_Component
    {
        // Cache commonly used constants
        private static readonly double GoldenRatio = (1 + Math.Sqrt(5)) / 2;

        // Cache decomposition planes to avoid recalculating them each time the component runs
        private Plane[][] _cachedPlanesA6;
        private Plane[][] _cachedPlanesB12;
        private Plane[][] _cachedPlanesF20;
        private Plane[][] _cachedPlanesK30;

        // Cache base breps and meshes
        private DataTree<Brep> _cachedBaseBreps;
        private DataTree<Mesh> _cachedBaseMeshes;

        // Cache scale - if scale changes, we need to recalculate the planes
        private double _lastScale = double.NaN;

        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public Aperiodic2TileComponent()
          : base("Aperiodic 2-Tile", "2-Tile",
            "Generate aperiodic 2-tile transformations (v1.2.0)",
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
            pManager.AddBooleanParameter("Include Interior", "includeInterior", "Boolean for whether to include tiles on the interior of the filter geometry (if it is a closed Brep). Default true. Note: interior may already be filtered out from the 4-tile component.", GH_ParamAccess.item, true);
            pManager.AddPlaneParameter("Input Planes", "inPlanes", "(Required) The output planes generated from the Aperiodic 4-Tile component. The tree structure contains a separate branch for each of the four tile types: {0} = rhombohedron; {1} = rhombic (Bilinski) dodecahedron; {2} = rhombic icosahedron; {3} = rhombic triacontahedron.", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Scale", "scale", "Scale factor (edge length) of the tiles. Default: 1.0", GH_ParamAccess.item, 1.0);
            pManager[0].Optional = true;
            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[4].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Base Meshes", "baseMeshes", "Set of base mesh geometry of the two tiles. Apply the input transformations to view tiling result.", GH_ParamAccess.tree);
            pManager.AddBrepParameter("Base Breps", "baseBreps", "Set of base brep geometry of the two tiles. Apply the input transformations to view tiling result.", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("Output Planes", "outPlanes", "Apply these output planes as plane-to-plane transformations to the baseMeshes, baseBreps, or other substitute geometry. The tree structure contains a separate branch for each of the two tile types: {0} = oblate rhombohedron aka flat tile; {1} = prolate rhombohedron aka long tile", GH_ParamAccess.tree);
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
            GH_Structure<GH_Plane> inPlanes = new GH_Structure<GH_Plane>();

            // Retrieve data from input parameters
            DA.GetData(0, ref geometryFilter);
            DA.GetData(1, ref filterDistance);
            DA.GetData(2, ref includeInterior);
            DA.GetDataTree(3, out inPlanes);
            DA.GetData(4, ref scale);

            if (inPlanes.IsEmpty || inPlanes.DataCount == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Missing input planes (inPlanes). Connect a valid tree of planes generated from the Aperiodic 4-Tile Component (outPlanes).");
            }

            if (scale <= RhinoMath.ZeroTolerance)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                    "Cannot scale with factor zero.");
                return;
            }

            // Get decomposition planes
            if (_cachedPlanesA6 == null || scale != _lastScale)
            {
                _cachedPlanesA6 = GetA6DecompositionPlanes(scale);
                _cachedPlanesB12 = GetB12DecompositionPlanes(scale);
                _cachedPlanesF20 = GetF20DecompositionPlanes(scale);
                _cachedPlanesK30 = GetK30DecompositionPlanes(scale);
            }

            // Collect all transformed planes first, then filter in batch
            var allO6Planes = new List<Plane>();
            var allA6Planes = new List<Plane>();

            // Work with 2D Array of planes
            Plane[][] inputTransformationPlanes = ExtractPlaneArrays(inPlanes);

            // Decompose A6 tiles
            foreach (Plane pl in inputTransformationPlanes[0])
            {
                Transform xform = Transform.PlaneToPlane(Plane.WorldXY, pl);
                foreach (Plane decompO6 in _cachedPlanesA6[0])
                {
                    Plane transformedO6 = decompO6;
                    transformedO6.Transform(xform);
                    allO6Planes.Add(transformedO6);
                }
                foreach (Plane decompA6 in _cachedPlanesA6[1])
                {
                    Plane transformedA6 = decompA6;
                    transformedA6.Transform(xform);
                    allA6Planes.Add(transformedA6);
                }
            }

            // Decompose B12 tiles
            foreach (Plane pl in inputTransformationPlanes[1])
            {
                Transform xform = Transform.PlaneToPlane(Plane.WorldXY, pl);
                foreach (Plane decompO6 in _cachedPlanesB12[0])
                {
                    Plane transformedO6 = decompO6;
                    transformedO6.Transform(xform);
                    allO6Planes.Add(transformedO6);
                }
                foreach (Plane decompA6 in _cachedPlanesB12[1])
                {
                    Plane transformedA6 = decompA6;
                    transformedA6.Transform(xform);
                    allA6Planes.Add(transformedA6);
                }
            }

            // Decompose F20 tiles
            foreach (Plane pl in inputTransformationPlanes[2])
            {
                Transform xform = Transform.PlaneToPlane(Plane.WorldXY, pl);
                foreach (Plane decompO6 in _cachedPlanesF20[0])
                {
                    Plane transformedO6 = decompO6;
                    transformedO6.Transform(xform);
                    allO6Planes.Add(transformedO6);
                }
                foreach (Plane decompA6 in _cachedPlanesF20[1])
                {
                    Plane transformedA6 = decompA6;
                    transformedA6.Transform(xform);
                    allA6Planes.Add(transformedA6);
                }
            }

            // Decompose K30 tiles
            foreach (Plane pl in inputTransformationPlanes[3])
            {
                Transform xform = Transform.PlaneToPlane(Plane.WorldXY, pl);
                foreach (Plane decompO6 in _cachedPlanesK30[0])
                {
                    Plane transformedO6 = decompO6;
                    transformedO6.Transform(xform);
                    allO6Planes.Add(transformedO6);
                }
                foreach (Plane decompA6 in _cachedPlanesK30[1])
                {
                    Plane transformedA6 = decompA6;
                    transformedA6.Transform(xform);
                    allA6Planes.Add(transformedA6);
                }
            }

            // Apply batch filtering (much faster - single bounding box computation, single brep conversion)
            List<Plane> filteredO6Planes = FilterPlanesBatch(geometryFilter, allO6Planes, filterDistance, includeInterior);
            List<Plane> filteredA6Planes = FilterPlanesBatch(geometryFilter, allA6Planes, filterDistance, includeInterior);

            // Build output tree using AddRange (faster than individual Add calls)
            DataTree<Plane> outputTransformations = new DataTree<Plane>();
            GH_Path pth0 = new GH_Path(0);
            GH_Path pth1 = new GH_Path(1);
            outputTransformations.AddRange(filteredO6Planes, pth0);
            outputTransformations.AddRange(filteredA6Planes, pth1);

            // Base mesh and brep generation for 2-tile system (use cached versions if available, otherwise generate and cache)

            // Generate Base Meshes
            if (_cachedBaseMeshes == null || scale != _lastScale)
            {
                // Generate meshes for each tile type
                DataTree<Mesh> baseMeshes = new DataTree<Mesh>();
                Mesh meshO6 = GenerateMeshO6(scale);
                Mesh meshA6 = GenerateMeshA6(scale);
                baseMeshes.Add(meshO6, new GH_Path(0));
                baseMeshes.Add(meshA6, new GH_Path(1));

                _cachedBaseMeshes = baseMeshes;
            }

            // Generate Base Breps
            if (_cachedBaseBreps == null || scale != _lastScale)
            {
                // Generate breps for each tile type
                DataTree<Brep> baseBreps = new DataTree<Brep>();
                Brep brepO6 = GenerateBrepO6(scale);
                Brep brepA6 = GenerateBrepA6(scale);
                baseBreps.Add(brepO6, new GH_Path(0));
                baseBreps.Add(brepA6, new GH_Path(1));

                _cachedBaseBreps = baseBreps;
            }

            _lastScale = scale;

            DA.SetDataTree(0, _cachedBaseMeshes);
            DA.SetDataTree(1, _cachedBaseBreps);
            DA.SetDataTree(2, outputTransformations);
        }

        /// <summary>
        /// Optimized batch filter check with bounding box pre-filtering
        /// </summary>
        public static List<Plane> FilterPlanesBatch(GeometryBase geometryFilter, List<Plane> planes, double filterDistance, bool includeInterior)
        {
            if (geometryFilter == null) return planes; // No filter, return all

            var result = new List<Plane>(planes.Count);

            // Pre-compute bounding box for fast rejection
            BoundingBox filterBBox = geometryFilter.GetBoundingBox(false);
            filterBBox.Inflate(filterDistance);

            // Pre-convert brep once if applicable
            Brep brepFilter = null;
            Mesh meshFilter = null;
            Curve crvFilter = null;

            bool isBrep = geometryFilter.HasBrepForm;
            bool isMesh = !isBrep && geometryFilter is Mesh;
            bool isCurve = !isBrep && !isMesh && geometryFilter is Curve;

            if (isBrep)
            {
                brepFilter = Brep.TryConvertBrep(geometryFilter);
            }
            else if (isMesh)
            {
                meshFilter = geometryFilter as Mesh;
            }
            else if (isCurve)
            {
                crvFilter = geometryFilter as Curve;
            }

            bool canCheckInsideMesh = includeInterior && meshFilter != null && meshFilter.IsClosed;

            foreach (var plane in planes)
            {
                Point3d tilePoint = plane.Origin;

                // Fast bounding box rejection
                if (!filterBBox.Contains(tilePoint))
                {
                    continue;
                }

                if (isBrep && brepFilter != null)
                {
                    if (includeInterior && brepFilter.IsSolid)
                    {
                        if (brepFilter.IsPointInside(tilePoint, 0.01, true))
                        {
                            result.Add(plane);
                            continue;
                        }
                    }

                    Point3d closestPoint;
                    ComponentIndex ci;
                    double s, t;
                    Vector3d normal;
                    brepFilter.ClosestPoint(tilePoint, out closestPoint, out ci, out s, out t, filterDistance, out normal);
                    double dist = closestPoint.DistanceTo(tilePoint);
                    if (dist > 0 && dist <= filterDistance)
                    {
                        result.Add(plane);
                    }
                }
                else if (isMesh && meshFilter != null)
                {
                    if (canCheckInsideMesh && meshFilter.IsPointInside(tilePoint, RhinoMath.SqrtEpsilon, false))
                    {
                        result.Add(plane);
                        continue;
                    }

                    Point3d closestPoint = meshFilter.ClosestPoint(tilePoint);
                    if (!closestPoint.IsValid) continue;

                    double dist = tilePoint.DistanceTo(closestPoint);
                    if (dist > 0 && dist <= filterDistance)
                        result.Add(plane);
                }
                else if (isCurve && crvFilter != null)
                {
                    double t;
                    if (crvFilter.ClosestPoint(tilePoint, out t, filterDistance))
                    {
                        result.Add(plane);
                    }
                }
                else
                {
                    result.Add(plane); // Unknown geometry type, include by default
                }
            }

            return result;
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

            Brep refB12 = GenerateBrepB12(scale);
            BoundingBox bbox = refB12.GetBoundingBox(true);
            double Xdist = bbox.Max.X;
            double Ydist = bbox.Max.Y;
            double Zdist = bbox.Max.Z;

            // Get O6 Planes using 3 points, then mirroring across WorldYZ
            Point3d minYpoint = GetClosestVertex(refB12, new Point3d(0, -Ydist, 0)).Location;
            Point3d topPoint = GetClosestVertex(refB12, new Point3d(Xdist, Ydist*0.5, 0)).Location;
            Point3d xPoint = GetClosestVertex(refB12, new Point3d(Xdist, -Ydist * 0.5, 0)).Location;
            Plane plane0601 = PlaneFromRhombohedronCoordinates(minYpoint, topPoint, xPoint);
            planes[0][0] = plane0601;
            plane0601.Transform(Transform.Mirror(Plane.WorldYZ));
            plane0601.Rotate(Math.PI, plane0601.Normal); // Rotate to match proper orientation
            planes[0][1] = plane0601;

            // Get A6 Planes using 3 points, then mirroring across WorldXY
            Point3d maxYpoint = GetClosestVertex(refB12, new Point3d(0, Ydist, 0)).Location;
            Point3d bottomPoint = GetClosestVertex(refB12, new Point3d(0, -Ydist, -Zdist)).Location;
            xPoint = GetClosestVertex(refB12, new Point3d(Xdist, 0, -Zdist)).Location;
            Plane planeA601 = PlaneFromRhombohedronCoordinates(bottomPoint, maxYpoint, xPoint);
            planes[1][0] = planeA601;
            planeA601.Transform(Transform.Mirror(Plane.WorldXY));
            planeA601.Rotate(Math.PI, planeA601.Normal); // Rotate to match proper orientation
            planes[1][1] = planeA601;

            return planes;
        }

        public static Plane[][] GetF20DecompositionPlanes(double scale)
        {
            Plane[][] planes = new Plane[2][];
            planes[0] = new Plane[5];  // 5 O6 tiles in F20 decomposition
            planes[1] = new Plane[5];  // 5 A6 tiles in F20 decomposition

            Brep refF20 = GenerateBrepF20(scale);
            BoundingBox bbox = refF20.GetBoundingBox(true);
            double Xdist = bbox.Max.X;
            double Ydist = bbox.Max.Y;
            double Zdist = bbox.Max.Z;

            // Get some vertices of the F20
            // Start with the top vertex
            BrepVertex maxZpointVertex = GetClosestVertex(refF20, new Point3d(0, 0, Zdist));
            Point3d maxZpoint = maxZpointVertex.Location;
            Point3d minZpoint = GetClosestVertex(refF20, new Point3d(0, 0, -Zdist)).Location;
            Point3d maxXpoint = GetClosestVertex(refF20, new Point3d(Xdist, 0, 0)).Location;
            Point3d minXpoint = GetClosestVertex(refF20, new Point3d(-Xdist, 0, 0)).Location;

            // Get adjacent vertices to top vertex - gives some vectors we can use to find different points
            int[] edgeIndices = maxZpointVertex.EdgeIndices();
            Point3d e0 = new Point3d(0, 0, 0);
            for (int i = 0; i < edgeIndices.Length; i++)
            {
                BrepEdge edge = maxZpointVertex.Brep.Edges[edgeIndices[i]];
                Point3d edgept = edge.EdgeCurve.PointAtEnd;
                if (edgept == maxZpointVertex.Location)
                {
                    edgept = edge.EdgeCurve.PointAtStart;
                }
                if (edgept.X > e0.X)
                {
                    e0 = edgept;
                }
            }

            Vector3d v0 = e0 - maxZpoint;
            Vector3d v1 = v0;
            v1.Rotate(Math.PI * 2 / 5, Vector3d.ZAxis);
            Vector3d v2 = v1;
            v2.Rotate(Math.PI * 2 / 5, Vector3d.ZAxis);
            Vector3d v3 = v2;
            v3.Rotate(Math.PI * 2 / 5, Vector3d.ZAxis);
            Vector3d v4 = v3;
            v4.Rotate(Math.PI * 2 / 5, Vector3d.ZAxis);

            // Get O6 Planes using 3 points
            planes[0][0] = PlaneFromRhombohedronCoordinates(maxZpoint, maxZpoint + v0 + v1 + v2, maxZpoint + v1);
            planes[0][1] = PlaneFromRhombohedronCoordinates(maxZpoint, maxZpoint + v0 + v4 + v3, maxZpoint + v4);
            planes[0][2] = PlaneFromRhombohedronCoordinates(minZpoint, maxZpoint + v0 + v1, minZpoint - v3);
            planes[0][3] = PlaneFromRhombohedronCoordinates(minZpoint, maxZpoint + v2 + v3, minZpoint - v0);
            planes[0][4] = PlaneFromRhombohedronCoordinates(maxZpoint + v0, minZpoint - v4, maxZpoint + v0 + v2);

            // Get A6 Planes using 3 points
            planes[1][0] = PlaneFromRhombohedronCoordinates(maxZpoint + v0, minXpoint, maxZpoint);
            planes[1][1] = PlaneFromRhombohedronCoordinates(minXpoint, maxZpoint + v0 + v1 + v2, maxZpoint + v2);
            planes[1][2] = PlaneFromRhombohedronCoordinates(minXpoint, maxZpoint + v0 + v4 + v3, maxZpoint + v3);
            planes[1][3] = PlaneFromRhombohedronCoordinates(maxZpoint + v0 + v4 + v3, maxZpoint + v0 + v1, minZpoint - v2);
            planes[1][4] = PlaneFromRhombohedronCoordinates(minZpoint - v4, minZpoint -v1 - v2, minZpoint);

            return planes;
        }

        public static Plane[][] GetK30DecompositionPlanes(double scale)
        {
            Plane[][] planes = new Plane[2][];
            planes[0] = new Plane[10];  // 5 O6 tiles in K30 decomposition
            planes[1] = new Plane[10];  // 5 A6 tiles in K30 decomposition

            Brep refK30 = GenerateBrepK30(scale);
            BoundingBox bbox = refK30.GetBoundingBox(true);
            double Zdist = bbox.Max.Z;

            // Get some vertices of the K30
            // Start with the top face
            int closeFaceIndex = GetClosestFace(refK30, new Point3d(0, 0, Zdist));

            // Get center and normal vector of the current face
            BrepFace closeFace = refK30.Faces[closeFaceIndex];
            Point3d centerFace = GetBrepFaceCenter(refK30, closeFaceIndex);

            // Get vertices from the face
            List<Point3d> faceVertices = new List<Point3d>();
            int edgeIndex = closeFace.AdjacentEdges()[0];
            BrepEdge edge = refK30.Edges[edgeIndex];
            Curve edgeCurve = edge.EdgeCurve;
            faceVertices.Add(edgeCurve.PointAtStart);
            faceVertices.Add(edgeCurve.PointAtEnd);
            Vector3d a = edgeCurve.PointAtStart - centerFace;
            Vector3d b = edgeCurve.PointAtEnd - centerFace;
            faceVertices.Add(centerFace - a);
            faceVertices.Add(centerFace - b);

            // Get vertices above and below XZ plane
            Point3d topFacePositiveY = new Point3d(0, 0, 0);
            Point3d topFaceNegativeY = new Point3d(0, 0, 0);
            for (int i = 0; i < 4; i++)
            {
                if (faceVertices[i].Y > 0.0001)
                {
                    topFacePositiveY = faceVertices[i];
                }
                else if (faceVertices[i].Y < -0.0001)
                {
                    topFaceNegativeY = faceVertices[i];
                }
            }
            BrepVertex topFacePositiveYVertex = GetClosestVertex(refK30, topFacePositiveY);
            BrepVertex topFaceNegativeYVertex = GetClosestVertex(refK30, topFaceNegativeY);

            // Get symmetric vertices on the bottom face by mirroring across XY plane
            Point3d bottomFacePositiveY = new Point3d(topFacePositiveY.X, topFacePositiveY.Y, -topFacePositiveY.Z);
            Point3d bottomFaceNegativeY = new Point3d(topFaceNegativeY.X, topFaceNegativeY.Y, -topFaceNegativeY.Z);

            // Get adjacent vertices and vector from topFaceNegativeY - look for the edge pointing in negative Y direction
            Point3d e0 = new Point3d(0, 0, 0);
            int[] edgeIndices = topFaceNegativeYVertex.EdgeIndices();
            for (int i = 0; i < edgeIndices.Length; i++)
            {
                edge = topFaceNegativeYVertex.Brep.Edges[edgeIndices[i]];
                Point3d edgept = edge.EdgeCurve.PointAtEnd;
                if (edgept == topFaceNegativeYVertex.Location)
                {
                    edgept = edge.EdgeCurve.PointAtStart;
                }
                if (edgept.Y < topFaceNegativeY.Y)
                {
                    e0 = edgept;
                }
            }

            // Get adjacent vertices and vector from topFacePositiveY - look for the edge pointing in positive Y direction
            Point3d e5 = new Point3d(0, 0, 0);
            edgeIndices = topFacePositiveYVertex.EdgeIndices();
            for (int i = 0; i < edgeIndices.Length; i++)
            {
                edge = topFacePositiveYVertex.Brep.Edges[edgeIndices[i]];
                Point3d edgept = edge.EdgeCurve.PointAtEnd;
                if (edgept == topFacePositiveYVertex.Location)
                {
                    edgept = edge.EdgeCurve.PointAtStart;
                }
                if (edgept.Y > topFacePositiveY.Y)
                {
                    e5 = edgept;
                }
            }

            Vector3d v5 = topFacePositiveY - e5;
            Vector3d v0 = e0 - topFaceNegativeY;
            Vector3d v1 = v0;
            v1.Rotate(Math.PI * 2 / 5, v5);
            Vector3d v2 = v1;
            v2.Rotate(Math.PI * 2 / 5, v5);
            Vector3d v3 = v2;
            v3.Rotate(Math.PI * 2 / 5, v5);
            Vector3d v4 = v3;
            v4.Rotate(Math.PI * 2 / 5, v5);

            // Get O6 Planes using 3 points
            planes[0][0] = PlaneFromRhombohedronCoordinates(topFaceNegativeY, topFacePositiveY + v1, topFaceNegativeY + v2);
            planes[0][1] = PlaneFromRhombohedronCoordinates(topFaceNegativeY, topFaceNegativeY + v0 + v4 + v3, topFaceNegativeY + v4);
            planes[0][2] = PlaneFromRhombohedronCoordinates(topFacePositiveY, topFacePositiveY + v1 - v5 + v4, topFacePositiveY - v5);
            planes[0][3] = PlaneFromRhombohedronCoordinates(bottomFacePositiveY, bottomFacePositiveY - v0 - v2 - v1, bottomFacePositiveY - v1);
            planes[0][4] = PlaneFromRhombohedronCoordinates(bottomFacePositiveY, bottomFacePositiveY - v0 + v5 - v2, bottomFacePositiveY + v5);
            planes[0][5] = PlaneFromRhombohedronCoordinates(bottomFacePositiveY, bottomFacePositiveY - v4 + v5 - v2, bottomFacePositiveY + v5);
            planes[0][6] = PlaneFromRhombohedronCoordinates(topFaceNegativeY + v3, bottomFaceNegativeY + v3 + v5, topFaceNegativeY + v3 + v0);
            planes[0][7] = PlaneFromRhombohedronCoordinates(bottomFacePositiveY - v4,topFaceNegativeY + v1 + v2 ,bottomFacePositiveY - v4 + v5);
            planes[0][8] = PlaneFromRhombohedronCoordinates(bottomFaceNegativeY + v3, bottomFaceNegativeY + v2 - v4, bottomFaceNegativeY + v3 - v4);
            planes[0][9] = PlaneFromRhombohedronCoordinates(bottomFaceNegativeY, bottomFaceNegativeY - v1 + v5 + v3, bottomFaceNegativeY - v1);

            // Get A6 Planes using 3 points
            planes[1][0] = PlaneFromRhombohedronCoordinates(bottomFacePositiveY - v0, topFacePositiveY + v4 - v2, bottomFacePositiveY - v0 - v1);
            planes[1][1] = PlaneFromRhombohedronCoordinates(bottomFaceNegativeY + v3, topFacePositiveY + v4 - v2, bottomFaceNegativeY + v3 + v5);
            planes[1][2] = PlaneFromRhombohedronCoordinates(topFaceNegativeY + v3 + v4, topFacePositiveY + v1, topFaceNegativeY + v3);
            planes[1][3] = PlaneFromRhombohedronCoordinates(topFacePositiveY + v1, bottomFaceNegativeY + v3 + v5, topFacePositiveY + v1 - v2);
            planes[1][4] = PlaneFromRhombohedronCoordinates(bottomFacePositiveY, topFacePositiveY + v1, bottomFacePositiveY + v5);
            planes[1][5] = PlaneFromRhombohedronCoordinates(topFaceNegativeY + v0 + v1, topFacePositiveY + v1, topFaceNegativeY + v0 + v1 + v3);
            planes[1][6] = PlaneFromRhombohedronCoordinates(topFaceNegativeY + v0 + v1, bottomFacePositiveY - v4, topFaceNegativeY + v0 + v1 + v3);
            planes[1][7] = PlaneFromRhombohedronCoordinates(topFaceNegativeY + v0 + v1, bottomFaceNegativeY + v3, topFaceNegativeY + v0 + v1 + v3);
            planes[1][8] = PlaneFromRhombohedronCoordinates(topFaceNegativeY + v0 + v1, topFaceNegativeY + v0 + v3 + v4, topFaceNegativeY + v0 + v1 + v3);
            planes[1][9] = PlaneFromRhombohedronCoordinates(topFaceNegativeY + v0 + v1, topFaceNegativeY + v3, topFaceNegativeY + v0 + v1 + v3);

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

        public static int GetClosestFace(Brep brep, Point3d reference)
        {
            int closestFaceIndex = 0;
            Point3d currentFaceCenter = new Point3d();
            double minDistance = double.MaxValue;
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
            double phi = GoldenRatio;

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
            double phi = GoldenRatio;

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
            double phi = GoldenRatio;
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
            double phi = GoldenRatio;
            // Rotate according to angle between long diagonal and face, so that long diagonal axis aligns with Z axis
            // Note: This rotation potentially introduces inaccuracies - maybe cleaner to generate the zonohedron already at this angle
            brepA6.Rotate(-Math.Acos(phi / Math.Sqrt(3)) - (Math.PI / 2), Vector3d.YAxis, Point3d.Origin);
            return brepA6;
        }

        public static Mesh GenerateZonohedronMeshFromStarVectors(List<Vector3d> starVectors, double scale)
        {
            double buildScale = (scale < 1.0) ? 1.0 : scale;

            // Scale star vectors
            List<Vector3d> scaledStarVectors = new List<Vector3d>();
            foreach (Vector3d vec in starVectors)
            {
                vec.Unitize();
                scaledStarVectors.Add(Vector3d.Multiply(vec, buildScale * 0.5));
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
                // Set up unitless comparison so scaling won't affect the p-representation
                Vector3d nUnit = n;
                nUnit.Unitize();

                // Set up face p-representation and its opposite
                List<int> face_p_representation = new List<int>();
                List<int> face_p_representationOpposite = new List<int>();

                // Loop through star vectors
                foreach (Vector3d v in scaledStarVectors)
                {
                    Vector3d vUnit = v;
                    vUnit.Unitize();

                    // Get dot product
                    double d = Vector3d.Multiply(nUnit, vUnit);

                    // Create p-representation entries
                    if (Math.Abs(d) < 1e-6)
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

            // Scale finished Mesh from buildScale to requested scale
            if (buildScale != scale)
            {
                Transform xform = Transform.Scale(Point3d.Origin, scale / buildScale);
                mesh.Transform(xform);
            }

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
            double buildScale = (scale < 1.0) ? 1.0 : scale;

            // Scale star vectors
            List<Vector3d> scaledStarVectors = new List<Vector3d>();
            foreach (Vector3d vec in starVectors)
            {
                vec.Unitize();
                scaledStarVectors.Add(Vector3d.Multiply(vec, buildScale * 0.5));
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

                // Set up unitless comparison so scaling won't affect the p-representation
                Vector3d nUnit = n;
                nUnit.Unitize();

                // Loop through star vectors
                foreach (Vector3d v in scaledStarVectors)
                {
                    Vector3d vUnit = v;
                    vUnit.Unitize();

                    // Get dot product
                    double d = Vector3d.Multiply(nUnit, vUnit);
                    // Create p-representation entries
                    if (Math.Abs(d) < 1e-6)
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
            Brep[] joined = Brep.JoinBreps(faces.ConvertAll(f => Brep.CreateFromSurface(f)), 0.001 * buildScale);

            if (joined == null || joined.Length == 0)
                return null;

            Brep brep = joined[0];

            // Scale finished Brep from buildScale to requested scale
            if (buildScale != scale)
            {
                Transform xform = Transform.Scale(Point3d.Origin, scale / buildScale);
                brep.Transform(xform);
            }

            return brep;
        }

        #endregion

        #region ---Preview---
        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            // No wireframe preview
        }

        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            // No mesh preview
        }
        #endregion

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => Resources.aperiodic2tile24px;

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("A1B2C3D4-E5F6-7890-ABCD-EF1234567890");
    }
}
