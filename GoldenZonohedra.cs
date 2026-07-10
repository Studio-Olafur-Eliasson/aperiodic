using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Numerics;
using Mesh = Rhino.Geometry.Mesh;

namespace Aperiodic
{
    public class GoldenZonohedra : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public GoldenZonohedra()
          : base("GoldenZonohedra", "GoldenZonohedra",
              "Generate the golden zonohedral tiles used in quasicrystal tilings, possibly with scale",
              "Aperiodic", "Aperiodic")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("scale", "scale", "Scale factor (default 1.0)", GH_ParamAccess.item, 1.0);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("meshO6", "O6", "Obtuse rhombohedron", GH_ParamAccess.list);
            pManager.AddMeshParameter("meshA6", "A6", "Acute rhombohedron", GH_ParamAccess.list);
            pManager.AddMeshParameter("meshB12", "B12", "Rhombic (Bilinksi) dodecahedron", GH_ParamAccess.list);
            pManager.AddMeshParameter("meshF20", "F20", "Rhombic icosahedron", GH_ParamAccess.list);
            pManager.AddMeshParameter("meshK30", "K30", "Rhombic triacontahedron", GH_ParamAccess.list);
            pManager.AddBrepParameter("brepO6", "O6", "Obtuse rhombohedron", GH_ParamAccess.list);
            pManager.AddBrepParameter("brepA6", "A6", "Acute rhombohedron", GH_ParamAccess.list);
            pManager.AddBrepParameter("brepB12", "B12", "Rhombic (Bilinksi) dodecahedron", GH_ParamAccess.list);
            pManager.AddBrepParameter("brepF20", "F20", "Rhombic icosahedron", GH_ParamAccess.list);
            pManager.AddBrepParameter("brepK30", "K30", "Rhombic triacontahedron", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Get scale
            double scale = 1.0;
            if (!DA.GetData(0, ref scale)) { return; }

            Mesh meshO6 = GenerateMeshO6(scale);
            Mesh meshA6 = GenerateMeshA6(scale);
            Mesh meshB12 = GenerateMeshB12(scale);
            Mesh meshF20 = GenerateMeshF20(scale);
            Mesh meshK30 = GenerateMeshK30(scale);

            Brep brepO6 = GenerateBrepO6(scale);
            Brep brepA6 = GenerateBrepA6(scale);
            Brep brepB12 = GenerateBrepB12(scale);
            Brep brepF20 = GenerateBrepF20(scale);
            Brep brepK30 = GenerateBrepK30(scale);

            // Declare outputs
            DA.SetData(0, meshO6);
            DA.SetData(1, meshA6);
            DA.SetData(2, meshB12);
            DA.SetData(3, meshF20);
            DA.SetData(4, meshK30);

            DA.SetData(5, brepO6);
            DA.SetData(6, brepA6);
            DA.SetData(7, brepB12);
            DA.SetData(8, brepF20);
            DA.SetData(9, brepK30);
        }

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
            brepO6.Rotate(((Math.PI / 2) - theta)/2, Vector3d.YAxis, Point3d.Origin);
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
            brepA6.Rotate(-Math.Acos(phi / Math.Sqrt(3))-(Math.PI / 2), Vector3d.YAxis, Point3d.Origin);
            return brepA6;
        }
        public static Mesh GenerateMeshB12(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(4, false);
            return GenerateZonohedronMeshFromStarVectors(starVectors, scale);
        }

        public static Brep GenerateBrepB12(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(4, false);
            return GenerateZonohedronBrepFromStarVectors(starVectors, scale);
        }

        public static Mesh GenerateMeshF20(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(5, false);
            Mesh meshF20 = GenerateZonohedronMeshFromStarVectors(starVectors, scale);
            meshF20.Rotate(Math.PI, Vector3d.ZAxis, Point3d.Origin);
            meshF20.Rotate(Math.Asin(Math.Sqrt((5 + Math.Sqrt(5)) / 10)), Vector3d.YAxis, Point3d.Origin);
            return meshF20;
        }

        public static Brep GenerateBrepF20(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(5, false);
            Brep brepF20 = GenerateZonohedronBrepFromStarVectors(starVectors, scale);
            brepF20.Rotate(Math.PI, Vector3d.ZAxis, Point3d.Origin);
            brepF20.Rotate(Math.Asin(Math.Sqrt((5+Math.Sqrt(5))/10)), Vector3d.YAxis, Point3d.Origin);
            return brepF20;
        }

        public static Mesh GenerateMeshK30(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(6, false);
            return GenerateZonohedronMeshFromStarVectors(starVectors, scale);
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
                scaledStarVectors.Add(Vector3d.Multiply(vec, buildScale*0.5));
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

        public override GH_Exposure Exposure => GH_Exposure.tertiary;

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("DC3C5F6A-7004-48B5-B226-1AAB99D2F259"); }
        }
    }
}