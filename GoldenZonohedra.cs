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
            pManager.AddBrepParameter("meshO6", "O6", "Obtuse rhombohedron", GH_ParamAccess.item);
            pManager.AddBrepParameter("meshA6", "A6", "Acute rhombohedron", GH_ParamAccess.item);
            pManager.AddBrepParameter("meshB12", "B12", "Rhombic (Bilinksi) dodecahedron", GH_ParamAccess.item);
            pManager.AddBrepParameter("meshF20", "F20", "Rhombic icosahedron", GH_ParamAccess.item);
            pManager.AddBrepParameter("meshK30", "K30", "Rhombic triacontahedron", GH_ParamAccess.item);
            pManager.AddMeshParameter("meshO6", "O6", "Obtuse rhombohedron", GH_ParamAccess.list);
            pManager.AddMeshParameter("meshA6", "A6", "Acute rhombohedron", GH_ParamAccess.list);
            pManager.AddMeshParameter("meshB12", "B12", "Rhombic (Bilinksi) dodecahedron", GH_ParamAccess.list);
            pManager.AddMeshParameter("meshF20", "F20", "Rhombic icosahedron", GH_ParamAccess.list);
            pManager.AddMeshParameter("meshK30", "K30", "Rhombic triacontahedron", GH_ParamAccess.list);
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
           
            Brep meshO6 = GenerateMeshO6(scale);
            Brep meshA6 = GenerateMeshA6(scale);
            Brep meshB12 = GenerateMeshB12(scale);
            Brep meshF20 = GenerateMeshF20(scale);
            Brep meshK30 = GenerateMeshK30(scale);

            // Declare outputs
            DA.SetData(0, meshO6);
            DA.SetData(1, meshA6);
            DA.SetData(2, meshB12);
            DA.SetData(3, meshF20);
            DA.SetData(4, meshK30);

            //Mesh meshmeshO6 = BrepToSingleMesh(meshO6, MeshingParameters.Default);
            //Mesh meshmeshA6 = BrepToSingleMesh(meshA6, MeshingParameters.Default);
            //Mesh meshmeshB12 = BrepToSingleMesh(meshB12, MeshingParameters.Default);
            //Mesh meshmeshF20 = BrepToSingleMesh(meshF20, MeshingParameters.Default);
            //Mesh meshmeshK30 = BrepToSingleMesh(meshK30, MeshingParameters.Default);

            //// Declare outputs
            //DA.SetData(5, meshmeshO6);
            //DA.SetData(6, meshmeshA6);
            //DA.SetData(7, meshmeshB12);
            //DA.SetData(8, meshmeshF20);
            //DA.SetData(9, meshmeshK30);
        }

        public static Brep GenerateMeshO6(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(3);
            return GenerateZonohedronFromStarVectors(starVectors, scale);
        }

        public static Brep GenerateMeshA6(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(3);
            return GenerateZonohedronFromStarVectors(starVectors, scale);
        }

        public static Brep GenerateMeshB12(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(4);
            return GenerateZonohedronFromStarVectors(starVectors, scale);
        }

        public static Brep GenerateMeshF20(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(5);
            return GenerateZonohedronFromStarVectors(starVectors, scale);
        }

        public static Brep GenerateMeshK30(double scale)
        {
            List<Vector3d> starVectors = GenerateStarVectors(6);
            return GenerateZonohedronFromStarVectors(starVectors, scale);
        }

        public static Brep GenerateZonohedronFromStarVectors(List<Vector3d> starVectors, double scale)
        {
            // Scale star vectors
            List<Vector3d> scaledStarVectors = new List<Vector3d>();
            foreach (Vector3d vec in starVectors)
            {
                vec.Unitize();
                scaledStarVectors.Add(Vector3d.Multiply(vec, scale*0.5));
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
            List<Surface> faces = new List<Surface>();
            foreach (Vector3d n in normalVectors)
            {
                List<int> face_p_representation = new List<int>();
                // Loop through star vectors
                foreach (Vector3d v in scaledStarVectors)
                {
                    // Get dot product
                    double d = Vector3d.Multiply(n, v);
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
                        // TODO: double check 1 and -1 assignments here
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

            // TODO: Generate mesh directly instead of going through Brep
            // return Mesh.CreateFromBrep(brep, MeshingParameters.Minimal)[0];
        }

        public static Mesh BrepToSingleMesh(Brep brep, MeshingParameters mp = null)
        {
            if (brep == null) return null;
            if (!brep.IsValid) brep.Repair(0.01); // optional, see notes

            mp = MeshingParameters.Default; // or .FastRenderMesh / .QualityRenderMesh

            Mesh[] parts = Mesh.CreateFromBrep(brep, mp);
            if (parts == null || parts.Length == 0) return null;

            var joined = new Mesh();
            foreach (var m in parts)
            {
                if (m == null) continue;
                if (m.Vertices.Count == 0) continue;
                joined.Append(m);
            }

            joined.Vertices.CombineIdentical(true, true);
            joined.Vertices.CullUnused();
            joined.Weld(Math.PI); // weld everything

            bool merged = joined.MergeAllCoplanarFaces(RhinoDoc.ActiveDoc.ModelAngleToleranceRadians);
            joined.UnifyNormals();
            joined.Normals.ComputeNormals();
            joined.Compact();
            return joined;
        }

        public static List<Vector3d> GenerateStarVectors(int numZones)
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

            // Select the required number of zones
            List<Vector3d> selectedVectors = allStarVectors.GetRange(0, numZones);
            return selectedVectors;
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
            get { return new Guid("DC3C5F6A-7004-48B5-B226-1AAB99D2F259"); }
        }
    }
}