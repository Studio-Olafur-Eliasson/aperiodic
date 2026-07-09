using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Aperiodic
{
    public class SeedOptions : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SeedOptions class.
        /// </summary>
        public SeedOptions()
          : base("SeedOptions", "Seeds",
              "Generates three different seed plane configurations based on scale",
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
            pManager.AddPlaneParameter("seedPlanes0", "0", "First set of seed planes", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("seedPlanes1", "1", "Second set of seed planes", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("seedPlanes2", "2", "Third set of seed planes", GH_ParamAccess.tree);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Structure<GH_Mesh> reference = new GH_Structure<GH_Mesh>();
            if (!DA.GetDataTree<GH_Mesh>(0, out reference)) { return; }

            GH_Structure<GH_Plane> seedPlanes0 = new GH_Structure<GH_Plane>();
            GH_Structure<GH_Plane> seedPlanes1 = new GH_Structure<GH_Plane>();
            GH_Structure<GH_Plane> seedPlanes2 = new GH_Structure<GH_Plane>();

            GH_Path pth0 = new GH_Path(0);
            GH_Path pth1 = new GH_Path(1);
            GH_Path pth2 = new GH_Path(2);
            GH_Path pth3 = new GH_Path(3);

            // TODO: Generate seed planes based on scale

            // Set up seed 0 option (a single plane for the K30 tile)
            seedPlanes0.EnsurePath(pth0);
            seedPlanes0.EnsurePath(pth1);
            seedPlanes0.EnsurePath(pth2);
            seedPlanes0.Append(new GH_Plane(Plane.WorldXY), pth3);

            // Set up seed 1 option (20 A6 tiles arranged as dodecahedron star)
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

            double a6HeightRef = GetMeshHeight(reference.Branches[0][0].Value);

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
                seedPlanes1.Append(new GH_Plane(seedPlane), pth0);

                Plane flippedSeedPlane = new Plane(seedPlane);
                flippedSeedPlane.Flip();
                flippedSeedPlane.Rotate(-Math.PI / 2, flippedSeedPlane.Normal);
                seedPlanes2.Append(new GH_Plane(flippedSeedPlane), pth0);
            }

            seedPlanes1.EnsurePath(pth1);
            seedPlanes1.EnsurePath(pth2);
            seedPlanes1.EnsurePath(pth3);

            // Set up seed 2 option (20 a6 tiles)
            seedPlanes2.EnsurePath(pth1);
            seedPlanes2.EnsurePath(pth2);
            seedPlanes2.EnsurePath(pth3);


            DA.SetDataTree(0, seedPlanes0);
            DA.SetDataTree(1, seedPlanes1);
            DA.SetDataTree(2, seedPlanes2);
        }

        public static double GetMeshHeight(Mesh mesh)
        {
            BoundingBox bbox = mesh.GetBoundingBox(true);
            return bbox.Max.Z - bbox.Min.Z;
        }

        /// <summary>
        /// Returns the signed angle between two vectors on a plane.
        /// Useful since Vector3d.VectorAngle only returns positive values.
        /// </summary>
        public static double GetSignedVectorAngle(Vector3d v1, Vector3d v2, Plane plane)
        {
            return Math.Atan2(Vector3d.CrossProduct(v1, v2) * plane.ZAxis, v1 * v2);
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
            get { return new Guid("8A2B3C4D-5E6F-7A8B-9C0D-1E2F3A4B5C6D"); }
        }
    }
}
