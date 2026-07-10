using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.IO;
using Mesh = Rhino.Geometry.Mesh;

namespace Aperiodic
{
    public class DeflationRuleA6Brep : GH_Component
    {
        // Cache commonly used constants
        private static readonly double GoldenRatio = (1 + Math.Sqrt(5)) / 2;
        private static readonly double DeflationScaleFactor = Math.Pow(GoldenRatio, 3);
        private static readonly double InverseDeflationScaleFactor = 1.0 / DeflationScaleFactor;

        /// <summary>
        /// Initializes a new instance of the DeflationRuleA6Brep class.
        /// </summary>
        public DeflationRuleA6Brep()
          : base("DeflationRuleA6Brep", "DefA6Brep",
              "Output planes corresponding the the deflation rules for the A6 tile (additionally output breps and points). Updated to work with Brep input and operations.",
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
            pManager.AddBrepParameter("outputBreps", "outBreps", "Output meshes after applying deflation rule A6", GH_ParamAccess.tree);
            pManager.AddPointParameter("outputPoints", "outPts", "Output points after applying deflation rule A6", GH_ParamAccess.tree);
            pManager.AddPlaneParameter("outputPlanes", "outPlanes", "Output planes after applying deflation rule A6", GH_ParamAccess.tree);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // NOTE: BREP VERSION HAS BETTER ACCURACY

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

            Brep brep = refA6.DuplicateBrep();
            Point3d basept = Point3d.Origin;

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
            // Translate the basept to the new scaled location - consistent with the brep itself, so further operations on the base pts can recurse well
            basept += inflateVec;

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
            double scale = refK30.Edges[0].PointAtEnd.DistanceTo(refK30.Edges[0].PointAtStart); //brep code

            // Set up base orientation for K30 transformation in step (a6-000)
            Point3d k30basecenter = GetClosestVertex(refK30, new Point3d(0, -1*scale, 0)).Location;
            Point3d k30basexaxis = GetClosestVertex(refK30, new Point3d(1*scale, 0, 0)).Location;
            Point3d k30baseyaxis = new Point3d(-k30basexaxis.X, 0, 0);
            Plane k30base = new Plane(k30basecenter, k30basexaxis, k30baseyaxis);

            // Set up base orientation for B12 transformation in step (a6-000)
            Point3d b12basecenter = GetClosestVertex(refB12, new Point3d(-0.5*scale, 0, 0)).Location;
            Point3d b12baseyaxis = GetClosestVertex(refB12, new Point3d(-0.5*scale, -0.5*scale, 1*scale)).Location;
            Point3d b12basexaxis = new Point3d(b12baseyaxis.X, -b12baseyaxis.Y, b12baseyaxis.Z);
            Plane b12base = new Plane(b12basecenter, b12basexaxis, b12baseyaxis);

            // This is rhombohedron (long tile)
            // Begin deflation for A6

            // TODO: Eventual order of operations for each new mesh/(pt)/pln - find new base plane first, then transform basemesh from worldXY to new base plane

            // Get centroid
            AreaMassProperties ampA6 = AreaMassProperties.Compute(brep);
            Point3d centroidA6 = ampA6.Centroid;

            // Scale up A6 unit to get general boundaries of the inflated shapes
            // Scale center point of geometry by a factor of golden ratio^3
            Transform xformScaleA6 = Transform.Scale(centroidA6, DeflationScaleFactor);
            Brep a6boundary = brep.DuplicateBrep();
            a6boundary.Transform(xformScaleA6);

            // Find vertex/point furthest from the base point
            Point3d furthestVertexpt = GetFurthestVertex(brep, basept).Location;
            Point3d scaledbasept = new Point3d(basept);
            scaledbasept.Transform(xformScaleA6);
            Point3d scaledFurthestVertexpt = new Point3d(furthestVertexpt);
            scaledFurthestVertexpt.Transform(xformScaleA6);
            Vector3d moveCloseCopy = scaledbasept - basept;
            Vector3d moveFarCopy = scaledFurthestVertexpt - furthestVertexpt;

            // Move one copy of A6 to the base point, and one to the far end of the inflated shape
            Brep a6a600 = brep.DuplicateBrep();
            Brep a6a601 = brep.DuplicateBrep();
            a6a600.Translate(moveCloseCopy);
            a6a601.Translate(moveFarCopy);

            // Add to brep list
            listA6.Add(a6a600);
            listA6.Add(a6a601);

            // Add to basepts list
            Point3d basepta6a600 = basept + moveCloseCopy;
            Point3d basepta6a601 = basept + moveFarCopy;
            ptsA6.Add(basepta6a600);
            ptsA6.Add(basepta6a601);

            // Add to baseplns list
            Plane baseplna6a600 = new Plane(Plane.WorldXY);
            Plane baseplna6a601 = new Plane(Plane.WorldXY);
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
            BrepVertex closeCopyInner = GetFurthestVertex(a6a600, basepta6a600);
            Point3d closeCopyInnerPt = closeCopyInner.Location;

            // Get orientation planes for the K30
            List<Point3d> edgePoints = GetAdjacentVertexPoints(closeCopyInner);

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
            Brep a6k300 = refK30.DuplicateBrep();
            Brep a6k301 = refK30.DuplicateBrep();
            Brep a6k302 = refK30.DuplicateBrep();
            a6k300.Transform(xforma6k300);
            a6k301.Transform(xforma6k301);
            a6k302.Transform(xforma6k302);

            // Add to brep list
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
            Brep a6a602 = a6a600.DuplicateBrep();
            a6a602.Transform(xforma6a602);
            a6a602.Transform(xforma6a602rot);

            // Add to brep list
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
            Brep a6a603 = a6a602.DuplicateBrep();
            Brep a6a604 = a6a602.DuplicateBrep();
            Brep a6a605 = a6a602.DuplicateBrep();
            a6a603.Transform(xforma6a603);
            a6a604.Transform(xforma6a604);
            a6a605.Transform(xforma6a605);

            // Add to brep list
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
            Brep a6b120 = refB12.DuplicateBrep();
            Brep a6b121 = refB12.DuplicateBrep();
            Brep a6b122 = refB12.DuplicateBrep();
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
            Brep a6a606 = a6a602.DuplicateBrep();
            Brep a6a607 = a6a602.DuplicateBrep();
            Brep a6a608 = a6a602.DuplicateBrep();
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
            Brep a6a6061 = a6a604.DuplicateBrep();
            Brep a6a6071 = a6a605.DuplicateBrep();
            Brep a6a6081 = a6a603.DuplicateBrep();
            Brep a6a6062 = a6a605.DuplicateBrep();
            Brep a6a6072 = a6a603.DuplicateBrep();
            Brep a6a6082 = a6a604.DuplicateBrep();
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
            Brep a6k303 = refK30.DuplicateBrep();
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
            Brep a6a609 = a6a603.DuplicateBrep();
            Brep a6a610 = a6a604.DuplicateBrep();
            Brep a6a611 = a6a605.DuplicateBrep();
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
            Brep a6a6091 = a6a609.DuplicateBrep();
            a6a6091.Transform(xforma6a609rot);
            Brep a6a6092 = a6a6091.DuplicateBrep();
            a6a6092.Transform(xforma6a609rot);
            Brep a6a6093 = a6a6092.DuplicateBrep();
            a6a6093.Transform(xforma6a609rot);
            Brep a6a6094 = a6a6093.DuplicateBrep();
            a6a6094.Transform(xforma6a609rot);
            Brep a6a6101 = a6a610.DuplicateBrep();
            a6a6101.Transform(xforma6a610rot);
            Brep a6a6102 = a6a6101.DuplicateBrep();
            a6a6102.Transform(xforma6a610rot);
            Brep a6a6103 = a6a6102.DuplicateBrep();
            a6a6103.Transform(xforma6a610rot);
            Brep a6a6104 = a6a6103.DuplicateBrep();
            a6a6104.Transform(xforma6a610rot);
            Brep a6a6111 = a6a611.DuplicateBrep();
            a6a6111.Transform(xforma6a611rot);
            Brep a6a6112 = a6a6111.DuplicateBrep();
            a6a6112.Transform(xforma6a611rot);
            Brep a6a6113 = a6a6112.DuplicateBrep();
            a6a6113.Transform(xforma6a611rot);
            Brep a6a6114 = a6a6113.DuplicateBrep();
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
            BrepVertex farVertex = GetFurthestVertex(a6a601, basept);

            // Get adjacent vertex points
            edgePoints = GetAdjacentVertexPoints(farVertex);

            // Create normal vectors
            Vector3d normala6f200 = edgePoints[0] - farVertex.Location;
            Vector3d normala6f201 = edgePoints[1] - farVertex.Location;
            Vector3d normala6f202 = edgePoints[2] - farVertex.Location;
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
            Brep a6f200 = refF20.DuplicateBrep();
            Brep a6f201 = refF20.DuplicateBrep();
            Brep a6f202 = refF20.DuplicateBrep();
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
            List<Brep> a6a6fora6f20 = new List<Brep> { a6a609, a6a610, a6a611 };
            List<Point3d> a6a6fora6f20pts = new List<Point3d> { basepta6a609, basepta6a610, basepta6a611 };
            List<Vector3d> a6a6fora6f20axes = new List<Vector3d> { axisa6a609, axisa6a610, axisa6a611 };
            for (int i = 0; i < 3; i++)
            {
                BrepVertex topVertex = GetClosestVertex(a6a6fora6f20[i], a6a6fora6f20pts[i]);

                // Get for vertex adjacent vertex points
                edgePoints = GetAdjacentVertexPoints(topVertex);

                for (int j = 0; j < 3; j++)
                {
                    Vector3d normala6f20b = edgePoints[j] - topVertex.Location;
                    if (normala6f20b.IsParallelTo(a6a6fora6f20axes[i]) == 0)
                    {
                        Plane planea6f20b = new Plane(edgePoints[j], normala6f20b);
                        Point3d planea6f20bxaxis = new Point3d(centroida6k303);
                        double anglea6f20b = GetSignedVectorAngle(planea6f20b.XAxis, planea6f20bxaxis - edgePoints[j], planea6f20b);
                        planea6f20b.Rotate(anglea6f20b + Math.PI, normala6f20b, edgePoints[j]);
                        Transform xforma6f20b = Transform.PlaneToPlane(Plane.WorldXY, planea6f20b);
                        Brep a6f203 = refF20.DuplicateBrep();
                        a6f203.Transform(xforma6f20b);

                        // Add to brep list
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

            // Final Z-offset for all planes based on the deflated A6 half-height
            double zTranslation = -a6HalfHeight * DeflationScaleFactor;
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

        public static List<Point3d> GetAdjacentVertexPoints(BrepVertex vertex)
        {
            List<Point3d> adjacentVertexPoints = new List<Point3d>();
            int[] edgeIndices = vertex.EdgeIndices();
            for (int i = 0; i < edgeIndices.Length; i++)
            {
                BrepEdge edge = vertex.Brep.Edges[edgeIndices[i]];
                Point3d start = edge.EdgeCurve.PointAtStart;
                Point3d end = edge.EdgeCurve.PointAtEnd;
                Point3d edgept = (start.DistanceTo(vertex.Location) > end.DistanceTo(vertex.Location)) ? start : end;
                adjacentVertexPoints.Add(edgept);
            }
            return adjacentVertexPoints;
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

        // Below function allows comparing angles on a plane without needing to project
        // Useful since Vector3d.VectorAngle will only ever return positive values
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
            brep.Translate(new Vector3d(0, 0, -minZ));
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
            get { return new Guid("A9224CE4-5B74-4D3A-94B6-3E8F1AA40B1A"); }
        }
    }
}