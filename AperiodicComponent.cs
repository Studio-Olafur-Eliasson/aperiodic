using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data; // Required for GH_Structure<T>
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Aperiodic
{
    public class AperiodicComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public AperiodicComponent()
          : base("GenerateTilings", "Generate",
            "Generate transformations according to inputs",
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
            pManager.AddNumberParameter("scale", "s", "Scale factor (edge length) of the tiles. Default: 1.0 (no scaling)", GH_ParamAccess.item, 1.0);
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

            // baseplns.SimplifyPaths(); // Ensure that basepln tree is simplified for further calculations to run (TODO: check if this still applies with GH_Structure<T>)

            List<GeometryBase> gfa = null;
            List<GeometryBase> gfaCopy = null;

            if (geometryFilter == null)
            {
                if (iterations > 2)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Missing geometry filter input (necessary for > 2 iterations).");
                    return; // Safeguard to prevent crashing when iterations > 2 with no geometry filter
                }

                gfa = null;
                gfaCopy = null;
            }
            else
            {
                // Populate geometry filter array based on scaling by deflation scale factor
                gfa = GetGeoFilterArray(geometryFilter, iterations, centerpln);
                gfaCopy = gfa.GetRange(0, gfa.Count);
            }

            DA.SetDataList(0, gfaCopy);

            // Call the recursive function
            GH_Structure<GH_Plane> outputplns = RecurseInflateGeometry(gfa, filterDistance, includeInterior, centerpln, baseplns, iterations, scale, deflationA6plns, deflationB12plns, deflationF20plns, deflationK30plns);

            // Return result - the tree of output planes for transformation
            DA.SetDataTree(1, outputplns);
        }

        // Recursive function to perform the inflation/deflation process
        public static GH_Structure<GH_Plane> RecurseInflateGeometry(List<GeometryBase> geometryFilterArray, double filterDistance, bool includeInterior, Plane centerpln, GH_Structure<GH_Plane> baseplns, int iterations, double scale, GH_Structure<GH_Plane> deflationA6plns, GH_Structure<GH_Plane> deflationB12plns, GH_Structure<GH_Plane> deflationF20plns, GH_Structure<GH_Plane> deflationK30plns)
        {
            // Base case: all iterations are completed, the baseplns remain unchanged
            if (iterations == 0)
            {
                return baseplns;
            }
            // Otherwise, recurse on the baseplns to produce 4 new lists of planes
            else
            {
                double goldenRatio = (1 + Math.Sqrt(5)) / 2;
                double deflationScaleFactor = Math.Pow(goldenRatio, 3);
                Transform scaleInflate = Transform.Scale(centerpln.Origin, deflationScaleFactor);

                // Scale all baseplns by a factor of golden ratio^3 (position for deflation)
                for (int i = 0; i < 4; i++)
                {
                    GH_Path pth = new GH_Path(i);
                    if (baseplns.Branches[i].Count > 0)
                    {
                        var planes = baseplns.Branches[i];
                        // List<Plane> planes = new List<Plane>(baseplns.Branches[i]);
                        baseplns.RemovePath(pth);

                        for (int j = 0; j < planes.Count; j++)
                        {
                            Plane scaledPlane = new Plane(planes[j].Value);
                            scaledPlane.Transform(scaleInflate);
                            baseplns.Append(new GH_Plane(scaledPlane), pth);
                        }
                    }
                }

                // Perform deflation on the inflated baseplns
                GH_Structure<GH_Plane> inflatedbaseplns = InflateGeometry(centerpln, baseplns, deflationA6plns, deflationB12plns, deflationF20plns, deflationK30plns);

                // Filter inflatedbaseplns by geometryFilterArray[iterations]
                GH_Structure<GH_Plane> filteredbaseplns;
                if (geometryFilterArray == null || geometryFilterArray.Count == 0)
                {
                    filteredbaseplns = inflatedbaseplns;
                }
                else
                {
                    // TODO: test/ensure that mesh geometry filter also works?
                    GeometryBase geometryFilter = geometryFilterArray[iterations - 1];
                    double buffer = scale;
                    if (geometryFilter.HasBrepForm)
                    {
                        Brep brepFilter = Brep.TryConvertBrep(geometryFilter);
                        filteredbaseplns = BrepFilterPlanes(inflatedbaseplns, brepFilter, filterDistance, includeInterior, iterations, buffer);
                    }
                    else
                    {
                        Curve crvFilter = geometryFilter as Curve;
                        filteredbaseplns = CrvFilterPlanes(inflatedbaseplns, crvFilter, filterDistance, iterations, buffer);
                    }
                    geometryFilterArray.RemoveAt(iterations - 1);
                }

                // Remove duplicates from the lists (within tolerance)
                GH_Structure<GH_Plane> culledbaseplns = CullDuplicatePlanes(filteredbaseplns, 0.1 * scale);

                // Decrement iterations, call recursion
                return RecurseInflateGeometry(geometryFilterArray, filterDistance, includeInterior, centerpln, culledbaseplns, iterations - 1, scale, deflationA6plns, deflationB12plns, deflationF20plns, deflationK30plns);
            }
        }


        // Deflation operation - replacement of one set of planes with another, by transforming planes from each of the deflation rules to the input baseplns
        public static GH_Structure<GH_Plane> InflateGeometry(Plane centerpln, GH_Structure<GH_Plane> baseplns, GH_Structure<GH_Plane> deflationA6plns, GH_Structure<GH_Plane> deflationB12plns, GH_Structure<GH_Plane> deflationF20plns, GH_Structure<GH_Plane> deflationK30plns)
        {
            GH_Structure<GH_Plane> inflatedbaseplns = new GH_Structure<GH_Plane>();
            GH_Path pth0 = new GH_Path(0);
            GH_Path pth1 = new GH_Path(1);
            GH_Path pth2 = new GH_Path(2);
            GH_Path pth3 = new GH_Path(3);

            inflatedbaseplns.EnsurePath(pth0);
            inflatedbaseplns.EnsurePath(pth1);
            inflatedbaseplns.EnsurePath(pth2);
            inflatedbaseplns.EnsurePath(pth3);

            // Loop through each branch (each type of unit cell) provided by the baseplns data tree
            for (int i = 0; i < 4; i++)
            {
                // Loop through all unit cells of a specific type (Branch(i))
                var planes = baseplns.Branches[i];
                if (planes.Count > 0)
                {

                    // Get set of planes based on the deflation rule for this tile
                    GH_Structure<GH_Plane> planesToCopy;

                    // Consider which type of tile is being inflated based on i / Branch number of the baseplns
                    switch (i)
                    {
                        case 0:
                            planesToCopy = deflationA6plns;
                            break;
                        case 1:
                            planesToCopy = deflationB12plns;
                            break;
                        case 2:
                            planesToCopy = deflationF20plns;
                            break;
                        case 3:
                            planesToCopy = deflationK30plns;
                            break;
                        default:
                            planesToCopy = deflationA6plns;
                            break;
                    }

                    // Loop through all tiles that need to be inflated (of a specific type)
                    for (int j = 0; j < planes.Count; j++)
                    {
                        if (planes[j].IsValid)
                        {
                            // Get the base transformation from WorldXY to the basepln
                            Transform xcopyPlane = Transform.PlaneToPlane(Plane.WorldXY, planes[j].Value);

                            // Copy all of the A6 tiles into inflatedbaseplns
                            for (int a = 0; a < planesToCopy.Branches[0].Count; a++)
                            {
                                Plane transformedPlane = planesToCopy.Branches[0][a].Value;
                                transformedPlane.Transform(xcopyPlane);
                                inflatedbaseplns.Append(new GH_Plane(transformedPlane), pth0);
                            }
                            // Copy all of the B12 tiles into inflatedbaseplns
                            for (int b = 0; b < planesToCopy.Branches[1].Count; b++)
                            {
                                Plane transformedPlane = planesToCopy.Branches[1][b].Value;
                                transformedPlane.Transform(xcopyPlane);
                                inflatedbaseplns.Append(new GH_Plane(transformedPlane), pth1);
                            }
                            // Copy all of the F20 tiles into inflatedbaseplns
                            for (int f = 0; f < planesToCopy.Branches[2].Count; f++)
                            {
                                Plane transformedPlane = planesToCopy.Branches[2][f].Value;
                                transformedPlane.Transform(xcopyPlane);
                                inflatedbaseplns.Append(new GH_Plane(transformedPlane), pth2);
                            }
                            // Copy all of the K30 tiles into inflatedbaseplns
                            for (int k = 0; k < planesToCopy.Branches[3].Count; k++)
                            {
                                Plane transformedPlane = planesToCopy.Branches[3][k].Value;
                                transformedPlane.Transform(xcopyPlane);
                                inflatedbaseplns.Append(new GH_Plane(transformedPlane), pth3);
                            }
                        }
                    }
                }
            }
            return inflatedbaseplns;
        }
        public static GH_Structure<GH_Plane> BrepFilterPlanes(GH_Structure<GH_Plane> inflatedbaseplns, Brep brepFilter, double filterDistance, bool includeInterior, int iterations, double buffer)
        {
            if (iterations == 1) buffer = 0;

            double goldenRatio = (1 + Math.Sqrt(5)) / 2;
            double deflationScaleFactor = 1 / Math.Pow(goldenRatio, 3);
            double filterDivisionFactor = Math.Pow(deflationScaleFactor, iterations - 1);

            GH_Structure<GH_Plane> filteredbaseplns = new GH_Structure<GH_Plane>();
            // Loop through each branch (each type of unit cell) provided by the baseplns data tree
            for (int i = 0; i < 4; i++)
            {
                GH_Path pth = new GH_Path(i);
                filteredbaseplns.EnsurePath(pth);
                var planes = inflatedbaseplns.Branches[i];
                if (planes.Count > 0)
                {
                    for (int j = 0; j < planes.Count; j++)
                    {
                        Point3d testPoint = planes[j].Value.Origin;
                        if (includeInterior)
                        {
                            if (brepFilter.IsPointInside(testPoint, RhinoMath.SqrtEpsilon, false))
                            {
                                filteredbaseplns.Append(planes[j], pth);
                                continue;
                            }
                        }
                        Point3d closestPoint = new Point3d();
                        ComponentIndex ci;
                        Double s, t;
                        Vector3d normal;
                        brepFilter.ClosestPoint(testPoint, out closestPoint, out ci, out s, out t, filterDistance * filterDivisionFactor + (buffer * 1.5), out normal);
                        //Check closeness but leave a tolerance for possible corners of tiles being near geometry, even if center point (plane origin) is at a distance
                        if (testPoint.DistanceTo(closestPoint) > 0 && testPoint.DistanceTo(closestPoint) < filterDistance * filterDivisionFactor + (buffer * 1.5))
                        {
                            filteredbaseplns.Append(planes[j], pth);
                        }
                    }
                }
            }
            return filteredbaseplns;
        }

        public static GH_Structure<GH_Plane> CrvFilterPlanes(GH_Structure<GH_Plane> inflatedbaseplns, Curve crvFilter, double filterDistance, int iterations, double buffer)
        {
            if (iterations == 1) buffer = 0;

            double goldenRatio = (1 + Math.Sqrt(5)) / 2;
            double deflationScaleFactor = 1 / Math.Pow(goldenRatio, 3);
            double filterDivisionFactor = Math.Pow(deflationScaleFactor, iterations - 1);

            GH_Structure<GH_Plane> filteredbaseplns = new GH_Structure<GH_Plane>();
            // Loop through each branch (each type of unit cell) provided by the baseplns data tree
            for (int i = 0; i < 4; i++)
            {
                GH_Path pth = new GH_Path(i);
                filteredbaseplns.EnsurePath(pth);
                var planes = inflatedbaseplns.Branches[i];
                if (planes.Count > 0)
                {
                    for (int j = 0; j < planes.Count; j++)
                    {
                        GH_Plane planeGH = planes[j];
                        Point3d testPoint = planeGH.Value.Origin;
                        Double t;
                        if (crvFilter.ClosestPoint(testPoint, out t, filterDistance * filterDivisionFactor + (buffer * 1.5)))
                        {
                            filteredbaseplns.Append(planeGH, pth);
                        }
                    }
                }
            }
            return filteredbaseplns;
        }

        public static GH_Structure<GH_Plane> CullDuplicatePlanes(GH_Structure<GH_Plane> filteredbaseplns, double tolerance)
        {
            GH_Structure<GH_Plane> culledbaseplns = new GH_Structure<GH_Plane>();
            // Loop through each branch (each type of unit cell) provided by the baseplns data tree
            for (int i = 0; i < 4; i++)
            {
                GH_Path pth = new GH_Path(i);
                culledbaseplns.EnsurePath(pth);
                var planes = filteredbaseplns.Branches[i];
                if (planes.Count > 0)
                {
                    for (int j = 0; j < planes.Count; j++)
                    {
                        GH_Plane planeGH = planes[j];
                        Point3d testPoint = planeGH.Value.Origin;
                        bool isDup = false;
                        if (culledbaseplns.Branches[i].Count > 0)
                        {
                            for (int k = 0; k < culledbaseplns.Branches[i].Count; k++)
                            {
                                Point3d testPoint2 = culledbaseplns.Branches[i][k].Value.Origin;
                                if (testPoint.DistanceTo(testPoint2) < tolerance) isDup = true;
                            }
                            if (!isDup) culledbaseplns.Append(planeGH, pth);
                        }
                        else culledbaseplns.Append(planeGH, pth);
                    }
                }
            }
            return culledbaseplns;
        }

        public static List<GeometryBase> GetGeoFilterArray(GeometryBase geo, int iterations, Plane centerpln)
        {
            double goldenRatio = (1 + Math.Sqrt(5)) / 2;
            double deflationScaleFactor = 1 / Math.Pow(goldenRatio, 3);
            Transform scaleInflate = Transform.Scale(centerpln.Origin, deflationScaleFactor);

            GeometryBase geoCopy = geo.Duplicate();
            List<GeometryBase> geoFilterArray = new List<GeometryBase>();
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