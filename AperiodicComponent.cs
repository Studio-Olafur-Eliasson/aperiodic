using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;

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
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => null;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("{7693d4a5-60af-4d28-a69d-739af538a058}");
    }
}