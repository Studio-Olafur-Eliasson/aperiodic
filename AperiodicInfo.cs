using System;
using System.Drawing;
using Grasshopper;
using Grasshopper.Kernel;

namespace Aperiodic
{
    public class AperiodicInfo : GH_AssemblyInfo
    {
        public override string Name => "Aperiodic";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => Resources.aperiodic2tile24px;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "A plugin for generating three-dimensional aperiodic space-filling structures.";

        public override Guid Id => new Guid("{40cb043b-5fb6-469a-a869-0370408bb339}");

        //Return a string identifying you or your company.
        public override string AuthorName => "Claire Djang";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "claire.djang@olafureliasson.net";

        //Return a string representing the version.  This returns the same version as the assembly.
        public override string AssemblyVersion => GetType().Assembly.GetName().Version.ToString();
    }
}