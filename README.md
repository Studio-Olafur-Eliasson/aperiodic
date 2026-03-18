# Aperiodic

A plugin for generating three-dimensional aperiodic space-filling structures.

## Description

**Aperiodic** generates space-filling, quasicrystal assemblies using golden rhombohedra and zonohedral tiles as base structures. It implements a recursive substitution system that fills bounded volumes, surfaces, or curves with three-dimensional tiles arranged in aperiodic patterns.

This work builds on research on space-filling polyhedra at Studio Olafur Eliasson. Aperiodic tiling systems, five-fold symmetry and golden rhombohedral assemblies have been investigated by Einar Thorsteinn and other collaborators at the studio for decades. The Advanced Geometry group at Studio Olafur Eliasson continues this research. This tool offers a computational implementation of the geometric systems explored conceptually at the studio for decades.

The plugin outputs transformation data for each tile instance, enabling efficient substitution with custom geometry and supporting designs that scale to over 100,000 discrete elements. By enabling the generation of non-periodic spatial systems directly within Rhino/Grasshopper, the plugin provides a new framework for exploring quasicrystalline structures in architectural and design workflows.

## Installation

1. Download the version matching your Rhino installation (Rhino 7 or Rhino 8).
2. Extract the zip file.
3. Copy the `.gha` file to the Grasshopper Libraries folder: C:\Users\[username]\AppData\Roaming\Grasshopper\Libraries
4. (Windows users): Right click > Properties, check Unblock
5. Restart Rhino and Grasshopper.

An example Grasshopper file is included to demonstrate basic usage.
Note: Assemblies are generated at a fixed scale in the current version. More flexible scaling options will be implemented in a future release.

## Research References

The concepts and methods implemented in this plugin are described in the following publications:

Claire Djang and Stefano Arrighi.
“Aperiodic Space-Filling Geometry as a Spatial Logic.”
Advances in Architectural Geometry (AAG 2025), MIT.
www.dropbox.com/scl/fi/o5vbqlep1y3zr7beyocgn/18.pdf?rlkey=ks78vsoy55wqq3fqjtz9ujhic&st=bxkx5y1i&dl=0

Claire Djang and Stefano Arrighi.
“Constructing Three-Dimensional Quasicrystalline Assemblies.”
Proceedings of the IASS Annual Symposium 2025.
https://www.ingentaconnect.com/contentone/iass/piass/2025/00002025/00000007/art00008

## Algorithm Reference

The substitution rules implemented in this plugin are based on the following work:

Alexey E. Madison. *Substitution rules for icosahedral quasicrystals.*  
RSC Advances 5, 5745–5753 (2015).  
https://doi.org/10.1039/C4RA09524C

## Video

An animation explaining the geometric concepts behind the tiling system:

Claire Djang.
"Quasicrystals in an Age of Digital Design"
Bridges 2023 Short Film Festival
https://gallery.bridgesmathart.org/exhibitions/2023-bridges-conference-short-film-festival/claire-djang

## Team

Claire Djang - software design and implementation

Stefano Arrighi - research collaboration

Developed at Studio Olafur Eliasson (Advanced Geometries)

## License

See LICENSE.txt for license information.
