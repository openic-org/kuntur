Layer Stackup
-------------

HDI: 2+N+2

   (TOP)
   F.Silkscreen - Top Silkscreen
   F.Paste - Top Solder Paste
   F.Mask - Top Solder Mask
1. F.Cu - Top Layer
2. L2.Cu - Layer 2
3. L3.Cu - Layer 3
4. L4.Cu - Layer 4
5. L5.Cu - Layer 5
6. L6.Cu - Layer 6
7. L7.Cu - Layer 7
8. B.Cu  - Bottom Layer
   B.Mask - Bottom Solder Mask
   B.Paste - Bottom Solder Paste
   B.Silkscreen - Bottom Silkscreen
   (BOTTOM)

Vias
----
- Mechanical Drill Via
  - Through Hole Via = L1-L8
  - Blind Via = L3 through L6
- Laser Via
  - All MicroVias = L1-L2, L1-L3, L2-L3, L6-L7, L6-L8, and L7-L8

Board Outline
-----------
- Gerber File = Edge.Cuts

Board Specs
-----------
- # of Layers = 8
- Trace W/S =  3/3 mil
- HDI = Yes
- uVia Diameter/Hole = 8.85/4.5 mil
- Via in Pad = Yes
- Stacked Vias = Yes
- Transmission lines =  Yes at 2.4 GHz
- Copper Weights = 0.5 or 1 oz
- Thickness <= 1 mm
- Surface Finish = ENIG

Component Placement
-------------------
- F.Fab = Front side components
- B.Fab = Back side components
- all-pos.csv = Pick-and-Place file