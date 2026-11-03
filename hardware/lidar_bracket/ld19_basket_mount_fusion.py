"""LD19 (D500 kit) LiDAR mount, hung upside down under the middle RASKOG basket, over the drive axle.

Run as a Fusion script (Utilities > Scripts and Add-Ins, or the Fusion MCP). It builds
components "LD19 Lower Plate" and "LD19 Upper Plate" in the active design. Edit the
constants below and rerun.

Coordinates (mm): origin on the LD19's rotation axis at the basket's underside,
+x toward the robot's front, +z up.

The drive axle is ahead of the basket, so the lower plate is an arm: its rear part
clamps the basket floor and its front part reaches out past the basket's front wall,
with the LD19 hanging from its tip. LD19_AHEAD sets how far the LD19's axis is ahead
of the wall; set it to the axle's distance and the LD19 sits over the axle.

How it holds:
  - The basket floor is clamped between two plates.
    - The lower plate is under the basket, and the LD19 hangs from it upside down.
      Its 3 lugs sit on locating bosses that fit their counterbores, held with
      3x M2x10 machine screws, up through the lug into CNC Kitchen M2 heat-set
      inserts pressed in flush from the plate's top face. The LD19's weight pulls
      each insert against the step at the bottom of its hole.
    - The upper plate is inside the basket.
  - 4x M3 bolts go down through both plates and the mesh, into nuts held in hex
    channels under the lower plate.
  - The bolt slots are angled across the mesh grid. Slide each bolt along its slot
    until it drops through an opening; the nut channel stops the nut turning
    wherever that is.
  - The connector faces the rear. The cable runs back under the arm and up through
    the cable hole, which is under the flat floor, to the D500 adapter board inside
    the basket.
  - The scan plane is SCAN_BELOW_MOUNT below the basket underside: plate plus the
    LD19's 27 mm. That is the 0.033 m scan_below_mount in
    ros2_xlerobot_2wheels robot_dimensions.yaml.
"""

import adsk.core, adsk.fusion, math

# --- Basket mesh. TODO measure on the cart ---
MESH_T = 1.5            # thickness of the basket floor between the plates
MESH_PITCH = 10.0       # only sets the slot length, which must cover >= ~2 pitches

# --- Cart ---
LD19_AHEAD = 20.0       # drive axle centre ahead of the middle basket's front wall (outside face); about 20 (2026-11-01), check with a plumb line
FLOOR_INSET = 15.0      # TODO measure: outside of the front wall back to where the flat mesh floor starts (wall plus the corner bend)

# --- Plates ---
FLOOR_EDGE = -LD19_AHEAD - FLOOR_INSET  # front of the flat floor in plate coordinates; clamping happens behind it
ARM_X = (FLOOR_EDGE - 10.0, 28.0)       # the arm out to the LD19, front edge just past its front lug
ARM_Y = (-30.0, 30.0)
CLAMP_X = (FLOOR_EDGE - 88.0, FLOOR_EDGE)  # the part under the floor, clamped by both plates
CLAMP_Y = (-52.0, 52.0)
CORNER_R = 6.0
LOWER_T = 6.0           # LD19 base to basket underside; with the LD19's 27 mm gives the 33 mm scan offset
UPPER_T = 4.0

# --- Clamp bolts, M3 ---
BOLT_D = 3.4
NUT_AF = 5.8            # M3 nut across flats plus clearance
NUT_DEPTH = 2.6
SLOT_ANGLE = math.degrees(math.atan2(1, 2))  # 26.6 deg: never parallel to a square grid's rows or diagonals
SLOT_L = max(24.0, 2.3 * MESH_PITCH)         # centre to centre of the slot ends
_SX = (FLOOR_EDGE - 16.0, FLOOR_EDGE - 72.0)
SLOTS = [(_SX[0], 38.0), (_SX[1], 38.0), (_SX[0], -38.0), (_SX[1], -38.0)]  # centres; +y slots at +angle, -y at -angle

# --- LD19 ---
LD19_SCAN = 27.0        # base to scan plane
# Holes relative to the rotation axis, seen from above: connector toward -x.
LD19_HOLES = [(-8.50, -23.40), (-8.50, 23.40), (23.42, 0.0)]
BOSS_D = 4.2            # fits the 4.46 counterbores
BOSS_H = 1.8
INSERT_D = 3.2          # CNC Kitchen M2 insert (M2x3, 3.6 OD): their recommended 3.2 hole
INSERT_DEPTH = 4.0      # their minimum, insert 3 long pressed flush with the top face
CLEAR_D = 2.2           # M2 clearance through the boss and the rest of the plate
CABLE_HOLE = (FLOOR_EDGE - 13.0, FLOOR_EDGE - 3.0, 7.0)  # x from, x to, half width in y; under the flat floor, between the slots


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent

    def P(x, y, z):
        return adsk.core.Point3D.create(x / 10, y / 10, z / 10)

    def mm(v):
        return adsk.core.ValueInput.createByString(f'{v} mm')

    Z = adsk.core.Vector3D.create(0, 0, 1)
    NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
    CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

    def new_comp(name):
        comp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
        comp.name = name
        return comp

    def sketch_at(comp, z, name):
        inp = comp.constructionPlanes.createInput()
        inp.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(z / 10))
        sk = comp.sketches.add(comp.constructionPlanes.add(inp))
        sk.name = name
        return sk

    def up(sk):
        # +1 if the sketch normal points up in the model, else -1.
        return 1 if sk.xDirection.crossProduct(sk.yDirection).dotProduct(Z) > 0 else -1

    def extrude(comp, sk, profiles, dist, op, bodies=None, name=None):
        coll = adsk.core.ObjectCollection.create()
        for p in profiles:
            coll.add(p)
        ei = comp.features.extrudeFeatures.createInput(coll, op)
        ei.setDistanceExtent(False, mm(dist * up(sk)))
        if bodies is not None:
            ei.participantBodies = bodies
        f = comp.features.extrudeFeatures.add(ei)
        if name:
            f.name = name
        return f

    def plate_outline(sk, z, xs, ys):
        x0, x1 = xs
        y0, y1 = ys
        r = CORNER_R
        lines = sk.sketchCurves.sketchLines
        arcs = sk.sketchCurves.sketchArcs
        S = lambda x, y: sk.modelToSketchSpace(P(x, y, z))
        lines.addByTwoPoints(S(x0 + r, y0), S(x1 - r, y0))
        lines.addByTwoPoints(S(x1, y0 + r), S(x1, y1 - r))
        lines.addByTwoPoints(S(x1 - r, y1), S(x0 + r, y1))
        lines.addByTwoPoints(S(x0, y1 - r), S(x0, y0 + r))
        for cx, cy, a0 in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
            a = math.radians(a0)
            arcs.addByCenterStartSweep(S(cx, cy), S(cx + r * math.cos(a), cy + r * math.sin(a)), math.pi / 2)

    def slot(sk, z, cx, cy, ang_deg, length, width):
        # A stadium: rectangle plus two end circles. Cut every profile it makes.
        a = math.radians(ang_deg)
        ux, uy = math.cos(a), math.sin(a)
        nx, ny = -uy, ux
        h, w = length / 2, width / 2
        S = lambda x, y: sk.modelToSketchSpace(P(x, y, z))
        pts = [(cx - ux * h + nx * w, cy - uy * h + ny * w), (cx + ux * h + nx * w, cy + uy * h + ny * w),
               (cx + ux * h - nx * w, cy + uy * h - ny * w), (cx - ux * h - nx * w, cy - uy * h - ny * w)]
        lines = sk.sketchCurves.sketchLines
        for p0, p1 in zip(pts, pts[1:] + pts[:1]):
            lines.addByTwoPoints(S(*p0), S(*p1))
        for s in (-1, 1):
            sk.sketchCurves.sketchCircles.addByCenterRadius(S(cx + s * ux * h, cy + s * uy * h), w / 10)

    def slots(sk, z, width):
        for cx, cy in SLOTS:
            slot(sk, z, cx, cy, SLOT_ANGLE if cy > 0 else -SLOT_ANGLE, SLOT_L, width)

    def cable_hole(sk, z):
        x0, x1, hw = CABLE_HOLE
        S = lambda x, y: sk.modelToSketchSpace(P(x, y, z))
        sk.sketchCurves.sketchLines.addTwoPointRectangle(S(x0, -hw), S(x1, hw))

    # --- Lower plate ---
    low = new_comp('LD19 Lower Plate')
    sk = sketch_at(low, -LOWER_T, 'lower plate')
    plate_outline(sk, -LOWER_T, CLAMP_X, CLAMP_Y)
    f = extrude(low, sk, [sk.profiles.item(0)], LOWER_T, NEW, name='Lower plate')
    lower = f.bodies.item(0)
    lower.name = 'LD19 Lower Plate'
    sk = sketch_at(low, -LOWER_T, 'arm')
    plate_outline(sk, -LOWER_T, ARM_X, ARM_Y)
    extrude(low, sk, [sk.profiles.item(0)], LOWER_T, JOIN, [lower], 'Arm')

    sk = sketch_at(low, -LOWER_T, 'bosses')
    for hx, hy in LD19_HOLES:
        sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(hx, hy, -LOWER_T)), BOSS_D / 20)
    extrude(low, sk, list(sk.profiles), -BOSS_H, JOIN, [lower], 'Bosses')

    sk = sketch_at(low, -LOWER_T - BOSS_H, 'screw clearance')
    for hx, hy in LD19_HOLES:
        sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(hx, hy, -LOWER_T - BOSS_H)), CLEAR_D / 20)
    extrude(low, sk, list(sk.profiles), LOWER_T + BOSS_H, CUT, [lower], 'M2 clearance')

    sk = sketch_at(low, 0, 'inserts')
    for hx, hy in LD19_HOLES:
        sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(hx, hy, 0)), INSERT_D / 20)
    extrude(low, sk, list(sk.profiles), -INSERT_DEPTH, CUT, [lower], 'M2 inserts')

    sk = sketch_at(low, 0, 'bolt slots')
    slots(sk, 0, BOLT_D)
    extrude(low, sk, list(sk.profiles), -LOWER_T, CUT, [lower], 'Bolt slots')

    sk = sketch_at(low, -LOWER_T, 'nut channels')
    slots(sk, -LOWER_T, NUT_AF)
    extrude(low, sk, list(sk.profiles), NUT_DEPTH, CUT, [lower], 'Nut channels')

    sk = sketch_at(low, 0, 'cable hole')
    cable_hole(sk, 0)
    extrude(low, sk, [sk.profiles.item(0)], -LOWER_T, CUT, [lower], 'Cable hole')

    # --- Upper plate ---
    upp = new_comp('LD19 Upper Plate')
    z0 = MESH_T
    sk = sketch_at(upp, z0, 'upper plate')
    plate_outline(sk, z0, CLAMP_X, CLAMP_Y)
    f = extrude(upp, sk, [sk.profiles.item(0)], UPPER_T, NEW, name='Upper plate')
    upper = f.bodies.item(0)
    upper.name = 'LD19 Upper Plate'

    sk = sketch_at(upp, z0 + UPPER_T, 'bolt slots')
    slots(sk, z0 + UPPER_T, BOLT_D)
    extrude(upp, sk, list(sk.profiles), -UPPER_T, CUT, [upper], 'Bolt slots')

    if CABLE_HOLE[0] < CLAMP_X[1]:
        # The cable hole reaches under the basket floor, so the upper plate needs it too.
        sk = sketch_at(upp, z0 + UPPER_T, 'cable hole')
        cable_hole(sk, z0 + UPPER_T)
        extrude(upp, sk, list(sk.profiles), -UPPER_T, CUT, [upper], 'Cable hole')

    for comp in (low, upp):
        for cp in comp.constructionPlanes:
            cp.isLightBulbOn = False
    print(f'scan plane {LOWER_T + LD19_SCAN:.1f} mm below the basket underside; LD19 head bottom '
          f'{LOWER_T + 34.8:.1f} mm below it; bolts M3x{math.ceil(UPPER_T + MESH_T + LOWER_T)} or the next size up')
    for b in (lower, upper):
        print(b.name, f'{b.volume * 1000:.0f} mm3')
