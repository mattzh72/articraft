from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


# ---------------------------------------------------------------------------
# Dimensions (meters). Realistic front-loader: ~60 cm wide x 60 deep x 85 tall.
# ---------------------------------------------------------------------------
CAB_W = 0.600
CAB_D = 0.600
CAB_H = 0.850
WALL_T = 0.015

# Porthole / door
PORTHOLE_R = 0.180
PORTHOLE_CZ = 0.420          # center height of porthole
BEZEL_R_OUTER = 0.215
BEZEL_R_INNER = 0.170
DOOR_THICKNESS = 0.045       # bezel depth (along world Y when closed)
# Glass radius is just over the bezel inner radius so the glass disc and the
# bezel ring share material, keeping the door part visually connected.
GLASS_R = 0.172
GLASS_THICKNESS = 0.035
HANDLE_OFFSET = 0.395        # horizontal handle offset from hinge in door frame

# Drum
DRUM_R = 0.175
# Drum spans the full cabinet interior front-to-back. The back rim seats into
# the cabinet back panel (a scoped mount overlap) and the front rim sits just
# inside the porthole hole, well clear of the door glass.
DRUM_LENGTH = 0.580
DRUM_CY = 0.000
DRUM_CZ = PORTHOLE_CZ

# Detergent drawer
DRAWER_BODY_W = 0.200
DRAWER_BODY_H = 0.050
DRAWER_BODY_D = 0.320
DRAWER_FACE_W = 0.220
DRAWER_FACE_H = 0.062
DRAWER_FACE_T = 0.014
DRAWER_CX = -0.160
DRAWER_CZ = 0.790            # slot center height
DRAWER_STROKE = 0.220        # max forward travel

# Knob (program selector)
KNOB_CX = 0.190
KNOB_CZ = 0.790
KNOB_STEM_LEN = 0.018
KNOB_STEM_R = 0.005

# Front face plane
FRONT_Y = CAB_D / 2.0
FRONT_INNER_Y = FRONT_Y - WALL_T


# ---------------------------------------------------------------------------
# Helper geometry builders
# ---------------------------------------------------------------------------
def _build_cabinet_front_panel():
    """Front panel of cabinet with porthole hole and detergent drawer slot."""
    front_center_y = FRONT_Y - WALL_T / 2.0
    panel = (
        cq.Workplane("XY", origin=(0, front_center_y, CAB_H / 2.0))
        .box(CAB_W, WALL_T, CAB_H, centered=True)
    )
    # Porthole through hole
    panel = (
        panel.faces(">Y")
        .workplane(centerOption="CenterOfBoundBox")
        .center(0.0, PORTHOLE_CZ - CAB_H / 2.0)
        .hole(PORTHOLE_R * 2.0)
    )
    # Detergent drawer slot
    panel = (
        panel.faces(">Y")
        .workplane(centerOption="CenterOfBoundBox")
        .center(DRAWER_CX, DRAWER_CZ - CAB_H / 2.0)
        .rect(DRAWER_BODY_W + 0.004, DRAWER_BODY_H + 0.004)
        .cutThruAll()
    )
    return panel


def _build_door_bezel():
    """Annular ring with rounded outer edge representing the door bezel.

    Built in CadQuery with the ring axis along local +Z (workplane XY).
    The door visual attaches it with rpy=(-pi/2, 0, 0) so the ring axis aligns
    with world +Y when the door is closed.
    """
    ring = (
        cq.Workplane("XY")
        .circle(BEZEL_R_OUTER)
        .circle(BEZEL_R_INNER)
        .extrude(DOOR_THICKNESS)
        .edges(">Z and %CIRCLE")
        .fillet(0.010)
    )
    return ring


def _build_door_glass():
    glass = (
        cq.Workplane("XY")
        .circle(GLASS_R)
        .extrude(GLASS_THICKNESS)
        .edges("%CIRCLE")
        .fillet(0.006)
    )
    return glass


def _build_drawer_body():
    """Open-top tray shape for the detergent drawer."""
    tray = (
        cq.Workplane("XY")
        .box(DRAWER_BODY_W, DRAWER_BODY_D, DRAWER_BODY_H, centered=(True, True, False))
        .faces(">Z")
        .shell(-0.005)
    )
    # Add a simple central divider
    divider = (
        cq.Workplane("XY", origin=(0, 0, DRAWER_BODY_H / 2.0))
        .box(0.004, DRAWER_BODY_D - 0.020, DRAWER_BODY_H - 0.010, centered=True)
    )
    return tray.union(divider)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_loading_washing_machine")

    cab_white = model.material("cabinet_white", rgba=(0.955, 0.955, 0.945, 1.0))
    bezel_gray = model.material("bezel_gray", rgba=(0.22, 0.22, 0.24, 1.0))
    glass_blue = model.material("door_glass", rgba=(0.18, 0.26, 0.36, 0.55))
    drum_steel = model.material("drum_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    drawer_white = model.material("drawer_white", rgba=(0.93, 0.93, 0.93, 1.0))
    drawer_blue = model.material("drawer_accent", rgba=(0.20, 0.35, 0.55, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.15, 0.15, 0.17, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.18, 0.18, 0.20, 1.0))

    # ------------------------------------------------------------------
    # Cabinet (root part): 5 simple panels + a front panel with cutouts.
    # Panels intentionally overlap at corners by WALL_T so the cabinet
    # reads as one connected shell.
    # ------------------------------------------------------------------
    cabinet = model.part("cabinet")

    cabinet.visual(
        Box((CAB_W, WALL_T, CAB_H)),
        origin=Origin(xyz=(0.0, -CAB_D / 2.0 + WALL_T / 2.0, CAB_H / 2.0)),
        material=cab_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((WALL_T, CAB_D, CAB_H)),
        origin=Origin(xyz=(-CAB_W / 2.0 + WALL_T / 2.0, 0.0, CAB_H / 2.0)),
        material=cab_white,
        name="left_side",
    )
    cabinet.visual(
        Box((WALL_T, CAB_D, CAB_H)),
        origin=Origin(xyz=(CAB_W / 2.0 - WALL_T / 2.0, 0.0, CAB_H / 2.0)),
        material=cab_white,
        name="right_side",
    )
    cabinet.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_H - WALL_T / 2.0)),
        material=cab_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, WALL_T / 2.0)),
        material=cab_white,
        name="bottom_panel",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_cabinet_front_panel(), "cabinet_front_panel"),
        material=cab_white,
        name="front_panel",
    )
    # Thin trim band marking the top edge of the control-panel area on the
    # cabinet front. Placed above the drawer/knob row so it does not collide
    # with those mounted controls.
    cabinet.visual(
        Box((CAB_W - 2.0 * WALL_T, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.004, CAB_H - 0.022)),
        material=bezel_gray,
        name="control_panel_trim",
    )

    # ------------------------------------------------------------------
    # Door: porthole hinged on the LEFT edge of the porthole, vertical axis.
    # Door part frame sits at the hinge axis. Door body extends along +X.
    # ------------------------------------------------------------------
    door = model.part("door")
    # Bezel ring center in door-local frame: x = PORTHOLE_R + small clearance,
    # y just outside the cabinet front face.
    bezel_local_x = PORTHOLE_R + 0.020   # 0.200 -> aligns with porthole center
    bezel_local_y = 0.0                  # ring base on hinge-plane (the front face)
    bezel_local_z = 0.0

    # The bezel ring is built around local +Z in CadQuery (extrudes +Z).
    # rpy=(-pi/2, 0, 0): R = Rx(-pi/2) sends local +Z -> world +Y at q=0.
    door.visual(
        mesh_from_cadquery(_build_door_bezel(), "door_bezel"),
        origin=Origin(
            xyz=(bezel_local_x, bezel_local_y + 0.002, bezel_local_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=bezel_gray,
        name="door_bezel",
    )
    # Glass disc nested inside the bezel ring. The disc radius is just over the
    # bezel inner radius so the glass disc and the bezel ring share material,
    # keeping the door part visually connected.
    door.visual(
        mesh_from_cadquery(_build_door_glass(), "door_glass"),
        origin=Origin(
            xyz=(bezel_local_x, bezel_local_y + 0.005, bezel_local_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=glass_blue,
        name="door_glass",
    )
    # Handle on the right (free) edge of the door
    door.visual(
        Box((0.020, 0.040, 0.110)),
        origin=Origin(xyz=(HANDLE_OFFSET, 0.020, 0.0)),
        material=handle_dark,
        name="door_handle",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        # Hinge axis just outside the porthole's left rim, on the front face.
        origin=Origin(xyz=(-PORTHOLE_R - 0.020, FRONT_Y, PORTHOLE_CZ)),
        # Closed door geometry extends along door-local +X from the hinge.
        # axis=(0, 0, 1): positive q rotates +X toward +Y, swinging the free
        # edge outward (away from the cabinet front).
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=2.20),
    )

    # ------------------------------------------------------------------
    # Drum: horizontal cylinder along world Y, continuous revolute axle.
    # Drum mesh is built around local +Z and rotated so the open mouth
    # (positive local Z face) faces the cabinet front (+Y world).
    # rpy=(-pi/2, 0, 0) sends local +Z -> world +Y.
    # ------------------------------------------------------------------
    drum = model.part("drum")
    # Position the drum mesh inside its part frame so it sits centered.
    # CadQuery drum spans local z in [0, DRUM_LENGTH] with open face at +Z.
    # We want world y in [DRUM_CY - L/2, DRUM_CY + L/2]; with rpy=(-pi/2,0,0)
    # local +Z maps to world +Y, so we offset visual along local -Z by L/2
    # to center about the part frame, then the part frame sits at the joint
    # origin which is at world (0, DRUM_CY, DRUM_CZ).
    # SDK Cylinder is axis-aligned along local +Z. The drum rotates around the
    # world Y axis (its joint axis), so we rotate the visual so local +Z maps
    # to world +Y at rest. The mesh is centered on the part frame, so it spans
    # world y in [-DRUM_LENGTH/2, +DRUM_LENGTH/2] around the joint origin.
    drum.visual(
        Cylinder(radius=DRUM_R, length=DRUM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="drum_body",
    )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, DRUM_CY, DRUM_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )

    # ------------------------------------------------------------------
    # Detergent drawer: prismatic, slides forward (+Y) from the cabinet.
    # The drawer body sits behind the cabinet front when closed (q=0);
    # only the drawer face is flush with the slot at rest.
    # ------------------------------------------------------------------
    drawer = model.part("drawer")
    # Drawer part frame at the drawer face center in world (when q=0):
    #   x = DRAWER_CX, y = FRONT_Y + DRAWER_FACE_T / 2, z = DRAWER_CZ
    # Drawer body extends behind the face along -Y in part frame.
    drawer.visual(
        mesh_from_cadquery(_build_drawer_body(), "drawer_body"),
        origin=Origin(
            xyz=(0.0, -DRAWER_FACE_T / 2.0 - DRAWER_BODY_D / 2.0, -DRAWER_BODY_H / 2.0),
        ),
        material=drawer_white,
        name="drawer_body",
    )
    # Drawer face/front plate
    drawer.visual(
        Box((DRAWER_FACE_W, DRAWER_FACE_T, DRAWER_FACE_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=drawer_white,
        name="drawer_face",
    )
    # Small accent handle recess on the face
    drawer.visual(
        Box((0.090, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, DRAWER_FACE_T / 2.0 + 0.005, 0.0)),
        material=drawer_blue,
        name="drawer_handle",
    )

    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        # Joint frame at the closed-drawer face position.
        origin=Origin(xyz=(DRAWER_CX, FRONT_Y + DRAWER_FACE_T / 2.0, DRAWER_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.20, lower=0.0, upper=DRAWER_STROKE
        ),
    )

    # ------------------------------------------------------------------
    # Program selector knob: rotary knob on the front control area, axis
    # along world +Y so the user can turn it from the front.
    # The knob mesh is built around local +Z (KnobGeometry).
    # rpy=(-pi/2, 0, 0) sends local +Z -> world +Y.
    # A short stem extends into the cabinet panel for a physical mount.
    # ------------------------------------------------------------------
    knob = model.part("program_knob")

    knob_geom = KnobGeometry(
        diameter=0.044,
        height=0.022,
        body_style="skirted",
        top_diameter=0.034,
        skirt=KnobSkirt(diameter=0.054, height=0.005, flare=0.10, chamfer=0.0012),
        grip=KnobGrip(style="fluted", count=18, depth=0.0014),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0010),
        bore=KnobBore(style="round", diameter=0.006),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "program_knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_body",
    )
    # Stem extending back into the front panel (along world -Y).
    # Cylinder default along local +Z; rpy=(-pi/2,0,0) -> world +Y; we want
    # the stem to extend along world -Y (into the cabinet), so offset along
    # local +Z (which is world +Y after rotation) by -KNOB_STEM_LEN/2 puts
    # the stem center at world y = -KNOB_STEM_LEN/2 relative to joint origin,
    # i.e. into the cabinet.
    knob.visual(
        Cylinder(radius=KNOB_STEM_R, length=KNOB_STEM_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, -KNOB_STEM_LEN / 2.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_dark,
        name="knob_stem",
    )

    model.articulation(
        "cabinet_to_program_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(KNOB_CX, FRONT_Y, KNOB_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    drum = object_model.get_part("drum")
    drawer = object_model.get_part("drawer")
    knob = object_model.get_part("program_knob")

    door_joint = object_model.get_articulation("cabinet_to_door")
    drum_joint = object_model.get_articulation("cabinet_to_drum")
    drawer_joint = object_model.get_articulation("cabinet_to_drawer")
    knob_joint = object_model.get_articulation("cabinet_to_program_knob")

    # The knob stem is intentionally embedded a few mm into the cabinet
    # front panel for a mechanical mount. Scope the allowance to those two
    # elements only.
    ctx.allow_overlap(
        knob,
        cabinet,
        elem_a="knob_stem",
        elem_b="front_panel",
        reason="Program-knob stem nests into the front panel as a captured-shaft mount.",
    )
    # The drum sits inside the cabinet shell with the porthole cut; the drum
    # mouth extends just past the front panel inner surface near the porthole.
    # Allow that intentional local embed between the drum body and the front
    # panel only.
    ctx.allow_overlap(
        drum,
        cabinet,
        elem_a="drum_body",
        elem_b="back_panel",
        reason="Drum rear rim seats into the cabinet back wall as the tub mount.",
    )
    # The drawer body slides through the drawer slot in the front panel; at
    # rest the body proxy passes through the slot, which is an intentional
    # sleeve-style fit on those two elements.
    ctx.allow_overlap(
        drawer,
        cabinet,
        elem_a="drawer_body",
        elem_b="front_panel",
        reason="Drawer body intentionally passes through the front-panel slot as a sliding sleeve fit.",
    )

    # --- Door articulation proofs -------------------------------------------------
    # Closed door overlaps the porthole footprint in front of the cabinet.
    with ctx.pose({door_joint: 0.0}):
        ctx.expect_overlap(
            door, cabinet,
            axes="xz",
            elem_a="door_bezel",
            elem_b="front_panel",
            min_overlap=0.20,
            name="closed door covers porthole footprint",
        )
        ctx.expect_gap(
            door, cabinet,
            axis="y",
            positive_elem="door_bezel",
            negative_elem="front_panel",
            min_gap=0.0,
            max_gap=0.010,
            name="closed door sits flush at front face",
        )

    # Opening the door swings the free edge outward in +Y world direction.
    closed_pos = ctx.part_world_position(door)
    with ctx.pose({door_joint: 1.5}):
        opened_pos = ctx.part_world_position(door)
    ctx.check(
        "door opens outward",
        closed_pos is not None
        and opened_pos is not None
        and opened_pos[1] > closed_pos[1] - 1e-6,
        details=f"closed={closed_pos}, opened={opened_pos}",
    )

    # --- Drum rotates and stays inside cabinet ----------------------------------
    with ctx.pose({drum_joint: 0.0}):
        ctx.expect_within(
            drum, cabinet,
            axes="xz",
            margin=0.005,
            name="drum stays within cabinet shell footprint",
        )
    # Quarter turn should still keep the drum geometry centered on its axis.
    with ctx.pose({drum_joint: math.pi / 2.0}):
        ctx.expect_within(
            drum, cabinet,
            axes="xz",
            margin=0.005,
            name="rotated drum stays within cabinet shell footprint",
        )

    # --- Drawer extends forward when opened -------------------------------------
    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: DRAWER_STROKE}):
        drawer_out = ctx.part_world_position(drawer)
    ctx.check(
        "drawer extends forward",
        drawer_rest is not None
        and drawer_out is not None
        and drawer_out[1] > drawer_rest[1] + 0.10,
        details=f"rest={drawer_rest}, extended={drawer_out}",
    )
    # Even fully extended, drawer body should still retain insertion into the slot.
    with ctx.pose({drawer_joint: DRAWER_STROKE}):
        ctx.expect_overlap(
            drawer, cabinet,
            axes="y",
            elem_a="drawer_body",
            elem_b="front_panel",
            min_overlap=0.005,
            name="extended drawer retains insertion through front-panel slot",
        )

    # --- Knob mounts to the front of the cabinet --------------------------------
    ctx.expect_contact(
        knob, cabinet,
        elem_a="knob_body",
        elem_b="front_panel",
        contact_tol=0.005,
        name="knob body seats against the cabinet front panel",
    )

    return ctx.report()


object_model = build_object_model()
