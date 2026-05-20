from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


# Real-world proportions for a recessed wall safe, in meters.
FRAME_W = 0.68
FRAME_H = 0.86
FRAME_T = 0.045
BODY_W = 0.54
BODY_H = 0.72
BODY_D = 0.32
BODY_WALL = 0.040

DOOR_W = 0.48
DOOR_H = 0.62
DOOR_T = 0.052
HINGE_X = DOOR_W / 2.0
HINGE_Y = FRAME_T + 0.010
HINGE_AXIS_Y = HINGE_Y + 0.030
DOOR_CENTER_X = -DOOR_W / 2.0 - 0.010
DOOR_LOCAL_Y = HINGE_Y - HINGE_AXIS_Y

DIAL_X = DOOR_CENTER_X
DIAL_Z = 0.155
DIAL_D = 0.135
DIAL_DEPTH = 0.032

FLAP_W = 0.280
FLAP_H = 0.105
FLAP_T = 0.014
FLAP_TOP_Z = -0.035
FLAP_X = DOOR_CENTER_X

HANDLE_Z = -0.205


def _safe_body_shell() -> cq.Workplane:
    """Hollow recessed steel box, open at the front behind the door."""

    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, -BODY_D / 2.0, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * BODY_WALL, BODY_D - BODY_WALL * 0.65, BODY_H - 2.0 * BODY_WALL)
        .translate((0.0, -(BODY_D - BODY_WALL * 0.65) / 2.0 + 0.010, 0.0))
    )
    return outer.cut(inner).edges("|Y").fillet(0.006)


def _front_flange() -> cq.Workplane:
    """One continuous broad front flange with a central opening."""

    outer = (
        cq.Workplane("XY")
        .box(FRAME_W, FRAME_T, FRAME_H)
        .translate((0.0, FRAME_T / 2.0, 0.0))
    )
    opening = (
        cq.Workplane("XY")
        .box(BODY_W, FRAME_T + 0.010, BODY_H)
        .translate((0.0, FRAME_T / 2.0, 0.0))
    )
    return outer.cut(opening).edges("|Y").fillet(0.008)


def _door_panel() -> cq.Workplane:
    """Thick safe door with a through-slot hidden behind the deposit flap."""

    slot_center_z = FLAP_TOP_Z - FLAP_H * 0.52
    base = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((DOOR_CENTER_X, DOOR_LOCAL_Y + DOOR_T / 2.0, 0.0))
    )
    slot_cut = (
        cq.Workplane("XY")
        .box(FLAP_W * 0.84, DOOR_T + 0.012, FLAP_H * 0.56)
        .translate((FLAP_X, DOOR_LOCAL_Y + DOOR_T / 2.0, slot_center_z))
    )
    return base.cut(slot_cut).edges("|Y").fillet(0.005)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_safe")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    blackened = model.material("blackened_edges", rgba=(0.015, 0.016, 0.018, 1.0))
    door_paint = model.material("door_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    brushed = model.material("brushed_metal", rgba=(0.70, 0.68, 0.62, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    red = model.material("red_index", rgba=(0.80, 0.05, 0.035, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_safe_body_shell(), "hollow_safe_body"),
        material=dark_steel,
        name="hollow_body",
    )
    frame.visual(
        mesh_from_cadquery(_front_flange(), "broad_front_flange"),
        material=dark_steel,
        name="front_flange",
    )
    # A dark back plate is visible when the main door is swung open.
    frame.visual(
        Box((BODY_W - 2.0 * BODY_WALL, 0.010, BODY_H - 2.0 * BODY_WALL)),
        origin=Origin(xyz=(0.0, -BODY_D + BODY_WALL + 0.001, 0.0)),
        material=blackened,
        name="interior_back",
    )
    # Fixed hinge-side leaves and a captured pin mounted to the right side of
    # the front flange. The moving door barrels occupy the alternating spaces.
    frame.visual(
        Cylinder(radius=0.006, length=0.620),
        origin=Origin(xyz=(HINGE_X, HINGE_AXIS_Y, 0.0)),
        material=brushed,
        name="hinge_pin",
    )
    for i, zc in enumerate((-0.112, 0.112)):
        frame.visual(
            Box((0.075, 0.046, 0.080)),
            origin=Origin(xyz=(HINGE_X + 0.032, FRAME_T + 0.023, zc)),
            material=brushed,
            name=f"frame_hinge_leaf_{i}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_panel(), "slotted_safe_door"),
        material=door_paint,
        name="door_panel",
    )
    # Raised door border, made as welded-on proud steel strips.
    door.visual(
        Box((DOOR_W - 0.030, 0.012, 0.030)),
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_LOCAL_Y + DOOR_T + 0.006, DOOR_H / 2.0 - 0.030)),
        material=dark_steel,
        name="top_raised_border",
    )
    door.visual(
        Box((DOOR_W - 0.030, 0.012, 0.030)),
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_LOCAL_Y + DOOR_T + 0.006, -DOOR_H / 2.0 + 0.030)),
        material=dark_steel,
        name="bottom_raised_border",
    )
    door.visual(
        Box((0.030, 0.012, DOOR_H - 0.030)),
        origin=Origin(xyz=(DOOR_CENTER_X - DOOR_W / 2.0 + 0.030, DOOR_LOCAL_Y + DOOR_T + 0.006, 0.0)),
        material=dark_steel,
        name="free_edge_border",
    )
    door.visual(
        Box((0.026, 0.012, DOOR_H - 0.030)),
        origin=Origin(xyz=(DOOR_CENTER_X + DOOR_W / 2.0 - 0.013, DOOR_LOCAL_Y + DOOR_T + 0.006, 0.0)),
        material=dark_steel,
        name="hinge_edge_border",
    )
    # Three hinge barrels rotate with the door about the same vertical axis.
    for i, zc in enumerate((-0.225, 0.0, 0.225)):
        door.visual(
            Cylinder(radius=0.012, length=0.128),
            origin=Origin(xyz=(0.0, 0.0, zc), rpy=(0.0, 0.0, 0.0)),
            material=brushed,
            name=f"door_hinge_barrel_{i}",
        )
    # Fixed pull handle below the deposit flap.
    for i, xoff in enumerate((-0.085, 0.085)):
        door.visual(
            Cylinder(radius=0.008, length=0.070),
            origin=Origin(
                xyz=(DOOR_CENTER_X + xoff, DOOR_LOCAL_Y + DOOR_T + 0.035, HANDLE_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed,
            name=f"handle_standoff_{i}",
        )
    door.visual(
        Cylinder(radius=0.013, length=0.220),
        origin=Origin(
            xyz=(DOOR_CENTER_X, DOOR_LOCAL_Y + DOOR_T + 0.070, HANDLE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brushed,
        name="pull_handle",
    )
    # Stationary red index mark above the rotating combination dial.
    door.visual(
        Box((0.018, 0.004, 0.024)),
        origin=Origin(xyz=(DIAL_X, DOOR_LOCAL_Y + DOOR_T + 0.002, DIAL_Z + DIAL_D / 2.0 + 0.020)),
        material=red,
        name="dial_index",
    )

    dial = model.part("combination_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            DIAL_D,
            DIAL_DEPTH,
            body_style="faceted",
            grip=KnobGrip(style="ribbed", count=40, depth=0.0018),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "combination_dial_body",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="dial_body",
    )
    dial.visual(
        Box((0.010, 0.004, 0.044)),
        origin=Origin(xyz=(0.0, DIAL_DEPTH + 0.002, DIAL_D * 0.22)),
        material=rubber,
        name="dial_pointer",
    )

    flap = model.part("deposit_flap")
    flap.visual(
        Box((FLAP_W, FLAP_T, FLAP_H)),
        origin=Origin(xyz=(0.0, FLAP_T / 2.0, -FLAP_H / 2.0)),
        material=door_paint,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.008, length=FLAP_W + 0.020),
        origin=Origin(xyz=(0.0, FLAP_T * 0.65, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="flap_top_barrel",
    )
    flap.visual(
        Box((FLAP_W - 0.030, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, FLAP_T + 0.002, -FLAP_H + 0.018)),
        material=brushed,
        name="flap_finger_lip",
    )

    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_AXIS_Y, 0.0)),
        # Closed door geometry extends left from a right-side hinge; negative Z
        # makes positive motion swing the free edge outward, toward +Y.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DIAL_X, DOOR_LOCAL_Y + DOOR_T, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0),
    )
    model.articulation(
        "door_to_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=flap,
        origin=Origin(xyz=(FLAP_X, DOOR_LOCAL_Y + DOOR_T, FLAP_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    dial = object_model.get_part("combination_dial")
    flap = object_model.get_part("deposit_flap")
    door_joint = object_model.get_articulation("frame_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    flap_joint = object_model.get_articulation("door_to_flap")

    ctx.check(
        "main door is right hinged",
        door_joint.axis == (0.0, 0.0, -1.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "combination dial is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )
    ctx.check(
        "deposit flap uses top horizontal hinge",
        flap_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={flap_joint.axis}",
    )
    for i in range(3):
        barrel = f"door_hinge_barrel_{i}"
        ctx.allow_overlap(
            frame,
            door,
            elem_a="hinge_pin",
            elem_b=barrel,
            reason="The fixed hinge pin is intentionally captured inside the rotating door barrel.",
        )
        ctx.expect_within(
            frame,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=barrel,
            margin=0.001,
            name=f"hinge pin centered in barrel {i}",
        )
        ctx.expect_overlap(
            frame,
            door,
            axes="z",
            min_overlap=0.10,
            elem_a="hinge_pin",
            elem_b=barrel,
            name=f"hinge pin retained through barrel {i}",
        )

    with ctx.pose({door_joint: 0.0, flap_joint: 0.0}):
        ctx.expect_gap(
            door,
            frame,
            axis="y",
            min_gap=0.006,
            max_gap=0.018,
            positive_elem="door_panel",
            negative_elem="front_flange",
            name="closed door stands just proud of flange",
        )
        ctx.expect_gap(
            dial,
            door,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem="dial_body",
            negative_elem="door_panel",
            name="dial sits on door face",
        )
        ctx.expect_gap(
            flap,
            door,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem="flap_panel",
            negative_elem="door_panel",
            name="deposit flap lies on door face",
        )
        ctx.expect_overlap(
            flap,
            door,
            axes="xz",
            min_overlap=0.08,
            elem_a="flap_panel",
            elem_b="door_panel",
            name="deposit flap covers the deposit slot area",
        )

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.35}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward from the wall",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > rest_door_aabb[1][1] + 0.14,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    rest_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 0.85}):
        open_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "deposit flap rotates outward",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > rest_flap_aabb[1][1] + 0.035,
        details=f"rest={rest_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
