from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
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


FRONT_Y = -0.31
CONTROL_FRONT_Y = -0.3445
DOOR_HINGE_Z = 0.20
DOOR_HEIGHT = 0.46


def _filleted_box(size: tuple[float, float, float], radius: float):
    """A softly radiused enamel-appliance box, authored in meters."""
    solid = cq.Workplane("XY").box(size[0], size[1], size[2])
    try:
        return solid.edges().fillet(radius)
    except Exception:
        return solid


def _oven_door_panel():
    """Rounded oven door skin with a real through-window opening."""
    width, thickness, height = 0.56, 0.055, DOOR_HEIGHT
    panel = cq.Workplane("XY").box(width, thickness, height)
    window_cutter = (
        cq.Workplane("XY")
        .box(0.31, thickness * 2.4, 0.145)
        .translate((0.0, 0.0, 0.035))
    )
    panel = panel.cut(window_cutter)
    try:
        return panel.edges().fillet(0.010)
    except Exception:
        return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_enamel_stove")

    mint = model.material("mint_enamel", rgba=(0.62, 0.86, 0.80, 1.0))
    ivory = model.material("ivory_enamel", rgba=(0.93, 0.88, 0.75, 1.0))
    chrome = model.material("warm_chrome", rgba=(0.78, 0.76, 0.70, 1.0))
    black = model.material("black_iron", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("smoky_oven_glass", rgba=(0.08, 0.13, 0.16, 0.55))
    red = model.material("red_enamel_trim", rgba=(0.72, 0.12, 0.08, 1.0))

    stove = model.part("stove")

    # Main rounded enamel cabinet, scaled like a compact 1950s domestic range.
    stove.visual(
        mesh_from_cadquery(_filleted_box((0.76, 0.62, 0.78), 0.030), "stove_cabinet"),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=mint,
        name="cabinet",
    )
    stove.visual(
        Box((0.78, 0.66, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.8425)),
        material=mint,
        name="cooktop",
    )
    stove.visual(
        Box((0.78, 0.040, 0.180)),
        origin=Origin(xyz=(0.0, 0.330, 0.950)),
        material=mint,
        name="backsplash",
    )
    stove.visual(
        Box((0.70, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.307, 1.010)),
        material=red,
        name="backsplash_trim",
    )

    # Recessed front control band and oven shadow opening.
    stove.visual(
        Box((0.68, 0.035, 0.120)),
        origin=Origin(xyz=(0.0, -0.327, 0.755)),
        material=ivory,
        name="control_panel",
    )
    stove.visual(
        Box((0.50, 0.006, 0.550)),
        origin=Origin(xyz=(0.0, -0.3065, 0.420)),
        material=black,
        name="oven_interior_shadow",
    )
    stove.visual(
        Box((0.60, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, -0.320, DOOR_HINGE_Z - 0.032)),
        material=chrome,
        name="lower_hinge_rail",
    )
    stove.visual(
        Box((0.62, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.337, 0.686)),
        material=chrome,
        name="door_header_trim",
    )

    # Four visible iron burners: dark caps plus crossed grate bars.
    burner_positions = (
        (-0.205, -0.145),
        (0.205, -0.145),
        (-0.205, 0.115),
        (0.205, 0.115),
    )
    for idx, (x, y) in enumerate(burner_positions):
        stove.visual(
            Cylinder(radius=0.083, length=0.012),
            origin=Origin(xyz=(x, y, 0.871)),
            material=black,
            name=f"burner_disc_{idx}",
        )
        stove.visual(
            Cylinder(radius=0.050, length=0.016),
            origin=Origin(xyz=(x, y, 0.885)),
            material=chrome,
            name=f"burner_cap_{idx}",
        )
        for bar_idx, yaw in enumerate((0.0, math.pi / 2.0)):
            stove.visual(
                Box((0.155, 0.018, 0.018)),
                origin=Origin(xyz=(x, y, 0.902), rpy=(0.0, 0.0, yaw)),
                material=black,
                name=f"grate_bar_{idx}_{bar_idx}",
            )

    # Windowed oven door.  Its child frame is the lower hinge line.
    door = model.part("oven_door")
    door.visual(
        mesh_from_cadquery(_oven_door_panel(), "oven_door_panel"),
        origin=Origin(xyz=(0.0, -0.0275, DOOR_HEIGHT / 2.0)),
        material=ivory,
        name="door_panel",
    )
    door.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.31, 0.145),
                (0.395, 0.230),
                0.014,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.018,
                outer_corner_radius=0.030,
                face=BezelFace(style="radiused_step", front_lip=0.002, fillet=0.002),
            ),
            "oven_window_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.057, 0.265), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="window_bezel",
    )
    door.visual(
        Box((0.330, 0.006, 0.160)),
        origin=Origin(xyz=(0.0, -0.057, 0.265)),
        material=glass,
        name="oven_window_glass",
    )
    door.visual(
        Box((0.040, 0.040, 0.060)),
        origin=Origin(xyz=(-0.205, -0.066, 0.385)),
        material=chrome,
        name="handle_mount_0",
    )
    door.visual(
        Box((0.040, 0.040, 0.060)),
        origin=Origin(xyz=(0.205, -0.066, 0.385)),
        material=chrome,
        name="handle_mount_1",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.460),
        origin=Origin(xyz=(0.0, -0.104, 0.400), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="door_handle",
    )
    door.visual(
        Cylinder(radius=0.020, length=0.575),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="door_hinge_barrel",
    )
    model.articulation(
        "stove_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=stove,
        child=door,
        origin=Origin(xyz=(0.0, FRONT_Y, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=1.35),
    )

    def make_knob(name: str, x: float, z: float, diameter: float, height: float) -> None:
        part = model.part(name)
        knob_mesh = mesh_from_geometry(
            KnobGeometry(
                diameter,
                height,
                body_style="skirted",
                top_diameter=diameter * 0.75,
                skirt=KnobSkirt(diameter * 1.20, height * 0.22, flare=0.08, chamfer=0.0012),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                bore=KnobBore(style="d_shaft", diameter=diameter * 0.16, flat_depth=0.001),
                center=False,
            ),
            f"{name}_cap",
        )
        part.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=ivory,
            name="knob_cap",
        )
        model.articulation(
            f"stove_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=stove,
            child=part,
            origin=Origin(xyz=(x, CONTROL_FRONT_Y, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=8.0),
        )

    make_knob("burner_knob_0", -0.285, 0.758, 0.052, 0.035)
    make_knob("burner_knob_1", -0.165, 0.758, 0.052, 0.035)
    make_knob("oven_knob", 0.0, 0.758, 0.075, 0.045)
    make_knob("burner_knob_2", 0.165, 0.758, 0.052, 0.035)
    make_knob("burner_knob_3", 0.285, 0.758, 0.052, 0.035)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.
    stove = object_model.get_part("stove")
    door = object_model.get_part("oven_door")
    door_joint = object_model.get_articulation("stove_to_oven_door")
    knob_joints = [
        object_model.get_articulation(f"stove_to_burner_knob_{idx}") for idx in range(4)
    ] + [object_model.get_articulation("stove_to_oven_knob")]

    ctx.check(
        "five front rotary knobs",
        len(knob_joints) == 5
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in knob_joints)
        and all(tuple(round(v, 6) for v in j.axis) == (0.0, -1.0, 0.0) for j in knob_joints),
        details=f"knob_joints={[(j.name, j.articulation_type, j.axis) for j in knob_joints]}",
    )
    ctx.check(
        "oven door lower hinge",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in door_joint.axis) == (1.0, 0.0, 0.0)
        and door_joint.motion_limits is not None
        and door_joint.motion_limits.lower == 0.0
        and door_joint.motion_limits.upper is not None
        and door_joint.motion_limits.upper > 1.0,
        details=f"door_joint={door_joint}",
    )
    ctx.expect_gap(
        stove,
        door,
        axis="y",
        positive_elem="oven_interior_shadow",
        negative_elem="door_panel",
        min_gap=0.0005,
        max_gap=0.012,
        name="closed door sits just ahead of oven opening",
    )
    ctx.expect_overlap(
        door,
        stove,
        axes="xz",
        elem_a="door_panel",
        elem_b="oven_interior_shadow",
        min_overlap=0.25,
        name="door covers the oven opening",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.20}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "oven door swings downward and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.10
        and open_aabb[1][2] < closed_aabb[1][2] - 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
