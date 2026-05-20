from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

try:
    os.chdir("/")
except Exception:
    pass

if not os.path.isabs(__file__):
    __file__ = "/" + __file__.lstrip("/")
if "__cached__" in globals() and isinstance(__cached__, str) and not os.path.isabs(__cached__):
    __cached__ = "/" + __cached__.lstrip("/")
if "__spec__" in globals() and getattr(__spec__, "origin", None) and not os.path.isabs(__spec__.origin):
    __spec__.origin = "/" + __spec__.origin.lstrip("/")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ValidationError,
)


WIDTH = 0.90
DEPTH = 0.64
CABINET_HEIGHT = 0.84
COOKTOP_THICKNESS = 0.04
COOKTOP_OVERHANG = 0.01
COOKTOP_TOP_Z = CABINET_HEIGHT + COOKTOP_THICKNESS
BACKGUARD_HEIGHT = 0.07
WALL_THICKNESS = 0.045
BACK_THICKNESS = 0.025
PLINTH_HEIGHT = 0.07
MID_DIVIDER_THICKNESS = 0.045
CONTROL_FASCIA_HEIGHT = 0.10
FRONT_FRAME_DEPTH = 0.025
LOWER_DOOR_HEIGHT = 0.38
UPPER_DOOR_HEIGHT = 0.26
LOWER_DOOR_BOTTOM_Z = PLINTH_HEIGHT
UPPER_DOOR_BOTTOM_Z = LOWER_DOOR_BOTTOM_Z + LOWER_DOOR_HEIGHT + MID_DIVIDER_THICKNESS
DOOR_WIDTH = 0.79
DOOR_THICKNESS = 0.045
KNOB_RADIUS = 0.019
KNOB_DEPTH = 0.032
KNOB_ROW_Z = CABINET_HEIGHT - 0.05
GRATE_WIDTH = 0.22
GRATE_DEPTH = 0.19
GRATE_LEG_RADIUS = 0.009
GRATE_SUPPORT_HEIGHT = 0.026
GRATE_BAR_RADIUS = 0.009
DOOR_OPEN_LIMIT = 1.38
KNOB_TURN_LIMIT = 4.71

GRATE_NAMES = (
    "front_left",
    "front_center",
    "front_right",
    "rear_left",
    "rear_center",
    "rear_right",
)

BURNER_POSITIONS = (
    (-0.27, -0.11),
    (0.0, -0.11),
    (0.27, -0.11),
    (-0.27, 0.11),
    (0.0, 0.11),
    (0.27, 0.11),
)

KNOB_X_POSITIONS = (-0.31, -0.19, -0.07, 0.07, 0.19, 0.31)


def _lookup_part(ctx: TestContext, name: str):
    try:
        part = object_model.get_part(name)
    except ValidationError as exc:
        ctx.fail(f"part {name} present", str(exc))
        return None
    ctx.check(f"part {name} present", True)
    return part


def _lookup_joint(ctx: TestContext, name: str):
    try:
        articulation = object_model.get_articulation(name)
    except ValidationError as exc:
        ctx.fail(f"articulation {name} present", str(exc))
        return None
    ctx.check(f"articulation {name} present", True)
    return articulation


def _lookup_visual(ctx: TestContext, part, name: str):
    try:
        visual = part.get_visual(name)
    except ValidationError as exc:
        ctx.fail(f"visual {part.name}.{name} present", str(exc))
        return None
    ctx.check(f"visual {part.name}.{name} present", True)
    return visual


def _add_grate_geometry(part, *, material) -> None:
    top_z = GRATE_SUPPORT_HEIGHT + GRATE_BAR_RADIUS - 0.002
    leg_z = GRATE_SUPPORT_HEIGHT / 2.0
    x_leg = (GRATE_WIDTH - 2.0 * GRATE_LEG_RADIUS) / 2.0
    y_leg = (GRATE_DEPTH - 2.0 * GRATE_LEG_RADIUS) / 2.0

    for index, (x_pos, y_pos) in enumerate(
        ((-x_leg, -y_leg), (x_leg, -y_leg), (-x_leg, y_leg), (x_leg, y_leg)),
        start=1,
    ):
        part.visual(
            Cylinder(radius=GRATE_LEG_RADIUS, length=GRATE_SUPPORT_HEIGHT),
            origin=Origin(xyz=(x_pos, y_pos, leg_z)),
            material=material,
            name=f"leg_{index}",
        )

    part.visual(
        Cylinder(radius=GRATE_BAR_RADIUS, length=GRATE_WIDTH),
        origin=Origin(
            xyz=(0.0, (GRATE_DEPTH - 2.0 * GRATE_BAR_RADIUS) / 2.0, top_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name="frame_front",
    )
    part.visual(
        Cylinder(radius=GRATE_BAR_RADIUS, length=GRATE_WIDTH),
        origin=Origin(
            xyz=(0.0, -(GRATE_DEPTH - 2.0 * GRATE_BAR_RADIUS) / 2.0, top_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name="frame_back",
    )
    part.visual(
        Cylinder(radius=GRATE_BAR_RADIUS, length=GRATE_DEPTH),
        origin=Origin(
            xyz=(-(GRATE_WIDTH - 2.0 * GRATE_BAR_RADIUS) / 2.0, 0.0, top_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name="frame_left",
    )
    part.visual(
        Cylinder(radius=GRATE_BAR_RADIUS, length=GRATE_DEPTH),
        origin=Origin(
            xyz=((GRATE_WIDTH - 2.0 * GRATE_BAR_RADIUS) / 2.0, 0.0, top_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name="frame_right",
    )
    part.visual(
        Cylinder(radius=GRATE_BAR_RADIUS, length=GRATE_WIDTH - 4.0 * GRATE_BAR_RADIUS + 0.002),
        origin=Origin(xyz=(0.0, 0.0, top_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="cross_x",
    )
    part.visual(
        Cylinder(radius=GRATE_BAR_RADIUS, length=GRATE_DEPTH - 4.0 * GRATE_BAR_RADIUS + 0.002),
        origin=Origin(xyz=(0.0, 0.0, top_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="cross_y",
    )

    grate_height = GRATE_SUPPORT_HEIGHT + 2.0 * GRATE_BAR_RADIUS
    part.inertial = Inertial.from_geometry(
        Box((GRATE_WIDTH, GRATE_DEPTH, grate_height)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, grate_height / 2.0)),
    )


def _add_door_geometry(part, *, width: float, height: float, shell_material, glass_material, trim_material) -> None:
    shell_center_y = -DOOR_THICKNESS / 2.0
    stile_width = 0.07
    top_rail_height = 0.055 if height <= 0.30 else 0.06
    bottom_rail_height = 0.08 if height <= 0.30 else 0.10
    opening_width = width - 2.0 * stile_width
    opening_height = height - top_rail_height - bottom_rail_height
    opening_center_z = bottom_rail_height + opening_height / 2.0
    glass_thickness = 0.008
    handle_radius = 0.011
    handle_length = width * 0.70

    part.visual(
        Box((stile_width, DOOR_THICKNESS, height)),
        origin=Origin(xyz=(-(width - stile_width) / 2.0, shell_center_y, height / 2.0)),
        material=shell_material,
        name="left_stile",
    )
    part.visual(
        Box((stile_width, DOOR_THICKNESS, height)),
        origin=Origin(xyz=((width - stile_width) / 2.0, shell_center_y, height / 2.0)),
        material=shell_material,
        name="right_stile",
    )
    part.visual(
        Box((opening_width, DOOR_THICKNESS, top_rail_height)),
        origin=Origin(xyz=(0.0, shell_center_y, height - top_rail_height / 2.0)),
        material=shell_material,
        name="top_rail",
    )
    part.visual(
        Box((opening_width, DOOR_THICKNESS, bottom_rail_height)),
        origin=Origin(xyz=(0.0, shell_center_y, bottom_rail_height / 2.0)),
        material=shell_material,
        name="bottom_rail",
    )
    part.visual(
        Box((opening_width, glass_thickness, opening_height)),
        origin=Origin(
            xyz=(
                0.0,
                -DOOR_THICKNESS + glass_thickness / 2.0 + 0.001,
                opening_center_z,
            ),
        ),
        material=glass_material,
        name="window",
    )
    part.visual(
        Cylinder(radius=handle_radius, length=handle_length),
        origin=Origin(
            xyz=(
                0.0,
                -DOOR_THICKNESS - handle_radius + 0.0015,
                height - top_rail_height * 0.45,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_material,
        name="handle",
    )
    part.inertial = Inertial.from_geometry(
        Box((width, DOOR_THICKNESS, height)),
        mass=9.0 if height > 0.30 else 7.0,
        origin=Origin(xyz=(0.0, shell_center_y, height / 2.0)),
    )


def _add_knob_geometry(part, *, knob_material, marker_material) -> None:
    part.visual(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
        origin=Origin(xyz=(0.0, -KNOB_DEPTH / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="body",
    )
    part.visual(
        Box((0.004, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -KNOB_DEPTH + 0.004, KNOB_RADIUS * 0.58)),
        material=marker_material,
        name="marker",
    )
    part.inertial = Inertial.from_geometry(
        Box((KNOB_RADIUS * 2.0, KNOB_DEPTH, KNOB_RADIUS * 2.0)),
        mass=0.15,
        origin=Origin(xyz=(0.0, -KNOB_DEPTH / 2.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="range_cooker")

    enamel = model.material("enamel", rgba=(0.17, 0.18, 0.20, 1.0))
    trim = model.material("trim", rgba=(0.73, 0.74, 0.76, 1.0))
    glass = model.material("glass", rgba=(0.10, 0.14, 0.18, 0.45))
    iron = model.material("cast_iron", rgba=(0.08, 0.08, 0.09, 1.0))
    burner = model.material("burner", rgba=(0.13, 0.13, 0.14, 1.0))
    fascia = model.material("fascia", rgba=(0.24, 0.25, 0.27, 1.0))

    body = model.part("body")
    inner_width = WIDTH - 2.0 * WALL_THICKNESS
    side_x = WIDTH / 2.0 - WALL_THICKNESS / 2.0
    stile_width = (inner_width - DOOR_WIDTH) / 2.0
    front_stile_height = CABINET_HEIGHT - CONTROL_FASCIA_HEIGHT - PLINTH_HEIGHT

    body.visual(
        Box((WALL_THICKNESS, DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-side_x, 0.0, CABINET_HEIGHT / 2.0)),
        material=enamel,
        name="left_side",
    )
    body.visual(
        Box((WALL_THICKNESS, DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(side_x, 0.0, CABINET_HEIGHT / 2.0)),
        material=enamel,
        name="right_side",
    )
    body.visual(
        Box((inner_width, BACK_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - BACK_THICKNESS / 2.0, CABINET_HEIGHT / 2.0)),
        material=enamel,
        name="back_panel",
    )
    body.visual(
        Box((inner_width, DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + WALL_THICKNESS / 2.0)),
        material=enamel,
        name="lower_floor",
    )
    body.visual(
        Box((inner_width, DEPTH, MID_DIVIDER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                LOWER_DOOR_BOTTOM_Z + LOWER_DOOR_HEIGHT + MID_DIVIDER_THICKNESS / 2.0,
            ),
        ),
        material=enamel,
        name="middle_divider",
    )
    body.visual(
        Box((inner_width, DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - WALL_THICKNESS / 2.0)),
        material=enamel,
        name="ceiling",
    )
    body.visual(
        Box((WIDTH, 0.04, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 + 0.02, PLINTH_HEIGHT / 2.0)),
        material=fascia,
        name="plinth",
    )
    body.visual(
        Box((stile_width, FRONT_FRAME_DEPTH, front_stile_height)),
        origin=Origin(
            xyz=(
                -(DOOR_WIDTH / 2.0 + stile_width / 2.0),
                -DEPTH / 2.0 + FRONT_FRAME_DEPTH / 2.0,
                PLINTH_HEIGHT + front_stile_height / 2.0,
            ),
        ),
        material=fascia,
        name="left_stile",
    )
    body.visual(
        Box((stile_width, FRONT_FRAME_DEPTH, front_stile_height)),
        origin=Origin(
            xyz=(
                DOOR_WIDTH / 2.0 + stile_width / 2.0,
                -DEPTH / 2.0 + FRONT_FRAME_DEPTH / 2.0,
                PLINTH_HEIGHT + front_stile_height / 2.0,
            ),
        ),
        material=fascia,
        name="right_stile",
    )
    body.visual(
        Box((inner_width, FRONT_FRAME_DEPTH, 0.02)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 + FRONT_FRAME_DEPTH / 2.0, LOWER_DOOR_BOTTOM_Z + 0.01)),
        material=trim,
        name="lower_hinge_rail",
    )
    body.visual(
        Box((inner_width, FRONT_FRAME_DEPTH, MID_DIVIDER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH / 2.0 + FRONT_FRAME_DEPTH / 2.0,
                LOWER_DOOR_BOTTOM_Z + LOWER_DOOR_HEIGHT + MID_DIVIDER_THICKNESS / 2.0,
            ),
        ),
        material=fascia,
        name="middle_rail",
    )
    body.visual(
        Box((inner_width, 0.045, CONTROL_FASCIA_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH / 2.0 + 0.0225,
                CABINET_HEIGHT - CONTROL_FASCIA_HEIGHT / 2.0,
            ),
        ),
        material=fascia,
        name="control_fascia",
    )
    body.visual(
        Box((WIDTH + 2.0 * COOKTOP_OVERHANG, DEPTH + 2.0 * COOKTOP_OVERHANG, COOKTOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT + COOKTOP_THICKNESS / 2.0)),
        material=fascia,
        name="cooktop",
    )
    body.visual(
        Box((WIDTH + 2.0 * COOKTOP_OVERHANG, 0.025, BACKGUARD_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                DEPTH / 2.0 - 0.0125,
                COOKTOP_TOP_Z + BACKGUARD_HEIGHT / 2.0,
            ),
        ),
        material=fascia,
        name="backsplash",
    )

    for index, (x_pos, y_pos) in enumerate(BURNER_POSITIONS, start=1):
        body.visual(
            Cylinder(radius=0.048, length=0.008),
            origin=Origin(xyz=(x_pos, y_pos, COOKTOP_TOP_Z + 0.004)),
            material=burner,
            name=f"burner_{index}",
        )
        body.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, COOKTOP_TOP_Z + 0.013)),
            material=trim,
            name=f"burner_cap_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((WIDTH + 2.0 * COOKTOP_OVERHANG, DEPTH + 2.0 * COOKTOP_OVERHANG, COOKTOP_TOP_Z + BACKGUARD_HEIGHT)),
        mass=112.0,
        origin=Origin(xyz=(0.0, 0.0, (COOKTOP_TOP_Z + BACKGUARD_HEIGHT) / 2.0)),
    )

    upper_door = model.part("upper_door")
    _add_door_geometry(
        upper_door,
        width=DOOR_WIDTH,
        height=UPPER_DOOR_HEIGHT,
        shell_material=enamel,
        glass_material=glass,
        trim_material=trim,
    )

    lower_door = model.part("lower_door")
    _add_door_geometry(
        lower_door,
        width=DOOR_WIDTH,
        height=LOWER_DOOR_HEIGHT,
        shell_material=enamel,
        glass_material=glass,
        trim_material=trim,
    )

    model.articulation(
        "upper_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_door,
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, UPPER_DOOR_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=DOOR_OPEN_LIMIT,
        ),
    )
    model.articulation(
        "lower_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_door,
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, LOWER_DOOR_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.5,
            lower=0.0,
            upper=DOOR_OPEN_LIMIT,
        ),
    )

    for grate_name, (x_pos, y_pos) in zip(GRATE_NAMES, BURNER_POSITIONS):
        grate = model.part(f"grate_{grate_name}")
        _add_grate_geometry(grate, material=iron)
        model.articulation(
            f"body_to_grate_{grate_name}",
            ArticulationType.FIXED,
            parent=body,
            child=grate,
            origin=Origin(xyz=(x_pos, y_pos, COOKTOP_TOP_Z)),
        )

    for index, x_pos in enumerate(KNOB_X_POSITIONS, start=1):
        knob = model.part(f"knob_{index}")
        _add_knob_geometry(knob, knob_material=trim, marker_material=burner)
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x_pos, -DEPTH / 2.0, KNOB_ROW_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=4.0,
                lower=0.0,
                upper=KNOB_TURN_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    body = _lookup_part(ctx, "body")
    upper_door = _lookup_part(ctx, "upper_door")
    lower_door = _lookup_part(ctx, "lower_door")
    upper_hinge = _lookup_joint(ctx, "upper_door_hinge")
    lower_hinge = _lookup_joint(ctx, "lower_door_hinge")

    grate_parts = [_lookup_part(ctx, f"grate_{name}") for name in GRATE_NAMES]
    knob_parts = [_lookup_part(ctx, f"knob_{index}") for index in range(1, 7)]
    knob_joints = [_lookup_joint(ctx, f"body_to_knob_{index}") for index in range(1, 7)]

    if any(obj is None for obj in [body, upper_door, lower_door, upper_hinge, lower_hinge, *grate_parts, *knob_parts, *knob_joints]):
        return ctx.report()

    cooktop = _lookup_visual(ctx, body, "cooktop")
    control_fascia = _lookup_visual(ctx, body, "control_fascia")
    middle_rail = _lookup_visual(ctx, body, "middle_rail")
    lower_hinge_rail = _lookup_visual(ctx, body, "lower_hinge_rail")
    upper_bottom_rail = _lookup_visual(ctx, upper_door, "bottom_rail")
    lower_bottom_rail = _lookup_visual(ctx, lower_door, "bottom_rail")
    upper_top_rail = _lookup_visual(ctx, upper_door, "top_rail")
    lower_top_rail = _lookup_visual(ctx, lower_door, "top_rail")
    upper_handle = _lookup_visual(ctx, upper_door, "handle")
    lower_handle = _lookup_visual(ctx, lower_door, "handle")
    knob_bodies = [_lookup_visual(ctx, knob, "body") for knob in knob_parts]

    if any(
        obj is None
        for obj in [
            cooktop,
            control_fascia,
            middle_rail,
            lower_hinge_rail,
            upper_bottom_rail,
            lower_bottom_rail,
            upper_top_rail,
            lower_top_rail,
            upper_handle,
            lower_handle,
            *knob_bodies,
        ]
    ):
        return ctx.report()

    ctx.check(
        "upper_door_hinge axis",
        tuple(upper_hinge.axis) == (1.0, 0.0, 0.0),
        f"axis was {upper_hinge.axis}",
    )
    ctx.check(
        "lower_door_hinge axis",
        tuple(lower_hinge.axis) == (1.0, 0.0, 0.0),
        f"axis was {lower_hinge.axis}",
    )
    ctx.check(
        "upper_door_hinge limits",
        upper_hinge.motion_limits is not None
        and abs((upper_hinge.motion_limits.lower or 0.0) - 0.0) < 1e-6
        and abs((upper_hinge.motion_limits.upper or 0.0) - DOOR_OPEN_LIMIT) < 1e-6,
        f"limits were {upper_hinge.motion_limits}",
    )
    ctx.check(
        "lower_door_hinge limits",
        lower_hinge.motion_limits is not None
        and abs((lower_hinge.motion_limits.lower or 0.0) - 0.0) < 1e-6
        and abs((lower_hinge.motion_limits.upper or 0.0) - DOOR_OPEN_LIMIT) < 1e-6,
        f"limits were {lower_hinge.motion_limits}",
    )

    ctx.expect_origin_distance(upper_door, body, axes="x", max_dist=0.001, name="upper door centered")
    ctx.expect_origin_distance(lower_door, body, axes="x", max_dist=0.001, name="lower door centered")
    ctx.expect_gap(
        body,
        upper_door,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=middle_rail,
        negative_elem=upper_bottom_rail,
        name="upper door closes against divider rail",
    )
    ctx.expect_gap(
        body,
        lower_door,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lower_hinge_rail,
        negative_elem=lower_bottom_rail,
        name="lower door closes against lower sill",
    )
    ctx.expect_gap(
        upper_door,
        lower_door,
        axis="z",
        min_gap=0.04,
        max_gap=0.05,
        positive_elem=upper_bottom_rail,
        negative_elem=lower_top_rail,
        name="two oven doors are vertically stacked with a divider gap",
    )
    ctx.expect_gap(
        body,
        upper_door,
        axis="z",
        min_gap=0.08,
        positive_elem=cooktop,
        negative_elem=upper_top_rail,
        name="cooktop sits above the upper oven opening",
    )

    for grate_name, grate in zip(GRATE_NAMES, grate_parts):
        ctx.expect_gap(
            grate,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            negative_elem=cooktop,
            name=f"{grate_name} grate stands on the cooktop",
        )
        ctx.expect_within(
            grate,
            body,
            axes="xy",
            outer_elem=cooktop,
            name=f"{grate_name} grate stays within the cooktop footprint",
        )

    for index, (knob, knob_joint, knob_body) in enumerate(zip(knob_parts, knob_joints, knob_bodies), start=1):
        ctx.check(
            f"knob_{index} axis",
            tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
            f"axis was {knob_joint.axis}",
        )
        ctx.check(
            f"knob_{index} limits",
            knob_joint.motion_limits is not None
            and abs((knob_joint.motion_limits.lower or 0.0) - 0.0) < 1e-6
            and abs((knob_joint.motion_limits.upper or 0.0) - KNOB_TURN_LIMIT) < 1e-6,
            f"limits were {knob_joint.motion_limits}",
        )
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=control_fascia,
            negative_elem=knob_body,
            name=f"knob_{index} is mounted to the control fascia",
        )
        ctx.expect_within(
            knob,
            body,
            axes="xz",
            outer_elem=control_fascia,
            margin=0.025,
            name=f"knob_{index} stays aligned within the fascia band",
        )

    door_hinges = (upper_hinge, lower_hinge)
    for hinge in door_hinges:
        limits = hinge.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({hinge: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{hinge.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{hinge.name}_lower_no_floating")
            with ctx.pose({hinge: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{hinge.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{hinge.name}_upper_no_floating")

    with ctx.pose({upper_hinge: DOOR_OPEN_LIMIT}):
        ctx.expect_gap(
            body,
            upper_door,
            axis="z",
            min_gap=0.18,
            positive_elem=control_fascia,
            negative_elem=upper_handle,
            name="upper door folds below the control fascia",
        )
        ctx.expect_gap(
            body,
            upper_door,
            axis="y",
            min_gap=0.18,
            positive_elem=control_fascia,
            negative_elem=upper_handle,
            name="upper door swings forward clear of the front face",
        )

    with ctx.pose({lower_hinge: DOOR_OPEN_LIMIT}):
        ctx.expect_gap(
            body,
            lower_door,
            axis="z",
            min_gap=0.28,
            positive_elem=middle_rail,
            negative_elem=lower_handle,
            name="lower door folds well below the divider rail",
        )
        ctx.expect_gap(
            body,
            lower_door,
            axis="y",
            min_gap=0.27,
            positive_elem=control_fascia,
            negative_elem=lower_handle,
            name="lower door swings forward clear of the cabinet front",
        )

    with ctx.pose({upper_hinge: DOOR_OPEN_LIMIT, lower_hinge: DOOR_OPEN_LIMIT}):
        ctx.fail_if_parts_overlap_in_current_pose(name="both_doors_open_no_overlap")
        ctx.fail_if_isolated_parts(name="both_doors_open_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
