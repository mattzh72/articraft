from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


TRAILER_LENGTH = 1.85
TRAILER_WIDTH = 0.98
DECK_THICKNESS = 0.12
WHEEL_RADIUS = 0.28
WHEEL_THICKNESS = 0.16
WHEEL_CENTER_X = -0.12
WHEEL_CENTER_Y = 0.58
WHEEL_CENTER_Z = -0.35

MAST_X = -0.42
MAST_PAD_TOP = 0.055
LOWER_FLANGE_HEIGHT = 0.04
LOWER_SHELL_Z0 = 0.03
LOWER_SHELL_HEIGHT = 1.60
LOWER_OPENING_Z = LOWER_SHELL_Z0 + LOWER_SHELL_HEIGHT

MID_STAGE_LENGTH = 1.75
MID_STAGE_REST_EXPOSED = 0.33
MID_STAGE_TRAVEL = 0.96
MID_STAGE_RETAINED = MID_STAGE_LENGTH - MID_STAGE_REST_EXPOSED - MID_STAGE_TRAVEL

UPPER_STAGE_LENGTH = 1.55
UPPER_STAGE_REST_EXPOSED = 0.30
UPPER_STAGE_TRAVEL = 0.82
UPPER_STAGE_RETAINED = (
    UPPER_STAGE_LENGTH - UPPER_STAGE_REST_EXPOSED - UPPER_STAGE_TRAVEL
)

TOP_STAGE_LENGTH = 1.35
TOP_STAGE_REST_EXPOSED = 0.25
TOP_STAGE_TRAVEL = 0.72
TOP_STAGE_RETAINED = TOP_STAGE_LENGTH - TOP_STAGE_REST_EXPOSED - TOP_STAGE_TRAVEL


def _add_square_tube_stage(
    part,
    *,
    outer_width: float,
    wall: float,
    length: float,
    material,
    name_prefix: str,
    z0: float = 0.0,
    hidden_below: float = 0.0,
) -> None:
    base_z = z0 - hidden_below
    z_center = base_z + length * 0.5
    wall_center = outer_width * 0.5 - wall * 0.5
    inner_span = outer_width - 2.0 * wall

    part.visual(
        Box((wall, outer_width, length)),
        origin=Origin(xyz=(wall_center, 0.0, z_center)),
        material=material,
        name=f"{name_prefix}_wall_pos_x",
    )
    part.visual(
        Box((wall, outer_width, length)),
        origin=Origin(xyz=(-wall_center, 0.0, z_center)),
        material=material,
        name=f"{name_prefix}_wall_neg_x",
    )
    part.visual(
        Box((inner_span, wall, length)),
        origin=Origin(xyz=(0.0, wall_center, z_center)),
        material=material,
        name=f"{name_prefix}_wall_pos_y",
    )
    part.visual(
        Box((inner_span, wall, length)),
        origin=Origin(xyz=(0.0, -wall_center, z_center)),
        material=material,
        name=f"{name_prefix}_wall_neg_y",
    )

    gusset_size = wall * 1.5
    gusset_height = min(0.06, max(0.035, length * 0.05))
    gusset_center = outer_width * 0.5 - gusset_size * 0.5
    gusset_z = base_z + gusset_height * 0.5
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            part.visual(
                Box((gusset_size, gusset_size, gusset_height)),
                origin=Origin(xyz=(sx * gusset_center, sy * gusset_center, gusset_z)),
                material=material,
                name=f"{name_prefix}_gusset_{'pos' if sx > 0 else 'neg'}x_{'pos' if sy > 0 else 'neg'}y",
            )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def _add_guide_strips(
    part,
    *,
    child_width: float,
    parent_inner_width: float,
    z_center: float,
    height: float,
    name_prefix: str,
    material,
) -> None:
    clearance = (parent_inner_width - child_width) * 0.5
    embed = min(0.0015, clearance * 0.45)
    strip_thickness = clearance + embed
    face_center = parent_inner_width * 0.5 - strip_thickness * 0.5
    span = child_width * 0.56

    part.visual(
        Box((strip_thickness, span, height)),
        origin=Origin(xyz=(face_center, 0.0, z_center)),
        material=material,
        name=f"{name_prefix}_pos_x",
    )
    part.visual(
        Box((strip_thickness, span, height)),
        origin=Origin(xyz=(-face_center, 0.0, z_center)),
        material=material,
        name=f"{name_prefix}_neg_x",
    )
    part.visual(
        Box((span, strip_thickness, height)),
        origin=Origin(xyz=(0.0, face_center, z_center)),
        material=material,
        name=f"{name_prefix}_pos_y",
    )
    part.visual(
        Box((span, strip_thickness, height)),
        origin=Origin(xyz=(0.0, -face_center, z_center)),
        material=material,
        name=f"{name_prefix}_neg_y",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mobile_security_camera_trailer_mast")

    trailer_grey = model.material("trailer_grey", rgba=(0.48, 0.50, 0.53, 1.0))
    dark_charcoal = model.material("dark_charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    galvanized = model.material("galvanized", rgba=(0.72, 0.73, 0.75, 1.0))
    off_white = model.material("off_white", rgba=(0.88, 0.89, 0.87, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.18, 0.24, 0.30, 1.0))
    hitch_black = model.material("hitch_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((TRAILER_LENGTH, TRAILER_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=trailer_grey,
        name="deck",
    )
    base.visual(
        Box((1.42, 0.22, 0.14)),
        origin=Origin(xyz=(-0.04, 0.0, -0.18)),
        material=dark_charcoal,
        name="spine_beam",
    )
    base.visual(
        Box((0.08, 1.00, 0.08)),
        origin=Origin(xyz=(WHEEL_CENTER_X, 0.0, -0.29)),
        material=dark_charcoal,
        name="axle_beam",
    )
    base.visual(
        Box((0.96, 0.12, 0.10)),
        origin=Origin(xyz=(1.395, 0.0, -0.05)),
        material=dark_charcoal,
        name="drawbar",
    )
    base.visual(
        Box((0.14, 0.08, 0.08)),
        origin=Origin(xyz=(1.945, 0.0, -0.04)),
        material=hitch_black,
        name="hitch_coupler",
    )
    base.visual(
        Box((0.72, 0.62, 0.55)),
        origin=Origin(xyz=(0.22, 0.0, 0.27)),
        material=trailer_grey,
        name="equipment_cabinet",
    )
    base.visual(
        Box((0.34, 0.34, 0.06)),
        origin=Origin(xyz=(MAST_X, 0.0, 0.025)),
        material=dark_charcoal,
        name="mast_pad",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_THICKNESS),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="left_tire",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_THICKNESS),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="right_tire",
    )

    lower_mast = model.part("lower_mast")
    lower_mast.visual(
        Box((0.30, 0.30, LOWER_FLANGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_FLANGE_HEIGHT * 0.5)),
        material=dark_charcoal,
        name="lower_flange",
    )
    _add_square_tube_stage(
        lower_mast,
        outer_width=0.20,
        wall=0.012,
        length=LOWER_SHELL_HEIGHT,
        z0=LOWER_SHELL_Z0,
        material=galvanized,
        name_prefix="lower_shell",
    )

    mid_mast = model.part("mid_mast")
    _add_square_tube_stage(
        mid_mast,
        outer_width=0.165,
        wall=0.010,
        length=MID_STAGE_LENGTH,
        hidden_below=MID_STAGE_LENGTH - MID_STAGE_REST_EXPOSED,
        material=galvanized,
        name_prefix="mid_shell",
    )
    _add_guide_strips(
        mid_mast,
        child_width=0.165,
        parent_inner_width=0.176,
        z_center=-1.19,
        height=0.20,
        name_prefix="mid_guide",
        material=dark_charcoal,
    )

    upper_mast = model.part("upper_mast")
    _add_square_tube_stage(
        upper_mast,
        outer_width=0.135,
        wall=0.009,
        length=UPPER_STAGE_LENGTH,
        hidden_below=UPPER_STAGE_LENGTH - UPPER_STAGE_REST_EXPOSED,
        material=galvanized,
        name_prefix="upper_shell",
    )
    _add_guide_strips(
        upper_mast,
        child_width=0.135,
        parent_inner_width=0.145,
        z_center=-1.05,
        height=0.18,
        name_prefix="upper_guide",
        material=dark_charcoal,
    )

    top_mast = model.part("top_mast")
    _add_square_tube_stage(
        top_mast,
        outer_width=0.108,
        wall=0.008,
        length=TOP_STAGE_LENGTH,
        hidden_below=TOP_STAGE_LENGTH - TOP_STAGE_REST_EXPOSED,
        material=galvanized,
        name_prefix="top_shell",
    )
    _add_guide_strips(
        top_mast,
        child_width=0.108,
        parent_inner_width=0.117,
        z_center=-0.93,
        height=0.16,
        name_prefix="top_guide",
        material=dark_charcoal,
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.065, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=dark_charcoal,
        name="slew_bearing",
    )
    pan_head.visual(
        Box((0.12, 0.22, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_charcoal,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.035, 0.020, 0.18)),
        origin=Origin(xyz=(0.0, 0.095, 0.19)),
        material=dark_charcoal,
        name="left_yoke_arm",
    )
    pan_head.visual(
        Box((0.035, 0.020, 0.18)),
        origin=Origin(xyz=(0.0, -0.095, 0.19)),
        material=dark_charcoal,
        name="right_yoke_arm",
    )
    pan_head.visual(
        Box((0.040, 0.22, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=dark_charcoal,
        name="top_crossbar",
    )

    camera = model.part("camera_unit")
    camera.visual(
        Box((0.19, 0.13, 0.14)),
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
        material=off_white,
        name="camera_shell",
    )
    camera.visual(
        Box((0.06, 0.10, 0.10)),
        origin=Origin(xyz=(-0.055, 0.0, 0.0)),
        material=off_white,
        name="rear_pod",
    )
    camera.visual(
        Cylinder(radius=0.040, length=0.080),
        origin=Origin(
            xyz=(0.205, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_glass,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(
            xyz=(0.0, 0.072, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_charcoal,
        name="left_trunnion",
    )
    camera.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(
            xyz=(0.0, -0.072, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_charcoal,
        name="right_trunnion",
    )

    model.articulation(
        "base_to_left_wheel",
        ArticulationType.FIXED,
        parent=base,
        child=left_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
    )
    model.articulation(
        "base_to_right_wheel",
        ArticulationType.FIXED,
        parent=base,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, -WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
    )
    model.articulation(
        "base_to_lower_mast",
        ArticulationType.FIXED,
        parent=base,
        child=lower_mast,
        origin=Origin(xyz=(MAST_X, 0.0, MAST_PAD_TOP)),
    )
    model.articulation(
        "lower_to_mid",
        ArticulationType.PRISMATIC,
        parent=lower_mast,
        child=mid_mast,
        origin=Origin(xyz=(0.0, 0.0, LOWER_OPENING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.20,
            lower=0.0,
            upper=MID_STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "mid_to_upper",
        ArticulationType.PRISMATIC,
        parent=mid_mast,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, MID_STAGE_REST_EXPOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.20,
            lower=0.0,
            upper=UPPER_STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "upper_to_top",
        ArticulationType.PRISMATIC,
        parent=upper_mast,
        child=top_mast,
        origin=Origin(xyz=(0.0, 0.0, UPPER_STAGE_REST_EXPOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.20,
            lower=0.0,
            upper=TOP_STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "mast_pan",
        ArticulationType.CONTINUOUS,
        parent=top_mast,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, TOP_STAGE_REST_EXPOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-1.05,
            upper=0.55,
        ),
    )

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

    base = object_model.get_part("base_frame")
    lower_mast = object_model.get_part("lower_mast")
    mid_mast = object_model.get_part("mid_mast")
    upper_mast = object_model.get_part("upper_mast")
    top_mast = object_model.get_part("top_mast")
    camera = object_model.get_part("camera_unit")

    lower_to_mid = object_model.get_articulation("lower_to_mid")
    mid_to_upper = object_model.get_articulation("mid_to_upper")
    upper_to_top = object_model.get_articulation("upper_to_top")
    mast_pan = object_model.get_articulation("mast_pan")
    camera_tilt = object_model.get_articulation("camera_tilt")

    mast_pad = base.get_visual("mast_pad")
    lower_flange = lower_mast.get_visual("lower_flange")
    mid_guide = mid_mast.get_visual("mid_guide_pos_x")
    upper_guide = upper_mast.get_visual("upper_guide_pos_x")
    top_guide = top_mast.get_visual("top_guide_pos_x")
    lens_barrel = camera.get_visual("lens_barrel")

    ctx.expect_contact(
        lower_mast,
        base,
        elem_a=lower_flange,
        elem_b=mast_pad,
        name="lower mast flange seats on trailer mast pad",
    )
    ctx.expect_contact(
        mid_mast,
        lower_mast,
        elem_a=mid_guide,
        contact_tol=1e-5,
        name="mid mast rides on lower sleeve guide strips",
    )
    ctx.expect_contact(
        upper_mast,
        mid_mast,
        elem_a=upper_guide,
        contact_tol=1e-5,
        name="upper mast rides on mid sleeve guide strips",
    )
    ctx.expect_contact(
        top_mast,
        upper_mast,
        elem_a=top_guide,
        contact_tol=1e-5,
        name="top mast rides on upper sleeve guide strips",
    )

    ctx.expect_within(
        mid_mast,
        lower_mast,
        axes="xy",
        margin=0.002,
        name="mid mast stays centered inside lower sleeve at rest",
    )
    ctx.expect_overlap(
        mid_mast,
        lower_mast,
        axes="z",
        min_overlap=1.20,
        name="mid mast remains deeply inserted at rest",
    )
    ctx.expect_within(
        upper_mast,
        mid_mast,
        axes="xy",
        margin=0.002,
        name="upper mast stays centered inside mid sleeve at rest",
    )
    ctx.expect_overlap(
        upper_mast,
        mid_mast,
        axes="z",
        min_overlap=1.00,
        name="upper mast remains deeply inserted at rest",
    )
    ctx.expect_within(
        top_mast,
        upper_mast,
        axes="xy",
        margin=0.002,
        name="top mast stays centered inside upper sleeve at rest",
    )
    ctx.expect_overlap(
        top_mast,
        upper_mast,
        axes="z",
        min_overlap=0.90,
        name="top mast remains deeply inserted at rest",
    )

    rest_lens_center = _aabb_center(
        ctx.part_element_world_aabb(camera, elem=lens_barrel)
    )
    with ctx.pose(
        {
            lower_to_mid: MID_STAGE_TRAVEL,
            mid_to_upper: UPPER_STAGE_TRAVEL,
            upper_to_top: TOP_STAGE_TRAVEL,
        }
    ):
        ctx.expect_within(
            mid_mast,
            lower_mast,
            axes="xy",
            margin=0.002,
            name="mid mast stays centered when fully extended",
        )
        ctx.expect_overlap(
            mid_mast,
            lower_mast,
            axes="z",
            min_overlap=MID_STAGE_RETAINED - 0.01,
            name="mid mast retains insertion when fully extended",
        )
        ctx.expect_within(
            upper_mast,
            mid_mast,
            axes="xy",
            margin=0.002,
            name="upper mast stays centered when fully extended",
        )
        ctx.expect_overlap(
            upper_mast,
            mid_mast,
            axes="z",
            min_overlap=UPPER_STAGE_RETAINED - 0.01,
            name="upper mast retains insertion when fully extended",
        )
        ctx.expect_within(
            top_mast,
            upper_mast,
            axes="xy",
            margin=0.002,
            name="top mast stays centered when fully extended",
        )
        ctx.expect_overlap(
            top_mast,
            upper_mast,
            axes="z",
            min_overlap=TOP_STAGE_RETAINED - 0.01,
            name="top mast retains insertion when fully extended",
        )
        extended_lens_center = _aabb_center(
            ctx.part_element_world_aabb(camera, elem=lens_barrel)
        )

    ctx.check(
        "telescoping mast lifts the camera assembly upward",
        rest_lens_center is not None
        and extended_lens_center is not None
        and extended_lens_center[2] > rest_lens_center[2] + 2.3,
        details=f"rest={rest_lens_center}, extended={extended_lens_center}",
    )

    with ctx.pose({mast_pan: math.pi / 2.0}):
        panned_lens_center = _aabb_center(
            ctx.part_element_world_aabb(camera, elem=lens_barrel)
        )
    ctx.check(
        "pan head swings the lens around the vertical mast axis",
        rest_lens_center is not None
        and panned_lens_center is not None
        and panned_lens_center[1] > rest_lens_center[1] + 0.12
        and panned_lens_center[0] < rest_lens_center[0] - 0.12,
        details=f"rest={rest_lens_center}, panned={panned_lens_center}",
    )

    with ctx.pose({camera_tilt: 0.45}):
        tilted_lens_center = _aabb_center(
            ctx.part_element_world_aabb(camera, elem=lens_barrel)
        )
    ctx.check(
        "tilt joint raises the camera nose for positive tilt angles",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > rest_lens_center[2] + 0.05,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
