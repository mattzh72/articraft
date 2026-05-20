from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
)


BASE_WIDTH = 0.90
BASE_DEPTH = 0.72
BASE_THICKNESS = 0.04
PEDESTAL_WIDTH = 0.30
PEDESTAL_DEPTH = 0.24
PEDESTAL_HEIGHT = 0.08

LOWER_WIDTH = 0.22
LOWER_DEPTH = 0.080
LOWER_RAIL_WIDTH = 0.028
LOWER_HEIGHT = 1.02
LOWER_TOP_BRIDGE_HEIGHT = 0.055
LOWER_BOTTOM_BRIDGE_HEIGHT = 0.060
LOWER_BOTTOM_BRIDGE_WIDTH = 0.18

MIDDLE_WIDTH = 0.14
MIDDLE_DEPTH = 0.055
MIDDLE_RAIL_WIDTH = 0.022
MIDDLE_LENGTH = 0.98
MIDDLE_HIDDEN = 0.34
MIDDLE_ABOVE = MIDDLE_LENGTH - MIDDLE_HIDDEN
MIDDLE_TRAVEL = 0.24
MIDDLE_TOP_BRIDGE_HEIGHT = 0.050
MIDDLE_BOTTOM_BRIDGE_HEIGHT = 0.045
MIDDLE_BOTTOM_BRIDGE_WIDTH = 0.11

INNER_WIDTH = 0.084
INNER_DEPTH = 0.040
INNER_RAIL_WIDTH = 0.018
INNER_LENGTH = 0.82
INNER_HIDDEN = 0.30
INNER_ABOVE = INNER_LENGTH - INNER_HIDDEN
INNER_TRAVEL = 0.20
INNER_TOP_BRIDGE_HEIGHT = 0.045
INNER_BOTTOM_BRIDGE_HEIGHT = 0.040
INNER_BOTTOM_BRIDGE_WIDTH = 0.064

MIDDLE_GUIDE_THICKNESS = (LOWER_WIDTH / 2.0 - LOWER_RAIL_WIDTH) - MIDDLE_WIDTH / 2.0
MIDDLE_GUIDE_DEPTH = 0.026
MIDDLE_GUIDE_BANDS = ((0.06, 0.10), (0.22, 0.08))

INNER_GUIDE_THICKNESS = (MIDDLE_WIDTH / 2.0 - MIDDLE_RAIL_WIDTH) - INNER_WIDTH / 2.0
INNER_GUIDE_DEPTH = 0.018
INNER_GUIDE_BANDS = ((0.05, 0.09), (0.17, 0.07))

PAN_BEARING_RADIUS = 0.045
PAN_BEARING_HEIGHT = 0.022
PAN_NECK_WIDTH = 0.050
PAN_NECK_DEPTH = 0.035
PAN_NECK_HEIGHT = 0.038
PAN_FACE_WIDTH = 0.18
PAN_FACE_THICKNESS = 0.014
PAN_FACE_HEIGHT = 0.14


def _stage_visual_specs(
    prefix: str,
    span_width: float,
    depth: float,
    rail_width: float,
    length: float,
    *,
    hidden: float,
    top_bridge_height: float,
    bottom_bridge_height: float,
    bottom_bridge_width: float,
    guide_thickness: float = 0.0,
    guide_depth: float = 0.0,
    guide_bands: tuple[tuple[float, float], ...] = (),
) -> list[tuple[Box, Origin, str]]:
    z_shift = -hidden
    rail_center_x = span_width / 2.0 - rail_width / 2.0
    top_cap_z = z_shift + length - top_bridge_height / 2.0
    specs: list[tuple[Box, Origin, str]] = [
        (
            Box((rail_width, depth, length)),
            Origin(xyz=(-rail_center_x, 0.0, z_shift + length / 2.0)),
            f"{prefix}_left_rail",
        ),
        (
            Box((rail_width, depth, length)),
            Origin(xyz=(rail_center_x, 0.0, z_shift + length / 2.0)),
            f"{prefix}_right_rail",
        ),
        (
            Box((rail_width, depth, top_bridge_height)),
            Origin(xyz=(-rail_center_x, 0.0, top_cap_z)),
            f"{prefix}_left_top_cap",
        ),
        (
            Box((rail_width, depth, top_bridge_height)),
            Origin(xyz=(rail_center_x, 0.0, top_cap_z)),
            f"{prefix}_right_top_cap",
        ),
        (
            Box((bottom_bridge_width, depth, bottom_bridge_height)),
            Origin(xyz=(0.0, 0.0, z_shift + bottom_bridge_height / 2.0)),
            f"{prefix}_bottom_bridge",
        ),
    ]
    if guide_thickness > 0.0:
        guide_center_x = span_width / 2.0 + guide_thickness / 2.0
        for idx, (z_base, height) in enumerate(guide_bands, start=1):
            z_center = z_shift + z_base + height / 2.0
            specs.append(
                (
                    Box((guide_thickness, guide_depth, height)),
                    Origin(xyz=(-guide_center_x, 0.0, z_center)),
                    f"{prefix}_left_guide_{idx}",
                )
            )
            specs.append(
                (
                    Box((guide_thickness, guide_depth, height)),
                    Origin(xyz=(guide_center_x, 0.0, z_center)),
                    f"{prefix}_right_guide_{idx}",
                )
            )
    return specs


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_lift_mast_pan_face")

    model.material("base_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("mast_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("stage_silver", rgba=(0.69, 0.72, 0.76, 1.0))
    model.material("face_dark", rgba=(0.24, 0.25, 0.28, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="base_steel",
        name="base_body",
    )
    base.visual(
        Box((PEDESTAL_WIDTH, PEDESTAL_DEPTH, PEDESTAL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0)),
        material="base_steel",
        name="base_pedestal",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS + PEDESTAL_HEIGHT)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + PEDESTAL_HEIGHT) / 2.0)),
    )

    lower_mast = model.part("lower_mast")
    for geometry, origin, name in _stage_visual_specs(
        "lower",
        LOWER_WIDTH,
        LOWER_DEPTH,
        LOWER_RAIL_WIDTH,
        LOWER_HEIGHT,
        hidden=0.0,
        top_bridge_height=LOWER_TOP_BRIDGE_HEIGHT,
        bottom_bridge_height=LOWER_BOTTOM_BRIDGE_HEIGHT,
        bottom_bridge_width=LOWER_BOTTOM_BRIDGE_WIDTH,
    ):
        lower_mast.visual(geometry, origin=origin, material="mast_steel", name=name)
    lower_mast.inertial = Inertial.from_geometry(
        Box((LOWER_WIDTH, LOWER_DEPTH, LOWER_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, LOWER_HEIGHT / 2.0)),
    )

    middle_stage = model.part("middle_stage")
    for geometry, origin, name in _stage_visual_specs(
        "middle",
        MIDDLE_WIDTH,
        MIDDLE_DEPTH,
        MIDDLE_RAIL_WIDTH,
        MIDDLE_LENGTH,
        hidden=MIDDLE_HIDDEN,
        top_bridge_height=MIDDLE_TOP_BRIDGE_HEIGHT,
        bottom_bridge_height=MIDDLE_BOTTOM_BRIDGE_HEIGHT,
        bottom_bridge_width=MIDDLE_BOTTOM_BRIDGE_WIDTH,
        guide_thickness=MIDDLE_GUIDE_THICKNESS,
        guide_depth=MIDDLE_GUIDE_DEPTH,
        guide_bands=MIDDLE_GUIDE_BANDS,
    ):
        middle_stage.visual(geometry, origin=origin, material="stage_silver", name=name)
    middle_stage.inertial = Inertial.from_geometry(
        Box((MIDDLE_WIDTH, MIDDLE_DEPTH, MIDDLE_LENGTH)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, (MIDDLE_ABOVE - MIDDLE_HIDDEN) / 2.0)),
    )

    inner_stage = model.part("inner_stage")
    for geometry, origin, name in _stage_visual_specs(
        "inner",
        INNER_WIDTH,
        INNER_DEPTH,
        INNER_RAIL_WIDTH,
        INNER_LENGTH,
        hidden=INNER_HIDDEN,
        top_bridge_height=INNER_TOP_BRIDGE_HEIGHT,
        bottom_bridge_height=INNER_BOTTOM_BRIDGE_HEIGHT,
        bottom_bridge_width=INNER_BOTTOM_BRIDGE_WIDTH,
        guide_thickness=INNER_GUIDE_THICKNESS,
        guide_depth=INNER_GUIDE_DEPTH,
        guide_bands=INNER_GUIDE_BANDS,
    ):
        inner_stage.visual(geometry, origin=origin, material="stage_silver", name=name)
    inner_stage.inertial = Inertial.from_geometry(
        Box((INNER_WIDTH, INNER_DEPTH, INNER_LENGTH)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, (INNER_ABOVE - INNER_HIDDEN) / 2.0)),
    )

    pan_face = model.part("pan_face")
    pan_face.visual(
        Cylinder(radius=PAN_BEARING_RADIUS, length=PAN_BEARING_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PAN_BEARING_HEIGHT / 2.0)),
        material="face_dark",
        name="pan_face_bearing",
    )
    pan_face.visual(
        Cylinder(radius=PAN_BEARING_RADIUS * 0.86, length=PAN_BEARING_HEIGHT * 0.45),
        origin=Origin(xyz=(0.0, 0.0, PAN_BEARING_HEIGHT + PAN_BEARING_HEIGHT * 0.225)),
        material="face_dark",
        name="pan_face_cap",
    )
    pan_face.visual(
        Box((PAN_NECK_WIDTH, PAN_NECK_DEPTH, PAN_NECK_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PAN_BEARING_HEIGHT + PAN_NECK_HEIGHT / 2.0)),
        material="face_dark",
        name="pan_face_neck",
    )
    pan_face.visual(
        Box((PAN_FACE_WIDTH, PAN_FACE_THICKNESS, PAN_FACE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                PAN_BEARING_HEIGHT + PAN_NECK_HEIGHT + PAN_FACE_HEIGHT / 2.0,
            )
        ),
        material="face_dark",
        name="pan_face_body",
    )
    pan_face.inertial = Inertial.from_geometry(
        Box((PAN_FACE_WIDTH, PAN_BEARING_RADIUS * 2.0, PAN_FACE_HEIGHT + PAN_BEARING_HEIGHT + PAN_NECK_HEIGHT)),
        mass=3.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (PAN_BEARING_HEIGHT + PAN_NECK_HEIGHT + PAN_FACE_HEIGHT) / 2.0,
            )
        ),
    )

    model.articulation(
        "base_to_lower_mast",
        ArticulationType.FIXED,
        parent=base,
        child=lower_mast,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT)),
    )
    model.articulation(
        "lower_to_middle_stage",
        ArticulationType.PRISMATIC,
        parent=lower_mast,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.30,
            lower=0.0,
            upper=MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner_stage",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_ABOVE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=0.30,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )
    model.articulation(
        "inner_stage_to_pan_face",
        ArticulationType.REVOLUTE,
        parent=inner_stage,
        child=pan_face,
        origin=Origin(xyz=(0.0, 0.0, INNER_ABOVE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.25,
            lower=-2.6,
            upper=2.6,
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
    base = object_model.get_part("base")
    lower_mast = object_model.get_part("lower_mast")
    middle_stage = object_model.get_part("middle_stage")
    inner_stage = object_model.get_part("inner_stage")
    pan_face = object_model.get_part("pan_face")
    lower_left_rail = lower_mast.get_visual("lower_left_rail")
    lower_right_rail = lower_mast.get_visual("lower_right_rail")
    middle_left_guide = middle_stage.get_visual("middle_left_guide_1")
    middle_right_guide = middle_stage.get_visual("middle_right_guide_1")
    inner_left_guide = inner_stage.get_visual("inner_left_guide_1")
    inner_right_guide = inner_stage.get_visual("inner_right_guide_1")
    middle_left_rail = middle_stage.get_visual("middle_left_rail")
    middle_right_rail = middle_stage.get_visual("middle_right_rail")

    lower_to_middle = object_model.get_articulation("lower_to_middle_stage")
    middle_to_inner = object_model.get_articulation("middle_to_inner_stage")
    pan_joint = object_model.get_articulation("inner_stage_to_pan_face")

    ctx.check(
        "lift joints translate along +Z",
        lower_to_middle.axis == (0.0, 0.0, 1.0) and middle_to_inner.axis == (0.0, 0.0, 1.0),
        details=f"lower_to_middle={lower_to_middle.axis}, middle_to_inner={middle_to_inner.axis}",
    )
    ctx.check(
        "pan joint rotates about +Z",
        pan_joint.axis == (0.0, 0.0, 1.0),
        details=f"pan_axis={pan_joint.axis}",
    )

    ctx.expect_gap(
        lower_mast,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        name="lower mast sits on the grounded base pedestal",
    )
    ctx.expect_within(
        middle_stage,
        lower_mast,
        axes="xy",
        margin=0.0,
        name="middle stage stays centered inside lower mast",
    )
    ctx.expect_contact(
        middle_stage,
        lower_mast,
        elem_a=middle_left_guide,
        elem_b=lower_left_rail,
        name="middle stage left guide bears on lower mast rail",
    )
    ctx.expect_overlap(
        middle_stage,
        lower_mast,
        axes="z",
        min_overlap=0.10,
        name="middle stage remains inserted in lower mast at rest",
    )
    ctx.expect_within(
        inner_stage,
        middle_stage,
        axes="xy",
        margin=0.0,
        name="inner stage stays centered inside middle stage",
    )
    ctx.expect_contact(
        inner_stage,
        middle_stage,
        elem_a=inner_left_guide,
        elem_b=middle_left_rail,
        name="inner stage left guide bears on middle stage rail",
    )
    ctx.expect_overlap(
        inner_stage,
        middle_stage,
        axes="z",
        min_overlap=0.12,
        name="inner stage remains inserted in middle stage at rest",
    )
    ctx.expect_gap(
        pan_face,
        inner_stage,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        name="pan face seats on top of the inner stage",
    )

    middle_rest = ctx.part_world_position(middle_stage)
    inner_rest = ctx.part_world_position(inner_stage)
    pan_rest_aabb = ctx.part_world_aabb(pan_face)
    with ctx.pose({lower_to_middle: MIDDLE_TRAVEL, middle_to_inner: INNER_TRAVEL, pan_joint: pi / 2.0}):
        ctx.expect_within(
            middle_stage,
            lower_mast,
            axes="xy",
            margin=0.0,
            name="extended middle stage stays centered inside lower mast",
        )
        ctx.expect_contact(
            middle_stage,
            lower_mast,
            elem_a=middle_right_guide,
            elem_b=lower_right_rail,
            name="middle stage retains rail support when extended",
        )
        ctx.expect_overlap(
            middle_stage,
            lower_mast,
            axes="z",
            min_overlap=0.09,
            name="extended middle stage still retains insertion in lower mast",
        )
        ctx.expect_within(
            inner_stage,
            middle_stage,
            axes="xy",
            margin=0.0,
            name="extended inner stage stays centered inside middle stage",
        )
        ctx.expect_contact(
            inner_stage,
            middle_stage,
            elem_a=inner_right_guide,
            elem_b=middle_right_rail,
            name="inner stage retains rail support when extended",
        )
        ctx.expect_overlap(
            inner_stage,
            middle_stage,
            axes="z",
            min_overlap=0.095,
            name="extended inner stage still retains insertion in middle stage",
        )
        middle_extended = ctx.part_world_position(middle_stage)
        inner_extended = ctx.part_world_position(inner_stage)
        pan_rotated_aabb = ctx.part_world_aabb(pan_face)

    ctx.check(
        "middle stage rises when extended",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[2] > middle_rest[2] + 0.10,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "inner stage rises when extended",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[2] > inner_rest[2] + 0.10,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    if pan_rest_aabb is None or pan_rotated_aabb is None:
        ctx.fail("pan face produces measurable footprint change", "missing faceplate AABB in one or more poses")
    else:
        rest_dx = pan_rest_aabb[1][0] - pan_rest_aabb[0][0]
        rest_dy = pan_rest_aabb[1][1] - pan_rest_aabb[0][1]
        rot_dx = pan_rotated_aabb[1][0] - pan_rotated_aabb[0][0]
        rot_dy = pan_rotated_aabb[1][1] - pan_rotated_aabb[0][1]
        ctx.check(
            "pan face yaws around the mast axis",
            rest_dx > rest_dy + 0.05 and rot_dy > rot_dx + 0.05,
            details=(
                f"rest_dx={rest_dx:.4f}, rest_dy={rest_dy:.4f}, "
                f"rot_dx={rot_dx:.4f}, rot_dy={rot_dy:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
