from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.17
BASE_THICKNESS = 0.044
BASE_BOSS_RADIUS = 0.072
BASE_BOSS_HEIGHT = 0.028
BASE_TOTAL_HEIGHT = BASE_THICKNESS + BASE_BOSS_HEIGHT

MAST_RADIUS = 0.027
MAST_HEIGHT = 0.37
MAST_FLANGE_RADIUS = 0.045
MAST_FLANGE_HEIGHT = 0.015
MAST_CAP_RADIUS = 0.032
MAST_CAP_HEIGHT = 0.014

CLAMP_RING_RADIUS = 0.051
CLAMP_HEIGHT = 0.042
CLAMP_BLOCK_WIDTH = 0.052
SUPPORT_AXIS_OFFSET = 0.104
SUPPORT_BLOCK_LENGTH = 0.034
SUPPORT_BLOCK_HEIGHT = 0.05
SUPPORT_BLOCK_WIDTH = 0.046

LONG_BRANCH_LENGTH = 0.34
LONG_BRANCH_WIDTH = 0.032
LONG_BRANCH_HEIGHT = 0.025
SHORT_BRANCH_LENGTH = 0.22
SHORT_BRANCH_WIDTH = 0.029
SHORT_BRANCH_HEIGHT = 0.022
BRANCH_HOUSING_RADIUS = 0.026
BRANCH_HOUSING_HEIGHT = 0.02
BRANCH_CAP_RADIUS = 0.018
BRANCH_CAP_HEIGHT = 0.01
BRANCH_ROOT_LENGTH = 0.052

LONG_COLLAR_Z = 0.31
SHORT_COLLAR_Z = 0.235

ROTARY_LIMIT = 2.5


def _base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .circle(BASE_RADIUS)
        .extrude(BASE_THICKNESS)
        .faces(">Z")
        .edges()
        .chamfer(0.007)
    )
    boss = (
        cq.Workplane("XY")
        .circle(BASE_BOSS_RADIUS)
        .extrude(BASE_BOSS_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    return foot.union(boss)


def _mast_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(MAST_FLANGE_RADIUS).extrude(MAST_FLANGE_HEIGHT)
    shaft = cq.Workplane("XY").circle(MAST_RADIUS).extrude(MAST_HEIGHT)
    cap = (
        cq.Workplane("XY")
        .circle(MAST_CAP_RADIUS)
        .extrude(MAST_CAP_HEIGHT)
        .translate((0.0, 0.0, MAST_HEIGHT))
    )
    return flange.union(shaft).union(_collar_block_shape(-1.0, SHORT_COLLAR_Z)).union(
        _collar_block_shape(1.0, LONG_COLLAR_Z)
    ).union(cap)


def _collar_block_shape(side: float, support_top_z: float) -> cq.Workplane:
    clamp_bottom_z = support_top_z - CLAMP_HEIGHT
    clamp_center_z = clamp_bottom_z + (CLAMP_HEIGHT / 2.0)
    support_center_x = side * (SUPPORT_AXIS_OFFSET - (SUPPORT_BLOCK_LENGTH / 2.0))
    support_center_z = support_top_z - (SUPPORT_BLOCK_HEIGHT / 2.0)
    bridge_start_x = side * CLAMP_RING_RADIUS
    bridge_end_x = side * (SUPPORT_AXIS_OFFSET - SUPPORT_BLOCK_LENGTH)
    bridge_length = abs(bridge_end_x - bridge_start_x)
    bridge_center_x = (bridge_start_x + bridge_end_x) / 2.0

    ring = (
        cq.Workplane("XY")
        .circle(CLAMP_RING_RADIUS)
        .extrude(CLAMP_HEIGHT)
        .translate((0.0, 0.0, clamp_bottom_z))
    )
    bridge = (
        cq.Workplane("XY")
        .box(bridge_length, CLAMP_BLOCK_WIDTH, CLAMP_HEIGHT)
        .translate((bridge_center_x, 0.0, clamp_center_z))
    )
    support_block = (
        cq.Workplane("XY")
        .box(SUPPORT_BLOCK_LENGTH, SUPPORT_BLOCK_WIDTH, SUPPORT_BLOCK_HEIGHT)
        .translate((support_center_x, 0.0, support_center_z))
    )
    rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (side * (CLAMP_RING_RADIUS - 0.01), clamp_bottom_z + 0.006),
                (side * (CLAMP_RING_RADIUS + 0.012), clamp_bottom_z + CLAMP_HEIGHT - 0.004),
                (side * (SUPPORT_AXIS_OFFSET - SUPPORT_BLOCK_LENGTH + 0.004), support_top_z - 0.008),
                (side * (SUPPORT_AXIS_OFFSET - SUPPORT_BLOCK_LENGTH + 0.004), clamp_bottom_z + 0.008),
            ]
        )
        .close()
        .extrude(CLAMP_BLOCK_WIDTH * 0.42)
        .translate((0.0, -(CLAMP_BLOCK_WIDTH * 0.21), 0.0))
    )

    return ring.union(bridge).union(support_block).union(rib)


def _branch_shape(
    *,
    arm_length: float,
    arm_width: float,
    arm_height: float,
    name_bias: str,
) -> cq.Workplane:
    root_width = arm_width * 1.35
    root_height = max(arm_height + 0.006, 0.028)
    root_block = (
        cq.Workplane("XY")
        .box(BRANCH_ROOT_LENGTH, root_width, root_height)
        .translate((BRANCH_ROOT_LENGTH / 2.0, 0.0, root_height / 2.0))
    )
    housing_center_x = BRANCH_HOUSING_RADIUS
    housing = (
        cq.Workplane("XY")
        .circle(BRANCH_HOUSING_RADIUS)
        .extrude(BRANCH_HOUSING_HEIGHT)
        .translate((housing_center_x, 0.0, 0.0))
    )
    hub_cap = (
        cq.Workplane("XY")
        .circle(BRANCH_CAP_RADIUS)
        .extrude(BRANCH_CAP_HEIGHT)
        .translate((housing_center_x, 0.0, BRANCH_HOUSING_HEIGHT))
    )

    beam_start_x = BRANCH_ROOT_LENGTH - 0.004
    beam_center_x = beam_start_x + (arm_length / 2.0)
    beam_center_z = max(root_height * 0.48, arm_height / 2.0)
    beam = cq.Workplane("XY").box(
        arm_length,
        arm_width,
        arm_height,
    ).translate((beam_center_x, 0.0, beam_center_z))

    tip_radius = max(arm_width * 0.72, 0.019 if name_bias == "short" else 0.022)
    tip_height = arm_height * 1.35
    tip = (
        cq.Workplane("XY")
        .circle(tip_radius)
        .extrude(tip_height)
        .translate(
            (
                beam_start_x + arm_length - (tip_radius * 0.18),
                0.0,
                0.0,
            )
        )
    )

    gusset_thickness = arm_width * 0.58
    gusset_end_x = beam_start_x + min(0.11, arm_length * 0.42)
    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, 0.0),
                (BRANCH_ROOT_LENGTH * 0.38, root_height + 0.003),
                (beam_start_x + 0.055, arm_height * 0.95),
                (gusset_end_x, arm_height * 0.24),
            ]
        )
        .close()
        .extrude(gusset_thickness)
        .translate((0.0, -gusset_thickness / 2.0, 0.0))
    )

    return root_block.union(housing).union(hub_cap).union(beam).union(tip).union(gusset)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_two_branch_rotary_head")

    model.material("powder_black", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("cast_aluminum", rgba=(0.68, 0.70, 0.73, 1.0))

    base_foot = model.part("base_foot")
    base_foot.visual(
        mesh_from_cadquery(_base_shape(), "base_foot"),
        material="powder_black",
        name="base_shell",
    )
    base_foot.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_TOTAL_HEIGHT),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT / 2.0)),
    )

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_mast_shape(), "mast"),
        material="dark_steel",
        name="mast_shell",
    )
    mast.inertial = Inertial.from_geometry(
        Cylinder(radius=MAST_FLANGE_RADIUS, length=MAST_HEIGHT + MAST_CAP_HEIGHT),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, (MAST_HEIGHT + MAST_CAP_HEIGHT) / 2.0)),
    )

    long_branch = model.part("long_branch")
    long_branch.visual(
        mesh_from_cadquery(
            _branch_shape(
                arm_length=LONG_BRANCH_LENGTH,
                arm_width=LONG_BRANCH_WIDTH,
                arm_height=LONG_BRANCH_HEIGHT,
                name_bias="long",
            ),
            "long_branch",
        ),
        material="cast_aluminum",
        name="long_branch_body",
    )

    short_branch = model.part("short_branch")
    short_branch.visual(
        mesh_from_cadquery(
            _branch_shape(
                arm_length=SHORT_BRANCH_LENGTH,
                arm_width=SHORT_BRANCH_WIDTH,
                arm_height=SHORT_BRANCH_HEIGHT,
                name_bias="short",
            ),
            "short_branch",
        ),
        material="cast_aluminum",
        name="short_branch_body",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.FIXED,
        parent=base_foot,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
    )
    model.articulation(
        "mast_to_long_branch",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=long_branch,
        origin=Origin(xyz=(SUPPORT_AXIS_OFFSET, 0.0, LONG_COLLAR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-ROTARY_LIMIT,
            upper=ROTARY_LIMIT,
            effort=22.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "mast_to_short_branch",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=short_branch,
        origin=Origin(xyz=(-SUPPORT_AXIS_OFFSET, 0.0, SHORT_COLLAR_Z), rpy=(0.0, 0.0, pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-ROTARY_LIMIT,
            upper=ROTARY_LIMIT,
            effort=18.0,
            velocity=1.8,
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

    base_foot = object_model.get_part("base_foot")
    mast = object_model.get_part("mast")
    long_branch = object_model.get_part("long_branch")
    short_branch = object_model.get_part("short_branch")

    long_joint = object_model.get_articulation("mast_to_long_branch")
    short_joint = object_model.get_articulation("mast_to_short_branch")

    ctx.expect_contact(mast, base_foot, name="mast seats directly on the heavy foot")
    ctx.expect_contact(long_branch, mast, name="long branch sits on its upper collar block")
    ctx.expect_contact(short_branch, mast, name="short branch sits on its lower collar block")
    ctx.expect_origin_gap(
        long_branch,
        short_branch,
        axis="z",
        min_gap=0.06,
        max_gap=0.09,
        name="the two collar blocks are stacked at separate mast heights",
    )

    ctx.check(
        "both branch joints use independent vertical support axes",
        tuple(long_joint.axis) == (0.0, 0.0, 1.0) and tuple(short_joint.axis) == (0.0, 0.0, 1.0),
        details=f"long_axis={long_joint.axis}, short_axis={short_joint.axis}",
    )
    ctx.check(
        "the two support axes sit on opposite sides of the mast",
        long_joint.origin.xyz[0] > 0.09 and short_joint.origin.xyz[0] < -0.09,
        details=f"long_origin={long_joint.origin.xyz}, short_origin={short_joint.origin.xyz}",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        return tuple((low + high) / 2.0 for low, high in zip(aabb[0], aabb[1]))

    long_rest = ctx.part_element_world_aabb(long_branch, elem="long_branch_body")
    short_rest = ctx.part_element_world_aabb(short_branch, elem="short_branch_body")
    ctx.check(
        "one branch is visibly longer than the other",
        long_rest is not None
        and short_rest is not None
        and (long_rest[1][0] - long_rest[0][0]) > (short_rest[1][0] - short_rest[0][0]) + 0.08,
        details=f"long_aabb={long_rest}, short_aabb={short_rest}",
    )

    with ctx.pose({long_joint: 1.1}):
        long_swung = ctx.part_element_world_aabb(long_branch, elem="long_branch_body")
        short_steady = ctx.part_element_world_aabb(short_branch, elem="short_branch_body")
        long_rest_center = _aabb_center(long_rest) if long_rest is not None else None
        long_swung_center = _aabb_center(long_swung) if long_swung is not None else None
        short_rest_center = _aabb_center(short_rest) if short_rest is not None else None
        short_steady_center = _aabb_center(short_steady) if short_steady is not None else None
        ctx.check(
            "moving the long branch does not drag the short branch",
            long_rest_center is not None
            and long_swung_center is not None
            and short_rest_center is not None
            and short_steady_center is not None
            and abs(long_swung_center[1] - long_rest_center[1]) > 0.07
            and abs(short_steady_center[1] - short_rest_center[1]) < 1e-6,
            details=(
                f"long_rest_center={long_rest_center}, long_swung_center={long_swung_center}, "
                f"short_rest_center={short_rest_center}, short_steady_center={short_steady_center}"
            ),
        )

    with ctx.pose({short_joint: -1.0}):
        long_steady = ctx.part_element_world_aabb(long_branch, elem="long_branch_body")
        short_swung = ctx.part_element_world_aabb(short_branch, elem="short_branch_body")
        long_rest_center = _aabb_center(long_rest) if long_rest is not None else None
        long_steady_center = _aabb_center(long_steady) if long_steady is not None else None
        short_rest_center = _aabb_center(short_rest) if short_rest is not None else None
        short_swung_center = _aabb_center(short_swung) if short_swung is not None else None
        ctx.check(
            "moving the short branch leaves the long branch independent",
            long_rest_center is not None
            and long_steady_center is not None
            and short_rest_center is not None
            and short_swung_center is not None
            and abs(short_swung_center[1] - short_rest_center[1]) > 0.05
            and abs(long_steady_center[1] - long_rest_center[1]) < 1e-6,
            details=(
                f"long_rest_center={long_rest_center}, long_steady_center={long_steady_center}, "
                f"short_rest_center={short_rest_center}, short_swung_center={short_swung_center}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
