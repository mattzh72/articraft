from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_THICKNESS = 0.020
SUPPORT_SHELF_TOP_Z = 0.100

OUTER_STAGE_SIZE = 0.080
OUTER_STAGE_WALL = 0.004
OUTER_STAGE_HEIGHT = 0.740
OUTER_COLLAR_HEIGHT = 0.030

MIDDLE_STAGE_SIZE = 0.064
MIDDLE_STAGE_WALL = 0.0035
MIDDLE_STAGE_BOTTOM = -0.420
MIDDLE_STAGE_TOP = 0.260
MIDDLE_STAGE_TRAVEL = 0.300

INNER_STAGE_SIZE = 0.050
INNER_STAGE_WALL = 0.003
INNER_STAGE_BOTTOM = -0.340
INNER_STAGE_TOP = 0.220
INNER_STAGE_TRAVEL = 0.240

TOP_MOUNT_SIZE = 0.090
TOP_MOUNT_THICKNESS = 0.008

FACEPLATE_RADIUS = 0.070
FACEPLATE_THICKNESS = 0.012


def _tube_shape(
    *,
    outer_size: float,
    wall: float,
    bottom_z: float,
    top_z: float,
    top_collar_outer: float | None = None,
    top_collar_inner: float | None = None,
    top_collar_height: float = 0.0,
) -> cq.Workplane:
    inner_size = outer_size - 2.0 * wall
    length = top_z - bottom_z
    tube = (
        cq.Workplane("XY")
        .box(outer_size, outer_size, length)
        .translate((0.0, 0.0, bottom_z + length / 2.0))
    )
    core = (
        cq.Workplane("XY")
        .box(inner_size, inner_size, length + 0.004)
        .translate((0.0, 0.0, bottom_z + length / 2.0))
    )
    tube = tube.cut(core)

    if (
        top_collar_outer is not None
        and top_collar_inner is not None
        and top_collar_height > 0.0
    ):
        collar = (
            cq.Workplane("XY")
            .box(top_collar_outer, top_collar_outer, top_collar_height)
            .translate((0.0, 0.0, top_z - top_collar_height / 2.0))
        )
        collar_core = (
            cq.Workplane("XY")
            .box(top_collar_inner, top_collar_inner, top_collar_height + 0.004)
            .translate((0.0, 0.0, top_z - top_collar_height / 2.0))
        )
        collar = collar.cut(collar_core)
        tube = tube.union(collar)

    return tube


def _rear_support_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.240, 0.180, BASE_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS / 2.0)
    )
    pedestal = cq.Workplane("XY").box(0.112, 0.112, 0.080).translate((0.0, 0.0, 0.060))
    rear_spine = cq.Workplane("XY").box(0.160, 0.016, 0.360).translate(
        (0.0, -0.058, 0.200)
    )
    left_fork = cq.Workplane("XY").box(0.016, 0.080, 0.180).translate(
        (-0.060, -0.018, 0.190)
    )
    right_fork = cq.Workplane("XY").box(0.016, 0.080, 0.180).translate(
        (0.060, -0.018, 0.190)
    )

    left_gusset = cq.Workplane("XY").box(0.020, 0.050, 0.140).translate(
        (-0.064, -0.024, 0.090)
    )
    right_gusset = cq.Workplane("XY").box(0.020, 0.050, 0.140).translate(
        (0.064, -0.024, 0.090)
    )

    support = (
        base.union(pedestal)
        .union(rear_spine)
        .union(left_fork)
        .union(right_fork)
        .union(left_gusset)
        .union(right_gusset)
    )

    return support


def _outer_stage_shape() -> cq.Workplane:
    stage = _tube_shape(
        outer_size=OUTER_STAGE_SIZE,
        wall=OUTER_STAGE_WALL,
        bottom_z=0.0,
        top_z=OUTER_STAGE_HEIGHT,
        top_collar_outer=OUTER_STAGE_SIZE + 0.010,
        top_collar_inner=MIDDLE_STAGE_SIZE + 0.004,
        top_collar_height=OUTER_COLLAR_HEIGHT,
    )
    bottom_cap = cq.Workplane("XY").box(OUTER_STAGE_SIZE, OUTER_STAGE_SIZE, 0.012).translate(
        (0.0, 0.0, 0.006)
    )
    return stage.union(bottom_cap)


def _middle_stage_shape() -> cq.Workplane:
    stage = _tube_shape(
        outer_size=MIDDLE_STAGE_SIZE,
        wall=MIDDLE_STAGE_WALL,
        bottom_z=MIDDLE_STAGE_BOTTOM,
        top_z=MIDDLE_STAGE_TOP,
        top_collar_outer=MIDDLE_STAGE_SIZE + 0.008,
        top_collar_inner=INNER_STAGE_SIZE + 0.004,
        top_collar_height=0.028,
    )
    guide_z = -0.255
    guide_h = 0.210
    side_pad = cq.Workplane("XY").box(0.005, 0.020, guide_h).translate(
        (0.0335, 0.0, guide_z)
    )
    fore_pad = cq.Workplane("XY").box(0.020, 0.005, guide_h).translate(
        (0.0, 0.0335, guide_z)
    )
    return (
        stage.union(side_pad)
        .union(side_pad.mirror(mirrorPlane="YZ"))
        .union(fore_pad)
        .union(fore_pad.mirror(mirrorPlane="XZ"))
    )


def _inner_stage_shape() -> cq.Workplane:
    stage = _tube_shape(
        outer_size=INNER_STAGE_SIZE,
        wall=INNER_STAGE_WALL,
        bottom_z=INNER_STAGE_BOTTOM,
        top_z=INNER_STAGE_TOP,
    )
    guide_z = -0.205
    guide_h = 0.185
    side_pad = cq.Workplane("XY").box(0.005, 0.016, guide_h).translate(
        (0.026, 0.0, guide_z)
    )
    fore_pad = cq.Workplane("XY").box(0.016, 0.005, guide_h).translate(
        (0.0, 0.026, guide_z)
    )
    bearing_pedestal = cq.Workplane("XY").cylinder(0.016, 0.028).translate(
        (0.0, 0.0, INNER_STAGE_TOP - 0.008)
    )
    top_mount = cq.Workplane("XY").box(
        TOP_MOUNT_SIZE, TOP_MOUNT_SIZE, TOP_MOUNT_THICKNESS
    ).translate((0.0, 0.0, INNER_STAGE_TOP + TOP_MOUNT_THICKNESS / 2.0))
    return (
        stage.union(side_pad)
        .union(side_pad.mirror(mirrorPlane="YZ"))
        .union(fore_pad)
        .union(fore_pad.mirror(mirrorPlane="XZ"))
        .union(bearing_pedestal)
        .union(top_mount)
    )


def _faceplate_shape() -> cq.Workplane:
    disk = cq.Workplane("XY").cylinder(FACEPLATE_THICKNESS, FACEPLATE_RADIUS).translate(
        (0.0, 0.0, FACEPLATE_THICKNESS / 2.0)
    )
    hub = cq.Workplane("XY").cylinder(0.026, 0.022).translate(
        (0.0, 0.0, FACEPLATE_THICKNESS + 0.013)
    )
    return disk.union(hub)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def _add_square_tube(
    part,
    *,
    prefix: str,
    outer_size: float,
    wall: float,
    bottom_z: float,
    top_z: float,
    material: str,
) -> None:
    length = top_z - bottom_z
    center_z = (bottom_z + top_z) / 2.0
    side_span = outer_size - 2.0 * wall

    part.visual(
        Box((outer_size, wall, length)),
        origin=Origin(xyz=(0.0, outer_size / 2.0 - wall / 2.0, center_z)),
        material=material,
        name=f"{prefix}_front",
    )
    part.visual(
        Box((outer_size, wall, length)),
        origin=Origin(xyz=(0.0, -outer_size / 2.0 + wall / 2.0, center_z)),
        material=material,
        name=f"{prefix}_rear",
    )
    part.visual(
        Box((wall, side_span, length)),
        origin=Origin(xyz=(outer_size / 2.0 - wall / 2.0, 0.0, center_z)),
        material=material,
        name=f"{prefix}_right",
    )
    part.visual(
        Box((wall, side_span, length)),
        origin=Origin(xyz=(-outer_size / 2.0 + wall / 2.0, 0.0, center_z)),
        material=material,
        name=f"{prefix}_left",
    )


def _add_guide_pads(
    part,
    *,
    prefix: str,
    outer_reach: float,
    inner_reach: float,
    pad_span: float,
    pad_height: float,
    center_z: float,
    material: str,
) -> None:
    thickness = outer_reach - inner_reach
    center = (outer_reach + inner_reach) / 2.0
    part.visual(
        Box((thickness, pad_span, pad_height)),
        origin=Origin(xyz=(center, 0.0, center_z)),
        material=material,
        name=f"{prefix}_right",
    )
    part.visual(
        Box((thickness, pad_span, pad_height)),
        origin=Origin(xyz=(-center, 0.0, center_z)),
        material=material,
        name=f"{prefix}_left",
    )
    part.visual(
        Box((pad_span, thickness, pad_height)),
        origin=Origin(xyz=(0.0, center, center_z)),
        material=material,
        name=f"{prefix}_front",
    )
    part.visual(
        Box((pad_span, thickness, pad_height)),
        origin=Origin(xyz=(0.0, -center, center_z)),
        material=material,
        name=f"{prefix}_rear",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_telescoping_mast")

    model.material("powder_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("mast_gray", rgba=(0.64, 0.67, 0.71, 1.0))
    model.material("mast_light", rgba=(0.77, 0.80, 0.83, 1.0))
    model.material("faceplate_gray", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("indicator_red", rgba=(0.72, 0.16, 0.12, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.240, 0.180, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="powder_black",
        name="support_base",
    )
    rear_support.visual(
        Box((0.112, 0.112, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material="powder_black",
        name="support_pedestal",
    )
    rear_support.visual(
        Box((0.160, 0.016, 0.360)),
        origin=Origin(xyz=(0.0, -0.058, 0.200)),
        material="powder_black",
        name="rear_spine",
    )
    rear_support.visual(
        Box((0.016, 0.080, 0.180)),
        origin=Origin(xyz=(-0.060, -0.018, 0.190)),
        material="powder_black",
        name="left_fork",
    )
    rear_support.visual(
        Box((0.016, 0.080, 0.180)),
        origin=Origin(xyz=(0.060, -0.018, 0.190)),
        material="powder_black",
        name="right_fork",
    )
    rear_support.visual(
        Box((0.020, 0.050, 0.140)),
        origin=Origin(xyz=(-0.064, -0.024, 0.090)),
        material="powder_black",
        name="left_brace",
    )
    rear_support.visual(
        Box((0.020, 0.050, 0.140)),
        origin=Origin(xyz=(0.064, -0.024, 0.090)),
        material="powder_black",
        name="right_brace",
    )
    rear_support.inertial = Inertial.from_geometry(
        Box((0.240, 0.180, 0.380)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
    )

    outer_stage = model.part("outer_stage")
    _add_square_tube(
        outer_stage,
        prefix="outer_sleeve",
        outer_size=OUTER_STAGE_SIZE,
        wall=OUTER_STAGE_WALL,
        bottom_z=0.0,
        top_z=OUTER_STAGE_HEIGHT,
        material="mast_gray",
    )
    _add_square_tube(
        outer_stage,
        prefix="outer_collar",
        outer_size=OUTER_STAGE_SIZE + 0.010,
        wall=0.005,
        bottom_z=OUTER_STAGE_HEIGHT - OUTER_COLLAR_HEIGHT,
        top_z=OUTER_STAGE_HEIGHT,
        material="mast_gray",
    )
    outer_stage.visual(
        Box((OUTER_STAGE_SIZE, OUTER_STAGE_SIZE, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="mast_gray",
        name="outer_base_cap",
    )
    outer_stage.inertial = Inertial.from_geometry(
        Box((OUTER_STAGE_SIZE, OUTER_STAGE_SIZE, OUTER_STAGE_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, OUTER_STAGE_HEIGHT / 2.0)),
    )

    middle_stage = model.part("middle_stage")
    _add_square_tube(
        middle_stage,
        prefix="middle_member",
        outer_size=MIDDLE_STAGE_SIZE,
        wall=MIDDLE_STAGE_WALL,
        bottom_z=MIDDLE_STAGE_BOTTOM,
        top_z=MIDDLE_STAGE_TOP,
        material="mast_light",
    )
    _add_square_tube(
        middle_stage,
        prefix="middle_collar",
        outer_size=MIDDLE_STAGE_SIZE + 0.008,
        wall=0.004,
        bottom_z=MIDDLE_STAGE_TOP - 0.028,
        top_z=MIDDLE_STAGE_TOP,
        material="mast_light",
    )
    _add_guide_pads(
        middle_stage,
        prefix="middle_guides",
        outer_reach=0.036,
        inner_reach=MIDDLE_STAGE_SIZE / 2.0,
        pad_span=0.018,
        pad_height=0.220,
        center_z=-0.250,
        material="mast_light",
    )
    middle_stage.inertial = Inertial.from_geometry(
        Box(
            (
                MIDDLE_STAGE_SIZE,
                MIDDLE_STAGE_SIZE,
                MIDDLE_STAGE_TOP - MIDDLE_STAGE_BOTTOM,
            )
        ),
        mass=2.7,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (MIDDLE_STAGE_BOTTOM + MIDDLE_STAGE_TOP) / 2.0,
            )
        ),
    )

    inner_stage = model.part("inner_stage")
    _add_square_tube(
        inner_stage,
        prefix="inner_member",
        outer_size=INNER_STAGE_SIZE,
        wall=INNER_STAGE_WALL,
        bottom_z=INNER_STAGE_BOTTOM,
        top_z=INNER_STAGE_TOP,
        material="mast_gray",
    )
    _add_guide_pads(
        inner_stage,
        prefix="inner_guides",
        outer_reach=(MIDDLE_STAGE_SIZE - 2.0 * MIDDLE_STAGE_WALL) / 2.0,
        inner_reach=INNER_STAGE_SIZE / 2.0 - 0.0005,
        pad_span=0.016,
        pad_height=0.180,
        center_z=-0.200,
        material="mast_gray",
    )
    inner_stage.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, INNER_STAGE_TOP - 0.008)),
        material="mast_gray",
        name="bearing_pedestal",
    )
    inner_stage.visual(
        Box((0.086, 0.086, TOP_MOUNT_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, INNER_STAGE_TOP + TOP_MOUNT_THICKNESS / 2.0)),
        material="mast_gray",
        name="top_mount",
    )
    inner_stage.inertial = Inertial.from_geometry(
        Box(
            (
                0.086,
                0.086,
                INNER_STAGE_TOP + TOP_MOUNT_THICKNESS - INNER_STAGE_BOTTOM,
            )
        ),
        mass=1.8,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (INNER_STAGE_BOTTOM + INNER_STAGE_TOP + TOP_MOUNT_THICKNESS) / 2.0,
            )
        ),
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=FACEPLATE_RADIUS, length=FACEPLATE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, FACEPLATE_THICKNESS / 2.0)),
        material="faceplate_gray",
        name="faceplate_disk",
    )
    faceplate.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, FACEPLATE_THICKNESS + 0.013)),
        material="faceplate_gray",
        name="faceplate_hub",
    )
    faceplate.visual(
        Box((0.028, 0.012, 0.018)),
        origin=Origin(
            xyz=(
                0.0,
                FACEPLATE_RADIUS - 0.014,
                FACEPLATE_THICKNESS + 0.009,
            )
        ),
        material="indicator_red",
        name="pointer_flag",
    )
    faceplate.inertial = Inertial.from_geometry(
        Cylinder(radius=FACEPLATE_RADIUS, length=FACEPLATE_THICKNESS + 0.026),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, (FACEPLATE_THICKNESS + 0.026) / 2.0)),
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=rear_support,
        child=outer_stage,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_SHELF_TOP_Z)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_stage,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_STAGE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_STAGE_TRAVEL,
            effort=80.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_STAGE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_STAGE_TRAVEL,
            effort=55.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "inner_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=inner_stage,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.0, INNER_STAGE_TOP + TOP_MOUNT_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=12.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    outer_stage = object_model.get_part("outer_stage")
    middle_stage = object_model.get_part("middle_stage")
    inner_stage = object_model.get_part("inner_stage")
    faceplate = object_model.get_part("faceplate")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_faceplate = object_model.get_articulation("inner_to_faceplate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        rear_support,
        outer_stage,
        contact_tol=0.002,
        name="rear support physically captures the mast root",
    )
    ctx.expect_within(
        middle_stage,
        outer_stage,
        axes="xy",
        margin=0.009,
        name="middle stage stays centered inside the outer sleeve",
    )
    ctx.expect_overlap(
        middle_stage,
        outer_stage,
        axes="z",
        min_overlap=0.120,
        name="middle stage retains insertion inside the outer sleeve at rest",
    )
    ctx.expect_within(
        inner_stage,
        middle_stage,
        axes="xy",
        margin=0.009,
        name="inner stage stays centered inside the middle sleeve",
    )
    ctx.expect_overlap(
        inner_stage,
        middle_stage,
        axes="z",
        min_overlap=0.100,
        name="inner stage retains insertion inside the middle sleeve at rest",
    )
    ctx.expect_gap(
        faceplate,
        inner_stage,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        name="faceplate seats on the top rotary mount without sinking into it",
    )

    middle_rest = ctx.part_world_position(middle_stage)
    inner_rest = ctx.part_world_position(inner_stage)
    pointer_rest = _aabb_center(
        ctx.part_element_world_aabb(faceplate, elem="pointer_flag")
    )

    with ctx.pose({outer_to_middle: MIDDLE_STAGE_TRAVEL}):
        ctx.expect_within(
            middle_stage,
            outer_stage,
            axes="xy",
            margin=0.009,
            name="extended middle stage stays guided inside the outer sleeve",
        )
        ctx.expect_overlap(
            middle_stage,
            outer_stage,
            axes="z",
            min_overlap=0.095,
            name="extended middle stage still retains insertion in the outer sleeve",
        )
        middle_extended = ctx.part_world_position(middle_stage)

    with ctx.pose(
        {
            outer_to_middle: MIDDLE_STAGE_TRAVEL,
            middle_to_inner: INNER_STAGE_TRAVEL,
        }
    ):
        ctx.expect_within(
            inner_stage,
            middle_stage,
            axes="xy",
            margin=0.009,
            name="extended inner stage stays guided inside the middle sleeve",
        )
        ctx.expect_overlap(
            inner_stage,
            middle_stage,
            axes="z",
            min_overlap=0.090,
            name="extended inner stage still retains insertion in the middle sleeve",
        )
        inner_extended = ctx.part_world_position(inner_stage)

    with ctx.pose(inner_to_faceplate=1.1):
        pointer_rotated = _aabb_center(
            ctx.part_element_world_aabb(faceplate, elem="pointer_flag")
        )

    ctx.check(
        "middle stage extends upward along +Z",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[2] > middle_rest[2] + 0.050,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "inner stage extends upward along +Z",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[2] > inner_rest[2] + 0.050,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )
    ctx.check(
        "faceplate articulation is a vertical revolute joint",
        tuple(round(v, 6) for v in inner_to_faceplate.axis) == (0.0, 0.0, 1.0)
        and inner_to_faceplate.motion_limits is not None
        and inner_to_faceplate.motion_limits.lower is not None
        and inner_to_faceplate.motion_limits.upper is not None
        and inner_to_faceplate.motion_limits.lower < 0.0 < inner_to_faceplate.motion_limits.upper,
        details=(
            f"axis={inner_to_faceplate.axis}, "
            f"limits={inner_to_faceplate.motion_limits}"
        ),
    )
    ctx.check(
        "faceplate visibly rotates about the mast axis",
        pointer_rest is not None
        and pointer_rotated is not None
        and abs(pointer_rotated[0] - pointer_rest[0]) > 0.020
        and abs(pointer_rotated[1] - pointer_rest[1]) > 0.020,
        details=f"rest={pointer_rest}, rotated={pointer_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
