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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pot_filler_faucet")

    brushed_brass = model.material("brushed_brass", rgba=(0.82, 0.73, 0.56, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.70, 0.60, 0.43, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.31, 0.27, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.014, 0.105, 0.165)),
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
        material=brushed_brass,
        name="wall_plate",
    )
    wall_bracket.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(-0.010, 0.0, -0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_brass,
        name="mount_boss",
    )
    wall_bracket.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=brushed_brass,
        name="pivot_hub",
    )
    wall_bracket.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(-0.027, 0.0, 0.050), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="upper_fastener",
    )
    wall_bracket.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(-0.027, 0.0, -0.050), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="lower_fastener",
    )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.070, 0.110, 0.170)),
        mass=1.4,
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
    )

    first_arm = model.part("first_arm")
    first_arm.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=brushed_brass,
        name="base_hub",
    )
    first_arm.visual(
        Cylinder(radius=0.010, length=0.184),
        origin=Origin(xyz=(0.108, 0.0, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_brass,
        name="arm_tube",
    )
    first_arm.visual(
        Box((0.046, 0.024, 0.020)),
        origin=Origin(xyz=(0.221, 0.0, -0.010)),
        material=warm_brass,
        name="distal_drop",
    )
    first_arm.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.230, 0.0, -0.010)),
        material=brushed_brass,
        name="distal_hub",
    )
    first_arm.inertial = Inertial.from_geometry(
        Box((0.250, 0.050, 0.050)),
        mass=0.9,
        origin=Origin(xyz=(0.115, 0.0, 0.002)),
    )

    second_arm = model.part("second_arm")
    second_arm.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=brushed_brass,
        name="proximal_hub",
    )
    second_arm.visual(
        Cylinder(radius=0.010, length=0.175),
        origin=Origin(xyz=(0.103, 0.0, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_brass,
        name="arm_tube",
    )
    second_arm.visual(
        Box((0.034, 0.024, 0.024)),
        origin=Origin(xyz=(0.196, 0.0, 0.010)),
        material=warm_brass,
        name="valve_neck",
    )
    second_arm.visual(
        Cylinder(radius=0.019, length=0.060),
        origin=Origin(xyz=(0.216, 0.0, -0.005)),
        material=brushed_brass,
        name="valve_body",
    )
    second_arm.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.216, 0.0, 0.036)),
        material=warm_brass,
        name="handle_stem",
    )
    second_arm.visual(
        Cylinder(radius=0.0085, length=0.108),
        origin=Origin(xyz=(0.228, 0.0, -0.089)),
        material=brushed_brass,
        name="spout_drop",
    )
    second_arm.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.228, 0.0, -0.152)),
        material=warm_brass,
        name="nozzle_tip",
    )
    second_arm.visual(
        Cylinder(radius=0.0048, length=0.012),
        origin=Origin(xyz=(0.228, 0.0, -0.167)),
        material=dark_metal,
        name="aerator",
    )
    second_arm.inertial = Inertial.from_geometry(
        Box((0.255, 0.060, 0.215)),
        mass=0.85,
        origin=Origin(xyz=(0.120, 0.0, -0.050)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_brass,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.0038, length=0.050),
        origin=Origin(xyz=(0.0, 0.025, 0.008), rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_brass,
        name="lever_bar",
    )
    handle.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.053, 0.008)),
        material=warm_brass,
        name="lever_tip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.028, 0.062, 0.020)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.030, 0.008)),
    )

    model.articulation(
        "wall_to_first_arm",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=first_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-1.75,
            upper=1.75,
        ),
    )
    model.articulation(
        "first_to_second_arm",
        ArticulationType.REVOLUTE,
        parent=first_arm,
        child=second_arm,
        origin=Origin(xyz=(0.230, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "spout_handle",
        ArticulationType.REVOLUTE,
        parent=second_arm,
        child=handle,
        origin=Origin(xyz=(0.216, 0.0, 0.047)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=pi / 2.0,
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

    wall_bracket = object_model.get_part("wall_bracket")
    first_arm = object_model.get_part("first_arm")
    second_arm = object_model.get_part("second_arm")
    handle = object_model.get_part("handle")

    wall_to_first = object_model.get_articulation("wall_to_first_arm")
    first_to_second = object_model.get_articulation("first_to_second_arm")
    spout_handle = object_model.get_articulation("spout_handle")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    for part_name in ("wall_bracket", "first_arm", "second_arm", "handle"):
        ctx.check(
            f"{part_name} exists",
            object_model.get_part(part_name) is not None,
            details=f"Missing part: {part_name}",
        )

    ctx.expect_overlap(
        first_arm,
        wall_bracket,
        axes="xy",
        elem_a="base_hub",
        elem_b="pivot_hub",
        min_overlap=0.030,
        name="base swivel hubs share a pivot footprint",
    )
    ctx.expect_gap(
        first_arm,
        wall_bracket,
        axis="z",
        positive_elem="base_hub",
        negative_elem="pivot_hub",
        max_gap=0.0005,
        max_penetration=0.0,
        name="base swivel hub seats on wall bracket hub",
    )
    ctx.expect_overlap(
        second_arm,
        first_arm,
        axes="xy",
        elem_a="proximal_hub",
        elem_b="distal_hub",
        min_overlap=0.030,
        name="elbow swivel hubs share a pivot footprint",
    )
    ctx.expect_gap(
        second_arm,
        first_arm,
        axis="z",
        positive_elem="proximal_hub",
        negative_elem="distal_hub",
        max_gap=0.0005,
        max_penetration=0.0,
        name="elbow swivel hub seats on first arm hub",
    )
    ctx.expect_overlap(
        handle,
        second_arm,
        axes="xy",
        elem_a="handle_hub",
        elem_b="handle_stem",
        min_overlap=0.010,
        name="handle hub is centered on the valve stem",
    )
    ctx.expect_gap(
        handle,
        second_arm,
        axis="z",
        positive_elem="handle_hub",
        negative_elem="handle_stem",
        max_gap=0.0005,
        max_penetration=0.0,
        name="handle hub seats on the valve stem",
    )
    ctx.expect_gap(
        second_arm,
        wall_bracket,
        axis="x",
        min_gap=0.180,
        name="rest pose projects the spout assembly away from the wall",
    )

    rest_first_center = _aabb_center(ctx.part_element_world_aabb(first_arm, elem="arm_tube"))
    with ctx.pose({wall_to_first: 1.0}):
        swung_first_center = _aabb_center(ctx.part_element_world_aabb(first_arm, elem="arm_tube"))
    ctx.check(
        "positive wall joint swings the first arm leftward",
        rest_first_center is not None
        and swung_first_center is not None
        and swung_first_center[1] > rest_first_center[1] + 0.080,
        details=f"rest={rest_first_center}, swung={swung_first_center}",
    )

    rest_spout_center = _aabb_center(ctx.part_element_world_aabb(second_arm, elem="spout_drop"))
    with ctx.pose({first_to_second: 1.0}):
        folded_spout_center = _aabb_center(ctx.part_element_world_aabb(second_arm, elem="spout_drop"))
    ctx.check(
        "positive elbow joint folds the outer arm leftward",
        rest_spout_center is not None
        and folded_spout_center is not None
        and folded_spout_center[1] > rest_spout_center[1] + 0.100,
        details=f"rest={rest_spout_center}, folded={folded_spout_center}",
    )

    rest_tip_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="lever_tip"))
    with ctx.pose({spout_handle: pi / 2.0}):
        turned_tip_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="lever_tip"))
    ctx.check(
        "quarter-turn handle rotates across the valve body",
        rest_tip_center is not None
        and turned_tip_center is not None
        and turned_tip_center[0] < rest_tip_center[0] - 0.035,
        details=f"rest={rest_tip_center}, turned={turned_tip_center}",
    )

    with ctx.pose({wall_to_first: 1.2, first_to_second: -1.5}):
        ctx.expect_gap(
            second_arm,
            wall_bracket,
            axis="x",
            max_penetration=0.0,
            name="folded pose keeps the spout assembly clear of the wall bracket",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
