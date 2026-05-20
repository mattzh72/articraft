from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pole_mounted_monitor_support")

    dark = _mat("matte_black_powder_coat", (0.015, 0.016, 0.018, 1.0))
    charcoal = _mat("charcoal_clamp", (0.055, 0.058, 0.062, 1.0))
    metal = _mat("brushed_steel", (0.62, 0.64, 0.62, 1.0))
    rubber = _mat("black_rubber_pads", (0.004, 0.004, 0.004, 1.0))
    screw = _mat("dark_screw_heads", (0.0, 0.0, 0.0, 1.0))

    half_pi = 1.57079632679
    first_len = 0.420
    second_len = 0.340

    # Root: the pole and the fixed clamp are one grounded assembly.  The arm's
    # first bearing is kept on a short standoff so the pole remains readable.
    mast_clamp = model.part("mast_clamp")
    mast_clamp.visual(
        Cylinder(radius=0.025, length=1.200),
        origin=Origin(xyz=(-0.120, 0.0, 0.000)),
        material=metal,
        name="vertical_pole",
    )
    mast_clamp.visual(
        Cylinder(radius=0.057, length=0.105),
        origin=Origin(xyz=(-0.120, 0.0, 0.000)),
        material=charcoal,
        name="split_collar",
    )
    mast_clamp.visual(
        Box((0.120, 0.046, 0.035)),
        origin=Origin(xyz=(-0.060, 0.0, -0.020)),
        material=charcoal,
        name="standoff_bridge",
    )
    mast_clamp.visual(
        Cylinder(radius=0.046, length=0.020),
        origin=Origin(xyz=(0.000, 0.0, -0.010)),
        material=charcoal,
        name="lower_bearing_seat",
    )
    mast_clamp.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.000, 0.0, 0.0275)),
        material=metal,
        name="base_pin",
    )
    mast_clamp.visual(
        Box((0.082, 0.026, 0.082)),
        origin=Origin(xyz=(-0.120, 0.066, 0.000)),
        material=charcoal,
        name="clamp_lug",
    )
    mast_clamp.visual(
        Cylinder(radius=0.007, length=0.086),
        origin=Origin(xyz=(-0.120, 0.067, 0.000), rpy=(half_pi, 0.0, 0.0)),
        material=metal,
        name="clamp_bolt",
    )

    # First arm: two parallel plates with large end bosses, like a commercial
    # monitor-support link.
    first_arm = model.part("first_arm")
    first_arm.visual(
        Cylinder(radius=0.042, length=0.050),
        origin=Origin(xyz=(0.000, 0.0, 0.030)),
        material=dark,
        name="root_hub",
    )
    first_arm.visual(
        Box((0.360, 0.012, 0.026)),
        origin=Origin(xyz=(first_len / 2.0, -0.024, 0.030)),
        material=dark,
        name="rail_0",
    )
    first_arm.visual(
        Box((0.360, 0.012, 0.026)),
        origin=Origin(xyz=(first_len / 2.0, 0.024, 0.030)),
        material=dark,
        name="rail_1",
    )
    first_arm.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(first_len, 0.0, 0.030)),
        material=dark,
        name="elbow_hub",
    )
    first_arm.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.000, 0.0, 0.058)),
        material=metal,
        name="root_cap",
    )
    first_arm.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(first_len, 0.0, 0.058)),
        material=metal,
        name="elbow_cap",
    )

    # Second arm is slightly shorter and stacked above the elbow bearing.
    second_arm = model.part("second_arm")
    second_arm.visual(
        Cylinder(radius=0.038, length=0.040),
        origin=Origin(xyz=(0.000, 0.0, 0.051)),
        material=dark,
        name="root_hub",
    )
    second_arm.visual(
        Box((0.290, 0.011, 0.024)),
        origin=Origin(xyz=(second_len / 2.0, -0.021, 0.051)),
        material=dark,
        name="rail_0",
    )
    second_arm.visual(
        Box((0.290, 0.011, 0.024)),
        origin=Origin(xyz=(second_len / 2.0, 0.021, 0.051)),
        material=dark,
        name="rail_1",
    )
    second_arm.visual(
        Cylinder(radius=0.036, length=0.040),
        origin=Origin(xyz=(second_len, 0.0, 0.051)),
        material=dark,
        name="head_hub",
    )
    second_arm.visual(
        Cylinder(radius=0.022, length=0.005),
        origin=Origin(xyz=(0.000, 0.0, 0.0735)),
        material=metal,
        name="root_cap",
    )
    second_arm.visual(
        Cylinder(radius=0.022, length=0.005),
        origin=Origin(xyz=(second_len, 0.0, 0.0735)),
        material=metal,
        name="head_cap",
    )

    # Compact head swivel hardware.  Its bearing and yoke are intentionally much
    # smaller than either arm link.
    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.000, 0.0, 0.045)),
        material=dark,
        name="swivel_hub",
    )
    head_swivel.visual(
        Box((0.045, 0.085, 0.020)),
        origin=Origin(xyz=(0.030, 0.0, 0.045)),
        material=dark,
        name="yoke_bridge",
    )
    head_swivel.visual(
        Box((0.035, 0.012, 0.050)),
        origin=Origin(xyz=(0.070, -0.037, 0.045)),
        material=dark,
        name="yoke_ear_0",
    )
    head_swivel.visual(
        Box((0.035, 0.012, 0.050)),
        origin=Origin(xyz=(0.070, 0.037, 0.045)),
        material=dark,
        name="yoke_ear_1",
    )
    head_swivel.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.070, 0.0, 0.045), rpy=(half_pi, 0.0, 0.0)),
        material=metal,
        name="tilt_pin",
    )

    monitor_plate = model.part("monitor_plate")
    monitor_plate.visual(
        Cylinder(radius=0.016, length=0.052),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(half_pi, 0.0, 0.0)),
        material=dark,
        name="tilt_barrel",
    )
    monitor_plate.visual(
        Box((0.030, 0.044, 0.026)),
        origin=Origin(xyz=(0.027, 0.0, 0.000)),
        material=dark,
        name="plate_neck",
    )
    monitor_plate.visual(
        Box((0.010, 0.140, 0.110)),
        origin=Origin(xyz=(0.045, 0.0, 0.000)),
        material=dark,
        name="vesa_plate",
    )
    for i, (y, z) in enumerate(
        ((-0.035, -0.025), (0.035, -0.025), (-0.035, 0.025), (0.035, 0.025))
    ):
        monitor_plate.visual(
            Cylinder(radius=0.0065, length=0.003),
            origin=Origin(xyz=(0.0505, y, z), rpy=(0.0, half_pi, 0.0)),
            material=screw,
            name=f"screw_head_{i}",
        )
    monitor_plate.visual(
        Box((0.004, 0.132, 0.012)),
        origin=Origin(xyz=(0.052, 0.0, 0.050)),
        material=rubber,
        name="top_rubber_strip",
    )
    monitor_plate.visual(
        Box((0.004, 0.132, 0.012)),
        origin=Origin(xyz=(0.052, 0.0, -0.050)),
        material=rubber,
        name="bottom_rubber_strip",
    )

    model.articulation(
        "mast_to_first",
        ArticulationType.REVOLUTE,
        parent=mast_clamp,
        child=first_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-1.6, upper=1.6),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.REVOLUTE,
        parent=first_arm,
        child=second_arm,
        origin=Origin(xyz=(first_len, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.4, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "second_to_head",
        ArticulationType.REVOLUTE,
        parent=second_arm,
        child=head_swivel,
        origin=Origin(xyz=(second_len, 0.0, 0.051)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.57, upper=1.57),
    )
    model.articulation(
        "head_to_plate",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=monitor_plate,
        origin=Origin(xyz=(0.070, 0.0, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.4, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast_clamp")
    first = object_model.get_part("first_arm")
    second = object_model.get_part("second_arm")
    head = object_model.get_part("head_swivel")
    plate = object_model.get_part("monitor_plate")

    mast_joint = object_model.get_articulation("mast_to_first")
    elbow_joint = object_model.get_articulation("first_to_second")
    swivel_joint = object_model.get_articulation("second_to_head")
    tilt_joint = object_model.get_articulation("head_to_plate")

    # These are small, local captured-pin intersections, not broad body
    # collisions.  Each allowance is paired with explicit fit/retention checks.
    ctx.allow_overlap(
        first,
        mast,
        elem_a="root_hub",
        elem_b="base_pin",
        reason="The first arm hub is intentionally captured by the clamp's vertical pivot pin.",
    )
    ctx.allow_overlap(
        head,
        plate,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        reason="The tilt pin intentionally passes through the compact monitor-plate barrel.",
    )

    ctx.check(
        "four revolute joints",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "joint limit set matches monitor arm",
        all(
            j.motion_limits is not None
            and j.motion_limits.lower is not None
            and j.motion_limits.upper is not None
            for j in (mast_joint, elbow_joint, swivel_joint, tilt_joint)
        ),
        details="Every user-facing revolute joint should have bounded motion.",
    )

    ctx.expect_within(
        mast,
        first,
        axes="xy",
        inner_elem="base_pin",
        outer_elem="root_hub",
        margin=0.001,
        name="base pin centered in first hub",
    )
    ctx.expect_overlap(
        mast,
        first,
        axes="z",
        elem_a="base_pin",
        elem_b="root_hub",
        min_overlap=0.045,
        name="base pin retains first hub height",
    )
    ctx.expect_within(
        head,
        plate,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="tilt_barrel",
        margin=0.001,
        name="tilt pin centered through barrel",
    )
    ctx.expect_overlap(
        head,
        plate,
        axes="y",
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        min_overlap=0.045,
        name="tilt pin retained through barrel width",
    )

    ctx.expect_gap(
        second,
        first,
        axis="z",
        positive_elem="root_hub",
        negative_elem="elbow_cap",
        max_gap=0.001,
        max_penetration=0.00002,
        name="second arm stacked on elbow washer",
    )
    ctx.expect_gap(
        head,
        second,
        axis="z",
        positive_elem="swivel_hub",
        negative_elem="head_cap",
        max_gap=0.001,
        max_penetration=0.00002,
        name="head swivel stacked on small head washer",
    )

    def _aabb_dims(aabb):
        if aabb is None:
            return None
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    first_dims = _aabb_dims(ctx.part_world_aabb(first))
    second_dims = _aabb_dims(ctx.part_world_aabb(second))
    head_dims = _aabb_dims(ctx.part_world_aabb(head))
    ctx.check(
        "compact head hardware smaller than arm links",
        first_dims is not None
        and second_dims is not None
        and head_dims is not None
        and first_dims[0] > 0.35
        and second_dims[0] > 0.28
        and head_dims[0] < 0.14,
        details=f"first={first_dims}, second={second_dims}, head={head_dims}",
    )

    rest_second_pos = ctx.part_world_position(second)
    with ctx.pose({mast_joint: 0.70}):
        swept_second_pos = ctx.part_world_position(second)
    ctx.check(
        "mast joint yaws the arm away from pole",
        rest_second_pos is not None
        and swept_second_pos is not None
        and abs(swept_second_pos[1] - rest_second_pos[1]) > 0.20,
        details=f"rest={rest_second_pos}, swept={swept_second_pos}",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({elbow_joint: 0.85}):
        bent_head_pos = ctx.part_world_position(head)
    ctx.check(
        "elbow joint swings the second arm",
        rest_head_pos is not None
        and bent_head_pos is not None
        and abs(bent_head_pos[1] - rest_head_pos[1]) > 0.20,
        details=f"rest={rest_head_pos}, bent={bent_head_pos}",
    )

    rest_plate_pos = ctx.part_world_position(plate)
    with ctx.pose({swivel_joint: 0.75}):
        swivel_plate_pos = ctx.part_world_position(plate)
    ctx.check(
        "head swivel moves plate about vertical axis",
        rest_plate_pos is not None
        and swivel_plate_pos is not None
        and abs(swivel_plate_pos[1] - rest_plate_pos[1]) > 0.035,
        details=f"rest={rest_plate_pos}, swivel={swivel_plate_pos}",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(plate, elem="vesa_plate")
    rest_plate_z = None if rest_plate_aabb is None else (rest_plate_aabb[0][2] + rest_plate_aabb[1][2]) / 2.0
    with ctx.pose({tilt_joint: 0.55}):
        tilted_plate_aabb = ctx.part_element_world_aabb(plate, elem="vesa_plate")
        tilted_plate_z = (
            None
            if tilted_plate_aabb is None
            else (tilted_plate_aabb[0][2] + tilted_plate_aabb[1][2]) / 2.0
        )
    ctx.check(
        "positive head tilt raises plate face",
        rest_plate_z is not None and tilted_plate_z is not None and tilted_plate_z > rest_plate_z + 0.015,
        details=f"rest_z={rest_plate_z}, tilted_z={tilted_plate_z}",
    )

    return ctx.report()


object_model = build_object_model()
