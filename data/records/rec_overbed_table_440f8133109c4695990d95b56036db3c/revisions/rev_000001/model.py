from __future__ import annotations

from math import pi

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


FRAME_TOP_Z = 0.137
FRAME_BOTTOM_Z = 0.103
POST_X = -0.12
POST_Y = -0.15
SLEEVE_BOTTOM_Z = 0.192
SLEEVE_HEIGHT = 0.400
SLEEVE_TOP_Z = SLEEVE_BOTTOM_Z + SLEEVE_HEIGHT
COLLAR_TOP_Z = 0.596
TOP_LOW_Z = 0.736
TOP_HIGH_Z = 0.956


def _add_fork_visuals(
    base_part,
    *,
    prefix: str,
    center: tuple[float, float],
    wheel_radius: float,
    wheel_width: float,
    crown_z: float,
    crown_height: float,
    material: str,
    add_lock_mount: bool = False,
) -> None:
    cx, cy = center
    stem_top_z = FRAME_BOTTOM_Z + 0.002
    stem_bottom_z = crown_z + crown_height * 0.5 - 0.002
    stem_height = stem_top_z - stem_bottom_z
    tine_thickness = 0.004
    tine_gap = wheel_width + 0.006
    tine_bottom_z = max(0.010, wheel_radius * 0.75)
    tine_top_z = crown_z - crown_height * 0.5 + 0.002
    tine_height = tine_top_z - tine_bottom_z
    tine_center_z = tine_bottom_z + tine_height * 0.5

    base_part.visual(
        Box((0.015, 0.015, stem_height)),
        origin=Origin(xyz=(cx, cy, stem_bottom_z + stem_height * 0.5)),
        material=material,
        name=f"{prefix}_stem",
    )
    base_part.visual(
        Box((tine_gap + tine_thickness * 2.0, 0.021, crown_height)),
        origin=Origin(xyz=(cx, cy, crown_z)),
        material=material,
        name=f"{prefix}_crown",
    )
    for side_index, side_sign in enumerate((-1.0, 1.0)):
        base_part.visual(
            Box((tine_thickness, 0.016, tine_height)),
            origin=Origin(
                xyz=(
                    cx + side_sign * (wheel_width * 0.5 + 0.003 + tine_thickness * 0.5),
                    cy,
                    tine_center_z,
                )
            ),
            material=material,
            name=f"{prefix}_tine_{side_index}",
        )
    if add_lock_mount:
        base_part.visual(
            Box((0.020, 0.044, 0.012)),
            origin=Origin(xyz=(cx, cy, crown_z + crown_height * 0.5 + 0.004)),
            material=material,
            name=f"{prefix}_lock_mount",
        )


def _add_wheel_part(
    model: ArticulatedObject,
    *,
    part_name: str,
    radius: float,
    width: float,
    tire_material: str,
    hub_material: str,
):
    wheel = model.part(part_name)
    wheel.visual(
        Cylinder(radius=radius, length=width),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=tire_material,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=radius * 0.58, length=width + 0.004),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=radius * 0.24, length=width + 0.006),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="axle_cap",
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overbed_table")

    model.material("frame_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("trim_gray", rgba=(0.60, 0.62, 0.64, 1.0))
    model.material("laminate", rgba=(0.93, 0.92, 0.88, 1.0))
    model.material("underside_gray", rgba=(0.84, 0.84, 0.82, 1.0))
    model.material("caster_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("caster_hub", rgba=(0.48, 0.49, 0.51, 1.0))
    model.material("knob_black", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("lock_red", rgba=(0.79, 0.16, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.68, 0.08, 0.034)),
        origin=Origin(xyz=(0.02, 0.0, 0.120)),
        material="frame_steel",
        name="main_beam",
    )
    base.visual(
        Box((0.12, 0.50, 0.034)),
        origin=Origin(xyz=(-0.24, 0.0, 0.120)),
        material="frame_steel",
        name="rear_beam",
    )
    base.visual(
        Box((0.10, 0.24, 0.030)),
        origin=Origin(xyz=(0.28, 0.0, 0.118)),
        material="frame_steel",
        name="front_beam",
    )
    base.visual(
        Box((0.16, 0.14, 0.070)),
        origin=Origin(xyz=(-0.12, -0.10, 0.157)),
        material="frame_steel",
        name="pedestal_block",
    )

    sleeve_outer_x = 0.056
    sleeve_outer_y = 0.044
    sleeve_wall = 0.006
    sleeve_center_z = SLEEVE_BOTTOM_Z + SLEEVE_HEIGHT * 0.5
    base.visual(
        Box((sleeve_wall, sleeve_outer_y, SLEEVE_HEIGHT)),
        origin=Origin(xyz=(POST_X - sleeve_outer_x * 0.5 + sleeve_wall * 0.5, POST_Y, sleeve_center_z)),
        material="frame_steel",
        name="sleeve_side_0",
    )
    base.visual(
        Box((sleeve_wall, sleeve_outer_y, SLEEVE_HEIGHT)),
        origin=Origin(xyz=(POST_X + sleeve_outer_x * 0.5 - sleeve_wall * 0.5, POST_Y, sleeve_center_z)),
        material="frame_steel",
        name="sleeve_side_1",
    )
    base.visual(
        Box((sleeve_outer_x - sleeve_wall * 2.0, sleeve_wall, SLEEVE_HEIGHT)),
        origin=Origin(xyz=(POST_X, POST_Y - sleeve_outer_y * 0.5 + sleeve_wall * 0.5, sleeve_center_z)),
        material="frame_steel",
        name="sleeve_front",
    )
    base.visual(
        Box((sleeve_outer_x - sleeve_wall * 2.0, sleeve_wall, SLEEVE_HEIGHT)),
        origin=Origin(xyz=(POST_X, POST_Y + sleeve_outer_y * 0.5 - sleeve_wall * 0.5, sleeve_center_z)),
        material="frame_steel",
        name="sleeve_rear",
    )

    collar_outer_x = 0.072
    collar_outer_y = 0.058
    collar_height = 0.050
    collar_center_z = COLLAR_TOP_Z - collar_height * 0.5
    base.visual(
        Box((sleeve_wall, collar_outer_y, collar_height)),
        origin=Origin(xyz=(POST_X - sleeve_outer_x * 0.5 + sleeve_wall * 0.5, POST_Y, collar_center_z)),
        material="trim_gray",
        name="collar_side_0",
    )
    base.visual(
        Box((sleeve_wall, collar_outer_y, collar_height)),
        origin=Origin(xyz=(POST_X + sleeve_outer_x * 0.5 - sleeve_wall * 0.5, POST_Y, collar_center_z)),
        material="trim_gray",
        name="collar_side_1",
    )
    base.visual(
        Box((collar_outer_x - sleeve_wall * 2.0, sleeve_wall, collar_height)),
        origin=Origin(xyz=(POST_X, POST_Y - sleeve_outer_y * 0.5 + sleeve_wall * 0.5, collar_center_z)),
        material="trim_gray",
        name="collar_front",
    )
    base.visual(
        Box((collar_outer_x - sleeve_wall * 2.0, sleeve_wall, collar_height)),
        origin=Origin(xyz=(POST_X, POST_Y + sleeve_outer_y * 0.5 - sleeve_wall * 0.5, collar_center_z)),
        material="trim_gray",
        name="collar_rear",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(
            xyz=(POST_X, POST_Y - 0.030, collar_center_z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="trim_gray",
        name="knob_hub",
    )

    _add_fork_visuals(
        base,
        prefix="front_fork_0",
        center=(0.29, -0.085),
        wheel_radius=0.036,
        wheel_width=0.020,
        crown_z=0.091,
        crown_height=0.010,
        material="frame_steel",
    )
    _add_fork_visuals(
        base,
        prefix="front_fork_1",
        center=(0.29, 0.085),
        wheel_radius=0.036,
        wheel_width=0.020,
        crown_z=0.091,
        crown_height=0.010,
        material="frame_steel",
    )
    _add_fork_visuals(
        base,
        prefix="rear_fork_0",
        center=(-0.26, -0.225),
        wheel_radius=0.042,
        wheel_width=0.022,
        crown_z=0.097,
        crown_height=0.010,
        material="frame_steel",
        add_lock_mount=True,
    )
    _add_fork_visuals(
        base,
        prefix="rear_fork_1",
        center=(-0.26, 0.225),
        wheel_radius=0.042,
        wheel_width=0.022,
        crown_z=0.097,
        crown_height=0.010,
        material="frame_steel",
        add_lock_mount=True,
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.034, 0.024, 0.580)),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material="frame_steel",
        name="mast",
    )
    mast.visual(
        Box((0.052, 0.052, 0.145)),
        origin=Origin(xyz=(0.0, 0.022, 0.072)),
        material="trim_gray",
        name="arm_block",
    )
    mast.visual(
        Box((0.072, 0.300, 0.028)),
        origin=Origin(xyz=(0.0, 0.175, 0.096)),
        material="frame_steel",
        name="arm",
    )
    mast.visual(
        Box((0.180, 0.180, 0.014)),
        origin=Origin(xyz=(0.0, 0.230, 0.129)),
        material="underside_gray",
        name="underplate",
    )
    mast.visual(
        Box((0.060, 0.110, 0.022)),
        origin=Origin(xyz=(0.0, 0.145, 0.117)),
        material="underside_gray",
        name="plate_brace",
    )
    mast.visual(
        Box((0.760, 0.410, 0.018)),
        origin=Origin(xyz=(0.04, 0.320, 0.140)),
        material="laminate",
        name="top",
    )
    mast.visual(
        Box((0.744, 0.394, 0.004)),
        origin=Origin(xyz=(0.04, 0.320, 0.129)),
        material="underside_gray",
        name="underside_panel",
    )

    collar_knob = model.part("collar_knob")
    collar_knob.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="knob_black",
        name="knob",
    )
    collar_knob.visual(
        Box((0.042, 0.007, 0.007)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material="knob_black",
        name="wing",
    )

    front_wheel_0 = _add_wheel_part(
        model,
        part_name="front_wheel_0",
        radius=0.036,
        width=0.020,
        tire_material="caster_rubber",
        hub_material="caster_hub",
    )
    front_wheel_1 = _add_wheel_part(
        model,
        part_name="front_wheel_1",
        radius=0.036,
        width=0.020,
        tire_material="caster_rubber",
        hub_material="caster_hub",
    )
    rear_wheel_0 = _add_wheel_part(
        model,
        part_name="rear_wheel_0",
        radius=0.042,
        width=0.022,
        tire_material="caster_rubber",
        hub_material="caster_hub",
    )
    rear_wheel_1 = _add_wheel_part(
        model,
        part_name="rear_wheel_1",
        radius=0.042,
        width=0.022,
        tire_material="caster_rubber",
        hub_material="caster_hub",
    )

    rear_lock_0 = model.part("rear_lock_0")
    rear_lock_0.visual(
        Box((0.010, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, -0.020, 0.002)),
        material="lock_red",
        name="tab",
    )
    rear_lock_0.visual(
        Box((0.016, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.036, -0.001)),
        material="lock_red",
        name="pedal",
    )

    rear_lock_1 = model.part("rear_lock_1")
    rear_lock_1.visual(
        Box((0.010, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, 0.020, 0.002)),
        material="lock_red",
        name="tab",
    )
    rear_lock_1.visual(
        Box((0.016, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.036, -0.001)),
        material="lock_red",
        name="pedal",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(POST_X, POST_Y, COLLAR_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.220),
    )
    model.articulation(
        "base_to_collar_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=collar_knob,
        origin=Origin(xyz=(POST_X, POST_Y - 0.039, collar_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )
    model.articulation(
        "base_to_front_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=front_wheel_0,
        origin=Origin(xyz=(0.29, -0.085, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "base_to_front_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=front_wheel_1,
        origin=Origin(xyz=(0.29, 0.085, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "base_to_rear_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=rear_wheel_0,
        origin=Origin(xyz=(-0.26, -0.225, 0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "base_to_rear_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=rear_wheel_1,
        origin=Origin(xyz=(-0.26, 0.225, 0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "base_to_rear_lock_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rear_lock_0,
        origin=Origin(xyz=(-0.26, -0.247, 0.111)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.5, lower=-0.20, upper=0.85),
    )
    model.articulation(
        "base_to_rear_lock_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rear_lock_1,
        origin=Origin(xyz=(-0.26, 0.247, 0.111)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.5, lower=-0.85, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    slide = object_model.get_articulation("base_to_mast")
    slide_limits = slide.motion_limits

    ctx.expect_gap(
        mast,
        base,
        axis="z",
        positive_elem="top",
        negative_elem="main_beam",
        min_gap=0.58,
        name="top clears the base frame at minimum height",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="x",
        elem_a="top",
        elem_b="main_beam",
        min_overlap=0.25,
        name="tabletop remains substantially over the pedestal footprint",
    )

    if slide_limits is not None and slide_limits.upper is not None:
        rest_pos = ctx.part_world_position(mast)
        with ctx.pose({slide: slide_limits.upper}):
            ctx.expect_gap(
                mast,
                base,
                axis="z",
                positive_elem="top",
                negative_elem="main_beam",
                min_gap=0.79,
                name="top reaches a standing-use height",
            )
            upper_pos = ctx.part_world_position(mast)
            mast_aabb = ctx.part_element_world_aabb(mast, elem="mast")
            sleeve_aabb = ctx.part_element_world_aabb(base, elem="sleeve_front")
            retained = None
            if mast_aabb is not None and sleeve_aabb is not None:
                retained = sleeve_aabb[1][2] - mast_aabb[0][2]
            ctx.check(
                "mast extends upward",
                rest_pos is not None and upper_pos is not None and upper_pos[2] > rest_pos[2] + 0.18,
                details=f"rest={rest_pos}, upper={upper_pos}",
            )
            ctx.check(
                "mast retains insertion at maximum height",
                retained is not None and retained >= 0.17,
                details=f"retained_insertion={retained}",
            )

    return ctx.report()


object_model = build_object_model()
