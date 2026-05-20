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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_wiper_assembly")

    housing_paint = model.material("housing_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    housing = model.part("motor_housing")
    housing.visual(
        Box((0.132, 0.084, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="mounting_plate",
    )
    housing.visual(
        _mesh(
            "gearbox_shell",
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.094, 0.064, 0.010, corner_segments=8),
                0.028,
                cap=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=housing_paint,
        name="gearbox_shell",
    )
    housing.visual(
        Cylinder(radius=0.020, length=0.058),
        origin=Origin(xyz=(-0.060, 0.0, 0.022), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="motor_can",
    )
    housing.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.018, 0.0, 0.043)),
        material=steel,
        name="spindle_tower",
    )
    housing.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.018, 0.0, 0.059)),
        material=steel,
        name="spindle_stub",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.132, 0.084, 0.066)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
    )

    arm = model.part("wiper_arm")
    arm.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="hub_collar",
    )
    arm.visual(
        _mesh(
            "arm_beam",
            sweep_profile_along_spline(
                [
                    (0.008, 0.0, 0.010),
                    (0.050, 0.0, 0.015),
                    (0.094, 0.0, 0.014),
                    (0.122, 0.0, 0.010),
                ],
                profile=rounded_rect_profile(0.012, 0.0035, radius=0.0012, corner_segments=6),
                samples_per_segment=16,
                cap_profile=True,
            ),
        ),
        material=satin_black,
        name="arm_beam",
    )
    arm.visual(
        Cylinder(radius=0.002, length=0.074),
        origin=Origin(xyz=(0.070, 0.0, 0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="spring_bar",
    )
    arm.visual(
        Box((0.007, 0.010, 0.012)),
        origin=Origin(xyz=(0.026, 0.0, 0.016)),
        material=satin_black,
        name="spring_post_root",
    )
    arm.visual(
        Box((0.007, 0.010, 0.010)),
        origin=Origin(xyz=(0.106, 0.0, 0.015)),
        material=satin_black,
        name="spring_post_tip",
    )
    arm.visual(
        Box((0.010, 0.016, 0.010)),
        origin=Origin(xyz=(0.127, 0.0, 0.010)),
        material=steel,
        name="tip_socket",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.140, 0.030, 0.030)),
        mass=0.12,
        origin=Origin(xyz=(0.070, 0.0, 0.012)),
    )

    blade = model.part("blade_carrier")
    blade.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    blade.visual(
        Box((0.014, 0.018, 0.006)),
        origin=Origin(xyz=(0.009, 0.0, -0.004)),
        material=satin_black,
        name="saddle_block",
    )
    blade.visual(
        Box((0.006, 0.030, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, -0.009)),
        material=steel,
        name="center_bridge",
    )
    blade.visual(
        _mesh(
            "blade_spine",
            sweep_profile_along_spline(
                [
                    (0.010, -0.056, -0.011),
                    (0.010, -0.028, -0.010),
                    (0.010, 0.000, -0.009),
                    (0.010, 0.028, -0.010),
                    (0.010, 0.056, -0.011),
                ],
                profile=rounded_rect_profile(0.008, 0.003, radius=0.0008, corner_segments=6),
                samples_per_segment=14,
                cap_profile=True,
            ),
        ),
        material=satin_black,
        name="blade_spine",
    )
    blade.visual(
        Box((0.003, 0.102, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, -0.018)),
        material=rubber,
        name="rubber_strip",
    )
    blade.visual(
        Box((0.005, 0.008, 0.008)),
        origin=Origin(xyz=(0.010, -0.051, -0.014)),
        material=satin_black,
        name="end_cap_neg",
    )
    blade.visual(
        Box((0.005, 0.008, 0.008)),
        origin=Origin(xyz=(0.010, 0.051, -0.014)),
        material=satin_black,
        name="end_cap_pos",
    )
    blade.inertial = Inertial.from_geometry(
        Box((0.022, 0.112, 0.028)),
        mass=0.06,
        origin=Origin(xyz=(0.010, 0.0, -0.012)),
    )

    model.articulation(
        "arm_sweep",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm,
        origin=Origin(xyz=(0.018, 0.0, 0.066)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "blade_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.132, 0.0, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-0.50, upper=0.50),
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

    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("wiper_arm")
    blade = object_model.get_part("blade_carrier")
    arm_sweep = object_model.get_articulation("arm_sweep")
    blade_roll = object_model.get_articulation("blade_roll")

    with ctx.pose({arm_sweep: 0.0, blade_roll: 0.0}):
        ctx.expect_gap(
            arm,
            housing,
            axis="z",
            positive_elem="hub_collar",
            negative_elem="spindle_stub",
            max_gap=0.001,
            max_penetration=0.0,
            name="arm hub seats on the spindle stub",
        )
        ctx.expect_gap(
            blade,
            arm,
            axis="x",
            positive_elem="pivot_barrel",
            negative_elem="tip_socket",
            max_gap=0.001,
            max_penetration=0.0,
            name="blade carrier mounts flush to the arm tip",
        )

    with ctx.pose({arm_sweep: -0.80, blade_roll: 0.0}):
        blade_low = ctx.part_world_position(blade)
    with ctx.pose({arm_sweep: 0.80, blade_roll: 0.0}):
        blade_high = ctx.part_world_position(blade)
    ctx.check(
        "arm sweep carries the blade tip across the spindle",
        blade_low is not None
        and blade_high is not None
        and blade_low[1] < -0.07
        and blade_high[1] > 0.07
        and blade_low[0] > 0.08
        and blade_high[0] > 0.08,
        details=f"low={blade_low}, high={blade_high}",
    )

    with ctx.pose({arm_sweep: 0.0, blade_roll: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(blade, elem="rubber_strip")
    with ctx.pose({arm_sweep: 0.0, blade_roll: 0.42}):
        rolled_aabb = ctx.part_element_world_aabb(blade, elem="rubber_strip")
    rest_dz = None if rest_aabb is None else rest_aabb[1][2] - rest_aabb[0][2]
    rolled_dz = None if rolled_aabb is None else rolled_aabb[1][2] - rolled_aabb[0][2]
    ctx.check(
        "blade roll changes the rubber strip pitch",
        rest_dz is not None and rolled_dz is not None and rolled_dz > rest_dz + 0.025,
        details=f"rest_dz={rest_dz}, rolled_dz={rolled_dz}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
