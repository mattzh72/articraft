from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="table_box_fan")

    def cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> CylinderGeometry:
        return CylinderGeometry(radius, length, radial_segments=32).rotate_y(math.pi / 2.0).translate(*center)

    def grid_mesh(
        x_pos: float,
        *,
        span: float,
        depth: float,
        offsets: tuple[float, ...],
    ):
        ring_outer = rounded_rect_profile(0.286, 0.286, 0.026, corner_segments=8)
        ring_inner = rounded_rect_profile(0.248, 0.248, 0.020, corner_segments=8)
        grid = ExtrudeWithHolesGeometry(ring_outer, [ring_inner], depth, center=True).rotate_y(math.pi / 2.0).translate(
            x_pos, 0.0, 0.0
        )
        grid.merge(BoxGeometry((depth, span, 0.012)).translate(x_pos, 0.0, offsets[0]))
        for z_off in offsets[1:]:
            grid.merge(BoxGeometry((depth, span, 0.012)).translate(x_pos, 0.0, z_off))
        for y_off in offsets:
            grid.merge(BoxGeometry((depth, 0.012, span)).translate(x_pos, y_off, 0.0))
        return grid

    model.material("warm_white", rgba=(0.92, 0.93, 0.90, 1.0))
    model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("dark_blade", rgba=(0.26, 0.28, 0.30, 1.0))
    model.material("knob_gray", rgba=(0.32, 0.33, 0.35, 1.0))

    base_radius = 0.135
    base_height = 0.018
    pivot_height = 0.180
    housing_size = 0.300
    housing_inner = 0.250
    housing_depth = 0.110
    boss_radius = 0.016
    boss_length = 0.027
    boss_center_y = 0.1625
    arm_pad_radius = 0.019
    arm_pad_length = 0.018
    arm_pad_center_y = 0.1845

    stand = model.part("stand")
    stand_geom = CylinderGeometry(base_radius, base_height, radial_segments=48).translate(0.0, 0.0, base_height / 2.0)
    stand_geom.merge(CylinderGeometry(0.052, 0.010, radial_segments=32).translate(0.0, 0.0, base_height + 0.005))
    for side in (-1.0, 1.0):
        arm_path = [
            (-0.075, side * 0.100, 0.009),
            (-0.102, side * 0.128, 0.070),
            (-0.095, side * 0.162, 0.125),
            (-0.050, side * 0.184, 0.168),
            (0.000, side * 0.190, pivot_height),
        ]
        stand_geom.merge(
            tube_from_spline_points(
                arm_path,
                radius=0.010,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            )
        )

    stand.visual(
        mesh_from_geometry(stand_geom, "fan_stand_frame"),
        material="charcoal",
        name="stand_frame",
    )
    stand.visual(
        Cylinder(radius=arm_pad_radius, length=arm_pad_length),
        origin=Origin(xyz=(0.0, arm_pad_center_y, pivot_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="charcoal",
        name="arm_pad_right",
    )
    stand.visual(
        Cylinder(radius=arm_pad_radius, length=arm_pad_length),
        origin=Origin(xyz=(0.0, -arm_pad_center_y, pivot_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="charcoal",
        name="arm_pad_left",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.320, 0.420, 0.220)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    housing = model.part("housing")
    shell_profile = rounded_rect_profile(housing_size, housing_size, 0.030, corner_segments=8)
    opening_profile = rounded_rect_profile(housing_inner, housing_inner, 0.022, corner_segments=8)
    housing_body_geom = ExtrudeWithHolesGeometry(shell_profile, [opening_profile], housing_depth, center=True).rotate_y(
        math.pi / 2.0
    )
    housing_body_geom.merge(grid_mesh(-(housing_depth / 2.0 - 0.004), span=0.248, depth=0.008, offsets=(-0.074, 0.0, 0.074)))
    housing_body_geom.merge(cylinder_x(0.046, 0.032, (-0.036, 0.0, 0.0)))
    housing_front_guard = grid_mesh(housing_depth / 2.0 - 0.004, span=0.248, depth=0.008, offsets=(-0.074, 0.0, 0.074))

    housing.visual(
        mesh_from_geometry(housing_body_geom, "fan_housing_body"),
        material="warm_white",
        name="housing_body",
    )
    housing.visual(
        mesh_from_geometry(housing_front_guard, "fan_front_guard"),
        material="warm_white",
        name="front_guard",
    )
    housing.visual(
        Cylinder(radius=boss_radius, length=boss_length),
        origin=Origin(xyz=(0.0, boss_center_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="warm_white",
        name="pivot_boss_right",
    )
    housing.visual(
        Cylinder(radius=boss_radius, length=boss_length),
        origin=Origin(xyz=(0.0, -boss_center_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="warm_white",
        name="pivot_boss_left",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_depth, 0.340, 0.320)),
        mass=1.3,
        origin=Origin(),
    )

    blade_assembly = model.part("blade_assembly")
    blade_geom = cylinder_x(0.027, 0.024, (0.0, 0.0, 0.0))
    blade_geom.merge(cylinder_x(0.015, 0.018, (0.014, 0.0, 0.0)))
    blade_geom.merge(cylinder_x(0.008, 0.020, (-0.014, 0.0, 0.0)))
    blade_profile = rounded_rect_profile(0.110, 0.038, 0.014, corner_segments=8)
    blade_paddle = ExtrudeGeometry(blade_profile, 0.006, center=True).rotate_y(math.pi / 2.0)
    blade_paddle.rotate_z(math.radians(18.0))
    blade_paddle.translate(0.0, 0.0, 0.067)
    for index in range(5):
        blade_geom.merge(blade_paddle.copy().rotate_x(index * 2.0 * math.pi / 5.0))

    blade_assembly.visual(
        mesh_from_geometry(blade_geom, "fan_blade_assembly"),
        material="dark_blade",
        name="blade_sweep",
    )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.130, length=0.024),
        mass=0.18,
        origin=Origin(),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.010, length=0.011),
        origin=Origin(xyz=(0.0, 0.0055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="knob_gray",
        name="knob_stem",
    )
    control_knob.visual(
        Cylinder(radius=0.019, length=0.020),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="knob_gray",
        name="knob_cap",
    )
    control_knob.visual(
        Box((0.004, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, 0.031, 0.014)),
        material="warm_white",
        name="knob_indicator",
    )
    control_knob.inertial = Inertial.from_geometry(
        Box((0.040, 0.042, 0.040)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, pivot_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=math.radians(-15.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade_assembly,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=28.0),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=control_knob,
        origin=Origin(xyz=(-0.020, housing_size / 2.0 - 0.0003, -0.088)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    housing = object_model.get_part("housing")
    blade_assembly = object_model.get_part("blade_assembly")
    control_knob = object_model.get_part("control_knob")

    tilt_joint = object_model.get_articulation("stand_to_housing")
    blade_joint = object_model.get_articulation("housing_to_blade")
    knob_joint = object_model.get_articulation("housing_to_knob")

    ctx.check(
        "tilt joint uses a horizontal revolute axis",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE and tuple(tilt_joint.axis) == (0.0, -1.0, 0.0),
        details=f"type={tilt_joint.articulation_type}, axis={tilt_joint.axis}",
    )
    ctx.check(
        "fan blade uses continuous spin",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS
        and blade_joint.motion_limits is not None
        and blade_joint.motion_limits.lower is None
        and blade_joint.motion_limits.upper is None
        and tuple(blade_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={blade_joint.articulation_type}, axis={blade_joint.axis}, limits={blade_joint.motion_limits}",
    )
    ctx.check(
        "side control knob uses continuous spin",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None
        and tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}, limits={knob_joint.motion_limits}",
    )

    ctx.expect_contact(
        stand,
        housing,
        elem_a="arm_pad_right",
        elem_b="pivot_boss_right",
        contact_tol=0.001,
        name="right tilt boss is seated in the right stand arm",
    )
    ctx.expect_contact(
        housing,
        stand,
        elem_a="pivot_boss_left",
        elem_b="arm_pad_left",
        contact_tol=0.001,
        name="left tilt boss is seated in the left stand arm",
    )
    ctx.expect_within(
        blade_assembly,
        housing,
        axes="yz",
        inner_elem="blade_sweep",
        outer_elem="front_guard",
        margin=0.012,
        name="blade sweep stays behind the front guard footprint",
    )
    ctx.expect_contact(
        control_knob,
        housing,
        elem_a="knob_stem",
        elem_b="housing_body",
        contact_tol=0.0015,
        name="control knob stem seats against the housing sidewall",
    )
    ctx.expect_origin_gap(
        control_knob,
        housing,
        axis="y",
        min_gap=0.145,
        max_gap=0.155,
        name="control knob is mounted on the housing right side",
    )

    def elem_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    front_guard_rest = ctx.part_element_world_aabb(housing, elem="front_guard")
    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
        front_guard_tilted = ctx.part_element_world_aabb(housing, elem="front_guard")

    rest_z = elem_center_z(front_guard_rest)
    tilted_z = elem_center_z(front_guard_tilted)
    ctx.check(
        "housing tilts upward about the stand hinge",
        rest_z is not None and tilted_z is not None and tilted_z > rest_z + 0.020,
        details=f"rest_z={rest_z}, tilted_z={tilted_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
