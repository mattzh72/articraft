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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="maksutov_altaz_telescope")

    mount_black = model.material("mount_black", rgba=(0.14, 0.15, 0.16, 1.0))
    tripod_metal = model.material("tripod_metal", rgba=(0.64, 0.66, 0.69, 1.0))
    tube_white = model.material("tube_white", rgba=(0.89, 0.90, 0.92, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    glass = model.material("glass", rgba=(0.35, 0.45, 0.52, 0.35))

    def rr_section(
        center_x: float,
        center_y: float,
        z: float,
        size_x: float,
        size_y: float,
        radius: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (center_x + x, center_y + y, z)
            for x, y in rounded_rect_profile(
                size_x,
                size_y,
                radius=radius,
                corner_segments=8,
            )
        ]

    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=0.060, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=mount_black,
        name="tripod_hub",
    )
    tripod_base.visual(
        Cylinder(radius=0.042, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
        material=mount_black,
        name="spreader_collar",
    )
    tripod_base.visual(
        Cylinder(radius=0.050, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.920)),
        material=mount_black,
        name="tripod_pier",
    )
    tripod_base.visual(
        Cylinder(radius=0.092, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.040)),
        material=mount_black,
        name="tripod_bearing_cap",
    )
    tripod_base.visual(
        Box((0.135, 0.135, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 1.015)),
        material=mount_black,
        name="tripod_head_plate",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.034 * c, 0.034 * s, 0.825),
                (0.145 * c, 0.145 * s, 0.560),
                (0.380 * c, 0.380 * s, 0.030),
            ],
            radius=0.013,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        tripod_base.visual(
            mesh_from_geometry(leg_mesh, f"tripod_leg_{index}"),
            material=tripod_metal,
            name=f"leg_{index}",
        )
        tripod_base.visual(
            Sphere(radius=0.020),
            origin=Origin(xyz=(0.380 * c, 0.380 * s, 0.028)),
            material=rubber,
            name=f"foot_{index}",
        )

    tripod_base.inertial = Inertial.from_geometry(
        Box((0.80, 0.80, 1.08)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
    )

    mount_head = model.part("mount_head")
    mount_head.visual(
        Cylinder(radius=0.090, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=mount_black,
        name="azimuth_rotor",
    )
    mount_head.visual(
        Cylinder(radius=0.074, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=mount_black,
        name="azimuth_motor_drum",
    )
    mount_head.visual(
        Box((0.160, 0.145, 0.080)),
        origin=Origin(xyz=(-0.015, -0.020, 0.120)),
        material=mount_black,
        name="electronics_pod",
    )

    arm_mesh = section_loft(
        [
            rr_section(-0.020, -0.107, 0.080, 0.165, 0.118, 0.024),
            rr_section(-0.005, -0.105, 0.180, 0.138, 0.102, 0.022),
            rr_section(0.018, -0.103, 0.290, 0.116, 0.082, 0.018),
            rr_section(0.035, -0.103, 0.382, 0.090, 0.070, 0.015),
        ]
    )
    mount_head.visual(
        mesh_from_geometry(arm_mesh, "single_arm_shell"),
        material=mount_black,
        name="single_arm_shell",
    )
    mount_head.visual(
        Cylinder(radius=0.040, length=0.085),
        origin=Origin(
            xyz=(0.034, -0.092, 0.382),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=mount_black,
        name="altitude_hub",
    )
    mount_head.visual(
        Box((0.122, 0.118, 0.060)),
        origin=Origin(xyz=(0.018, -0.028, 0.252)),
        material=mount_black,
        name="saddle_bridge",
    )
    mount_head.visual(
        Box((0.126, 0.060, 0.012)),
        origin=Origin(xyz=(0.018, 0.032, 0.282)),
        material=dark_trim,
        name="saddle_base",
    )
    mount_head.visual(
        Box((0.108, 0.026, 0.046)),
        origin=Origin(xyz=(0.022, -0.031, 0.265)),
        material=mount_black,
        name="outer_clamp_jaw",
    )
    mount_head.visual(
        Box((0.100, 0.032, 0.032)),
        origin=Origin(xyz=(0.024, 0.066, 0.285)),
        material=mount_black,
        name="inner_clamp_jaw",
    )
    mount_head.visual(
        Cylinder(radius=0.009, length=0.032),
        origin=Origin(
            xyz=(0.062, -0.060, 0.266),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="clamp_knob",
    )
    mount_head.inertial = Inertial.from_geometry(
        Box((0.25, 0.22, 0.44)),
        mass=5.4,
        origin=Origin(xyz=(0.0, -0.040, 0.220)),
    )

    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        Cylinder(radius=0.083, length=0.278),
        origin=Origin(
            xyz=(0.0, 0.110, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=tube_white,
        name="main_tube",
    )
    optical_tube.visual(
        Cylinder(radius=0.092, length=0.066),
        origin=Origin(
            xyz=(-0.160, 0.110, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="rear_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.095, length=0.046),
        origin=Origin(
            xyz=(0.156, 0.110, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="corrector_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.074, length=0.004),
        origin=Origin(
            xyz=(0.178, 0.110, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=glass,
        name="meniscus_plate",
    )
    optical_tube.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(
            xyz=(0.179, 0.110, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="secondary_spot",
    )
    optical_tube.visual(
        Box((0.168, 0.034, 0.014)),
        origin=Origin(xyz=(0.0, 0.110, -0.087)),
        material=dark_trim,
        name="dovetail_rail",
    )
    optical_tube.visual(
        Cylinder(radius=0.016, length=0.118),
        origin=Origin(
            xyz=(0.030, 0.110, 0.113),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="finder_scope_body",
    )
    optical_tube.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(
            xyz=(0.088, 0.110, 0.113),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="finder_objective_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(
            xyz=(-0.046, 0.110, 0.113),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="finder_eyepiece",
    )
    optical_tube.visual(
        Box((0.070, 0.020, 0.014)),
        origin=Origin(xyz=(0.020, 0.110, 0.087)),
        material=dark_trim,
        name="finder_shoe",
    )
    optical_tube.visual(
        Box((0.014, 0.022, 0.020)),
        origin=Origin(xyz=(-0.004, 0.110, 0.100)),
        material=dark_trim,
        name="finder_ring_rear",
    )
    optical_tube.visual(
        Box((0.014, 0.022, 0.020)),
        origin=Origin(xyz=(0.056, 0.110, 0.100)),
        material=dark_trim,
        name="finder_ring_front",
    )
    optical_tube.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(
            xyz=(-0.150, 0.188, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="focus_knob",
    )
    optical_tube.inertial = Inertial.from_geometry(
        Box((0.380, 0.220, 0.220)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.110, 0.0)),
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.CONTINUOUS,
        parent=tripod_base,
        child=mount_head,
        origin=Origin(xyz=(0.0, 0.0, 1.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=mount_head,
        child=optical_tube,
        origin=Origin(xyz=(0.034, -0.078, 0.382)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=-0.20,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod_base = object_model.get_part("tripod_base")
    mount_head = object_model.get_part("mount_head")
    optical_tube = object_model.get_part("optical_tube")
    azimuth_axis = object_model.get_articulation("azimuth_axis")
    altitude_axis = object_model.get_articulation("altitude_axis")

    ctx.check(
        "azimuth articulation is continuous about vertical axis",
        azimuth_axis.articulation_type == ArticulationType.CONTINUOUS
        and tuple(azimuth_axis.axis) == (0.0, 0.0, 1.0),
        details=f"type={azimuth_axis.articulation_type}, axis={azimuth_axis.axis}",
    )
    ctx.check(
        "altitude articulation pitches upward about the arm-top trunnion",
        altitude_axis.articulation_type == ArticulationType.REVOLUTE
        and tuple(altitude_axis.axis) == (0.0, -1.0, 0.0)
        and altitude_axis.motion_limits is not None
        and altitude_axis.motion_limits.upper is not None
        and altitude_axis.motion_limits.upper > 1.0,
        details=(
            f"type={altitude_axis.articulation_type}, axis={altitude_axis.axis}, "
            f"limits={altitude_axis.motion_limits}"
        ),
    )

    ctx.expect_contact(
        mount_head,
        tripod_base,
        elem_a="azimuth_rotor",
        elem_b="tripod_bearing_cap",
        name="azimuth rotor sits on the tripod bearing cap",
    )
    ctx.expect_contact(
        optical_tube,
        mount_head,
        elem_a="dovetail_rail",
        elem_b="saddle_base",
        name="tube dovetail rests in the saddle base",
    )
    ctx.expect_overlap(
        optical_tube,
        mount_head,
        axes="x",
        elem_a="dovetail_rail",
        elem_b="saddle_base",
        min_overlap=0.11,
        name="saddle keeps a substantial grip along the rail length",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_front = aabb_center(
        ctx.part_element_world_aabb(optical_tube, elem="corrector_cell")
    )
    with ctx.pose({altitude_axis: 0.95}):
        raised_front = aabb_center(
            ctx.part_element_world_aabb(optical_tube, elem="corrector_cell")
        )
    ctx.check(
        "positive altitude motion raises the telescope nose",
        rest_front is not None
        and raised_front is not None
        and raised_front[2] > rest_front[2] + 0.12
        and raised_front[0] < rest_front[0] - 0.05,
        details=f"rest_front={rest_front}, raised_front={raised_front}",
    )

    rest_pivot = ctx.part_world_position(optical_tube)
    with ctx.pose({azimuth_axis: math.pi / 2.0}):
        turned_pivot = ctx.part_world_position(optical_tube)
    ctx.check(
        "azimuth motion carries the arm and tube around the vertical bearing",
        rest_pivot is not None
        and turned_pivot is not None
        and rest_pivot[1] < -0.05
        and turned_pivot[0] > 0.05
        and turned_pivot[1] > 0.0,
        details=f"rest_pivot={rest_pivot}, turned_pivot={turned_pivot}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
