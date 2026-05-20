from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(-x, y, z) for x, y, z in points]

    def tube_mesh(
        name: str,
        points: list[tuple[float, float, float]],
        *,
        radius: float,
        samples_per_segment: int = 16,
        radial_segments: int = 18,
    ):
        return save_mesh(
            name,
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=samples_per_segment,
                radial_segments=radial_segments,
                cap_ends=True,
            ),
        )

    def add_wheel_visuals(
        part,
        prefix: str,
        *,
        radius: float,
        width: float,
        hub_radius: float,
        tire_material,
        rim_material,
        hub_material,
    ) -> None:
        half_width = width * 0.5
        tire_profile = [
            (radius * 0.72, -half_width * 0.96),
            (radius * 0.87, -half_width),
            (radius * 0.98, -half_width * 0.52),
            (radius, 0.0),
            (radius * 0.98, half_width * 0.52),
            (radius * 0.87, half_width),
            (radius * 0.72, half_width * 0.96),
            (radius * 0.57, half_width * 0.52),
            (radius * 0.52, 0.0),
            (radius * 0.57, -half_width * 0.52),
            (radius * 0.72, -half_width * 0.96),
        ]
        tire = save_mesh(
            f"{prefix}_tire.obj",
            LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0),
        )
        spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
        part.visual(tire, material=tire_material, name="tire")
        part.visual(
            Cylinder(radius=radius * 0.79, length=width * 0.86),
            origin=spin_origin,
            material=rim_material,
            name="rim_shell",
        )
        part.visual(
            Cylinder(radius=radius * 0.57, length=width * 0.22),
            origin=Origin(xyz=(width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hub_material,
            name="outer_face",
        )
        part.visual(
            Cylinder(radius=radius * 0.57, length=width * 0.22),
            origin=Origin(xyz=(-width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hub_material,
            name="inner_face",
        )
        part.visual(
            Cylinder(radius=hub_radius, length=width),
            origin=spin_origin,
            material=hub_material,
            name="hub",
        )
        part.visual(
            Cylinder(radius=hub_radius * 0.44, length=width * 1.04),
            origin=spin_origin,
            material=rim_material,
            name="axle_cap",
        )

    model = ArticulatedObject(name="premium_rolling_walker", assets=ASSETS)

    frame_satin = model.material("frame_satin", rgba=(0.77, 0.75, 0.70, 1.0))
    graphite_satin = model.material("graphite_satin", rgba=(0.31, 0.33, 0.35, 1.0))
    rubber_matte = model.material("rubber_matte", rgba=(0.08, 0.08, 0.08, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    nylon_trim = model.material("nylon_trim", rgba=(0.18, 0.19, 0.21, 1.0))
    seat_fabric = model.material("seat_fabric", rgba=(0.26, 0.27, 0.30, 1.0))
    silver_hub = model.material("silver_hub", rgba=(0.72, 0.74, 0.76, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((0.64, 0.72, 0.92)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    left_lower_points = [
        (0.230, -0.220, 0.180),
        (0.240, -0.100, 0.180),
        (0.246, 0.075, 0.184),
        (0.244, 0.165, 0.192),
        (0.240, 0.205, 0.205),
    ]
    left_front_upright_points = [
        (0.240, 0.205, 0.205),
        (0.228, 0.190, 0.335),
        (0.230, 0.160, 0.515),
        (0.236, 0.115, 0.720),
    ]
    left_rear_upright_points = [
        (0.230, -0.220, 0.180),
        (0.228, -0.170, 0.470),
        (0.220, -0.110, 0.820),
    ]
    left_upper_side_points = [
        (0.236, 0.115, 0.720),
        (0.233, 0.020, 0.760),
        (0.228, -0.060, 0.800),
        (0.220, -0.110, 0.820),
    ]
    left_front_seat_support = [
        (0.240, -0.010, 0.185),
        (0.215, 0.010, 0.340),
        (0.195, 0.030, 0.545),
    ]
    left_rear_seat_support = [
        (0.235, -0.170, 0.180),
        (0.215, -0.120, 0.335),
        (0.195, -0.050, 0.545),
    ]
    cross_brace_a = [
        (0.225, -0.150, 0.190),
        (0.075, -0.060, 0.310),
        (-0.060, 0.000, 0.410),
        (-0.195, 0.030, 0.545),
    ]
    cross_brace_b = [
        (-0.225, -0.150, 0.190),
        (-0.075, -0.060, 0.310),
        (0.060, 0.000, 0.410),
        (0.195, 0.030, 0.545),
    ]

    chassis.visual(
        tube_mesh("walker_left_lower_rail.obj", left_lower_points, radius=0.0115),
        material=frame_satin,
        name="left_lower_rail",
    )
    chassis.visual(
        tube_mesh("walker_right_lower_rail.obj", mirror_x(left_lower_points), radius=0.0115),
        material=frame_satin,
        name="right_lower_rail",
    )
    chassis.visual(
        tube_mesh("walker_left_front_upright.obj", left_front_upright_points, radius=0.0110),
        material=frame_satin,
        name="left_front_upright",
    )
    chassis.visual(
        tube_mesh("walker_right_front_upright.obj", mirror_x(left_front_upright_points), radius=0.0110),
        material=frame_satin,
        name="right_front_upright",
    )
    chassis.visual(
        tube_mesh("walker_left_rear_upright.obj", left_rear_upright_points, radius=0.0110),
        material=frame_satin,
        name="left_rear_upright",
    )
    chassis.visual(
        tube_mesh("walker_right_rear_upright.obj", mirror_x(left_rear_upright_points), radius=0.0110),
        material=frame_satin,
        name="right_rear_upright",
    )
    chassis.visual(
        tube_mesh("walker_left_upper_side.obj", left_upper_side_points, radius=0.0105),
        material=frame_satin,
        name="left_upper_side",
    )
    chassis.visual(
        tube_mesh("walker_right_upper_side.obj", mirror_x(left_upper_side_points), radius=0.0105),
        material=frame_satin,
        name="right_upper_side",
    )
    chassis.visual(
        tube_mesh("walker_left_front_seat_support.obj", left_front_seat_support, radius=0.0090),
        material=frame_satin,
        name="left_front_seat_support",
    )
    chassis.visual(
        tube_mesh("walker_right_front_seat_support.obj", mirror_x(left_front_seat_support), radius=0.0090),
        material=frame_satin,
        name="right_front_seat_support",
    )
    chassis.visual(
        tube_mesh("walker_left_rear_seat_support.obj", left_rear_seat_support, radius=0.0090),
        material=frame_satin,
        name="left_rear_seat_support",
    )
    chassis.visual(
        tube_mesh("walker_right_rear_seat_support.obj", mirror_x(left_rear_seat_support), radius=0.0090),
        material=frame_satin,
        name="right_rear_seat_support",
    )
    chassis.visual(
        tube_mesh("walker_cross_brace_a.obj", cross_brace_a, radius=0.0070, samples_per_segment=14),
        material=graphite_satin,
        name="cross_brace_a",
    )
    chassis.visual(
        tube_mesh("walker_cross_brace_b.obj", cross_brace_b, radius=0.0070, samples_per_segment=14),
        material=graphite_satin,
        name="cross_brace_b",
    )

    chassis.visual(
        Cylinder(radius=0.0105, length=0.470),
        origin=Origin(xyz=(0.0, 0.205, 0.205), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_satin,
        name="front_lower_crossbar",
    )
    chassis.visual(
        Cylinder(radius=0.0100, length=0.444),
        origin=Origin(xyz=(0.0, -0.110, 0.820), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_satin,
        name="handle_crossbar",
    )
    chassis.visual(
        Cylinder(radius=0.0090, length=0.390),
        origin=Origin(xyz=(0.0, 0.030, 0.526), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite_satin,
        name="seat_front_crossbar",
    )
    chassis.visual(
        Cylinder(radius=0.0090, length=0.390),
        origin=Origin(xyz=(0.0, -0.050, 0.526), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite_satin,
        name="seat_rear_crossbar",
    )
    chassis.visual(
        Box((0.280, 0.100, 0.012)),
        origin=Origin(xyz=(0.0, -0.010, 0.541)),
        material=nylon_trim,
        name="seat_support_bridge",
    )

    chassis.visual(
        Cylinder(radius=0.0170, length=0.038),
        origin=Origin(xyz=(0.255, 0.285, 0.267)),
        material=graphite_satin,
        name="front_left_head_tube",
    )
    chassis.visual(
        Cylinder(radius=0.0170, length=0.038),
        origin=Origin(xyz=(-0.255, 0.285, 0.267)),
        material=graphite_satin,
        name="front_right_head_tube",
    )
    chassis.visual(
        tube_mesh(
            "walker_left_head_gusset.obj",
            [
                (0.240, 0.205, 0.205),
                (0.241, 0.220, 0.236),
                (0.245, 0.238, 0.258),
                (0.250, 0.258, 0.274),
                (0.255, 0.276, 0.278),
            ],
            radius=0.0080,
            samples_per_segment=14,
            radial_segments=16,
        ),
        material=frame_satin,
        name="left_head_gusset",
    )
    chassis.visual(
        tube_mesh(
            "walker_right_head_gusset.obj",
            [
                (-0.240, 0.205, 0.205),
                (-0.241, 0.220, 0.236),
                (-0.245, 0.238, 0.258),
                (-0.250, 0.258, 0.274),
                (-0.255, 0.276, 0.278),
            ],
            radius=0.0080,
            samples_per_segment=14,
            radial_segments=16,
        ),
        material=frame_satin,
        name="right_head_gusset",
    )

    chassis.visual(
        Box((0.050, 0.060, 0.180)),
        origin=Origin(xyz=(0.257, -0.220, 0.180)),
        material=graphite_satin,
        name="left_rear_axle_plate",
    )
    chassis.visual(
        Box((0.050, 0.060, 0.180)),
        origin=Origin(xyz=(-0.257, -0.220, 0.180)),
        material=graphite_satin,
        name="right_rear_axle_plate",
    )
    chassis.visual(
        Box((0.014, 0.022, 0.058)),
        origin=Origin(xyz=(0.257, -0.205, 0.126)),
        material=nylon_trim,
        name="left_brake_caliper",
    )
    chassis.visual(
        Box((0.014, 0.022, 0.058)),
        origin=Origin(xyz=(-0.257, -0.205, 0.126)),
        material=nylon_trim,
        name="right_brake_caliper",
    )

    chassis.visual(
        Box((0.028, 0.048, 0.014)),
        origin=Origin(xyz=(0.236, -0.145, 0.816)),
        material=graphite_satin,
        name="left_lever_mount",
    )
    chassis.visual(
        Box((0.028, 0.048, 0.014)),
        origin=Origin(xyz=(-0.236, -0.145, 0.816)),
        material=graphite_satin,
        name="right_lever_mount",
    )
    chassis.visual(
        tube_mesh(
            "walker_left_handle_support.obj",
            [
                (0.220, -0.110, 0.820),
                (0.232, -0.145, 0.822),
                (0.246, -0.172, 0.830),
                (0.255, -0.198, 0.837),
            ],
            radius=0.0100,
            samples_per_segment=14,
            radial_segments=16,
        ),
        material=frame_satin,
        name="left_handle_support",
    )
    chassis.visual(
        tube_mesh(
            "walker_right_handle_support.obj",
            [
                (-0.220, -0.110, 0.820),
                (-0.232, -0.145, 0.822),
                (-0.246, -0.172, 0.830),
                (-0.255, -0.198, 0.837),
            ],
            radius=0.0100,
            samples_per_segment=14,
            radial_segments=16,
        ),
        material=frame_satin,
        name="right_handle_support",
    )
    chassis.visual(
        Cylinder(radius=0.0170, length=0.120),
        origin=Origin(xyz=(0.255, -0.198, 0.837), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_matte,
        name="left_grip",
    )
    chassis.visual(
        Cylinder(radius=0.0170, length=0.120),
        origin=Origin(xyz=(-0.255, -0.198, 0.837), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_matte,
        name="right_grip",
    )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.390, 0.220, 0.036)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.110, 0.018)),
    )
    seat_shell_mesh = save_mesh(
        "walker_seat_shell.obj",
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.390, 0.220, 0.030, corner_segments=8),
            0.018,
        ),
    )
    seat_pad_mesh = save_mesh(
        "walker_seat_pad.obj",
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.370, 0.200, 0.040, corner_segments=10),
            0.020,
        ),
    )
    seat.visual(
        seat_shell_mesh,
        origin=Origin(xyz=(0.0, -0.110, 0.009)),
        material=nylon_trim,
        name="seat_shell",
    )
    seat.visual(
        seat_pad_mesh,
        origin=Origin(xyz=(0.0, -0.110, 0.028)),
        material=seat_fabric,
        name="seat_pad",
    )
    seat.visual(
        Box((0.340, 0.028, 0.024)),
        origin=Origin(xyz=(0.0, -0.192, 0.020)),
        material=seat_fabric,
        name="rear_lip",
    )
    seat.visual(
        Box((0.036, 0.024, 0.024)),
        origin=Origin(xyz=(0.140, -0.008, 0.012)),
        material=graphite_satin,
        name="left_hinge_tab",
    )
    seat.visual(
        Box((0.036, 0.024, 0.024)),
        origin=Origin(xyz=(-0.140, -0.008, 0.012)),
        material=graphite_satin,
        name="right_hinge_tab",
    )

    lever_blade_mesh = save_mesh(
        "walker_brake_lever_blade.obj",
        tube_from_spline_points(
            [
                (0.0, -0.004, -0.008),
                (0.0, -0.018, -0.024),
                (0.0, -0.031, -0.048),
                (0.0, -0.028, -0.088),
            ],
            radius=0.0045,
            samples_per_segment=18,
            radial_segments=12,
            cap_ends=True,
        ),
    )

    left_brake = model.part("left_brake_lever")
    left_brake.inertial = Inertial.from_geometry(
        Box((0.030, 0.090, 0.030)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.030, -0.035)),
    )
    left_brake.visual(
        Cylinder(radius=0.0090, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite_satin,
        name="pivot_boss",
    )
    left_brake.visual(lever_blade_mesh, material=graphite_satin, name="lever_blade")
    left_brake.visual(
        Box((0.010, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, -0.010)),
        material=graphite_satin,
        name="blade_bridge",
    )
    left_brake.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(0.015, 0.005, -0.012)),
        material=silver_hub,
        name="parking_lock",
    )

    right_brake = model.part("right_brake_lever")
    right_brake.inertial = Inertial.from_geometry(
        Box((0.030, 0.090, 0.030)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.030, -0.035)),
    )
    right_brake.visual(
        Cylinder(radius=0.0090, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite_satin,
        name="pivot_boss",
    )
    right_brake.visual(lever_blade_mesh, material=graphite_satin, name="lever_blade")
    right_brake.visual(
        Box((0.010, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, -0.010)),
        material=graphite_satin,
        name="blade_bridge",
    )
    right_brake.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(-0.015, 0.005, -0.012)),
        material=silver_hub,
        name="parking_lock",
    )

    front_left_caster = model.part("front_left_caster")
    front_left_caster.inertial = Inertial.from_geometry(
        Box((0.050, 0.040, 0.180)),
        mass=0.60,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )
    front_left_caster.visual(
        Cylinder(radius=0.0110, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=graphite_satin,
        name="swivel_stem",
    )
    front_left_caster.visual(
        Cylinder(radius=0.0200, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=nylon_trim,
        name="swivel_collar",
    )
    front_left_caster.visual(
        Box((0.036, 0.026, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=graphite_satin,
        name="crown",
    )
    front_left_caster.visual(
        Box((0.012, 0.014, 0.100)),
        origin=Origin(xyz=(0.024, 0.0, -0.110)),
        material=graphite_satin,
        name="fork_left",
    )
    front_left_caster.visual(
        Box((0.012, 0.014, 0.100)),
        origin=Origin(xyz=(-0.024, 0.0, -0.110)),
        material=graphite_satin,
        name="fork_right",
    )
    front_left_caster.visual(
        Cylinder(radius=0.0060, length=0.010),
        origin=Origin(xyz=(0.023, 0.0, -0.150), rpy=(0.0, pi / 2.0, 0.0)),
        material=silver_hub,
        name="axle_stub_outer",
    )
    front_left_caster.visual(
        Cylinder(radius=0.0060, length=0.010),
        origin=Origin(xyz=(-0.023, 0.0, -0.150), rpy=(0.0, pi / 2.0, 0.0)),
        material=silver_hub,
        name="axle_stub_inner",
    )

    front_right_caster = model.part("front_right_caster")
    front_right_caster.inertial = Inertial.from_geometry(
        Box((0.050, 0.040, 0.180)),
        mass=0.60,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )
    front_right_caster.visual(
        Cylinder(radius=0.0110, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=graphite_satin,
        name="swivel_stem",
    )
    front_right_caster.visual(
        Cylinder(radius=0.0200, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=nylon_trim,
        name="swivel_collar",
    )
    front_right_caster.visual(
        Box((0.036, 0.026, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=graphite_satin,
        name="crown",
    )
    front_right_caster.visual(
        Box((0.012, 0.014, 0.100)),
        origin=Origin(xyz=(0.024, 0.0, -0.110)),
        material=graphite_satin,
        name="fork_left",
    )
    front_right_caster.visual(
        Box((0.012, 0.014, 0.100)),
        origin=Origin(xyz=(-0.024, 0.0, -0.110)),
        material=graphite_satin,
        name="fork_right",
    )
    front_right_caster.visual(
        Cylinder(radius=0.0060, length=0.010),
        origin=Origin(xyz=(0.023, 0.0, -0.150), rpy=(0.0, pi / 2.0, 0.0)),
        material=silver_hub,
        name="axle_stub_outer",
    )
    front_right_caster.visual(
        Cylinder(radius=0.0060, length=0.010),
        origin=Origin(xyz=(-0.023, 0.0, -0.150), rpy=(0.0, pi / 2.0, 0.0)),
        material=silver_hub,
        name="axle_stub_inner",
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.036),
        mass=0.72,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel_visuals(
        front_left_wheel,
        "walker_front_left_wheel",
        radius=0.090,
        width=0.036,
        hub_radius=0.026,
        tire_material=tire_rubber,
        rim_material=silver_hub,
        hub_material=graphite_satin,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.036),
        mass=0.72,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel_visuals(
        front_right_wheel,
        "walker_front_right_wheel",
        radius=0.090,
        width=0.036,
        hub_radius=0.026,
        tire_material=tire_rubber,
        rim_material=silver_hub,
        hub_material=graphite_satin,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.034),
        mass=0.90,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel_visuals(
        rear_left_wheel,
        "walker_rear_left_wheel",
        radius=0.100,
        width=0.034,
        hub_radius=0.028,
        tire_material=tire_rubber,
        rim_material=silver_hub,
        hub_material=graphite_satin,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.034),
        mass=0.90,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel_visuals(
        rear_right_wheel,
        "walker_rear_right_wheel",
        radius=0.100,
        width=0.034,
        hub_radius=0.028,
        tire_material=tire_rubber,
        rim_material=silver_hub,
        hub_material=graphite_satin,
    )

    model.articulation(
        "seat_fold",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=seat,
        origin=Origin(xyz=(0.0, 0.055, 0.547)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.20, upper=0.05),
    )
    model.articulation(
        "left_brake_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=left_brake,
        origin=Origin(xyz=(0.236, -0.160, 0.803)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=-0.45, upper=0.12),
    )
    model.articulation(
        "right_brake_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=right_brake,
        origin=Origin(xyz=(-0.236, -0.160, 0.803)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=-0.45, upper=0.12),
    )
    model.articulation(
        "front_left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=front_left_caster,
        origin=Origin(xyz=(0.255, 0.285, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "front_right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=front_right_caster,
        origin=Origin(xyz=(-0.255, 0.285, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_left_caster,
        child=front_left_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_right_caster,
        child=front_right_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.299, -0.220, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.299, -0.220, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    seat = object_model.get_part("seat")
    left_brake = object_model.get_part("left_brake_lever")
    right_brake = object_model.get_part("right_brake_lever")
    front_left_caster = object_model.get_part("front_left_caster")
    front_right_caster = object_model.get_part("front_right_caster")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    seat_fold = object_model.get_articulation("seat_fold")
    left_brake_hinge = object_model.get_articulation("left_brake_hinge")
    right_brake_hinge = object_model.get_articulation("right_brake_hinge")
    front_left_caster_swivel = object_model.get_articulation("front_left_caster_swivel")
    front_right_caster_swivel = object_model.get_articulation("front_right_caster_swivel")
    front_left_wheel_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_wheel_spin = object_model.get_articulation("front_right_wheel_spin")

    seat_shell = seat.get_visual("seat_shell")
    rear_lip = seat.get_visual("rear_lip")
    left_pivot = left_brake.get_visual("pivot_boss")
    right_pivot = right_brake.get_visual("pivot_boss")
    left_blade = left_brake.get_visual("lever_blade")
    right_blade = right_brake.get_visual("lever_blade")
    left_mount = chassis.get_visual("left_lever_mount")
    right_mount = chassis.get_visual("right_lever_mount")
    left_grip = chassis.get_visual("left_grip")
    right_grip = chassis.get_visual("right_grip")
    support_bridge = chassis.get_visual("seat_support_bridge")
    left_head_tube = chassis.get_visual("front_left_head_tube")
    right_head_tube = chassis.get_visual("front_right_head_tube")
    left_collar = front_left_caster.get_visual("swivel_collar")
    right_collar = front_right_caster.get_visual("swivel_collar")
    left_fork_blade = front_left_caster.get_visual("fork_left")
    right_fork_blade = front_right_caster.get_visual("fork_left")
    front_left_hub = front_left_wheel.get_visual("hub")
    front_right_hub = front_right_wheel.get_visual("hub")
    rear_left_hub = rear_left_wheel.get_visual("hub")
    rear_right_hub = rear_right_wheel.get_visual("hub")
    left_axle_plate = chassis.get_visual("left_rear_axle_plate")
    right_axle_plate = chassis.get_visual("right_rear_axle_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_origin_distance(
        front_left_caster,
        front_right_caster,
        axes="x",
        min_dist=0.48,
        max_dist=0.54,
        name="front_track_is_realistic",
    )
    ctx.expect_origin_distance(
        front_left_caster,
        rear_left_wheel,
        axes="y",
        min_dist=0.42,
        max_dist=0.55,
        name="wheelbase_is_realistic",
    )
    ctx.expect_origin_distance(
        left_brake,
        right_brake,
        axes="x",
        min_dist=0.46,
        max_dist=0.56,
        name="handle_spacing_is_realistic",
    )
    ctx.expect_origin_distance(
        seat,
        chassis,
        axes="x",
        max_dist=0.005,
        name="seat_is_centered",
    )

    ctx.expect_gap(
        seat,
        chassis,
        axis="z",
        positive_elem=seat_shell,
        negative_elem=support_bridge,
        max_gap=0.001,
        max_penetration=0.0,
        name="seat_rests_on_support_bridge",
    )
    ctx.expect_overlap(
        seat,
        chassis,
        axes="xy",
        elem_a=seat_shell,
        elem_b=support_bridge,
        min_overlap=0.08,
        name="seat_overlaps_support_bridge_footprint",
    )
    with ctx.pose({seat_fold: -1.05}):
        ctx.expect_gap(
            seat,
            chassis,
            axis="z",
            positive_elem=rear_lip,
            negative_elem=support_bridge,
            min_gap=0.10,
            name="seat_lifts_clear_when_folded",
        )

    ctx.expect_contact(
        left_brake,
        chassis,
        elem_a=left_pivot,
        elem_b=left_mount,
        contact_tol=0.0005,
        name="left_brake_pivot_is_mounted",
    )
    ctx.expect_contact(
        right_brake,
        chassis,
        elem_a=right_pivot,
        elem_b=right_mount,
        contact_tol=0.0005,
        name="right_brake_pivot_is_mounted",
    )
    ctx.expect_gap(
        chassis,
        left_brake,
        axis="z",
        positive_elem=left_grip,
        negative_elem=left_blade,
        min_gap=0.015,
        max_gap=0.035,
        name="left_brake_has_rest_gap",
    )
    ctx.expect_gap(
        chassis,
        right_brake,
        axis="z",
        positive_elem=right_grip,
        negative_elem=right_blade,
        min_gap=0.015,
        max_gap=0.035,
        name="right_brake_has_rest_gap",
    )
    with ctx.pose({left_brake_hinge: -0.40, right_brake_hinge: -0.40}):
        ctx.expect_gap(
            chassis,
            left_brake,
            axis="z",
            positive_elem=left_grip,
            negative_elem=left_blade,
            min_gap=0.008,
            max_gap=0.028,
            name="left_brake_squeezes_toward_grip",
        )
        ctx.expect_gap(
            chassis,
            right_brake,
            axis="z",
            positive_elem=right_grip,
            negative_elem=right_blade,
            min_gap=0.008,
            max_gap=0.028,
            name="right_brake_squeezes_toward_grip",
        )

    ctx.expect_gap(
        chassis,
        front_left_caster,
        axis="z",
        positive_elem=left_head_tube,
        negative_elem=left_collar,
        max_gap=0.0005,
        max_penetration=1e-5,
        name="left_caster_swivel_seats_under_head_tube",
    )
    ctx.expect_gap(
        chassis,
        front_right_caster,
        axis="z",
        positive_elem=right_head_tube,
        negative_elem=right_collar,
        max_gap=0.0005,
        max_penetration=1e-5,
        name="right_caster_swivel_seats_under_head_tube",
    )
    ctx.expect_overlap(
        front_left_caster,
        chassis,
        axes="xy",
        elem_a=left_collar,
        elem_b=left_head_tube,
        min_overlap=0.020,
        name="left_caster_swivel_is_axially_aligned",
    )
    ctx.expect_overlap(
        front_right_caster,
        chassis,
        axes="xy",
        elem_a=right_collar,
        elem_b=right_head_tube,
        min_overlap=0.020,
        name="right_caster_swivel_is_axially_aligned",
    )
    ctx.expect_contact(
        front_left_wheel,
        front_left_caster,
        elem_a=front_left_hub,
        elem_b=left_fork_blade,
        contact_tol=0.0005,
        name="left_front_wheel_is_captured_by_fork",
    )
    ctx.expect_contact(
        front_right_wheel,
        front_right_caster,
        elem_a=front_right_hub,
        elem_b=right_fork_blade,
        contact_tol=0.0005,
        name="right_front_wheel_is_captured_by_fork",
    )
    with ctx.pose({front_left_caster_swivel: 0.80, front_right_caster_swivel: -0.65}):
        ctx.expect_contact(
            front_left_wheel,
            front_left_caster,
            elem_a=front_left_hub,
            elem_b=left_fork_blade,
            contact_tol=0.0005,
            name="left_front_wheel_stays_mounted_when_swiveled",
        )
        ctx.expect_contact(
            front_right_wheel,
            front_right_caster,
            elem_a=front_right_hub,
            elem_b=right_fork_blade,
            contact_tol=0.0005,
            name="right_front_wheel_stays_mounted_when_swiveled",
        )
    with ctx.pose({front_left_wheel_spin: 1.3, front_right_wheel_spin: -0.9}):
        ctx.expect_contact(
            front_left_wheel,
            front_left_caster,
            elem_a=front_left_hub,
            elem_b=left_fork_blade,
            contact_tol=0.0005,
            name="left_front_wheel_stays_on_axle_when_spun",
        )
        ctx.expect_contact(
            front_right_wheel,
            front_right_caster,
            elem_a=front_right_hub,
            elem_b=right_fork_blade,
            contact_tol=0.0005,
            name="right_front_wheel_stays_on_axle_when_spun",
        )

    ctx.expect_gap(
        rear_left_wheel,
        chassis,
        axis="x",
        positive_elem=rear_left_hub,
        negative_elem=left_axle_plate,
        max_gap=0.0005,
        max_penetration=1e-5,
        name="left_rear_wheel_meets_axle_plate",
    )
    ctx.expect_gap(
        chassis,
        rear_right_wheel,
        axis="x",
        positive_elem=right_axle_plate,
        negative_elem=rear_right_hub,
        max_gap=0.0005,
        max_penetration=1e-5,
        name="right_rear_wheel_meets_axle_plate",
    )
    ctx.expect_contact(
        rear_left_wheel,
        chassis,
        elem_a=rear_left_hub,
        elem_b=left_axle_plate,
        contact_tol=0.0005,
        name="left_rear_wheel_is_mounted",
    )
    ctx.expect_contact(
        rear_right_wheel,
        chassis,
        elem_a=rear_right_hub,
        elem_b=right_axle_plate,
        contact_tol=0.0005,
        name="right_rear_wheel_is_mounted",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
