from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stick_vacuum_fold_joint", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    smoked_poly = model.material("smoked_poly", rgba=(0.34, 0.38, 0.42, 0.58))
    soft_black = model.material("soft_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.56, 0.58, 0.61, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def xy_section(
        size_x: float,
        size_y: float,
        radius: float,
        z: float,
        *,
        x_shift: float = 0.0,
        y_shift: float = 0.0,
        corner_segments: int = 8,
    ) -> list[tuple[float, float, float]]:
        return [
            (x + x_shift, y + y_shift, z)
            for x, y in rounded_rect_profile(size_x, size_y, radius, corner_segments=corner_segments)
        ]

    motor_body = model.part("motor_body")

    body_shell_geom = repair_loft(
        section_loft(
            [
                xy_section(0.058, 0.054, 0.014, 0.012, x_shift=-0.004),
                xy_section(0.066, 0.058, 0.016, 0.115, x_shift=0.000),
                xy_section(0.074, 0.062, 0.018, 0.215, x_shift=0.008),
                xy_section(0.054, 0.046, 0.013, 0.292, x_shift=0.014),
            ]
        )
    )
    motor_body.visual(
        save_mesh("vac_motor_body_shell.obj", body_shell_geom),
        material=matte_graphite,
        name="body_shell",
    )

    handle_geom = tube_from_spline_points(
        [
            (-0.024, 0.0, 0.098),
            (-0.050, 0.0, 0.182),
            (-0.044, 0.0, 0.274),
            (0.000, 0.0, 0.288),
        ],
        radius=0.0105,
        samples_per_segment=18,
        radial_segments=18,
    )
    motor_body.visual(
        save_mesh("vac_handle_spine.obj", handle_geom),
        material=matte_graphite,
        name="handle_spine",
    )

    dust_bin_geom = LatheGeometry(
        [
            (0.0, -0.050),
            (0.026, -0.050),
            (0.029, -0.040),
            (0.029, 0.012),
            (0.026, 0.036),
            (0.016, 0.052),
            (0.0, 0.058),
        ],
        segments=52,
    ).rotate_y(math.pi / 2.0)
    motor_body.visual(
        save_mesh("vac_dust_bin.obj", dust_bin_geom),
        origin=Origin(xyz=(0.065, 0.0, 0.166)),
        material=smoked_poly,
        name="dust_bin",
    )
    motor_body.visual(
        Cylinder(radius=0.031, length=0.016),
        origin=Origin(xyz=(0.018, 0.0, 0.166), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="dust_bin_collar",
    )
    motor_body.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.118, 0.0, 0.166), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="dust_bin_nose",
    )
    motor_body.visual(
        save_mesh(
            "vac_battery_pack.obj",
            repair_loft(
                section_loft(
                    [
                        xy_section(0.042, 0.048, 0.010, 0.000, x_shift=-0.004),
                        xy_section(0.045, 0.050, 0.011, 0.030, x_shift=-0.003),
                        xy_section(0.041, 0.046, 0.010, 0.076, x_shift=0.001),
                        xy_section(0.032, 0.038, 0.008, 0.094, x_shift=0.006),
                    ]
                )
            ),
        ),
        origin=Origin(xyz=(-0.031, 0.0, 0.022)),
        material=soft_black,
        name="battery_pack",
    )
    motor_body.visual(
        Cylinder(radius=0.034, length=0.006),
        origin=Origin(xyz=(0.004, 0.0, 0.114)),
        material=satin_steel,
        name="service_ring",
    )
    motor_body.visual(
        save_mesh(
            "vac_trigger_pod.obj",
            repair_loft(
                section_loft(
                    [
                        xy_section(0.015, 0.019, 0.004, 0.000, x_shift=-0.001),
                        xy_section(0.018, 0.022, 0.005, 0.010, x_shift=0.000),
                        xy_section(0.020, 0.024, 0.006, 0.020, x_shift=0.002),
                        xy_section(0.014, 0.018, 0.004, 0.032, x_shift=0.004),
                    ]
                )
            ),
        ),
        origin=Origin(xyz=(0.028, 0.0, 0.138)),
        material=soft_black,
        name="trigger_pod",
    )
    motor_body.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.302)),
        material=satin_steel,
        name="filter_cap",
    )
    motor_body.visual(
        Cylinder(radius=0.023, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_aluminum,
        name="wand_socket_collar",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.19, 0.10, 0.34)),
        mass=2.4,
        origin=Origin(xyz=(0.020, 0.0, 0.155)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=satin_aluminum,
        name="top_spigot",
    )
    upper_wand.visual(
        Cylinder(radius=0.017, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=satin_aluminum,
        name="upper_tube",
    )
    upper_wand.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=satin_steel,
        name="upper_seam_ring",
    )
    upper_wand.visual(
        Box((0.030, 0.034, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.311)),
        material=matte_graphite,
        name="hinge_bridge",
    )
    upper_wand.visual(
        Box((0.026, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, 0.014, -0.332)),
        material=matte_graphite,
        name="hinge_fork_left",
    )
    upper_wand.visual(
        Box((0.026, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, -0.014, -0.332)),
        material=matte_graphite,
        name="hinge_fork_right",
    )
    upper_wand.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.020, -0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hinge_axle_cap_left",
    )
    upper_wand.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, -0.020, -0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hinge_axle_cap_right",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.350),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hinge_barrel",
    )
    lower_wand.visual(
        Box((0.028, 0.020, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=matte_graphite,
        name="hinge_saddle",
    )
    lower_wand.visual(
        Cylinder(radius=0.017, length=0.302),
        origin=Origin(xyz=(0.0, 0.0, -0.171)),
        material=satin_aluminum,
        name="lower_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.132)),
        material=satin_steel,
        name="lower_seam_ring",
    )
    lower_wand.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.013, 0.0, -0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="link_anchor_front",
    )
    lower_wand.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(-0.013, 0.0, -0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="link_anchor_rear",
    )
    lower_wand.visual(
        Box((0.028, 0.034, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.313)),
        material=matte_graphite,
        name="head_bridge",
    )
    lower_wand.visual(
        Box((0.024, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.014, -0.332)),
        material=matte_graphite,
        name="head_fork_left",
    )
    lower_wand.visual(
        Box((0.024, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -0.014, -0.332)),
        material=matte_graphite,
        name="head_fork_right",
    )
    lower_wand.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(0.0, 0.020, -0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="head_axle_cap_left",
    )
    lower_wand.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(0.0, -0.020, -0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="head_axle_cap_right",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.350),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pivot_barrel",
    )
    floor_head.visual(
        save_mesh(
            "vac_head_neck_tower.obj",
            repair_loft(
                section_loft(
                    [
                        xy_section(0.026, 0.018, 0.0045, -0.050, x_shift=-0.004),
                        xy_section(0.026, 0.018, 0.0050, -0.028, x_shift=-0.004),
                        xy_section(0.020, 0.016, 0.0045, -0.006, x_shift=-0.003),
                        xy_section(0.017, 0.014, 0.0040, 0.008, x_shift=-0.002),
                    ]
                )
            ),
        ),
        material=matte_graphite,
        name="neck_tower",
    )
    head_shell_geom = repair_loft(
        section_loft(
            [
                xy_section(0.108, 0.272, 0.016, -0.086, x_shift=0.010),
                xy_section(0.104, 0.282, 0.020, -0.068, x_shift=0.015),
                xy_section(0.088, 0.262, 0.018, -0.046, x_shift=0.022),
            ]
        )
    )
    floor_head.visual(
        save_mesh("vac_floor_head_shell.obj", head_shell_geom),
        material=matte_graphite,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.050, 0.110, 0.014)),
        origin=Origin(xyz=(0.016, 0.0, -0.090)),
        material=soft_black,
        name="intake_throat",
    )
    floor_head.visual(
        Box((0.010, 0.236, 0.012)),
        origin=Origin(xyz=(0.060, 0.0, -0.078)),
        material=soft_black,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.060, 0.182, 0.014)),
        origin=Origin(xyz=(0.012, 0.0, -0.053)),
        material=satin_steel,
        name="brush_window",
    )
    floor_head.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(-0.020, 0.134, -0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="wheel_left",
    )
    floor_head.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(-0.020, -0.134, -0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="wheel_right",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.12, 0.29, 0.10)),
        mass=0.7,
        origin=Origin(xyz=(0.014, 0.0, -0.060)),
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.FIXED,
        parent=motor_body,
        child=upper_wand,
        origin=Origin(),
    )
    model.articulation(
        "wand_fold",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.0, 0.0, -0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-1.22,
            upper=0.18,
        ),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=-0.40,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    motor_body = object_model.get_part("motor_body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_head = object_model.get_part("floor_head")

    wand_fold = object_model.get_articulation("wand_fold")
    head_pitch = object_model.get_articulation("head_pitch")

    body_socket = motor_body.get_visual("wand_socket_collar")
    upper_spigot = upper_wand.get_visual("top_spigot")
    upper_left_fork = upper_wand.get_visual("hinge_fork_left")
    upper_right_fork = upper_wand.get_visual("hinge_fork_right")
    lower_barrel = lower_wand.get_visual("hinge_barrel")
    lower_left_fork = lower_wand.get_visual("head_fork_left")
    lower_right_fork = lower_wand.get_visual("head_fork_right")
    head_barrel = floor_head.get_visual("pivot_barrel")
    front_bumper = floor_head.get_visual("front_bumper")

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

    ctx.expect_contact(motor_body, upper_wand, elem_a=body_socket, elem_b=upper_spigot)
    ctx.expect_gap(
        motor_body,
        upper_wand,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=body_socket,
        negative_elem=upper_spigot,
    )
    ctx.expect_overlap(
        motor_body,
        upper_wand,
        axes="xy",
        min_overlap=0.030,
        elem_a=body_socket,
        elem_b=upper_spigot,
    )

    ctx.expect_contact(upper_wand, lower_wand, elem_a=upper_left_fork, elem_b=lower_barrel)
    ctx.expect_contact(upper_wand, lower_wand, elem_a=upper_right_fork, elem_b=lower_barrel)
    ctx.expect_overlap(
        upper_wand,
        lower_wand,
        axes="xz",
        min_overlap=0.020,
        elem_a=upper_left_fork,
        elem_b=lower_barrel,
    )

    ctx.expect_contact(lower_wand, floor_head, elem_a=lower_left_fork, elem_b=head_barrel)
    ctx.expect_contact(lower_wand, floor_head, elem_a=lower_right_fork, elem_b=head_barrel)
    ctx.expect_overlap(
        lower_wand,
        floor_head,
        axes="xz",
        min_overlap=0.015,
        elem_a=lower_left_fork,
        elem_b=head_barrel,
    )

    ctx.expect_origin_gap(motor_body, floor_head, axis="z", min_gap=0.66, max_gap=0.70)
    ctx.expect_origin_distance(motor_body, floor_head, axes="xy", max_dist=0.01)

    rest_head_pos = ctx.part_world_position(floor_head)
    rest_bumper_aabb = ctx.part_element_world_aabb(floor_head, elem=front_bumper)
    assert rest_head_pos is not None
    assert rest_bumper_aabb is not None

    with ctx.pose({wand_fold: -1.0}):
        folded_head_pos = ctx.part_world_position(floor_head)
        assert folded_head_pos is not None
        ctx.check(
            "wand_fold_moves_floor_head_forward",
            folded_head_pos[0] > rest_head_pos[0] + 0.22,
            details=f"expected folded head x > {rest_head_pos[0] + 0.22:.3f}, got {folded_head_pos[0]:.3f}",
        )
        ctx.check(
            "wand_fold_lifts_floor_head",
            folded_head_pos[2] > rest_head_pos[2] + 0.12,
            details=f"expected folded head z > {rest_head_pos[2] + 0.12:.3f}, got {folded_head_pos[2]:.3f}",
        )
        ctx.expect_contact(upper_wand, lower_wand, elem_a=upper_left_fork, elem_b=lower_barrel)
        ctx.expect_contact(upper_wand, lower_wand, elem_a=upper_right_fork, elem_b=lower_barrel)

    with ctx.pose({head_pitch: 0.55}):
        pitched_bumper_aabb = ctx.part_element_world_aabb(floor_head, elem=front_bumper)
        assert pitched_bumper_aabb is not None
        ctx.check(
            "head_pitch_drops_floor_head_nose",
            pitched_bumper_aabb[0][2] < rest_bumper_aabb[0][2] - 0.015,
            details=(
                f"expected pitched bumper min z < {rest_bumper_aabb[0][2] - 0.015:.3f}, "
                f"got {pitched_bumper_aabb[0][2]:.3f}"
            ),
        )
        ctx.expect_contact(lower_wand, floor_head, elem_a=lower_left_fork, elem_b=head_barrel)
        ctx.expect_contact(lower_wand, floor_head, elem_a=lower_right_fork, elem_b=head_barrel)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
