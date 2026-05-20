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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _cylindrical_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    lip: float = 0.0,
    segments: int = 56,
):
    outer = [
        (outer_radius * 0.98, 0.0),
        (outer_radius + lip, min(height * 0.08, 0.02)),
        (outer_radius + lip, max(height - min(height * 0.08, 0.02), 0.0)),
        (outer_radius * 0.98, height),
    ]
    inner = [
        (inner_radius, min(height * 0.04, 0.01)),
        (inner_radius, max(height - min(height * 0.04, 0.01), 0.0)),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _ring_shell_mesh(name: str, *, inner_radius: float, outer_radius: float, depth: float):
    outer = [
        (outer_radius - 0.014, -depth * 0.50),
        (outer_radius - 0.004, -depth * 0.24),
        (outer_radius, 0.0),
        (outer_radius - 0.004, depth * 0.24),
        (outer_radius - 0.014, depth * 0.50),
    ]
    inner = [
        (inner_radius + 0.010, -depth * 0.38),
        (inner_radius + 0.002, -depth * 0.12),
        (inner_radius, 0.0),
        (inner_radius + 0.002, depth * 0.12),
        (inner_radius + 0.010, depth * 0.38),
    ]
    geom = LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    geom.rotate_y(math.pi / 2.0)
    return _save_mesh(name, geom)


def _approx_axis(actual: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(a - e) <= tol for a, e in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_light_boom_stand")

    stand_black = model.material("stand_black", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    ring_white = model.material("ring_white", rgba=(0.92, 0.93, 0.95, 1.0))
    diffuser_dark = model.material("diffuser_dark", rgba=(0.80, 0.82, 0.84, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    base_chassis = model.part("base_chassis")
    base_chassis.visual(
        Cylinder(radius=0.070, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=dark_metal,
        name="tripod_hub",
    )
    base_chassis.visual(
        Cylinder(radius=0.050, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        material=stand_black,
        name="receiver_collar",
    )
    base_chassis.visual(
        _cylindrical_shell_mesh(
            "stand_outer_sleeve",
            outer_radius=0.036,
            inner_radius=0.031,
            height=0.660,
            lip=0.0015,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        material=stand_black,
        name="outer_sleeve",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        base_chassis.visual(
            Box((0.034, 0.034, 0.028)),
            origin=Origin(
                xyz=(0.086 * math.cos(angle), 0.086 * math.sin(angle), 0.340),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"leg_mount_{index}",
        )
    base_chassis.inertial = Inertial.from_geometry(
        Box((1.25, 1.25, 1.18)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
    )

    leg_angles = {
        "front_leg": 0.0,
        "left_leg": 2.0 * math.pi / 3.0,
        "right_leg": 4.0 * math.pi / 3.0,
    }
    leg_mesh = _save_mesh(
        "tripod_leg_tube",
        tube_from_spline_points(
            [
                (0.030, 0.0, 0.000),
                (0.130, 0.0, -0.050),
                (0.330, 0.0, -0.165),
                (0.610, 0.0, -0.315),
            ],
            radius=0.012,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    for leg_name in leg_angles:
        leg = model.part(leg_name)
        leg.visual(
            Box((0.056, 0.022, 0.024)),
            origin=Origin(xyz=(0.030, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_block",
        )
        leg.visual(
            Cylinder(radius=0.012, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stand_black,
            name="hinge_barrel",
        )
        leg.visual(leg_mesh, material=stand_black, name="leg_tube")
        leg.visual(
            Cylinder(radius=0.018, length=0.060),
            origin=Origin(xyz=(0.620, 0.0, -0.315), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stand_black,
            name="foot_socket",
        )
        leg.visual(
            Box((0.055, 0.028, 0.016)),
            origin=Origin(xyz=(0.635, 0.0, -0.332)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.700, 0.080, 0.380)),
            mass=0.55,
            origin=Origin(xyz=(0.330, 0.0, -0.165)),
        )

    center_mast = model.part("center_mast")
    center_mast.visual(
        Cylinder(radius=0.025, length=1.120),
        material=satin_metal,
        name="inner_mast",
    )
    center_mast.visual(
        Cylinder(radius=0.036, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=stand_black,
        name="mast_stop_collar",
    )
    center_mast.visual(
        Cylinder(radius=0.034, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=dark_metal,
        name="mast_top_cap",
    )
    center_mast.visual(
        Cylinder(radius=0.038, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        material=dark_metal,
        name="yaw_turntable",
    )
    center_mast.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=1.280),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    boom_arm = model.part("boom_arm")
    boom_arm.visual(
        _cylindrical_shell_mesh(
            "boom_yaw_collar",
            outer_radius=0.045,
            inner_radius=0.038,
            height=0.072,
            lip=0.001,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        material=stand_black,
        name="yaw_collar",
    )
    boom_arm.visual(
        Cylinder(radius=0.040, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=dark_metal,
        name="thrust_washer",
    )
    boom_arm.visual(
        Box((0.060, 0.044, 0.020)),
        origin=Origin(xyz=(0.070, 0.0, -0.056)),
        material=dark_metal,
        name="yaw_neck",
    )
    boom_arm.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.090, 0.037, -0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="yaw_plate",
    )
    boom_arm.visual(
        Box((0.110, 0.060, 0.040)),
        origin=Origin(xyz=(0.115, 0.0, -0.040)),
        material=dark_metal,
        name="boom_clamp_block",
    )
    boom_arm.visual(
        Cylinder(radius=0.018, length=0.730),
        origin=Origin(xyz=(0.400, 0.0, -0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_black,
        name="front_boom_tube",
    )
    boom_arm.visual(
        Cylinder(radius=0.014, length=0.260),
        origin=Origin(xyz=(-0.175, 0.0, -0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_black,
        name="counterweight_stub",
    )
    boom_arm.visual(
        Cylinder(radius=0.050, length=0.090),
        origin=Origin(xyz=(-0.350, 0.0, -0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight",
    )
    boom_arm.inertial = Inertial.from_geometry(
        Box((1.160, 0.120, 0.120)),
        mass=1.9,
        origin=Origin(xyz=(0.180, 0.0, 0.020)),
    )

    boom_yoke = model.part("boom_yoke")
    boom_yoke.visual(
        Box((0.022, 0.206, 0.050)),
        origin=Origin(xyz=(0.011, 0.0, 0.020)),
        material=dark_metal,
        name="yoke_body",
    )
    boom_yoke.visual(
        Box((0.090, 0.018, 0.140)),
        origin=Origin(xyz=(0.056, 0.104, 0.0)),
        material=stand_black,
        name="left_arm",
    )
    boom_yoke.visual(
        Box((0.090, 0.018, 0.140)),
        origin=Origin(xyz=(0.056, -0.104, 0.0)),
        material=stand_black,
        name="right_arm",
    )
    boom_yoke.inertial = Inertial.from_geometry(
        Box((0.130, 0.220, 0.160)),
        mass=0.35,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        _ring_shell_mesh(
            "ring_light_shell",
            inner_radius=0.180,
            outer_radius=0.238,
            depth=0.056,
        ),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=ring_white,
        name="ring_shell",
    )
    ring_head.visual(
        Cylinder(radius=0.014, length=0.168),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_axle",
    )
    ring_head.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.087, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    ring_head.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, -0.087, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    ring_head.visual(
        Box((0.030, 0.060, 0.240)),
        origin=Origin(xyz=(0.000, 0.0, -0.120)),
        material=dark_metal,
        name="lower_bracket",
    )
    ring_head.visual(
        Box((0.050, 0.082, 0.070)),
        origin=Origin(xyz=(-0.016, 0.0, -0.275)),
        material=diffuser_dark,
        name="control_box",
    )
    ring_head.inertial = Inertial.from_geometry(
        Box((0.090, 0.480, 0.620)),
        mass=1.2,
        origin=Origin(xyz=(0.010, 0.0, -0.030)),
    )

    mast_slide = model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base_chassis,
        child=center_mast,
        origin=Origin(xyz=(0.0, 0.0, 1.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.18, lower=0.0, upper=0.350),
    )
    model.articulation(
        "boom_yaw",
        ArticulationType.CONTINUOUS,
        parent=center_mast,
        child=boom_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.2),
    )
    model.articulation(
        "boom_to_yoke",
        ArticulationType.FIXED,
        parent=boom_arm,
        child=boom_yoke,
        origin=Origin(xyz=(0.730, 0.0, -0.018)),
    )
    model.articulation(
        "ring_tilt",
        ArticulationType.REVOLUTE,
        parent=boom_yoke,
        child=ring_head,
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=-1.10,
            upper=0.75,
        ),
    )

    for leg_name, angle in leg_angles.items():
        model.articulation(
            f"{leg_name}_swing",
            ArticulationType.REVOLUTE,
            parent=base_chassis,
            child=leg_name,
            origin=Origin(
                xyz=(0.115 * math.cos(angle), 0.115 * math.sin(angle), 0.340),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.0,
                lower=-0.25,
                upper=0.45,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    base_chassis = object_model.get_part("base_chassis")
    center_mast = object_model.get_part("center_mast")
    boom_arm = object_model.get_part("boom_arm")
    boom_yoke = object_model.get_part("boom_yoke")
    ring_head = object_model.get_part("ring_head")
    front_leg = object_model.get_part("front_leg")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")

    mast_slide = object_model.get_articulation("mast_slide")
    boom_yaw = object_model.get_articulation("boom_yaw")
    ring_tilt = object_model.get_articulation("ring_tilt")
    front_leg_swing = object_model.get_articulation("front_leg_swing")
    left_leg_swing = object_model.get_articulation("left_leg_swing")
    right_leg_swing = object_model.get_articulation("right_leg_swing")

    ctx.check(
        "mast_slide_axis_is_vertical",
        _approx_axis(mast_slide.axis, (0.0, 0.0, 1.0)),
        f"axis={mast_slide.axis}",
    )
    ctx.check(
        "boom_yaw_axis_is_vertical",
        _approx_axis(boom_yaw.axis, (0.0, 0.0, 1.0)),
        f"axis={boom_yaw.axis}",
    )
    ctx.check(
        "ring_tilt_axis_is_horizontal",
        _approx_axis(ring_tilt.axis, (0.0, 1.0, 0.0)),
        f"axis={ring_tilt.axis}",
    )
    for joint in (front_leg_swing, left_leg_swing, right_leg_swing):
        ctx.check(
            f"{joint.name}_axis_is_hinge_like",
            _approx_axis(joint.axis, (0.0, 1.0, 0.0)),
            f"axis={joint.axis}",
        )

    ctx.expect_contact(center_mast, base_chassis, contact_tol=0.0025)
    ctx.expect_contact(boom_arm, center_mast, contact_tol=0.0025)
    ctx.expect_contact(boom_yoke, boom_arm, contact_tol=0.0025)
    ctx.expect_contact(ring_head, boom_yoke, contact_tol=0.0025)
    ctx.expect_contact(front_leg, base_chassis, contact_tol=0.0025)
    ctx.expect_contact(left_leg, base_chassis, contact_tol=0.0025)
    ctx.expect_contact(right_leg, base_chassis, contact_tol=0.0025)
    ctx.expect_origin_distance(ring_head, center_mast, axes="xy", min_dist=0.75, max_dist=0.90)

    mast_rest = ctx.part_world_position(center_mast)
    ring_rest = ctx.part_world_position(ring_head)
    assert mast_rest is not None
    assert ring_rest is not None

    with ctx.pose({mast_slide: 0.300}):
        mast_raised = ctx.part_world_position(center_mast)
        assert mast_raised is not None
        ctx.check(
            "mast_slide_raises_center_mast",
            mast_raised[2] > mast_rest[2] + 0.290,
            f"rest={mast_rest}, raised={mast_raised}",
        )
        ctx.expect_contact(boom_arm, center_mast, contact_tol=0.0025)

    with ctx.pose({boom_yaw: math.pi / 2.0}):
        ring_side = ctx.part_world_position(ring_head)
        assert ring_side is not None
        ctx.check(
            "boom_yaw_swings_ring_around_stand",
            ring_side[1] > ring_rest[1] + 0.70 and abs(ring_side[0]) < 0.25,
            f"rest={ring_rest}, yawed={ring_side}",
        )
        ctx.expect_contact(boom_arm, center_mast, contact_tol=0.0025)

    ring_rest_aabb = ctx.part_world_aabb(ring_head)
    assert ring_rest_aabb is not None
    ring_rest_x = ring_rest_aabb[1][0] - ring_rest_aabb[0][0]
    ring_rest_z = ring_rest_aabb[1][2] - ring_rest_aabb[0][2]
    with ctx.pose({ring_tilt: 0.75}):
        ring_tilted_aabb = ctx.part_world_aabb(ring_head)
        assert ring_tilted_aabb is not None
        ring_tilted_x = ring_tilted_aabb[1][0] - ring_tilted_aabb[0][0]
        ring_tilted_z = ring_tilted_aabb[1][2] - ring_tilted_aabb[0][2]
        ctx.check(
            "ring_tilt_changes_ring_pose",
            ring_tilted_x > ring_rest_x + 0.12 and ring_tilted_z < ring_rest_z - 0.08,
            f"rest=({ring_rest_x:.3f}, {ring_rest_z:.3f}), tilted=({ring_tilted_x:.3f}, {ring_tilted_z:.3f})",
        )
        ctx.expect_contact(ring_head, boom_yoke, contact_tol=0.0025)

    front_leg_rest = ctx.part_world_aabb(front_leg)
    assert front_leg_rest is not None
    with ctx.pose({front_leg_swing: 0.300}):
        front_leg_folded = ctx.part_world_aabb(front_leg)
        assert front_leg_folded is not None
        delta_x = abs(front_leg_folded[1][0] - front_leg_rest[1][0])
        delta_z = abs(front_leg_folded[0][2] - front_leg_rest[0][2])
        ctx.check(
            "tripod_leg_hinge_moves_leg",
            delta_x > 0.040 or delta_z > 0.040,
            f"delta_x={delta_x:.3f}, delta_z={delta_z:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
