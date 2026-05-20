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
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

GRILL_BOWL_RIM_OUTER = 0.233
GRILL_BOWL_RIM_INNER = 0.219
GRILL_LID_RIM_INNER = 0.233
GRILL_LID_RIM_OUTER = 0.245
RIM_HEIGHT = 0.55
RING_RADIUS = 0.217
RING_Z = 0.43
FRAME_TUBE_RADIUS = 0.009
WHEEL_RADIUS = 0.09
WHEEL_WIDTH = 0.044
WHEEL_TRACK_HALF = 0.19
AXLE_Y = -0.16
AXLE_Z = WHEEL_RADIUS


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_points(radius: float, z: float, *, count: int = 16) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(count):
        angle = (2.0 * math.pi * index) / count
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return points


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kettle_charcoal_grill", assets=ASSETS)

    enamel_black = model.material("enamel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    frame_black = model.material("frame_black", rgba=(0.18, 0.18, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.66, 0.69, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    handle_black = model.material("handle_black", rgba=(0.14, 0.14, 0.14, 1.0))

    stand = model.part("stand")
    stand.inertial = Inertial.from_geometry(
        Box((0.56, 0.46, 0.68)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
    )

    stand.visual(
        _save_mesh(
            "grill_top_ring.obj",
            tube_from_spline_points(
                _circle_points(RING_RADIUS, RING_Z, count=18),
                radius=FRAME_TUBE_RADIUS,
                samples_per_segment=8,
                closed_spline=True,
                cap_ends=False,
                radial_segments=16,
            ),
        ),
        material=frame_black,
        name="top_ring",
    )

    rear_leg_points = [
        (0.145, -0.145, RING_Z),
        (0.152, -0.145, 0.27),
        (0.155, AXLE_Y, AXLE_Z),
    ]
    stand.visual(
        _save_mesh(
            "grill_rear_right_leg.obj",
            tube_from_spline_points(
                rear_leg_points,
                radius=FRAME_TUBE_RADIUS,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_black,
        name="rear_right_leg",
    )
    stand.visual(
        _save_mesh(
            "grill_rear_left_leg.obj",
            tube_from_spline_points(
                _mirror_x(rear_leg_points),
                radius=FRAME_TUBE_RADIUS,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_black,
        name="rear_left_leg",
    )

    stand.visual(
        _save_mesh(
            "grill_front_leg.obj",
            tube_from_spline_points(
                [
                    (0.0, RING_RADIUS, RING_Z),
                    (0.0, 0.202, 0.24),
                    (0.0, 0.205, 0.027),
                ],
                radius=FRAME_TUBE_RADIUS,
                samples_per_segment=14,
                radial_segments=16,
            ),
        ),
        material=frame_black,
        name="front_leg",
    )

    brace_points = [
        (0.155, AXLE_Y, AXLE_Z),
        (0.110, -0.085, 0.17),
        (0.0, 0.165, 0.17),
    ]
    stand.visual(
        _save_mesh(
            "grill_right_lower_brace.obj",
            tube_from_spline_points(
                brace_points,
                radius=0.008,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_black,
        name="right_lower_brace",
    )
    stand.visual(
        _save_mesh(
            "grill_left_lower_brace.obj",
            tube_from_spline_points(
                _mirror_x(brace_points),
                radius=0.008,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_black,
        name="left_lower_brace",
    )

    stand.visual(
        Box((0.060, 0.030, 0.029)),
        origin=Origin(xyz=(0.0, 0.215, 0.0145)),
        material=frame_black,
        name="front_foot_pad",
    )
    stand.visual(
        Cylinder(radius=0.009, length=0.420),
        origin=Origin(xyz=(0.0, AXLE_Y, AXLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_rod",
    )

    bowl = model.part("bowl")
    bowl.inertial = Inertial.from_geometry(
        Box((0.48, 0.48, 0.24)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
    )
    bowl.visual(
        _save_mesh(
            "grill_bowl_shell.obj",
            LatheGeometry.from_shell_profiles(
                [
                    (GRILL_BOWL_RIM_OUTER, 0.0),
                    (0.228, -0.016),
                    (0.210, -0.060),
                    (0.175, -0.120),
                    (0.120, -0.180),
                    (0.050, -0.215),
                    (0.0, -0.225),
                ],
                [
                    (GRILL_BOWL_RIM_INNER, 0.0),
                    (0.214, -0.014),
                    (0.197, -0.055),
                    (0.163, -0.110),
                    (0.112, -0.165),
                    (0.047, -0.198),
                    (0.0, -0.208),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=enamel_black,
        name="bowl_shell",
    )
    bowl.visual(
        Box((0.070, 0.030, 0.012)),
        origin=Origin(xyz=(-0.205, 0.060, -0.105)),
        material=steel,
        name="mount_left_ear",
    )
    bowl.visual(
        Box((0.070, 0.030, 0.012)),
        origin=Origin(xyz=(0.205, 0.060, -0.105)),
        material=steel,
        name="mount_right_ear",
    )
    bowl.visual(
        Box((0.030, 0.070, 0.012)),
        origin=Origin(xyz=(0.0, -0.205, -0.105)),
        material=steel,
        name="mount_rear_ear",
    )
    bowl.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.0, 0.185, -0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="ash_vent_housing",
    )
    bowl.visual(
        Box((0.030, 0.020, 0.022)),
        origin=Origin(xyz=(-0.060, -0.227, -0.014)),
        material=steel,
        name="hinge_left_bracket",
    )
    bowl.visual(
        Box((0.030, 0.020, 0.022)),
        origin=Origin(xyz=(0.060, -0.227, -0.014)),
        material=steel,
        name="hinge_right_bracket",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.50, 0.50, 0.20)),
        mass=3.5,
        origin=Origin(xyz=(0.0, GRILL_LID_RIM_OUTER, 0.10)),
    )
    lid.visual(
        _save_mesh(
            "grill_lid_shell.obj",
            LatheGeometry.from_shell_profiles(
                [
                    (GRILL_LID_RIM_OUTER, 0.0),
                    (0.242, 0.020),
                    (0.220, 0.070),
                    (0.176, 0.125),
                    (0.105, 0.166),
                    (0.032, 0.184),
                    (0.0, 0.188),
                ],
                [
                    (GRILL_LID_RIM_INNER, 0.0),
                    (0.230, 0.018),
                    (0.210, 0.065),
                    (0.168, 0.116),
                    (0.100, 0.151),
                    (0.028, 0.166),
                    (0.0, 0.171),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        origin=Origin(xyz=(0.0, GRILL_LID_RIM_OUTER, 0.0)),
        material=enamel_black,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.0, 0.000, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(xyz=(-0.055, 0.350, 0.128)),
        material=steel,
        name="lid_handle_left_post",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(xyz=(0.055, 0.350, 0.128)),
        material=steel,
        name="lid_handle_right_post",
    )
    lid.visual(
        Cylinder(radius=0.013, length=0.150),
        origin=Origin(xyz=(0.0, 0.350, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="lid_handle_grip",
    )
    lid.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.0, GRILL_LID_RIM_OUTER, 0.189)),
        material=steel,
        name="top_vent_plate",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, GRILL_LID_RIM_OUTER, 0.199)),
        material=handle_black,
        name="top_vent_knob",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    left_wheel.visual(
        _save_mesh(
            "grill_wheel_tire.obj",
            LatheGeometry(
                [
                    (0.060, -0.018),
                    (0.074, -0.022),
                    (0.086, -0.017),
                    (0.090, -0.006),
                    (0.090, 0.006),
                    (0.086, 0.017),
                    (0.074, 0.022),
                    (0.060, 0.018),
                    (0.054, 0.008),
                    (0.052, 0.0),
                    (0.054, -0.008),
                ],
                segments=56,
            ).rotate_y(math.pi / 2.0),
        ),
        material=dark_rubber,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    left_wheel.visual(
        Box((0.008, 0.012, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=steel,
        name="spoke_top",
    )
    left_wheel.visual(
        Box((0.008, 0.012, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=steel,
        name="spoke_bottom",
    )
    left_wheel.visual(
        Box((0.008, 0.042, 0.012)),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=steel,
        name="spoke_front",
    )
    left_wheel.visual(
        Box((0.008, 0.042, 0.012)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=steel,
        name="spoke_back",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    right_wheel.visual(
        _save_mesh(
            "grill_wheel_tire.obj",
            LatheGeometry(
                [
                    (0.060, -0.018),
                    (0.074, -0.022),
                    (0.086, -0.017),
                    (0.090, -0.006),
                    (0.090, 0.006),
                    (0.086, 0.017),
                    (0.074, 0.022),
                    (0.060, 0.018),
                    (0.054, 0.008),
                    (0.052, 0.0),
                    (0.054, -0.008),
                ],
                segments=56,
            ).rotate_y(math.pi / 2.0),
        ),
        material=dark_rubber,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    right_wheel.visual(
        Box((0.008, 0.012, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=steel,
        name="spoke_top",
    )
    right_wheel.visual(
        Box((0.008, 0.012, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=steel,
        name="spoke_bottom",
    )
    right_wheel.visual(
        Box((0.008, 0.042, 0.012)),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=steel,
        name="spoke_front",
    )
    right_wheel.visual(
        Box((0.008, 0.042, 0.012)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=steel,
        name="spoke_back",
    )

    model.articulation(
        "stand_to_bowl",
        ArticulationType.FIXED,
        parent=stand,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, RIM_HEIGHT)),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, -GRILL_LID_RIM_OUTER, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=left_wheel,
        origin=Origin(xyz=(-WHEEL_TRACK_HALF, AXLE_Y, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_TRACK_HALF, AXLE_Y, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    stand = object_model.get_part("stand")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    lid_hinge = object_model.get_articulation("bowl_to_lid")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

    axle_rod = stand.get_visual("axle_rod")
    top_ring = stand.get_visual("top_ring")
    bowl_shell = bowl.get_visual("bowl_shell")
    mount_left_ear = bowl.get_visual("mount_left_ear")
    mount_right_ear = bowl.get_visual("mount_right_ear")
    mount_rear_ear = bowl.get_visual("mount_rear_ear")
    hinge_left_bracket = bowl.get_visual("hinge_left_bracket")
    hinge_right_bracket = bowl.get_visual("hinge_right_bracket")
    lid_shell = lid.get_visual("lid_shell")
    hinge_barrel = lid.get_visual("hinge_barrel")
    lid_handle_grip = lid.get_visual("lid_handle_grip")
    left_hub = left_wheel.get_visual("hub")
    right_hub = right_wheel.get_visual("hub")

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
    ctx.allow_overlap(
        stand,
        left_wheel,
        elem_a=axle_rod,
        elem_b=left_hub,
        reason="The left wheel hub spins around the shared stand-mounted axle rod.",
    )
    ctx.allow_overlap(
        stand,
        right_wheel,
        elem_a=axle_rod,
        elem_b=right_hub,
        reason="The right wheel hub spins around the shared stand-mounted axle rod.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(bowl, stand, axes="xy", max_dist=0.001)
    ctx.expect_contact(bowl, stand, elem_a=mount_left_ear, elem_b=top_ring)
    ctx.expect_contact(bowl, stand, elem_a=mount_right_ear, elem_b=top_ring)
    ctx.expect_contact(bowl, stand, elem_a=mount_rear_ear, elem_b=top_ring)

    ctx.expect_overlap(lid, bowl, axes="xy", min_overlap=0.45, elem_a=lid_shell, elem_b=bowl_shell)
    ctx.expect_gap(
        lid,
        bowl,
        axis="z",
        max_gap=0.003,
        max_penetration=0.001,
        positive_elem=lid_shell,
        negative_elem=bowl_shell,
        name="lid_seats_on_bowl_rim_plane",
    )

    ctx.expect_contact(left_wheel, stand, elem_a=left_hub, elem_b=axle_rod)
    ctx.expect_contact(right_wheel, stand, elem_a=right_hub, elem_b=axle_rod)
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="yz", max_dist=0.001)
    ctx.expect_origin_gap(
        right_wheel,
        left_wheel,
        axis="x",
        min_gap=0.34,
        max_gap=0.40,
    )

    ctx.check(
        "lid_hinge_axis_is_rear_crossbar",
        tuple(round(v, 3) for v in lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "lid_hinge_opens_about_100_deg",
        lid_hinge.motion_limits is not None
        and abs(lid_hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(lid_hinge.motion_limits.upper - math.radians(100.0)) < 1e-6,
        f"limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "wheel_joints_are_continuous",
        left_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        "Wheel articulations must be continuous.",
    )
    ctx.check(
        "wheel_spin_axes_are_axle_aligned",
        tuple(round(v, 3) for v in left_wheel_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 3) for v in right_wheel_spin.axis) == (1.0, 0.0, 0.0),
        f"left={left_wheel_spin.axis}, right={right_wheel_spin.axis}",
    )

    hinge_barrel_rest = ctx.part_element_world_aabb(lid, elem=hinge_barrel)
    hinge_left_rest = ctx.part_element_world_aabb(bowl, elem=hinge_left_bracket)
    hinge_right_rest = ctx.part_element_world_aabb(bowl, elem=hinge_right_bracket)
    lid_handle_rest = ctx.part_element_world_aabb(lid, elem=lid_handle_grip)
    assert hinge_barrel_rest is not None
    assert hinge_left_rest is not None
    assert hinge_right_rest is not None
    assert lid_handle_rest is not None
    ctx.check(
        "rear_hinge_barrel_sits_between_side_brackets",
        hinge_left_rest[1][0] < hinge_barrel_rest[0][0] and hinge_barrel_rest[1][0] < hinge_right_rest[0][0],
        f"left={hinge_left_rest}, barrel={hinge_barrel_rest}, right={hinge_right_rest}",
    )
    left_wheel_rest = ctx.part_world_position(left_wheel)
    right_wheel_rest = ctx.part_world_position(right_wheel)
    assert left_wheel_rest is not None
    assert right_wheel_rest is not None

    with ctx.pose({lid_hinge: math.radians(100.0)}):
        lid_handle_open = ctx.part_element_world_aabb(lid, elem=lid_handle_grip)
        assert lid_handle_open is not None
        assert lid_handle_open[1][2] > lid_handle_rest[1][2] + 0.06
        assert lid_handle_open[0][1] < lid_handle_rest[0][1] - 0.10
        ctx.expect_gap(
            lid,
            bowl,
            axis="z",
            min_gap=0.16,
            positive_elem=lid_handle_grip,
            negative_elem=bowl_shell,
            name="open_lid_handle_clears_bowl",
        )

    with ctx.pose({left_wheel_spin: math.pi / 2.0, right_wheel_spin: math.pi}):
        left_wheel_spun = ctx.part_world_position(left_wheel)
        right_wheel_spun = ctx.part_world_position(right_wheel)
        assert left_wheel_spun is not None
        assert right_wheel_spun is not None
        assert abs(left_wheel_spun[0] - left_wheel_rest[0]) < 1e-9
        assert abs(left_wheel_spun[1] - left_wheel_rest[1]) < 1e-9
        assert abs(left_wheel_spun[2] - left_wheel_rest[2]) < 1e-9
        assert abs(right_wheel_spun[0] - right_wheel_rest[0]) < 1e-9
        assert abs(right_wheel_spun[1] - right_wheel_rest[1]) < 1e-9
        assert abs(right_wheel_spun[2] - right_wheel_rest[2]) < 1e-9
        ctx.expect_contact(left_wheel, stand, elem_a=left_hub, elem_b=axle_rod)
        ctx.expect_contact(right_wheel, stand, elem_a=right_hub, elem_b=axle_rod)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
