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
    DomeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _annulus_mesh(name: str, *, outer_diameter: float, inner_diameter: float, thickness: float):
    outer = superellipse_profile(outer_diameter, outer_diameter, exponent=2.0, segments=64)
    inner = superellipse_profile(inner_diameter, inner_diameter, exponent=2.0, segments=64)
    geom = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        thickness,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _door_glass_mesh(name: str, *, radius: float, depth_scale: float):
    geom = DomeGeometry(
        radius=radius,
        radial_segments=56,
        height_segments=18,
        closed=True,
    )
    geom.scale(1.0, 1.0, depth_scale)
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _aabb_center(aabb):
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washer_with_steam_drawer")

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.96, 0.97, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.62, 0.65, 0.68, 1.0))
    drawer_blue = model.material("drawer_blue", rgba=(0.70, 0.86, 0.95, 0.72))
    control_glass = model.material("control_glass", rgba=(0.18, 0.23, 0.28, 0.55))
    glass_tint = model.material("glass_tint", rgba=(0.62, 0.76, 0.86, 0.28))
    gasket_black = model.material("gasket_black", rgba=(0.09, 0.09, 0.10, 1.0))

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((0.60, 0.64, 0.85)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
    )

    cabinet.visual(
        Box((0.60, 0.64, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cabinet_white,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.022, 0.615, 0.770)),
        origin=Origin(xyz=(-0.289, 0.0, 0.440)),
        material=cabinet_white,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((0.022, 0.615, 0.770)),
        origin=Origin(xyz=(0.289, 0.0, 0.440)),
        material=cabinet_white,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((0.556, 0.010, 0.770)),
        origin=Origin(xyz=(0.0, 0.315, 0.440)),
        material=cabinet_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((0.600, 0.640, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.8375)),
        material=cabinet_white,
        name="top_cover",
    )
    cabinet.visual(
        Box((0.556, 0.600, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=cabinet_white,
        name="inner_base_pan",
    )
    cabinet.visual(
        Box((0.090, 0.035, 0.520)),
        origin=Origin(xyz=(-0.255, -0.3025, 0.420)),
        material=cabinet_white,
        name="left_front_pillar",
    )
    cabinet.visual(
        Box((0.080, 0.028, 0.060)),
        origin=Origin(xyz=(0.260, -0.300, 0.190)),
        material=cabinet_white,
        name="right_lower_jamb",
    )
    cabinet.visual(
        Box((0.080, 0.028, 0.240)),
        origin=Origin(xyz=(0.260, -0.300, 0.410)),
        material=cabinet_white,
        name="right_mid_jamb",
    )
    cabinet.visual(
        Box((0.080, 0.028, 0.080)),
        origin=Origin(xyz=(0.260, -0.300, 0.635)),
        material=cabinet_white,
        name="right_upper_jamb",
    )
    cabinet.visual(
        Box((0.600, 0.035, 0.110)),
        origin=Origin(xyz=(0.0, -0.3025, 0.105)),
        material=cabinet_white,
        name="bottom_front_rail",
    )
    cabinet.visual(
        Box((0.297, 0.045, 0.040)),
        origin=Origin(xyz=(-0.1555, -0.2975, 0.805)),
        material=cabinet_white,
        name="drawer_top_strip",
    )
    cabinet.visual(
        Box((0.297, 0.045, 0.024)),
        origin=Origin(xyz=(-0.1555, -0.2975, 0.675)),
        material=cabinet_white,
        name="drawer_bottom_strip",
    )
    cabinet.visual(
        Box((0.052, 0.045, 0.130)),
        origin=Origin(xyz=(-0.278, -0.2975, 0.740)),
        material=cabinet_white,
        name="drawer_left_jamb",
    )
    cabinet.visual(
        Box((0.030, 0.045, 0.130)),
        origin=Origin(xyz=(-0.022, -0.2975, 0.740)),
        material=cabinet_white,
        name="drawer_right_jamb",
    )
    cabinet.visual(
        Box((0.307, 0.045, 0.130)),
        origin=Origin(xyz=(0.1465, -0.2975, 0.740)),
        material=cabinet_white,
        name="control_panel_block",
    )
    cabinet.visual(
        Box((0.220, 0.220, 0.012)),
        origin=Origin(xyz=(-0.145, -0.165, 0.681)),
        material=cabinet_white,
        name="drawer_shelf",
    )
    cabinet.visual(
        Box((0.160, 0.095, 0.160)),
        origin=Origin(xyz=(0.0, 0.2625, 0.410)),
        material=cabinet_white,
        name="rear_bearing_block",
    )

    cabinet.visual(
        _annulus_mesh(
            "washer_front_bezel",
            outer_diameter=0.500,
            inner_diameter=0.360,
            thickness=0.018,
        ),
        origin=Origin(xyz=(0.0, -0.293, 0.410)),
        material=cabinet_white,
        name="door_bezel",
    )
    cabinet.visual(
        _annulus_mesh(
            "washer_boot_ring",
            outer_diameter=0.395,
            inner_diameter=0.308,
            thickness=0.045,
        ),
        origin=Origin(xyz=(0.0, -0.2615, 0.410)),
        material=gasket_black,
        name="door_boot",
    )

    cabinet.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.270, -0.260, 0.570)),
        material=hinge_metal,
        name="upper_hinge_pin",
    )
    cabinet.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.270, -0.260, 0.250)),
        material=hinge_metal,
        name="lower_hinge_pin",
    )

    cabinet.visual(
        Box((0.120, 0.006, 0.050)),
        origin=Origin(xyz=(0.060, -0.320, 0.758)),
        material=control_glass,
        name="display_window",
    )
    cabinet.visual(
        Cylinder(radius=0.044, length=0.026),
        origin=Origin(xyz=(0.195, -0.307, 0.748), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="program_knob",
    )
    cabinet.visual(
        Box((0.080, 0.008, 0.010)),
        origin=Origin(xyz=(0.115, -0.320, 0.698)),
        material=trim_dark,
        name="button_row",
    )

    drum = model.part("drum")
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.215, length=0.340),
        mass=18.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    drum.visual(
        Cylinder(radius=0.205, length=0.340),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.190, length=0.020),
        origin=Origin(xyz=(0.0, 0.160, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_plate",
    )
    drum.visual(
        Cylinder(radius=0.195, length=0.025),
        origin=Origin(xyz=(0.0, -0.1575, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_lip",
    )
    drum.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.170, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="axle_stub",
    )
    paddle_radius = 0.152
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.050, 0.250, 0.018)),
            origin=Origin(
                xyz=(paddle_radius * math.sin(angle), 0.0, paddle_radius * math.cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=steel,
            name=f"paddle_{index}",
        )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((0.500, 0.100, 0.500)),
        mass=5.4,
        origin=Origin(xyz=(-0.238, -0.085, 0.0)),
    )
    door.visual(
        _annulus_mesh(
            "washer_door_outer_ring",
            outer_diameter=0.490,
            inner_diameter=0.340,
            thickness=0.055,
        ),
        origin=Origin(xyz=(-0.238, -0.087, 0.0)),
        material=cabinet_white,
        name="door_outer_ring",
    )
    door.visual(
        _annulus_mesh(
            "washer_door_inner_trim",
            outer_diameter=0.340,
            inner_diameter=0.300,
            thickness=0.018,
        ),
        origin=Origin(xyz=(-0.238, -0.0685, 0.0)),
        material=trim_dark,
        name="door_inner_trim",
    )
    door.visual(
        _door_glass_mesh("washer_door_glass", radius=0.150, depth_scale=0.05),
        origin=Origin(xyz=(-0.238, -0.067, 0.0)),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Box((0.024, 0.0485, 0.034)),
        origin=Origin(xyz=(-0.005, -0.02925, 0.160)),
        material=hinge_metal,
        name="upper_hinge_link",
    )
    door.visual(
        Box((0.048, 0.024, 0.034)),
        origin=Origin(xyz=(-0.041, -0.055, 0.160)),
        material=hinge_metal,
        name="upper_hinge_bridge",
    )
    door.visual(
        Box((0.024, 0.0485, 0.034)),
        origin=Origin(xyz=(-0.005, -0.02925, -0.160)),
        material=hinge_metal,
        name="lower_hinge_link",
    )
    door.visual(
        Box((0.048, 0.024, 0.034)),
        origin=Origin(xyz=(-0.041, -0.055, -0.160)),
        material=hinge_metal,
        name="lower_hinge_bridge",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=hinge_metal,
        name="upper_hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.160)),
        material=hinge_metal,
        name="lower_hinge_barrel",
    )
    door.visual(
        Box((0.110, 0.026, 0.030)),
        origin=Origin(xyz=(-0.070, -0.100, 0.020)),
        material=cabinet_white,
        name="door_handle",
    )

    drawer = model.part("steam_drawer")
    drawer.inertial = Inertial.from_geometry(
        Box((0.206, 0.220, 0.074)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
    )
    drawer.visual(
        Box((0.206, 0.220, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cabinet_white,
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.210, 0.018, 0.074)),
        origin=Origin(xyz=(0.0, -0.119, 0.037)),
        material=cabinet_white,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.110, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.132, 0.037)),
        material=trim_dark,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.160, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, 0.015, 0.030)),
        material=drawer_blue,
        name="drawer_insert",
    )
    drawer.visual(
        Box((0.018, 0.180, 0.012)),
        origin=Origin(xyz=(-0.074, 0.0, 0.006)),
        material=hinge_metal,
        name="runner_left",
    )
    drawer.visual(
        Box((0.018, 0.180, 0.012)),
        origin=Origin(xyz=(0.074, 0.0, 0.006)),
        material=hinge_metal,
        name="runner_right",
    )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, 0.020, 0.410)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=8.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.270, -0.260, 0.410)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "cabinet_to_steam_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(-0.145, -0.185, 0.687)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.12,
            lower=0.0,
            upper=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drawer = object_model.get_part("steam_drawer")
    drum_joint = object_model.get_articulation("cabinet_to_drum")
    door_joint = object_model.get_articulation("cabinet_to_door")
    drawer_joint = object_model.get_articulation("cabinet_to_steam_drawer")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
        cabinet,
        door,
        elem_a="upper_hinge_pin",
        elem_b="upper_hinge_barrel",
        reason="Upper barrel hinge pin runs concentrically inside the upper door hinge barrel.",
    )
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="lower_hinge_pin",
        elem_b="lower_hinge_barrel",
        reason="Lower barrel hinge pin runs concentrically inside the lower door hinge barrel.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(drum, cabinet, elem_a="axle_stub", elem_b="rear_bearing_block")
    ctx.expect_contact(door, cabinet, elem_a="upper_hinge_barrel", elem_b="upper_hinge_pin")
    ctx.expect_contact(door, cabinet, elem_a="lower_hinge_barrel", elem_b="lower_hinge_pin")
    ctx.expect_contact(drawer, cabinet, elem_a="runner_left", elem_b="drawer_shelf")
    ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.34, elem_a="door_outer_ring", elem_b="door_bezel")

    ctx.check(
        "drum_joint_axis",
        tuple(drum_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected drum axis (0, 1, 0), got {drum_joint.axis}.",
    )
    ctx.check(
        "door_joint_axis",
        tuple(door_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected door hinge axis (0, 0, 1), got {door_joint.axis}.",
    )
    ctx.check(
        "drawer_joint_axis",
        tuple(drawer_joint.axis) == (0.0, -1.0, 0.0),
        f"Expected drawer slide axis (0, -1, 0), got {drawer_joint.axis}.",
    )

    door_rest_aabb = ctx.part_element_world_aabb(door, elem="door_outer_ring")
    if door_rest_aabb is None:
        ctx.fail("door_outer_ring_aabb", "Could not resolve closed door outer ring AABB.")
    else:
        door_rest_center = _aabb_center(door_rest_aabb)
        with ctx.pose({door_joint: 1.25}):
            door_open_aabb = ctx.part_element_world_aabb(door, elem="door_outer_ring")
            if door_open_aabb is None:
                ctx.fail("door_open_outer_ring_aabb", "Could not resolve open door outer ring AABB.")
            else:
                door_open_center = _aabb_center(door_open_aabb)
                ctx.check(
                    "door_swings_outward",
                    door_open_center[0] > door_rest_center[0] + 0.14,
                    (
                        f"Expected open door center x to increase by > 0.14 m; "
                        f"closed={door_rest_center[0]:.3f}, open={door_open_center[0]:.3f}."
                    ),
                )
                ctx.expect_contact(door, cabinet, elem_a="upper_hinge_barrel", elem_b="upper_hinge_pin")

    drawer_rest = ctx.part_world_position(drawer)
    if drawer_rest is None:
        ctx.fail("drawer_rest_position", "Could not resolve steam drawer rest position.")
    else:
        with ctx.pose({drawer_joint: 0.10}):
            drawer_open = ctx.part_world_position(drawer)
            if drawer_open is None:
                ctx.fail("drawer_open_position", "Could not resolve steam drawer open position.")
            else:
                ctx.check(
                    "drawer_slides_forward",
                    drawer_open[1] < drawer_rest[1] - 0.08,
                    (
                        f"Expected drawer origin to move at least 0.08 m forward; "
                        f"closed y={drawer_rest[1]:.3f}, open y={drawer_open[1]:.3f}."
                    ),
                )
                ctx.expect_contact(drawer, cabinet, elem_a="runner_right", elem_b="drawer_shelf")

    paddle_rest_aabb = ctx.part_element_world_aabb(drum, elem="paddle_0")
    if paddle_rest_aabb is None:
        ctx.fail("paddle_rest_aabb", "Could not resolve drum paddle AABB.")
    else:
        paddle_rest_center = _aabb_center(paddle_rest_aabb)
        with ctx.pose({drum_joint: 1.10}):
            paddle_spin_aabb = ctx.part_element_world_aabb(drum, elem="paddle_0")
            if paddle_spin_aabb is None:
                ctx.fail("paddle_spin_aabb", "Could not resolve rotated drum paddle AABB.")
            else:
                paddle_spin_center = _aabb_center(paddle_spin_aabb)
                ctx.check(
                    "drum_rotates_paddle",
                    abs(paddle_spin_center[0] - paddle_rest_center[0]) > 0.08
                    or abs(paddle_spin_center[2] - paddle_rest_center[2]) > 0.08,
                    (
                        "Expected the named drum paddle to move appreciably under drum rotation; "
                        f"rest={paddle_rest_center}, rotated={paddle_spin_center}."
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
