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
)


def _annular_ring_mesh(name: str, *, outer_radius: float, inner_radius: float, depth: float):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (outer_radius, -depth * 0.5),
                (outer_radius, depth * 0.5),
            ],
            [
                (inner_radius, -depth * 0.5),
                (inner_radius, depth * 0.5),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _drum_shell_mesh():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.080, 0.340),
                (0.170, 0.332),
                (0.290, 0.315),
                (0.338, 0.280),
                (0.338, -0.280),
                (0.320, -0.320),
                (0.305, -0.350),
            ],
            [
                (0.000, 0.340),
                (0.220, 0.308),
                (0.296, 0.264),
                (0.296, -0.305),
                (0.278, -0.350),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "commercial_dryer_drum_shell",
    )


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) < 1e-6 for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coin_operated_commercial_dryer")

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    stainless_dark = model.material("stainless_dark", rgba=(0.55, 0.58, 0.61, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.10, 0.11, 0.12, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.34, 0.42, 0.48, 0.35))
    indicator_black = model.material("indicator_black", rgba=(0.08, 0.09, 0.10, 1.0))

    cabinet_width = 0.95
    cabinet_depth = 0.98
    cabinet_height = 1.28
    front_face_y = -cabinet_depth * 0.5
    door_center_z = 0.63
    drum_center_y = -0.05

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.030, cabinet_depth, 1.20)),
        origin=Origin(xyz=(-0.460, 0.000, 0.600)),
        material=stainless,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((0.030, cabinet_depth, 1.20)),
        origin=Origin(xyz=(0.460, 0.000, 0.600)),
        material=stainless,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((0.890, cabinet_depth, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 0.025)),
        material=stainless_dark,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.890, cabinet_depth, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, 1.265)),
        material=stainless,
        name="top_panel",
    )
    cabinet.visual(
        Box((0.890, 0.030, 1.200)),
        origin=Origin(xyz=(0.000, 0.475, 0.600)),
        material=stainless_dark,
        name="back_panel",
    )
    cabinet.visual(
        Box((0.890, 0.040, 0.280)),
        origin=Origin(xyz=(0.000, -0.470, 0.140)),
        material=stainless,
        name="lower_front_panel",
    )
    cabinet.visual(
        Box((0.125, 0.040, 0.700)),
        origin=Origin(xyz=(-0.4125, -0.470, door_center_z)),
        material=stainless,
        name="left_front_stile",
    )
    cabinet.visual(
        Box((0.125, 0.040, 0.700)),
        origin=Origin(xyz=(0.4125, -0.470, door_center_z)),
        material=stainless,
        name="right_front_stile",
    )
    cabinet.visual(
        Box((0.890, 0.040, 0.070)),
        origin=Origin(xyz=(0.000, -0.470, 1.015)),
        material=stainless,
        name="upper_front_bridge",
    )
    cabinet.visual(
        Box((0.890, 0.090, 0.200)),
        origin=Origin(xyz=(0.000, -0.445, 1.150)),
        material=stainless,
        name="control_console",
    )
    cabinet.visual(
        Box((0.240, 0.004, 0.080)),
        origin=Origin(xyz=(-0.170, -0.488, 1.140)),
        material=indicator_black,
        name="display_window",
    )
    cabinet.visual(
        _annular_ring_mesh(
            "dryer_front_bezel",
            outer_radius=0.350,
            inner_radius=0.305,
            depth=0.040,
        ),
        origin=Origin(xyz=(0.000, -0.455, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="front_bezel",
    )
    cabinet.visual(
        _annular_ring_mesh(
            "dryer_gasket_ring",
            outer_radius=0.312,
            inner_radius=0.285,
            depth=0.022,
        ),
        origin=Origin(xyz=(0.000, -0.447, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="door_gasket",
    )
    cabinet.visual(
        Box((0.018, 0.030, 0.100)),
        origin=Origin(xyz=(0.315, -0.475, 0.750)),
        material=stainless_dark,
        name="upper_hinge_pad",
    )
    cabinet.visual(
        Box((0.032, 0.030, 0.110)),
        origin=Origin(xyz=(0.339, -0.475, 0.750)),
        material=stainless_dark,
        name="upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.018, 0.030, 0.100)),
        origin=Origin(xyz=(0.315, -0.475, 0.510)),
        material=stainless_dark,
        name="lower_hinge_pad",
    )
    cabinet.visual(
        Box((0.032, 0.030, 0.110)),
        origin=Origin(xyz=(0.339, -0.475, 0.510)),
        material=stainless_dark,
        name="lower_hinge_mount",
    )
    cabinet.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=Origin(xyz=(0.315, -0.472, door_center_z)),
        material=stainless_dark,
        name="barrel_hinge_spine",
    )
    cabinet.visual(
        Box((0.016, 0.030, 0.140)),
        origin=Origin(xyz=(0.180, -0.475, 1.110)),
        material=stainless_dark,
        name="coin_panel_hinge_pad",
    )
    cabinet.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.000, 0.450, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless_dark,
        name="rear_bearing_housing",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=95.0,
        origin=Origin(xyz=(0.000, 0.000, cabinet_height * 0.5)),
    )

    drum = model.part("drum")
    drum.visual(
        _drum_shell_mesh(),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless_dark,
        name="drum_shell",
    )
    for index, angle in enumerate((math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.040, 0.560, 0.030)),
            origin=Origin(
                xyz=(0.277 * math.cos(angle), 0.000, 0.277 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=charcoal,
            name=f"baffle_{index}",
        )
    drum.visual(
        Cylinder(radius=0.295, length=0.020),
        origin=Origin(xyz=(0.000, 0.335, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless_dark,
        name="rear_bulkhead",
    )
    drum.visual(
        Cylinder(radius=0.050, length=0.200),
        origin=Origin(xyz=(0.000, 0.370, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless_dark,
        name="rear_axle_hub",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.340, length=0.700),
        mass=18.0,
        origin=Origin(),
    )

    door = model.part("door")
    door.visual(
        _annular_ring_mesh(
            "dryer_door_ring",
            outer_radius=0.315,
            inner_radius=0.185,
            depth=0.058,
        ),
        origin=Origin(xyz=(-0.315, -0.029, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.185, length=0.012),
        origin=Origin(xyz=(-0.315, -0.021, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoked_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.034, 0.030, 0.095)),
        origin=Origin(xyz=(-0.540, -0.043, 0.000)),
        material=charcoal,
        name="door_handle",
    )
    door.visual(
        Box((0.018, 0.040, 0.100)),
        origin=Origin(xyz=(0.000, -0.020, 0.120)),
        material=stainless_dark,
        name="upper_hinge_leaf",
    )
    door.visual(
        Box((0.018, 0.040, 0.100)),
        origin=Origin(xyz=(0.000, -0.020, -0.120)),
        material=stainless_dark,
        name="lower_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(xyz=(0.000, -0.018, 0.130)),
        material=stainless_dark,
        name="upper_barrel_hinge",
    )
    door.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(xyz=(0.000, -0.018, -0.130)),
        material=stainless_dark,
        name="lower_barrel_hinge",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.660, 0.060, 0.680)),
        mass=7.5,
        origin=Origin(xyz=(-0.315, -0.030, 0.000)),
    )

    coin_panel = model.part("coin_panel")
    coin_panel.visual(
        Box((0.170, 0.018, 0.140)),
        origin=Origin(xyz=(0.085, -0.009, 0.000)),
        material=stainless_dark,
        name="coin_panel_door",
    )
    coin_panel.visual(
        Box((0.014, 0.030, 0.140)),
        origin=Origin(xyz=(0.000, -0.015, 0.000)),
        material=stainless_dark,
        name="coin_panel_hinge_leaf",
    )
    coin_panel.visual(
        Box((0.070, 0.004, 0.012)),
        origin=Origin(xyz=(0.112, -0.010, 0.030)),
        material=indicator_black,
        name="coin_slot",
    )
    coin_panel.visual(
        Box((0.030, 0.022, 0.016)),
        origin=Origin(xyz=(0.136, -0.018, -0.030)),
        material=charcoal,
        name="coin_panel_pull",
    )
    coin_panel.inertial = Inertial.from_geometry(
        Box((0.180, 0.030, 0.150)),
        mass=1.5,
        origin=Origin(xyz=(0.090, -0.015, 0.000)),
    )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.000, drum_center_y, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=12.0),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.315, front_face_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "cabinet_to_coin_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=coin_panel,
        origin=Origin(xyz=(0.180, front_face_y, 1.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=-1.10,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    coin_panel = object_model.get_part("coin_panel")

    drum_joint = object_model.get_articulation("cabinet_to_drum")
    door_joint = object_model.get_articulation("cabinet_to_door")
    coin_joint = object_model.get_articulation("cabinet_to_coin_panel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        drum,
        cabinet,
        elem_a="rear_axle_hub",
        elem_b="rear_bearing_housing",
        name="drum_axle_contacts_bearing",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a="upper_hinge_leaf",
        elem_b="upper_hinge_pad",
        name="upper_door_hinge_is_connected",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a="lower_hinge_leaf",
        elem_b="lower_hinge_pad",
        name="lower_door_hinge_is_connected",
    )
    ctx.expect_contact(
        coin_panel,
        cabinet,
        elem_a="coin_panel_hinge_leaf",
        elem_b="coin_panel_hinge_pad",
        name="coin_panel_hinge_is_connected",
    )

    ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.58, name="door_covers_opening")
    ctx.expect_overlap(coin_panel, cabinet, axes="xz", min_overlap=0.12, name="coin_panel_sits_on_console")
    ctx.expect_within(drum, cabinet, axes="xz", margin=0.02, name="drum_stays_within_body_width_and_height")

    ctx.check(
        "drum_axis_is_depthwise",
        _axis_matches(drum_joint.axis, (0.0, 1.0, 0.0)),
        details=f"Expected drum axis (0, 1, 0), got {drum_joint.axis}.",
    )
    ctx.check(
        "door_axis_is_vertical_right_edge_hinge",
        _axis_matches(door_joint.axis, (0.0, 0.0, 1.0)),
        details=f"Expected door axis (0, 0, 1), got {door_joint.axis}.",
    )
    ctx.check(
        "coin_panel_axis_is_vertical",
        _axis_matches(coin_joint.axis, (0.0, 0.0, 1.0)),
        details=f"Expected coin-panel axis (0, 0, 1), got {coin_joint.axis}.",
    )
    door_limits = door_joint.motion_limits
    coin_limits = coin_joint.motion_limits
    ctx.check(
        "door_opens_wide_enough",
        door_limits is not None and door_limits.upper is not None and door_limits.upper >= 1.20,
        details=f"Door upper limit should be at least 1.20 rad, got {None if door_limits is None else door_limits.upper}.",
    )
    ctx.check(
        "coin_panel_has_service_swing",
        coin_limits is not None
        and coin_limits.lower is not None
        and coin_limits.lower <= -0.80
        and coin_limits.upper == 0.0,
        details=(
            "Coin-panel hinge should swing outward from the closed position. "
            f"Got limits {None if coin_limits is None else (coin_limits.lower, coin_limits.upper)}."
        ),
    )

    door_rest_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    coin_rest_aabb = ctx.part_element_world_aabb(coin_panel, elem="coin_panel_door")
    drum_rest_aabb = ctx.part_element_world_aabb(drum, elem="baffle_0")
    assert door_rest_aabb is not None
    assert coin_rest_aabb is not None
    assert drum_rest_aabb is not None

    door_rest_center = _aabb_center(door_rest_aabb)
    coin_rest_center = _aabb_center(coin_rest_aabb)
    drum_rest_center = _aabb_center(drum_rest_aabb)

    with ctx.pose({door_joint: 1.20}):
        door_open_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
        assert door_open_aabb is not None
        door_open_center = _aabb_center(door_open_aabb)
        ctx.check(
            "door_swings_outward_on_right_hinge",
            door_open_center[1] < door_rest_center[1] - 0.18 and door_open_center[0] > door_rest_center[0] + 0.02,
            details=(
                "Expected the door to move outward and toward +X when opened from the right edge. "
                f"rest={door_rest_center}, open={door_open_center}"
            ),
        )

    with ctx.pose({coin_joint: -1.00}):
        coin_open_aabb = ctx.part_element_world_aabb(coin_panel, elem="coin_panel_door")
        assert coin_open_aabb is not None
        coin_open_center = _aabb_center(coin_open_aabb)
        ctx.check(
            "coin_panel_swings_outward",
            coin_open_center[1] < coin_rest_center[1] - 0.05 and coin_open_center[0] < coin_rest_center[0] - 0.03,
            details=(
                "Expected the coin panel to open outward from its front-face hinge. "
                f"rest={coin_rest_center}, open={coin_open_center}"
            ),
        )

    with ctx.pose({drum_joint: math.pi / 2.0}):
        drum_spun_aabb = ctx.part_element_world_aabb(drum, elem="baffle_0")
        assert drum_spun_aabb is not None
        drum_spun_center = _aabb_center(drum_spun_aabb)
        ctx.check(
            "drum_rotates_about_depth_axis",
            drum_spun_center[0] > drum_rest_center[0] + 0.18 and drum_spun_center[2] < drum_rest_center[2] - 0.18,
            details=(
                "Expected an internal baffle to move around the visible drum circumference. "
                f"rest={drum_rest_center}, spun={drum_spun_center}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
