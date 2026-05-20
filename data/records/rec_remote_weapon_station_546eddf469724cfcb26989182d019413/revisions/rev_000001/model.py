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
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_band_mesh(*, outer_radius: float, inner_radius: float, height: float):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, height)],
        [(inner_radius, 0.0), (inner_radius, height)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _side_frame_mesh(side_y: float):
    return wire_from_points(
        [
            (-0.24, side_y, 0.08),
            (0.18, side_y, 0.08),
            (0.18, side_y, 0.44),
            (0.12, side_y, 0.70),
            (-0.24, side_y, 0.70),
        ],
        radius=0.024,
        radial_segments=16,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.055,
        corner_segments=10,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station")

    pedestal_gray = model.material("pedestal_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    armor_green = model.material("armor_green", rgba=(0.37, 0.43, 0.31, 1.0))
    shield_green = model.material("shield_green", rgba=(0.41, 0.47, 0.34, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    optics_black = model.material("optics_black", rgba=(0.07, 0.08, 0.09, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.20, 0.29, 0.34, 1.0))

    ring_band_mesh = _mesh(
        "rws_ring_band",
        _ring_band_mesh(outer_radius=0.40, inner_radius=0.22, height=0.06),
    )
    left_frame_mesh = _mesh("rws_left_side_frame", _side_frame_mesh(0.31))
    right_frame_mesh = _mesh("rws_right_side_frame", _side_frame_mesh(-0.31))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.72, 0.72, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=pedestal_gray,
        name="base_plinth",
    )
    pedestal.visual(
        Box((0.52, 0.52, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=pedestal_gray,
        name="lower_pedestal",
    )
    pedestal.visual(
        Cylinder(radius=0.19, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        material=pedestal_gray,
        name="mast",
    )
    pedestal.visual(
        Cylinder(radius=0.33, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.945)),
        material=gunmetal,
        name="top_bearing_plate",
    )
    pedestal.visual(
        Box((0.24, 0.18, 0.24)),
        origin=Origin(xyz=(-0.22, 0.0, 0.58)),
        material=gunmetal,
        name="drive_cabinet",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 1.00)),
        mass=280.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    ring_mount = model.part("ring_mount")
    ring_mount.visual(ring_band_mesh, material=armor_green, name="azimuth_race")
    ring_mount.visual(left_frame_mesh, material=armor_green, name="left_side_frame")
    ring_mount.visual(right_frame_mesh, material=armor_green, name="right_side_frame")
    ring_mount.visual(
        Box((0.24, 0.24, 0.12)),
        origin=Origin(xyz=(-0.16, 0.0, 0.12)),
        material=gunmetal,
        name="azimuth_drive_housing",
    )
    ring_mount.visual(
        Box((0.14, 0.05, 0.22)),
        origin=Origin(xyz=(0.10, 0.29, 0.35)),
        material=armor_green,
        name="left_lug_plate",
    )
    ring_mount.visual(
        Box((0.14, 0.05, 0.22)),
        origin=Origin(xyz=(0.10, -0.29, 0.35)),
        material=armor_green,
        name="right_lug_plate",
    )
    ring_mount.visual(
        Cylinder(radius=0.055, length=0.06),
        origin=Origin(xyz=(0.10, 0.295, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_lug_bearing",
    )
    ring_mount.visual(
        Cylinder(radius=0.055, length=0.06),
        origin=Origin(xyz=(0.10, -0.295, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_lug_bearing",
    )
    ring_mount.visual(
        Box((0.14, 0.66, 0.035)),
        origin=Origin(xyz=(0.12, 0.0, 0.71)),
        material=armor_green,
        name="roof_front_rail",
    )
    ring_mount.visual(
        Box((0.18, 0.66, 0.035)),
        origin=Origin(xyz=(-0.20, 0.0, 0.74)),
        material=armor_green,
        name="roof_rear_rail",
    )
    ring_mount.visual(
        Box((0.32, 0.66, 0.04)),
        origin=Origin(xyz=(-0.01, 0.0, 0.755)),
        material=armor_green,
        name="roof_center_support",
    )
    ring_mount.visual(
        Box((0.78, 0.70, 0.024)),
        origin=Origin(xyz=(-0.04, 0.0, 0.787)),
        material=shield_green,
        name="roof_panel",
    )
    ring_mount.inertial = Inertial.from_geometry(
        Box((0.82, 0.82, 0.82)),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
    )

    gun_cradle = model.part("gun_cradle")
    gun_cradle.visual(
        Cylinder(radius=0.05, length=0.53),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_axle",
    )
    gun_cradle.visual(
        Box((0.22, 0.05, 0.18)),
        origin=Origin(xyz=(0.02, 0.12, -0.04)),
        material=gunmetal,
        name="left_cradle_cheek",
    )
    gun_cradle.visual(
        Box((0.22, 0.05, 0.18)),
        origin=Origin(xyz=(0.02, -0.12, -0.04)),
        material=gunmetal,
        name="right_cradle_cheek",
    )
    gun_cradle.visual(
        Box((0.54, 0.18, 0.16)),
        origin=Origin(xyz=(0.20, 0.0, -0.03)),
        material=gunmetal,
        name="receiver_body",
    )
    gun_cradle.visual(
        Box((0.34, 0.12, 0.05)),
        origin=Origin(xyz=(0.08, 0.0, 0.08)),
        material=dark_steel,
        name="receiver_top_cover",
    )
    gun_cradle.visual(
        Box((0.18, 0.12, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, -0.02)),
        material=dark_steel,
        name="rear_breech_block",
    )
    gun_cradle.visual(
        Cylinder(radius=0.022, length=0.42),
        origin=Origin(xyz=(0.46, 0.055, 0.065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_recoil_guide",
    )
    gun_cradle.visual(
        Cylinder(radius=0.022, length=0.42),
        origin=Origin(xyz=(0.46, -0.055, 0.065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_recoil_guide",
    )
    gun_cradle.visual(
        Cylinder(radius=0.038, length=0.92),
        origin=Origin(xyz=(0.88, 0.0, -0.01), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="barrel_shroud",
    )
    gun_cradle.visual(
        Cylinder(radius=0.022, length=0.14),
        origin=Origin(xyz=(1.41, 0.0, -0.01), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="muzzle_tube",
    )
    gun_cradle.visual(
        Cylinder(radius=0.029, length=0.08),
        origin=Origin(xyz=(1.52, 0.0, -0.01), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="flash_hider",
    )
    gun_cradle.visual(
        Box((0.28, 0.12, 0.20)),
        origin=Origin(xyz=(0.08, 0.18, -0.07)),
        material=gunmetal,
        name="ammo_box",
    )
    gun_cradle.visual(
        Box((0.18, 0.10, 0.04)),
        origin=Origin(xyz=(0.39, -0.14, 0.06)),
        material=gunmetal,
        name="sight_mount_arm",
    )
    gun_cradle.visual(
        Box((0.04, 0.20, 0.04)),
        origin=Origin(xyz=(0.50, -0.24, 0.05)),
        material=gunmetal,
        name="sight_mount_bracket",
    )
    gun_cradle.visual(
        Box((0.04, 0.01, 0.08)),
        origin=Origin(xyz=(0.54, -0.275, 0.10)),
        material=gunmetal,
        name="sight_mount_outer",
    )
    gun_cradle.visual(
        Box((0.04, 0.01, 0.08)),
        origin=Origin(xyz=(0.54, -0.205, 0.10)),
        material=gunmetal,
        name="sight_mount_inner",
    )
    gun_cradle.inertial = Inertial.from_geometry(
        Box((1.62, 0.54, 0.34)),
        mass=68.0,
        origin=Origin(xyz=(0.68, 0.0, -0.01)),
    )

    sight_unit = model.part("sight_unit")
    sight_unit.visual(
        Cylinder(radius=0.016, length=0.06),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="sight_tilt_pin",
    )
    sight_unit.visual(
        Box((0.10, 0.04, 0.08)),
        origin=Origin(xyz=(0.05, 0.0, -0.03)),
        material=gunmetal,
        name="sight_mount_block",
    )
    sight_unit.visual(
        Box((0.20, 0.10, 0.14)),
        origin=Origin(xyz=(0.18, -0.06, 0.00)),
        material=optics_black,
        name="sight_housing",
    )
    sight_unit.visual(
        Box((0.10, 0.10, 0.11)),
        origin=Origin(xyz=(0.31, -0.06, 0.01)),
        material=optics_black,
        name="sensor_head",
    )
    sight_unit.visual(
        Cylinder(radius=0.026, length=0.03),
        origin=Origin(xyz=(0.36, -0.06, 0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sensor_glass,
        name="front_lens",
    )
    sight_unit.inertial = Inertial.from_geometry(
        Box((0.44, 0.12, 0.16)),
        mass=8.0,
        origin=Origin(xyz=(0.19, -0.04, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=ring_mount,
        origin=Origin(xyz=(0.0, 0.0, 0.97)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=1.1),
    )
    model.articulation(
        "elevation_trunnion",
        ArticulationType.REVOLUTE,
        parent=ring_mount,
        child=gun_cradle,
        origin=Origin(xyz=(0.10, 0.0, 0.42)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.9,
            lower=-0.25,
            upper=1.05,
        ),
    )
    model.articulation(
        "sight_tilt",
        ArticulationType.REVOLUTE,
        parent=gun_cradle,
        child=sight_unit,
        origin=Origin(xyz=(0.54, -0.24, 0.10)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=-0.45,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    ring_mount = object_model.get_part("ring_mount")
    gun_cradle = object_model.get_part("gun_cradle")
    sight_unit = object_model.get_part("sight_unit")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_trunnion")
    sight_tilt = object_model.get_articulation("sight_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "azimuth_axis_vertical",
        azimuth.axis == (0.0, 0.0, 1.0),
        details=f"axis={azimuth.axis}",
    )
    ctx.check(
        "elevation_axis_horizontal",
        elevation.axis == (0.0, -1.0, 0.0),
        details=f"axis={elevation.axis}",
    )
    ctx.check(
        "sight_tilt_axis_horizontal",
        sight_tilt.axis == (0.0, -1.0, 0.0),
        details=f"axis={sight_tilt.axis}",
    )

    ctx.expect_contact(
        ring_mount,
        pedestal,
        elem_a="azimuth_race",
        elem_b="top_bearing_plate",
        name="azimuth_race_seated_on_pedestal",
    )
    ctx.expect_contact(
        gun_cradle,
        ring_mount,
        elem_a="trunnion_axle",
        elem_b="left_lug_bearing",
        name="left_trunnion_supported",
    )
    ctx.expect_contact(
        gun_cradle,
        ring_mount,
        elem_a="trunnion_axle",
        elem_b="right_lug_bearing",
        name="right_trunnion_supported",
    )
    ctx.expect_contact(
        sight_unit,
        gun_cradle,
        elem_a="sight_tilt_pin",
        elem_b="sight_mount_outer",
        name="sight_pin_bears_on_outer_ear",
    )
    ctx.expect_contact(
        sight_unit,
        gun_cradle,
        elem_a="sight_tilt_pin",
        elem_b="sight_mount_inner",
        name="sight_pin_bears_on_inner_ear",
    )

    with ctx.pose({elevation: 0.95}):
        ctx.expect_contact(
            gun_cradle,
            ring_mount,
            elem_a="trunnion_axle",
            elem_b="left_lug_bearing",
            name="left_trunnion_supported_when_elevated",
        )
        ctx.expect_contact(
            gun_cradle,
            ring_mount,
            elem_a="trunnion_axle",
            elem_b="right_lug_bearing",
            name="right_trunnion_supported_when_elevated",
        )
        ctx.expect_gap(
            ring_mount,
            gun_cradle,
            axis="z",
            positive_elem="roof_panel",
            negative_elem="receiver_top_cover",
            min_gap=0.05,
            name="receiver_clears_roof_when_elevated",
        )

    ctx.expect_gap(
        gun_cradle,
        sight_unit,
        axis="y",
        positive_elem="right_cradle_cheek",
        negative_elem="sight_housing",
        min_gap=0.08,
        name="sight_housing_stays_outboard_of_receiver",
    )

    with ctx.pose({sight_tilt: 0.25}):
        ctx.expect_contact(
            sight_unit,
            gun_cradle,
            elem_a="sight_tilt_pin",
            elem_b="sight_mount_outer",
            name="sight_pin_bears_on_outer_ear_when_tilted",
        )
        ctx.expect_contact(
            sight_unit,
            gun_cradle,
            elem_a="sight_tilt_pin",
            elem_b="sight_mount_inner",
            name="sight_pin_bears_on_inner_ear_when_tilted",
        )
        ctx.expect_gap(
            gun_cradle,
            sight_unit,
            axis="y",
            positive_elem="right_cradle_cheek",
            negative_elem="sight_housing",
            min_gap=0.08,
            name="sight_housing_stays_outboard_when_tilted",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
