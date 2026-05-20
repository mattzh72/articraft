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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_remote_weapon_station")

    armor = model.material("armor", rgba=(0.34, 0.37, 0.33, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.08, 1.0))
    optic_glass = model.material("optic_glass", rgba=(0.20, 0.32, 0.44, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.22, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="mounting_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.16, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=armor,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_steel,
        name="slew_pad",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.44, 0.44, 0.16)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    rotating_base = model.part("rotating_base")
    rotating_base.visual(
        Cylinder(radius=0.18, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="azimuth_ring",
    )
    rotating_base.visual(
        Box((0.28, 0.34, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=armor,
        name="turret_deck",
    )
    rotating_base.visual(
        Box((0.10, 0.18, 0.08)),
        origin=Origin(xyz=(-0.16, 0.0, 0.12)),
        material=armor,
        name="rear_housing",
    )
    rotating_base.visual(
        Box((0.20, 0.03, 0.22)),
        origin=Origin(xyz=(0.0, -0.18, 0.19)),
        material=armor,
        name="left_support",
    )
    rotating_base.visual(
        Box((0.20, 0.03, 0.22)),
        origin=Origin(xyz=(0.0, 0.18, 0.19)),
        material=armor,
        name="right_support",
    )
    rotating_base.visual(
        Box((0.12, 0.24, 0.03)),
        origin=Origin(xyz=(0.04, 0.0, 0.09)),
        material=armor,
        name="front_crossmember",
    )
    rotating_base.visual(
        Cylinder(radius=0.038, length=0.03),
        origin=Origin(xyz=(0.02, -0.18, 0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_bearing",
    )
    rotating_base.visual(
        Cylinder(radius=0.038, length=0.03),
        origin=Origin(xyz=(0.02, 0.18, 0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_bearing",
    )
    rotating_base.inertial = Inertial.from_geometry(
        Box((0.36, 0.40, 0.30)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.038, length=0.03),
        origin=Origin(xyz=(0.0, -0.15, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    cradle.visual(
        Cylinder(radius=0.038, length=0.03),
        origin=Origin(xyz=(0.0, 0.15, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    cradle.visual(
        Box((0.18, 0.28, 0.08)),
        origin=Origin(xyz=(-0.04, 0.0, -0.05)),
        material=armor,
        name="lower_frame",
    )
    cradle.visual(
        Box((0.18, 0.26, 0.11)),
        origin=Origin(xyz=(0.02, 0.0, 0.02)),
        material=armor,
        name="receiver_box",
    )
    cradle.visual(
        Box((0.12, 0.18, 0.06)),
        origin=Origin(xyz=(-0.02, 0.0, 0.085)),
        material=armor,
        name="top_cover",
    )
    cradle.visual(
        Box((0.06, 0.18, 0.04)),
        origin=Origin(xyz=(0.10, 0.0, -0.055)),
        material=armor,
        name="armor_mount",
    )
    cradle.visual(
        Box((0.020, 0.18, 0.06)),
        origin=Origin(xyz=(0.119, 0.0, 0.11)),
        material=armor,
        name="front_armor",
    )
    cradle.visual(
        Box((0.018, 0.05, 0.12)),
        origin=Origin(xyz=(0.117, -0.07, 0.02)),
        material=armor,
        name="left_armor_cheek",
    )
    cradle.visual(
        Box((0.018, 0.05, 0.12)),
        origin=Origin(xyz=(0.117, 0.07, 0.02)),
        material=armor,
        name="right_armor_cheek",
    )
    cradle.visual(
        Cylinder(radius=0.007, length=0.16),
        origin=Origin(xyz=(0.128, 0.0, 0.14), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shield_hinge_rod",
    )
    cradle.visual(
        Cylinder(radius=0.026, length=0.50),
        origin=Origin(xyz=(0.31, 0.0, 0.01), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="barrel",
    )
    cradle.visual(
        Cylinder(radius=0.034, length=0.14),
        origin=Origin(xyz=(0.47, 0.0, 0.01), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="barrel_shroud",
    )
    cradle.visual(
        Cylinder(radius=0.019, length=0.05),
        origin=Origin(xyz=(0.585, 0.0, 0.01), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="muzzle",
    )
    cradle.visual(
        Box((0.14, 0.09, 0.09)),
        origin=Origin(xyz=(0.03, 0.095, 0.035)),
        material=dark_steel,
        name="sensor_box",
    )
    cradle.visual(
        Box((0.05, 0.094, 0.04)),
        origin=Origin(xyz=(0.115, 0.095, 0.038)),
        material=dark_steel,
        name="sensor_hood",
    )
    cradle.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.14, 0.095, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=optic_glass,
        name="main_optic",
    )
    cradle.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.14, 0.095, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=optic_glass,
        name="secondary_optic",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.64, 0.30, 0.22)),
        mass=34.0,
        origin=Origin(xyz=(0.18, 0.0, 0.02)),
    )

    shield_flap = model.part("shield_flap")
    shield_flap.visual(
        Box((0.012, 0.10, 0.026)),
        origin=Origin(xyz=(0.010, 0.0, -0.013)),
        material=dark_steel,
        name="top_link",
    )
    shield_flap.visual(
        Box((0.010, 0.018, 0.016)),
        origin=Origin(xyz=(0.012, -0.038, 0.0)),
        material=dark_steel,
        name="left_hinge_ear",
    )
    shield_flap.visual(
        Box((0.010, 0.018, 0.016)),
        origin=Origin(xyz=(0.012, 0.038, 0.0)),
        material=dark_steel,
        name="right_hinge_ear",
    )
    shield_flap.visual(
        Box((0.008, 0.09, 0.058)),
        origin=Origin(xyz=(0.008, 0.0, -0.055)),
        material=armor,
        name="flap_panel",
    )
    shield_flap.visual(
        Box((0.014, 0.010, 0.058)),
        origin=Origin(xyz=(0.010, -0.040, -0.055)),
        material=armor,
        name="left_flap_stiffener",
    )
    shield_flap.visual(
        Box((0.014, 0.010, 0.058)),
        origin=Origin(xyz=(0.010, 0.040, -0.055)),
        material=armor,
        name="right_flap_stiffener",
    )
    shield_flap.inertial = Inertial.from_geometry(
        Box((0.024, 0.10, 0.08)),
        mass=3.5,
        origin=Origin(xyz=(0.010, 0.0, -0.042)),
    )

    model.articulation(
        "pedestal_to_rotating_base",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=rotating_base,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6),
    )
    model.articulation(
        "base_to_cradle",
        ArticulationType.REVOLUTE,
        parent=rotating_base,
        child=cradle,
        origin=Origin(xyz=(0.02, 0.0, 0.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=math.radians(-20.0),
            upper=math.radians(63.0),
        ),
    )
    model.articulation(
        "cradle_to_shield_flap",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=shield_flap,
        origin=Origin(xyz=(0.128, 0.0, 0.14)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=math.radians(-12.0),
            upper=math.radians(58.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    rotating_base = object_model.get_part("rotating_base")
    cradle = object_model.get_part("cradle")
    shield_flap = object_model.get_part("shield_flap")

    yaw_joint = object_model.get_articulation("pedestal_to_rotating_base")
    elevation_joint = object_model.get_articulation("base_to_cradle")
    flap_joint = object_model.get_articulation("cradle_to_shield_flap")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "azimuth_axis_is_vertical",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        f"expected vertical azimuth axis, got {yaw_joint.axis}",
    )
    ctx.check(
        "elevation_axis_is_horizontal",
        elevation_joint.axis == (0.0, -1.0, 0.0),
        f"expected lateral elevation axis, got {elevation_joint.axis}",
    )
    ctx.check(
        "shield_hinge_axis_is_horizontal",
        flap_joint.axis == (0.0, -1.0, 0.0),
        f"expected top-edge flap hinge axis, got {flap_joint.axis}",
    )

    ctx.expect_contact(rotating_base, pedestal, elem_a="azimuth_ring", elem_b="slew_pad")
    ctx.expect_contact(cradle, rotating_base, elem_a="left_trunnion", elem_b="left_bearing")
    ctx.expect_contact(cradle, rotating_base, elem_a="right_trunnion", elem_b="right_bearing")
    ctx.expect_contact(shield_flap, cradle, elem_a="left_hinge_ear", elem_b="shield_hinge_rod")
    ctx.expect_contact(shield_flap, cradle, elem_a="right_hinge_ear", elem_b="shield_hinge_rod")
    ctx.expect_overlap(cradle, rotating_base, axes="xy", min_overlap=0.10)
    ctx.expect_overlap(shield_flap, cradle, axes="y", min_overlap=0.08)

    sensor_rest = ctx.part_element_world_aabb(cradle, elem="sensor_box")
    barrel_rest = ctx.part_element_world_aabb(cradle, elem="barrel")
    flap_rest = ctx.part_element_world_aabb(shield_flap, elem="flap_panel")
    assert sensor_rest is not None
    assert barrel_rest is not None
    assert flap_rest is not None
    sensor_rest_center = aabb_center(sensor_rest)
    barrel_rest_center = aabb_center(barrel_rest)
    flap_rest_center = aabb_center(flap_rest)

    with ctx.pose({yaw_joint: math.pi / 2.0}):
        sensor_yawed = ctx.part_element_world_aabb(cradle, elem="sensor_box")
        assert sensor_yawed is not None
        sensor_yawed_center = aabb_center(sensor_yawed)
        ctx.expect_contact(rotating_base, pedestal, elem_a="azimuth_ring", elem_b="slew_pad")
        ctx.check(
            "sensor_box_swings_with_azimuth",
            sensor_rest_center[0] > 0.0
            and sensor_yawed_center[0] < -0.05
            and sensor_yawed_center[1] > 0.0,
            f"sensor center did not rotate as expected: rest={sensor_rest_center}, yawed={sensor_yawed_center}",
        )

    with ctx.pose({elevation_joint: math.radians(45.0)}):
        barrel_elevated = ctx.part_element_world_aabb(cradle, elem="barrel")
        assert barrel_elevated is not None
        barrel_elevated_center = aabb_center(barrel_elevated)
        ctx.expect_contact(cradle, rotating_base, elem_a="left_trunnion", elem_b="left_bearing")
        ctx.expect_contact(cradle, rotating_base, elem_a="right_trunnion", elem_b="right_bearing")
        ctx.check(
            "barrel_rises_with_elevation",
            barrel_elevated_center[2] > barrel_rest_center[2] + 0.12,
            f"barrel center did not rise enough: rest={barrel_rest_center}, elevated={barrel_elevated_center}",
        )

    with ctx.pose({flap_joint: math.radians(55.0)}):
        flap_open = ctx.part_element_world_aabb(shield_flap, elem="flap_panel")
        assert flap_open is not None
        flap_open_center = aabb_center(flap_open)
        ctx.expect_contact(shield_flap, cradle, elem_a="left_hinge_ear", elem_b="shield_hinge_rod")
        ctx.expect_contact(shield_flap, cradle, elem_a="right_hinge_ear", elem_b="shield_hinge_rod")
        ctx.check(
            "shield_flap_deploys_forward",
            flap_open_center[0] > flap_rest_center[0] + 0.02
            and flap_open_center[2] > flap_rest_center[2] + 0.02,
            f"shield flap did not deploy from hinge as expected: rest={flap_rest_center}, open={flap_open_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
