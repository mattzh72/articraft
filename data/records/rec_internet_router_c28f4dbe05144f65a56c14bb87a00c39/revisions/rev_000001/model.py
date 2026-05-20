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
    model = ArticulatedObject(name="gaming_router")

    body_black = model.material("body_black", rgba=(0.10, 0.11, 0.13, 1.0))
    body_dark = model.material("body_dark", rgba=(0.16, 0.17, 0.20, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.05, 0.05, 0.06, 1.0))
    accent_red = model.material("accent_red", rgba=(0.72, 0.08, 0.08, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.368, 0.256, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_black,
        name="housing_lower",
    )
    housing.visual(
        Box((0.356, 0.214, 0.012)),
        origin=Origin(xyz=(0.0, 0.018, 0.016)),
        material=body_black,
        name="housing_mid",
    )
    housing.visual(
        Box((0.340, 0.166, 0.012)),
        origin=Origin(xyz=(0.0, 0.046, 0.026)),
        material=body_black,
        name="housing_upper",
    )
    housing.visual(
        Box((0.324, 0.114, 0.010)),
        origin=Origin(xyz=(0.0, 0.074, 0.037)),
        material=body_black,
        name="housing_top",
    )
    housing.visual(
        Box((0.334, 0.156, 0.008)),
        origin=Origin(xyz=(0.0, 0.044, 0.034), rpy=(-0.22, 0.0, 0.0)),
        material=body_dark,
        name="roof_plane",
    )
    housing.visual(
        Box((0.306, 0.032, 0.008)),
        origin=Origin(xyz=(0.0, 0.104, 0.046)),
        material=body_dark,
        name="rear_mount_deck",
    )
    housing.visual(
        Box((0.320, 0.204, 0.003)),
        origin=Origin(xyz=(0.0, 0.004, 0.0015)),
        material=body_dark,
        name="bottom_plate",
    )
    for x_sign in (-1.0, 1.0):
        housing.visual(
            Box((0.074, 0.006, 0.003)),
            origin=Origin(
                xyz=(x_sign * 0.050, -0.002, 0.033),
                rpy=(-0.14, 0.0, x_sign * 0.22),
            ),
            material=accent_red,
            name=f"top_accent_{'left' if x_sign < 0 else 'right'}",
        )

    switch_center_y = 0.078
    switch_center_z = 0.021
    housing.visual(
        Box((0.003, 0.003, 0.014)),
        origin=Origin(xyz=(0.1835, switch_center_y - 0.0095, switch_center_z)),
        material=body_dark,
        name="switch_front_tab",
    )
    housing.visual(
        Box((0.003, 0.003, 0.014)),
        origin=Origin(xyz=(0.1835, switch_center_y + 0.0095, switch_center_z)),
        material=body_dark,
        name="switch_rear_tab",
    )
    housing.visual(
        Box((0.003, 0.020, 0.003)),
        origin=Origin(xyz=(0.1835, switch_center_y, switch_center_z + 0.0085)),
        material=body_dark,
        name="switch_upper_tab",
    )
    housing.visual(
        Box((0.003, 0.020, 0.003)),
        origin=Origin(xyz=(0.1835, switch_center_y, switch_center_z - 0.0085)),
        material=body_dark,
        name="switch_lower_tab",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.370, 0.255, 0.056)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    antenna_specs = [
        ("left_outer", -0.145),
        ("left_mid", -0.109),
        ("left_inner", -0.073),
        ("right_inner", 0.073),
        ("right_mid", 0.109),
        ("right_outer", 0.145),
    ]

    for index, (suffix, x_pos) in enumerate(antenna_specs, start=1):
        base = model.part(f"antenna_base_{suffix}")
        base.visual(
            Box((0.024, 0.018, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=body_dark,
            name="pod_block",
        )
        base.visual(
            Box((0.020, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, 0.003, 0.008)),
            material=body_dark,
            name="hinge_saddle",
        )
        for x_knuckle, knuckle_name in ((-0.005, "left"), (0.005, "right")):
            base.visual(
                Cylinder(radius=0.004, length=0.006),
                origin=Origin(
                    xyz=(x_knuckle, 0.004, 0.010),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=body_dark,
                name=f"{knuckle_name}_knuckle",
            )
        base.inertial = Inertial.from_geometry(
            Box((0.026, 0.020, 0.014)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.002, 0.007)),
        )

        antenna = model.part(f"antenna_{suffix}")
        antenna.visual(
            Cylinder(radius=0.0038, length=0.004),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_dark,
            name="hinge_hub",
        )
        antenna.visual(
            Box((0.008, 0.006, 0.018)),
            origin=Origin(xyz=(0.0, -0.003, 0.010)),
            material=rubber_dark,
            name="root_neck",
        )
        antenna.visual(
            Box((0.010, 0.006, 0.072)),
            origin=Origin(xyz=(0.0, -0.004, 0.046)),
            material=rubber_dark,
            name="lower_blade",
        )
        antenna.visual(
            Box((0.008, 0.005, 0.094)),
            origin=Origin(xyz=(0.0, -0.006, 0.128)),
            material=rubber_dark,
            name="upper_blade",
        )
        antenna.visual(
            Box((0.006, 0.004, 0.018)),
            origin=Origin(xyz=(0.0, -0.0065, 0.184)),
            material=rubber_dark,
            name="tip",
        )
        antenna.inertial = Inertial.from_geometry(
            Box((0.012, 0.010, 0.192)),
            mass=0.06,
            origin=Origin(xyz=(0.0, -0.005, 0.096)),
        )

        model.articulation(
            f"housing_to_antenna_base_{suffix}",
            ArticulationType.FIXED,
            parent=housing,
            child=base,
            origin=Origin(xyz=(x_pos, 0.108, 0.050)),
        )
        model.articulation(
            f"antenna_base_to_antenna_{suffix}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=antenna,
            origin=Origin(xyz=(0.0, 0.004, 0.010)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.5,
                lower=-1.15,
                upper=0.28,
            ),
            meta={"group_index": index},
        )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Cylinder(radius=0.0018, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_dark,
        name="switch_axle",
    )
    power_switch.visual(
        Box((0.006, 0.018, 0.012)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=body_dark,
        name="switch_body",
    )
    power_switch.visual(
        Box((0.002, 0.018, 0.006)),
        origin=Origin(xyz=(0.005, 0.0, 0.003)),
        material=accent_red,
        name="switch_cap",
    )
    power_switch.inertial = Inertial.from_geometry(
        Box((0.010, 0.020, 0.014)),
        mass=0.02,
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=power_switch,
        origin=Origin(xyz=(0.182, switch_center_y, switch_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=2.0,
            lower=-0.28,
            upper=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    power_switch = object_model.get_part("power_switch")
    switch_joint = object_model.get_articulation("housing_to_power_switch")

    antenna_suffixes = (
        "left_outer",
        "left_mid",
        "left_inner",
        "right_inner",
        "right_mid",
        "right_outer",
    )

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
        "switch_axis_is_side_to_side",
        tuple(switch_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected power switch axis (0, 1, 0), got {switch_joint.axis}",
    )
    ctx.expect_contact(power_switch, housing, name="power_switch_is_captured_in_side_opening")

    switch_pos = ctx.part_world_position(power_switch)
    if switch_pos is None:
        ctx.fail("power_switch_position_available", "Power switch has no world position.")
    else:
        ctx.check(
            "power_switch_is_on_right_side",
            switch_pos[0] > 0.17 and switch_pos[1] > 0.05 and 0.012 < switch_pos[2] < 0.032,
            f"Unexpected power switch position: {switch_pos}",
        )

    folded_pose = {switch_joint: 0.20}

    for suffix in antenna_suffixes:
        base = object_model.get_part(f"antenna_base_{suffix}")
        antenna = object_model.get_part(f"antenna_{suffix}")
        antenna_joint = object_model.get_articulation(f"antenna_base_to_antenna_{suffix}")

        ctx.expect_contact(base, housing, name=f"{suffix}_base_contacts_housing")
        ctx.expect_contact(antenna, base, name=f"{suffix}_antenna_is_captured_by_base")
        ctx.check(
            f"{suffix}_antenna_axis_is_x",
            tuple(antenna_joint.axis) == (1.0, 0.0, 0.0),
            f"Expected antenna axis (1, 0, 0), got {antenna_joint.axis}",
        )

        base_pos = ctx.part_world_position(base)
        if base_pos is None:
            ctx.fail(f"{suffix}_base_position_available", "Antenna base has no world position.")
        else:
            expected_sign = -1.0 if suffix.startswith("left") else 1.0
            ctx.check(
                f"{suffix}_base_is_along_rear_edge",
                base_pos[1] > 0.09 and expected_sign * base_pos[0] > 0.06 and 0.048 <= base_pos[2] <= 0.065,
                f"Unexpected antenna base position: {base_pos}",
            )

        folded_pose[antenna_joint] = -1.02

    with ctx.pose({switch_joint: -0.20}):
        ctx.expect_contact(power_switch, housing, name="power_switch_contact_off_pose")

    with ctx.pose({switch_joint: 0.20}):
        ctx.expect_contact(power_switch, housing, name="power_switch_contact_on_pose")

    with ctx.pose(folded_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_has_no_overlaps")
        for suffix in antenna_suffixes:
            antenna = object_model.get_part(f"antenna_{suffix}")
            base = object_model.get_part(f"antenna_base_{suffix}")
            ctx.expect_contact(antenna, base, name=f"{suffix}_antenna_stays_captured_when_folded")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
