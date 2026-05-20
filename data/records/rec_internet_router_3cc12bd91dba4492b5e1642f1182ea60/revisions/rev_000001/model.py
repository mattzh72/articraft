from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_router")

    body_material = model.material("router_body", rgba=(0.10, 0.11, 0.12, 1.0))
    base_material = model.material("router_base", rgba=(0.07, 0.08, 0.09, 1.0))
    antenna_material = model.material("router_antenna", rgba=(0.14, 0.15, 0.16, 1.0))
    switch_material = model.material("router_switch", rgba=(0.18, 0.19, 0.20, 1.0))

    base_h = 0.010
    body_h = 0.205
    body_joint_x = 0.0505
    upper_antenna_y = 0.007
    lower_antenna_y = -0.007

    def rounded_rect_section(width: float, depth: float, radius: float, z: float):
        return tuple(
            (x, y, z)
            for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)
        )

    base = model.part("base")
    base_mesh = section_loft(
        [
            rounded_rect_section(0.092, 0.060, 0.010, 0.000),
            rounded_rect_section(0.088, 0.056, 0.009, 0.004),
            rounded_rect_section(0.084, 0.052, 0.008, base_h),
        ]
    )
    base.visual(
        mesh_from_geometry(base_mesh, "router_base_shell"),
        material=base_material,
        name="base_shell",
    )

    body = model.part("body")
    body_mesh = section_loft(
        [
            rounded_rect_section(0.080, 0.038, 0.008, 0.000),
            rounded_rect_section(0.078, 0.036, 0.007, 0.115),
            rounded_rect_section(0.074, 0.034, 0.006, body_h),
        ]
    )
    body.visual(
        mesh_from_geometry(body_mesh, "router_body_shell"),
        material=body_material,
        name="body_shell",
    )

    def add_side_pod(side: int, label: str, joint_y: float, joint_z: float) -> None:
        x_sign = float(side)
        body.visual(
            Box((0.012, 0.010, 0.018)),
            origin=Origin(xyz=(x_sign * 0.0415, joint_y, joint_z)),
            material=body_material,
            name=f"{label}_pod_block",
        )
        for suffix, ear_y in (("front", joint_y + 0.0065), ("rear", joint_y - 0.0065)):
            body.visual(
                Box((0.006, 0.003, 0.018)),
                origin=Origin(xyz=(x_sign * body_joint_x, ear_y, joint_z)),
                material=body_material,
                name=f"{label}_{suffix}_ear",
            )

    add_side_pod(+1, "right_upper", upper_antenna_y, 0.178)
    add_side_pod(+1, "right_lower", lower_antenna_y, 0.152)
    add_side_pod(-1, "left_upper", upper_antenna_y, 0.178)
    add_side_pod(-1, "left_lower", lower_antenna_y, 0.152)

    body.visual(
        Box((0.022, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.0205, 0.024)),
        material=body_material,
        name="switch_bezel",
    )
    for x_center, name in ((-0.0085, "switch_left_ear"), (0.0085, "switch_right_ear")):
        body.visual(
            Box((0.003, 0.006, 0.010)),
            origin=Origin(xyz=(x_center, 0.0260, 0.024)),
            material=body_material,
            name=name,
        )

    model.articulation(
        "base_to_body",
        ArticulationType.FIXED,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, base_h)),
    )

    def add_antenna(name: str, side: int, joint_y: float, joint_z: float) -> None:
        antenna = model.part(name)
        x_sign = float(side)
        antenna.visual(
            Cylinder(radius=0.003, length=0.010),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=antenna_material,
            name="root_barrel",
        )
        antenna.visual(
            Box((0.008, 0.004, 0.014)),
            origin=Origin(xyz=(x_sign * 0.006, 0.0, 0.007)),
            material=antenna_material,
            name="neck",
        )
        antenna.visual(
            Box((0.016, 0.004, 0.095)),
            origin=Origin(xyz=(x_sign * 0.009, 0.0, 0.0615)),
            material=antenna_material,
            name="blade",
        )
        antenna.visual(
            Box((0.011, 0.004, 0.028)),
            origin=Origin(xyz=(x_sign * 0.008, 0.0, 0.123)),
            material=antenna_material,
            name="blade_tip",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=antenna,
            origin=Origin(xyz=(x_sign * body_joint_x, joint_y, joint_z)),
            axis=(0.0, x_sign, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.5,
                lower=-0.15,
                upper=1.00,
            ),
        )

    add_antenna("right_upper_antenna", +1, upper_antenna_y, 0.178)
    add_antenna("right_lower_antenna", +1, lower_antenna_y, 0.152)
    add_antenna("left_upper_antenna", -1, upper_antenna_y, 0.178)
    add_antenna("left_lower_antenna", -1, lower_antenna_y, 0.152)

    switch = model.part("front_switch")
    switch.visual(
        Cylinder(radius=0.0015, length=0.014),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=switch_material,
        name="axle",
    )
    switch.visual(
        Box((0.013, 0.006, 0.017)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=switch_material,
        name="rocker_cap",
    )
    model.articulation(
        "body_to_front_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=switch,
        origin=Origin(xyz=(0.0, 0.0260, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=-0.22,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    front_switch = object_model.get_part("front_switch")
    antenna_names = (
        "right_upper_antenna",
        "right_lower_antenna",
        "left_upper_antenna",
        "left_lower_antenna",
    )
    antenna_joints = {
        name: object_model.get_articulation(f"body_to_{name}")
        for name in antenna_names
    }
    switch_joint = object_model.get_articulation("body_to_front_switch")

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

    ctx.expect_contact(body, base, name="body_is_seated_on_base")
    for antenna_name in antenna_names:
        ctx.expect_contact(
            object_model.get_part(antenna_name),
            body,
            name=f"{antenna_name}_captured_in_pod",
        )
    ctx.expect_contact(front_switch, body, name="front_switch_captured_in_bezel")

    ctx.check(
        "antenna_joint_axes",
        {name: antenna_joints[name].axis for name in antenna_names}
        == {
            "right_upper_antenna": (0.0, 1.0, 0.0),
            "right_lower_antenna": (0.0, 1.0, 0.0),
            "left_upper_antenna": (0.0, -1.0, 0.0),
            "left_lower_antenna": (0.0, -1.0, 0.0),
        },
        details=str({name: antenna_joints[name].axis for name in antenna_names}),
    )
    ctx.check(
        "switch_joint_axis",
        switch_joint.axis == (1.0, 0.0, 0.0),
        details=str(switch_joint.axis),
    )

    with ctx.pose(
        body_to_right_upper_antenna=0.85,
        body_to_right_lower_antenna=0.60,
        body_to_left_upper_antenna=0.85,
        body_to_left_lower_antenna=0.60,
        body_to_front_switch=0.18,
    ):
        ctx.expect_gap(
            "right_upper_antenna",
            body,
            axis="x",
            min_gap=0.003,
            positive_elem="blade",
            negative_elem="body_shell",
            name="right_upper_blade_stays_outboard",
        )
        ctx.expect_gap(
            "right_lower_antenna",
            body,
            axis="x",
            min_gap=0.003,
            positive_elem="blade",
            negative_elem="body_shell",
            name="right_lower_blade_stays_outboard",
        )
        ctx.expect_gap(
            body,
            "left_upper_antenna",
            axis="x",
            min_gap=0.003,
            positive_elem="body_shell",
            negative_elem="blade",
            name="left_upper_blade_stays_outboard",
        )
        ctx.expect_gap(
            body,
            "left_lower_antenna",
            axis="x",
            min_gap=0.003,
            positive_elem="body_shell",
            negative_elem="blade",
            name="left_lower_blade_stays_outboard",
        )
        ctx.expect_gap(
            "right_upper_antenna",
            "right_lower_antenna",
            axis="y",
            min_gap=0.002,
            positive_elem="blade",
            negative_elem="blade",
            name="right_side_antennas_keep_separation",
        )
        ctx.expect_gap(
            "left_upper_antenna",
            "left_lower_antenna",
            axis="y",
            min_gap=0.002,
            positive_elem="blade",
            negative_elem="blade",
            name="left_side_antennas_keep_separation",
        )
        ctx.expect_gap(
            front_switch,
            body,
            axis="y",
            min_gap=0.0005,
            positive_elem="rocker_cap",
            negative_elem="switch_bezel",
            name="rocker_cap_sits_proud_of_bezel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
