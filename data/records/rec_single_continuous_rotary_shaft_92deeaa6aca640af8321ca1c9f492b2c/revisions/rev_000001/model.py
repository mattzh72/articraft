from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_rotary_shaft")

    plate_w = 0.18
    plate_h = 0.22
    plate_t = 0.014
    mount_hole_d = 0.011

    rail_w = 0.020
    rail_h = 0.052
    rail_x = 0.038
    rail_front_y = 0.106
    rail_len = rail_front_y - plate_t

    shaft_d = 0.018
    bearing_t = 0.016
    rear_bearing_y = 0.035
    front_bearing_y = 0.080
    bearing_bridge_w = 2.0 * rail_x + rail_w
    bearing_pad_h = 0.015
    bearing_pad_z = shaft_d / 2.0 + 0.002 + bearing_pad_h / 2.0

    shaft_rear_y = rear_bearing_y - bearing_t / 2.0
    shaft_len = 0.112
    collar_d = 0.034
    collar_t = 0.006
    collar_rear_local_y = bearing_t
    hub_d = 0.036
    hub_t = 0.010
    hub_rear_local_y = 0.088
    faceplate_d = 0.110
    faceplate_t = 0.012
    faceplate_rear_local_y = 0.100
    half_pi = 1.5707963267948966

    support_color = model.material("support_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    bearing_color = model.material("bearing_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    rotor_color = model.material("rotor_steel", rgba=(0.63, 0.65, 0.69, 1.0))

    backplate_shape = (
        cq.Workplane("XY")
        .box(plate_w, plate_t, plate_h)
        .translate((0.0, plate_t / 2.0, 0.0))
        .edges("|Y")
        .fillet(0.009)
        .faces(">Y")
        .workplane()
        .pushPoints(
            [
                (-plate_w * 0.34, -plate_h * 0.36),
                (-plate_w * 0.34, plate_h * 0.36),
                (plate_w * 0.34, -plate_h * 0.36),
                (plate_w * 0.34, plate_h * 0.36),
            ]
        )
        .hole(mount_hole_d)
    )
    support = model.part("support")
    support.visual(
        mesh_from_cadquery(backplate_shape, "support_backplate"),
        material=support_color,
        name="backplate",
    )
    support.visual(
        Box((rail_w, rail_len, rail_h)),
        origin=Origin(xyz=(rail_x, plate_t + rail_len / 2.0, 0.0)),
        material=support_color,
        name="left_rail",
    )
    support.visual(
        Box((rail_w, rail_len, rail_h)),
        origin=Origin(xyz=(-rail_x, plate_t + rail_len / 2.0, 0.0)),
        material=support_color,
        name="right_rail",
    )
    support.visual(
        Box((bearing_bridge_w, bearing_t, bearing_pad_h)),
        origin=Origin(xyz=(0.0, rear_bearing_y, bearing_pad_z)),
        material=bearing_color,
        name="rear_bearing_top",
    )
    support.visual(
        Box((bearing_bridge_w, bearing_t, bearing_pad_h)),
        origin=Origin(xyz=(0.0, rear_bearing_y, -bearing_pad_z)),
        material=bearing_color,
        name="rear_bearing_bottom",
    )
    support.visual(
        Box((bearing_bridge_w, bearing_t, bearing_pad_h)),
        origin=Origin(xyz=(0.0, front_bearing_y, bearing_pad_z)),
        material=bearing_color,
        name="front_bearing_top",
    )
    support.visual(
        Box((bearing_bridge_w, bearing_t, bearing_pad_h)),
        origin=Origin(xyz=(0.0, front_bearing_y, -bearing_pad_z)),
        material=bearing_color,
        name="front_bearing_bottom",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=shaft_d / 2.0, length=shaft_len),
        origin=Origin(xyz=(0.0, shaft_len / 2.0, 0.0), rpy=(half_pi, 0.0, 0.0)),
        material=rotor_color,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=collar_d / 2.0, length=collar_t),
        origin=Origin(
            xyz=(0.0, collar_rear_local_y + collar_t / 2.0, 0.0),
            rpy=(half_pi, 0.0, 0.0),
        ),
        material=rotor_color,
        name="rear_collar",
    )
    rotor.visual(
        Cylinder(radius=hub_d / 2.0, length=hub_t),
        origin=Origin(
            xyz=(0.0, hub_rear_local_y + hub_t / 2.0, 0.0),
            rpy=(half_pi, 0.0, 0.0),
        ),
        material=rotor_color,
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=faceplate_d / 2.0, length=faceplate_t),
        origin=Origin(
            xyz=(0.0, faceplate_rear_local_y + faceplate_t / 2.0, 0.0),
            rpy=(half_pi, 0.0, 0.0),
        ),
        material=rotor_color,
        name="faceplate",
    )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=rotor,
        origin=Origin(xyz=(0.0, shaft_rear_y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    rotor = object_model.get_part("rotor")
    shaft_spin = object_model.get_articulation("shaft_spin")

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
        "shaft_joint_is_continuous_about_centerline",
        shaft_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in shaft_spin.axis) == (0.0, 1.0, 0.0),
        f"expected continuous +Y spin axis, got {shaft_spin.articulation_type} axis={shaft_spin.axis}",
    )
    ctx.expect_contact(
        rotor,
        support,
        elem_a="rear_collar",
        elem_b="rear_bearing_top",
        name="rear_collar_seats_on_rear_bearing_top_face",
    )
    ctx.expect_contact(
        rotor,
        support,
        elem_a="rear_collar",
        elem_b="rear_bearing_bottom",
        name="rear_collar_seats_on_rear_bearing_bottom_face",
    )
    ctx.expect_gap(
        support,
        rotor,
        axis="z",
        positive_elem="front_bearing_top",
        negative_elem="shaft",
        min_gap=0.0015,
        max_gap=0.003,
        name="front_top_bearing_pad_clears_shaft",
    )
    ctx.expect_gap(
        rotor,
        support,
        axis="z",
        positive_elem="shaft",
        negative_elem="front_bearing_bottom",
        min_gap=0.0015,
        max_gap=0.003,
        name="front_bottom_bearing_pad_clears_shaft",
    )
    ctx.expect_gap(
        rotor,
        support,
        axis="y",
        positive_elem="faceplate",
        negative_elem="backplate",
        min_gap=0.09,
        name="faceplate_projects_forward_of_wall_mount",
    )

    with ctx.pose({shaft_spin: 1.4}):
        ctx.expect_gap(
            rotor,
            support,
            axis="y",
            positive_elem="faceplate",
            negative_elem="backplate",
            min_gap=0.09,
            name="faceplate_standoff_persists_while_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
