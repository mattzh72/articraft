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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


BASE_L = 0.220
BASE_W = 0.062
BASE_T = 0.012

ROD_Z = 0.045
ROD_RADIUS = 0.006
ROD_HOME_X = -0.060
ROD_STROKE = 0.055
ROD_BODY_L = 0.136
ROD_COLLAR_R = 0.0085
ROD_COLLAR_L = 0.010
ROD_COLLAR_X = 0.081
ROD_NOSE_R = 0.006
ROD_NOSE_X = 0.136

PIN_X = 0.108
PIN_Z = 0.050
HUB_R = 0.007
HUB_L = 0.026
SHAFT_R = 0.005
SHAFT_L = 0.014
LEVER_OPEN = 0.70


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][2] + aabb[1][2]) / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_pushrod_chain")

    model.material("painted_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("machined_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("dark_oxide", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("amber_cap", rgba=(0.82, 0.52, 0.18, 1.0))

    support = model.part("rear_support")
    support.visual(
        Box((BASE_L, BASE_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="painted_steel",
        name="base_plate",
    )
    support.visual(
        Box((0.014, 0.010, 0.054)),
        origin=Origin(xyz=(-0.072, 0.012, 0.039)),
        material="painted_steel",
        name="rear_post_left",
    )
    support.visual(
        Box((0.014, 0.010, 0.054)),
        origin=Origin(xyz=(-0.072, -0.012, 0.039)),
        material="painted_steel",
        name="rear_post_right",
    )
    support.visual(
        Box((0.014, 0.034, 0.010)),
        origin=Origin(xyz=(-0.072, 0.0, 0.068)),
        material="painted_steel",
        name="rear_bridge_cap",
    )
    support.visual(
        Box((0.090, 0.008, 0.018)),
        origin=Origin(xyz=(-0.020, 0.010, ROD_Z)),
        material="painted_steel",
        name="guide_rail_left",
    )
    support.visual(
        Box((0.090, 0.008, 0.018)),
        origin=Origin(xyz=(-0.020, -0.010, ROD_Z)),
        material="painted_steel",
        name="guide_rail_right",
    )
    support.visual(
        Box((0.146, 0.016, 0.010)),
        origin=Origin(xyz=(0.002, 0.0, 0.068)),
        material="painted_steel",
        name="top_bridge",
    )
    support.visual(
        Box((0.010, 0.014, 0.014)),
        origin=Origin(xyz=(0.031, 0.0, 0.057)),
        material="painted_steel",
        name="bridge_stop",
    )
    support.visual(
        Box((0.036, 0.012, 0.024)),
        origin=Origin(xyz=(0.090, 0.014, 0.056)),
        material="painted_steel",
        name="fork_cheek_left",
    )
    support.visual(
        Box((0.036, 0.012, 0.024)),
        origin=Origin(xyz=(0.090, -0.014, 0.056)),
        material="painted_steel",
        name="fork_cheek_right",
    )
    support.visual(
        Box((0.018, 0.008, 0.032)),
        origin=Origin(xyz=(PIN_X, 0.017, PIN_Z)),
        material="painted_steel",
        name="ear_left",
    )
    support.visual(
        Box((0.018, 0.008, 0.032)),
        origin=Origin(xyz=(PIN_X, -0.017, PIN_Z)),
        material="painted_steel",
        name="ear_right",
    )
    support.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(PIN_X, 0.024, PIN_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="pin_end_left",
    )
    support.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(PIN_X, -0.024, PIN_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="pin_end_right",
    )
    support.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.080)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    rod = model.part("pushrod")
    rod.visual(
        Cylinder(radius=ROD_RADIUS, length=ROD_BODY_L),
        origin=Origin(xyz=(ROD_BODY_L / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="rod_shaft",
    )
    rod.visual(
        Cylinder(radius=ROD_COLLAR_R, length=ROD_COLLAR_L),
        origin=Origin(xyz=(ROD_COLLAR_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="rod_collar",
    )
    rod.visual(
        Sphere(radius=ROD_NOSE_R),
        origin=Origin(xyz=(ROD_NOSE_X, 0.0, 0.0)),
        material="machined_steel",
        name="rod_nose",
    )
    rod.inertial = Inertial.from_geometry(
        Box((ROD_BODY_L + 0.010, 0.018, 0.018)),
        mass=0.32,
        origin=Origin(xyz=((ROD_BODY_L + 0.010) / 2.0, 0.0, 0.0)),
    )

    lever = model.part("front_lever")
    lever.visual(
        Cylinder(radius=HUB_R, length=HUB_L),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_oxide",
        name="lever_hub",
    )
    lever.visual(
        Box((0.048, 0.007, 0.012)),
        origin=Origin(xyz=(0.024, 0.0, 0.006)),
        material="dark_oxide",
        name="lever_blade",
    )
    lever.visual(
        Box((0.020, 0.007, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, -0.006)),
        material="dark_oxide",
        name="lever_tail",
    )
    lever.visual(
        Box((0.010, 0.010, 0.008)),
        origin=Origin(xyz=(0.046, 0.0, 0.014)),
        material="amber_cap",
        name="lever_tip",
    )
    lever.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_L),
        origin=Origin(xyz=(-0.017, 0.0, -0.006), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="cam_roller",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.070, 0.040, 0.022)),
        mass=0.18,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_pushrod",
        ArticulationType.PRISMATIC,
        parent=support,
        child=rod,
        origin=Origin(xyz=(ROD_HOME_X, 0.0, ROD_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=ROD_STROKE, effort=80.0, velocity=0.25),
    )
    model.articulation(
        "support_to_lever",
        ArticulationType.REVOLUTE,
        parent=support,
        child=lever,
        origin=Origin(xyz=(PIN_X, 0.0, PIN_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.82, effort=12.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("rear_support")
    rod = object_model.get_part("pushrod")
    lever = object_model.get_part("front_lever")
    rod_joint = object_model.get_articulation("support_to_pushrod")
    lever_joint = object_model.get_articulation("support_to_lever")
    guide_rail_left = support.get_visual("guide_rail_left")
    bridge_stop = support.get_visual("bridge_stop")
    ear_left = support.get_visual("ear_left")
    rod_shaft = rod.get_visual("rod_shaft")
    rod_collar = rod.get_visual("rod_collar")
    rod_nose = rod.get_visual("rod_nose")
    lever_hub = lever.get_visual("lever_hub")
    lever_tip = lever.get_visual("lever_tip")
    cam_roller = lever.get_visual("cam_roller")

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
        "pushrod joint translates along +X",
        rod_joint.axis == (1.0, 0.0, 0.0),
        details=f"unexpected pushrod axis {rod_joint.axis}",
    )
    ctx.check(
        "lever joint rotates on transverse Y pin",
        lever_joint.axis == (0.0, -1.0, 0.0),
        details=f"unexpected lever axis {lever_joint.axis}",
    )
    ctx.expect_contact(
        lever,
        support,
        elem_a=lever_hub,
        elem_b=ear_left,
        name="lever hub bears against fork ear",
    )
    with ctx.pose({rod_joint: 0.0, lever_joint: 0.0}):
        ctx.expect_contact(
            rod,
            support,
            elem_a=rod_shaft,
            elem_b=guide_rail_left,
            name="pushrod shaft rides on guide rail",
        )
        ctx.expect_contact(
            rod,
            support,
            elem_a=rod_collar,
            elem_b=bridge_stop,
            name="pushrod collar seats against bridge stop",
        )
        ctx.expect_overlap(
            lever,
            rod,
            axes="yz",
            min_overlap=0.009,
            elem_a=cam_roller,
            elem_b=rod_nose,
            name="pushrod nose aligns with lever roller",
        )
        ctx.expect_gap(
            lever,
            rod,
            axis="x",
            positive_elem=cam_roller,
            negative_elem=rod_nose,
            min_gap=0.002,
            max_gap=0.008,
            name="pushrod nose sits just behind lever roller",
        )

    rod_home_pos = ctx.part_world_position(rod)
    with ctx.pose({rod_joint: ROD_STROKE}):
        rod_extended_pos = ctx.part_world_position(rod)
    rod_slides_forward = (
        rod_home_pos is not None
        and rod_extended_pos is not None
        and rod_extended_pos[0] > rod_home_pos[0] + 0.045
    )
    ctx.check(
        "pushrod extends forward through its stroke",
        rod_slides_forward,
        details=f"home={rod_home_pos}, extended={rod_extended_pos}",
    )

    tip_home = ctx.part_element_world_aabb(lever, elem=lever_tip)
    with ctx.pose({lever_joint: LEVER_OPEN}):
        tip_open = ctx.part_element_world_aabb(lever, elem=lever_tip)
    tip_home_z = _aabb_center_z(tip_home)
    tip_open_z = _aabb_center_z(tip_open)
    lever_tip_rises = (
        tip_home_z is not None and tip_open_z is not None and tip_open_z > tip_home_z + 0.015
    )
    ctx.check(
        "positive lever rotation lifts the front tip",
        lever_tip_rises,
        details=f"tip_z_home={tip_home_z}, tip_z_open={tip_open_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
