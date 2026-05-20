from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PIN_AXIS = (0.0, 1.0, 0.0)

PIN_HOLE_RADIUS = 0.0035
EYE_RADIUS = 0.0095
EYE_THICKNESS = 0.006
SHANK_WIDTH = 0.010
FORK_CHEEK_THICKNESS = 0.006
FORK_OUTER_WIDTH = EYE_THICKNESS + 2.0 * FORK_CHEEK_THICKNESS
FORK_LUG_HEIGHT = 0.022
FORK_BRIDGE_HEIGHT = 0.006
FORK_BRIDGE_OFFSET = 0.014
LUG_WIDTH_X = 0.012

LINK_1_LENGTH = 0.076
LINK_2_LENGTH = 0.062
END_TAB_DROP = 0.028

BRACKET_TOP_WIDTH = 0.052
BRACKET_TOP_DEPTH = 0.024
BRACKET_TOP_THICKNESS = 0.006
BRACKET_TOP_CENTER_Z = 0.026
BRACKET_EAR_WIDTH = 0.016
BRACKET_EAR_THICKNESS = 0.004
BRACKET_EAR_HEIGHT = 0.028
BRACKET_EAR_CENTER_Z = 0.013
BRACKET_INNER_GAP = EYE_THICKNESS


Y_CYLINDER_RPY = (1.5707963267948966, 0.0, 0.0)


def add_y_cylinder(part, *, radius: float, length: float, xyz: tuple[float, float, float], material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=Y_CYLINDER_RPY),
        material=material,
        name=name,
    )


def add_bracket_visuals(part, material) -> None:
    ear_y = BRACKET_INNER_GAP / 2.0 + BRACKET_EAR_THICKNESS / 2.0
    part.visual(
        Box((BRACKET_TOP_WIDTH, BRACKET_TOP_DEPTH, BRACKET_TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_TOP_CENTER_Z)),
        material=material,
        name="top_plate",
    )
    part.visual(
        Box((BRACKET_EAR_WIDTH, BRACKET_EAR_THICKNESS, BRACKET_EAR_HEIGHT)),
        origin=Origin(xyz=(0.0, ear_y, BRACKET_EAR_CENTER_Z)),
        material=material,
        name="left_ear",
    )
    part.visual(
        Box((BRACKET_EAR_WIDTH, BRACKET_EAR_THICKNESS, BRACKET_EAR_HEIGHT)),
        origin=Origin(xyz=(0.0, -ear_y, BRACKET_EAR_CENTER_Z)),
        material=material,
        name="right_ear",
    )


def add_link_visuals(part, *, length: float, material, prefix: str) -> None:
    shank_length = length - FORK_BRIDGE_OFFSET
    lug_center_y = EYE_THICKNESS / 2.0 + FORK_CHEEK_THICKNESS / 2.0
    bridge_center_z = -length + 0.015
    lug_center_z = -length + 0.005

    add_y_cylinder(
        part,
        radius=EYE_RADIUS,
        length=EYE_THICKNESS,
        xyz=(0.0, 0.0, 0.0),
        material=material,
        name=f"{prefix}_top_eye",
    )
    part.visual(
        Box((SHANK_WIDTH, EYE_THICKNESS, shank_length)),
        origin=Origin(xyz=(0.0, 0.0, -shank_length / 2.0)),
        material=material,
        name=f"{prefix}_shank",
    )
    part.visual(
        Box((LUG_WIDTH_X, FORK_OUTER_WIDTH, FORK_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, bridge_center_z)),
        material=material,
        name=f"{prefix}_lower_bridge",
    )
    part.visual(
        Box((LUG_WIDTH_X, FORK_CHEEK_THICKNESS, FORK_LUG_HEIGHT)),
        origin=Origin(xyz=(0.0, lug_center_y, lug_center_z)),
        material=material,
        name=f"{prefix}_left_lug",
    )
    part.visual(
        Box((LUG_WIDTH_X, FORK_CHEEK_THICKNESS, FORK_LUG_HEIGHT)),
        origin=Origin(xyz=(0.0, -lug_center_y, lug_center_z)),
        material=material,
        name=f"{prefix}_right_lug",
    )


def add_end_tab_visuals(part, material) -> None:
    stem_height = END_TAB_DROP - 0.012
    add_y_cylinder(
        part,
        radius=EYE_RADIUS * 0.94,
        length=EYE_THICKNESS,
        xyz=(0.0, 0.0, 0.0),
        material=material,
        name="tab_eye",
    )
    part.visual(
        Box((0.010, EYE_THICKNESS, stem_height)),
        origin=Origin(xyz=(0.0, 0.0, -stem_height / 2.0)),
        material=material,
        name="tab_stem",
    )
    part.visual(
        Box((0.016, EYE_THICKNESS, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -(END_TAB_DROP - 0.006))),
        material=material,
        name="tab_flag",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_lever_chain")

    dark_bracket = model.material("bracket_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    steel = model.material("zinc_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_tab = model.material("end_tab_finish", rgba=(0.18, 0.18, 0.19, 1.0))

    bracket = model.part("support_bracket")
    add_bracket_visuals(bracket, dark_bracket)

    first_link = model.part("first_link")
    add_link_visuals(first_link, length=LINK_1_LENGTH, material=steel, prefix="first")

    second_link = model.part("second_link")
    add_link_visuals(second_link, length=LINK_2_LENGTH, material=steel, prefix="second")

    end_tab = model.part("end_tab")
    add_end_tab_visuals(end_tab, dark_tab)

    model.articulation(
        "bracket_to_first_link",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=first_link,
        origin=Origin(),
        axis=PIN_AXIS,
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=-1.45,
            upper=1.45,
        ),
    )
    model.articulation(
        "first_to_second_link",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(0.0, 0.0, -LINK_1_LENGTH)),
        axis=PIN_AXIS,
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=3.5,
            lower=-1.6,
            upper=1.6,
        ),
    )
    model.articulation(
        "second_link_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=end_tab,
        origin=Origin(xyz=(0.0, 0.0, -LINK_2_LENGTH)),
        axis=PIN_AXIS,
        motion_limits=MotionLimits(
            effort=9.0,
            velocity=4.0,
            lower=-1.75,
            upper=1.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("support_bracket")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    end_tab = object_model.get_part("end_tab")

    upper_joint = object_model.get_articulation("bracket_to_first_link")
    middle_joint = object_model.get_articulation("first_to_second_link")
    lower_joint = object_model.get_articulation("second_link_to_end_tab")

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

    ctx.expect_contact(bracket, first_link, contact_tol=1e-5, name="bracket_pin_contacts_first_link")
    ctx.expect_contact(first_link, second_link, contact_tol=1e-5, name="first_pin_contacts_second_link")
    ctx.expect_contact(second_link, end_tab, contact_tol=1e-5, name="second_pin_contacts_end_tab")

    for joint_name, joint in (
        ("upper_joint_is_revolute", upper_joint),
        ("middle_joint_is_revolute", middle_joint),
        ("lower_joint_is_revolute", lower_joint),
    ):
        ctx.check(
            joint_name,
            joint.articulation_type == ArticulationType.REVOLUTE,
            f"{joint.name} should be revolute, got {joint.articulation_type}",
        )

    expected_axis = PIN_AXIS
    for joint_name, joint in (
        ("upper_joint_axis_parallel", upper_joint),
        ("middle_joint_axis_parallel", middle_joint),
        ("lower_joint_axis_parallel", lower_joint),
    ):
        ctx.check(
            joint_name,
            tuple(round(v, 6) for v in joint.axis) == expected_axis,
            f"{joint.name} axis {joint.axis} is not parallel to expected {expected_axis}",
        )

    def z_center(part) -> float:
        aabb = ctx.part_world_aabb(part)
        assert aabb is not None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    ctx.check(
        "links_hang_below_support_in_rest_pose",
        z_center(bracket) > z_center(first_link) > z_center(second_link) > z_center(end_tab),
        "Part centers should descend from bracket to end tab in the rest pose.",
    )

    with ctx.pose({upper_joint: 0.6}):
        second_origin = ctx.part_world_position(second_link)
        end_origin = ctx.part_world_position(end_tab)
        ctx.check(
            "positive_upper_joint_swings_chain_sideways",
            second_origin is not None and second_origin[0] < -0.025,
            f"Expected second link origin to swing to negative x, got {second_origin}",
        )
        ctx.check(
            "end_tab_follows_upper_joint_motion",
            end_origin is not None and end_origin[0] < -0.025,
            f"Expected end tab origin to follow the swung chain, got {end_origin}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
