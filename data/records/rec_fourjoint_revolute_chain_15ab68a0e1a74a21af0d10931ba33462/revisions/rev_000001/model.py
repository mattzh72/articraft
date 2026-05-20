from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

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


BOSS_RADIUS = 0.0088
INNER_BOSS_RADIUS = 0.0072
CENTER_THICKNESS = 0.0080
BODY_THICKNESS = 0.0058
PLATE_THICKNESS = 0.0032
SLOT_WIDTH = CENTER_THICKNESS
OUTER_WIDTH = SLOT_WIDTH + 2.0 * PLATE_THICKNESS
PLATE_CENTER_Y = SLOT_WIDTH / 2.0 + PLATE_THICKNESS / 2.0

ROOT_PLATE_HEIGHT = 0.020
LINK_BODY_HEIGHT = 0.0138

LINK_LENGTHS = (0.078, 0.072, 0.066, 0.060, 0.054)
JOINT_LIMITS = MotionLimits(
    effort=18.0,
    velocity=2.0,
    lower=-radians(72.0),
    upper=radians(63.0),
)


def add_box(part, name: str, size_xyz: tuple[float, float, float], center_xyz: tuple[float, float, float], material):
    part.visual(Box(size_xyz), origin=Origin(xyz=center_xyz), material=material, name=name)


def add_y_cylinder(part, name: str, radius: float, length: float, center_xyz: tuple[float, float, float], material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center_xyz, rpy=(radians(90.0), 0.0, 0.0)),
        material=material,
        name=name,
    )


def author_root_bracket(part, material):
    add_box(part, "back_plate", (0.010, OUTER_WIDTH + 0.018, 0.066), (-0.033, 0.0, 0.0), material)
    add_box(part, "foot", (0.022, OUTER_WIDTH + 0.022, 0.012), (-0.022, 0.0, -0.028), material)
    add_box(part, "spine", (0.016, CENTER_THICKNESS + 0.002, 0.010), (-0.024, 0.0, 0.0), material)
    add_box(part, "upper_rib", (0.018, CENTER_THICKNESS + 0.002, 0.010), (-0.024, 0.0, 0.020), material)
    add_box(part, "lower_rib", (0.018, CENTER_THICKNESS + 0.002, 0.010), (-0.024, 0.0, -0.020), material)
    add_box(part, "upper_stop", (0.010, CENTER_THICKNESS, 0.0038), (-0.010, 0.0, ROOT_PLATE_HEIGHT / 2.0 + 0.0013), material)
    add_box(part, "left_cheek", (0.020, PLATE_THICKNESS, ROOT_PLATE_HEIGHT), (-0.009, PLATE_CENTER_Y, 0.0), material)
    add_box(part, "right_cheek", (0.020, PLATE_THICKNESS, ROOT_PLATE_HEIGHT), (-0.009, -PLATE_CENTER_Y, 0.0), material)
    add_y_cylinder(part, "left_boss", BOSS_RADIUS, PLATE_THICKNESS, (0.0, PLATE_CENTER_Y, 0.0), material)
    add_y_cylinder(part, "right_boss", BOSS_RADIUS, PLATE_THICKNESS, (0.0, -PLATE_CENTER_Y, 0.0), material)


def author_link(part, length: float, material, *, index: int, tip_link: bool = False):
    body_height = LINK_BODY_HEIGHT - 0.0006 * (index - 1)
    side_shift = 0.0036 if index % 2 else -0.0036

    add_y_cylinder(part, "proximal_boss", INNER_BOSS_RADIUS, CENTER_THICKNESS, (0.0, 0.0, 0.0), material)
    add_box(part, "proximal_pad", (0.024, CENTER_THICKNESS, body_height * 0.92), (0.012, 0.0, 0.0), material)
    add_box(part, "shoulder", (0.014, BODY_THICKNESS, body_height), (0.027, side_shift, 0.0), material)

    web_end = length - (0.010 if tip_link else 0.024)
    web_length = max(0.016, web_end - 0.034)
    web_center = 0.034 + web_length / 2.0
    add_box(part, "main_web", (web_length, BODY_THICKNESS, body_height * 0.74), (web_center, side_shift, 0.0), material)
    add_box(part, "top_rib", (web_length * 0.82, BODY_THICKNESS, 0.0036), (web_center, side_shift, body_height * 0.34), material)
    add_box(part, "proximal_stop", (0.008, CENTER_THICKNESS, 0.0036), (0.016, 0.0, body_height / 2.0 + 0.0012), material)

    if tip_link:
        add_box(part, "fork_base", (0.018, BODY_THICKNESS, body_height * 0.64), (length - 0.005, side_shift, 0.0), material)
        add_box(part, "fork_root", (0.012, BODY_THICKNESS, 0.0090), (length + 0.001, side_shift, 0.0), material)
        add_box(part, "fork_upper", (0.024, BODY_THICKNESS, 0.0028), (length + 0.013, side_shift, 0.0042), material)
        add_box(part, "fork_lower", (0.024, BODY_THICKNESS, 0.0028), (length + 0.013, side_shift, -0.0042), material)
    else:
        plate_height = max(body_height + 0.002, 0.015)
        add_box(part, "clevis_bridge", (0.010, CENTER_THICKNESS, body_height * 0.72), (length - 0.022, 0.0, 0.0), material)
        add_box(part, "left_plate", (0.022, PLATE_THICKNESS, plate_height), (length - 0.009, PLATE_CENTER_Y, 0.0), material)
        add_box(part, "right_plate", (0.022, PLATE_THICKNESS, plate_height), (length - 0.009, -PLATE_CENTER_Y, 0.0), material)
        add_y_cylinder(part, "left_boss", BOSS_RADIUS, PLATE_THICKNESS, (length, PLATE_CENTER_Y, 0.0), material)
        add_y_cylinder(part, "right_boss", BOSS_RADIUS, PLATE_THICKNESS, (length, -PLATE_CENTER_Y, 0.0), material)
        add_box(part, "distal_stop", (0.008, CENTER_THICKNESS, 0.0036), (length - 0.014, 0.0, -(body_height / 2.0 + 0.0020)), material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="accordion_hinge_arm")

    bracket_mat = model.material("bracket_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    link_mat = model.material("machined_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    tip_mat = model.material("fork_steel", rgba=(0.58, 0.60, 0.63, 1.0))

    root_bracket = model.part("root_bracket")
    author_root_bracket(root_bracket, bracket_mat)

    links = []
    for index, length in enumerate(LINK_LENGTHS, start=1):
        part = model.part(f"link_{index}")
        is_tip = index == len(LINK_LENGTHS)
        author_link(part, length, tip_mat if is_tip else link_mat, index=index, tip_link=is_tip)
        links.append(part)

    model.articulation(
        "root_to_link_1",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=links[0],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=JOINT_LIMITS,
    )

    for index, length in enumerate(LINK_LENGTHS[:-1], start=1):
        model.articulation(
            f"link_{index}_to_link_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=links[index - 1],
            child=links[index],
            origin=Origin(xyz=(length, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=JOINT_LIMITS,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_bracket")
    links = [object_model.get_part(f"link_{index}") for index in range(1, 6)]
    hinges = [
        object_model.get_articulation("root_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_link_3"),
        object_model.get_articulation("link_3_to_link_4"),
        object_model.get_articulation("link_4_to_link_5"),
    ]
    supported_pairs = [
        (root, links[0], "root_support"),
        (links[0], links[1], "hinge_1_support"),
        (links[1], links[2], "hinge_2_support"),
        (links[2], links[3], "hinge_3_support"),
        (links[3], links[4], "hinge_4_support"),
    ]

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

    for hinge in hinges:
        ctx.check(
            f"{hinge.name}_axis_family",
            tuple(hinge.axis) == (0.0, 1.0, 0.0),
            f"expected axis (0, 1, 0), got {hinge.axis}",
        )

    for parent, child, label in supported_pairs:
        ctx.expect_contact(parent, child, contact_tol=5e-5, name=f"{label}_rest_contact")

    open_pose = {
        hinges[0]: -radians(12.0),
        hinges[1]: -radians(10.0),
        hinges[2]: -radians(8.0),
        hinges[3]: -radians(6.0),
        hinges[4]: -radians(4.0),
    }
    folded_pose = {
        hinges[0]: -radians(68.8),
        hinges[1]: radians(57.3),
        hinges[2]: radians(45.8),
        hinges[3]: -radians(45.8),
        hinges[4]: -radians(34.4),
    }

    with ctx.pose(open_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_clearance")
        for parent, child, label in supported_pairs:
            ctx.expect_contact(parent, child, contact_tol=5e-5, name=f"{label}_open_contact")
        ctx.expect_origin_distance(links[-1], root, axes="x", min_dist=0.24, name="open_pose_reach")
        ctx.expect_origin_gap(links[-1], root, axis="z", min_gap=0.06, max_gap=0.18, name="open_pose_arc_height")

    with ctx.pose(folded_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_clearance")
        for parent, child, label in supported_pairs:
            ctx.expect_contact(parent, child, contact_tol=5e-5, name=f"{label}_folded_contact")
        ctx.expect_origin_distance(links[-1], root, axes="xz", max_dist=0.23, name="folded_pose_compactness")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
