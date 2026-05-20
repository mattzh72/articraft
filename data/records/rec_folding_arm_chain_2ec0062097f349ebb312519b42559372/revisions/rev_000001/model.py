from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LINK_THICKNESS = 0.004
LINK_WIDTH = 0.022
LINK_BAR_WIDTH = 0.018
BRACKET_THICKNESS = 0.006
BRACKET_WIDTH = 0.046
BRACKET_HEIGHT = 0.092
BRACKET_CENTER_X = -0.020
BRACKET_LUG_LENGTH = 0.022
BRACKET_LUG_HEIGHT = 0.018
EAR_THICKNESS = 0.004
FORK_GAP = LINK_THICKNESS
EAR_CENTER_Y = FORK_GAP / 2.0 + EAR_THICKNESS / 2.0
PIN_RADIUS = 0.0030
HOLE_RADIUS = 0.00345
PIN_SPAN = 2.0 * EAR_CENTER_Y + EAR_THICKNESS

LINK1_LENGTH = 0.140
LINK2_LENGTH = 0.118
LINK3_BAR_LENGTH = 0.098
FORK_LENGTH = 0.038
FORK_OVERLAP = 0.000
PAD_LENGTH = 0.030
PAD_HEIGHT = 0.036


def centered_box(size_x: float, size_y: float, size_z: float, center: tuple[float, float, float]):
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center)


def cylinder_y(radius: float, length: float, center: tuple[float, float, float]):
    return cq.Workplane("XZ", origin=center).circle(radius).extrude(length / 2.0, both=True)


def cylinder_x(radius: float, length: float, center: tuple[float, float, float]):
    return cq.Workplane("YZ", origin=center).circle(radius).extrude(length / 2.0, both=True)


def make_base_bracket():
    plate = (
        cq.Workplane("YZ", origin=(BRACKET_CENTER_X, 0.0, 0.0))
        .rect(BRACKET_WIDTH, BRACKET_HEIGHT)
        .extrude(BRACKET_THICKNESS / 2.0, both=True)
    )
    mount_holes = (
        cq.Workplane("YZ", origin=(BRACKET_CENTER_X, 0.0, 0.0))
        .pushPoints([(0.0, -0.028), (0.0, 0.028)])
        .circle(0.0032)
        .extrude(BRACKET_THICKNESS, both=True)
    )
    plate = plate.cut(mount_holes)

    lug_center_x = (BRACKET_CENTER_X + BRACKET_THICKNESS / 2.0 + 0.0) / 2.0
    lug_length = 0.0 - (BRACKET_CENTER_X + BRACKET_THICKNESS / 2.0)
    bracket = plate
    for sign in (-1.0, 1.0):
        lug = centered_box(
            lug_length,
            EAR_THICKNESS,
            BRACKET_LUG_HEIGHT,
            (lug_center_x, sign * EAR_CENTER_Y, 0.0),
        )
        bracket = bracket.union(lug)

    pivot_pin = cylinder_y(PIN_RADIUS, PIN_SPAN, center=(0.0, 0.0, 0.0))
    return bracket.union(pivot_pin)


def make_fork_link(length: float):
    fork_start = length - FORK_LENGTH
    cheek_start = fork_start - FORK_OVERLAP
    cheek_length = length - cheek_start
    cheek_center_x = cheek_start + cheek_length / 2.0

    link = centered_box(fork_start, LINK_THICKNESS, LINK_WIDTH, (fork_start / 2.0, 0.0, 0.0))
    link = link.union(cylinder_y(LINK_WIDTH / 2.0, LINK_THICKNESS, center=(0.0, 0.0, 0.0)))

    for sign in (-1.0, 1.0):
        cheek = centered_box(
            cheek_length,
            EAR_THICKNESS,
            LINK_WIDTH,
            (cheek_center_x, sign * EAR_CENTER_Y, 0.0),
        )
        end_cap = cylinder_y(LINK_WIDTH / 2.0, EAR_THICKNESS, center=(length, sign * EAR_CENTER_Y, 0.0))
        link = link.union(cheek).union(end_cap)

    proximal_hole = cylinder_y(HOLE_RADIUS, LINK_THICKNESS * 3.0, center=(0.0, 0.0, 0.0))
    distal_pin = cylinder_y(PIN_RADIUS, PIN_SPAN, center=(length, 0.0, 0.0))
    return link.cut(proximal_hole).union(distal_pin)


def make_terminal_link():
    bar = centered_box(LINK3_BAR_LENGTH, LINK_THICKNESS, LINK_WIDTH, (LINK3_BAR_LENGTH / 2.0, 0.0, 0.0))
    bar = bar.union(cylinder_y(LINK_WIDTH / 2.0, LINK_THICKNESS, center=(0.0, 0.0, 0.0)))
    bar = bar.union(cylinder_y(LINK_WIDTH / 2.0, LINK_THICKNESS, center=(LINK3_BAR_LENGTH, 0.0, 0.0)))

    neck_length = 0.010
    neck_center_x = LINK3_BAR_LENGTH + neck_length / 2.0
    pad_center_x = LINK3_BAR_LENGTH + neck_length + PAD_LENGTH / 2.0
    neck = centered_box(neck_length, LINK_THICKNESS, LINK_WIDTH * 0.78, (neck_center_x, 0.0, 0.0))
    pad = centered_box(PAD_LENGTH, LINK_THICKNESS, PAD_HEIGHT, (pad_center_x, 0.0, 0.0))
    link = bar.union(neck).union(pad)

    hole_cuts = cylinder_y(HOLE_RADIUS, LINK_THICKNESS * 3.0, center=(0.0, 0.0, 0.0))
    hole_cuts = hole_cuts.union(
        cq.Workplane("XZ", origin=(pad_center_x, 0.0, 0.0))
        .pushPoints([(0.0, -0.010), (0.0, 0.010)])
        .circle(0.0026)
        .extrude(LINK_THICKNESS * 1.5, both=True)
    )
    return link.cut(hole_cuts)


def _axis_is_parallel_to_y(axis: tuple[float, float, float]) -> bool:
    return abs(axis[0]) < 1e-9 and abs(axis[1] - 1.0) < 1e-9 and abs(axis[2]) < 1e-9


def _combined_x_span(ctx: TestContext, parts: list[object]) -> tuple[float, float] | None:
    mins: list[float] = []
    maxs: list[float] = []
    for part in parts:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        mins.append(aabb[0][0])
        maxs.append(aabb[1][0])
    return min(mins), max(maxs)


def add_box_visual(
    part_obj,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material,
    name: str,
):
    part_obj.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_support_arm")

    dark_bracket = model.material("dark_bracket", rgba=(0.18, 0.18, 0.20, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.71, 0.73, 0.76, 1.0))

    bracket = model.part("base_bracket")
    add_box_visual(
        bracket,
        size=(BRACKET_THICKNESS, BRACKET_WIDTH, BRACKET_HEIGHT),
        center=(BRACKET_CENTER_X, 0.0, 0.0),
        material=dark_bracket,
        name="bracket_plate",
    )
    lug_center_x = -0.009
    add_box_visual(
        bracket,
        size=(BRACKET_LUG_LENGTH, EAR_THICKNESS, BRACKET_LUG_HEIGHT),
        center=(lug_center_x, EAR_CENTER_Y, 0.0),
        material=dark_bracket,
        name="bracket_ear_upper",
    )
    add_box_visual(
        bracket,
        size=(BRACKET_LUG_LENGTH, EAR_THICKNESS, BRACKET_LUG_HEIGHT),
        center=(lug_center_x, -EAR_CENTER_Y, 0.0),
        material=dark_bracket,
        name="bracket_ear_lower",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.042, 0.046, BRACKET_HEIGHT)),
        mass=0.22,
        origin=Origin(xyz=(-0.011, 0.0, 0.0)),
    )

    link1 = model.part("link_1")
    add_box_visual(
        link1,
        size=(0.110, LINK_THICKNESS, LINK_BAR_WIDTH),
        center=(0.055, 0.0, 0.0),
        material=zinc_steel,
        name="link_1_bar",
    )
    add_box_visual(
        link1,
        size=(FORK_LENGTH, EAR_THICKNESS, LINK_WIDTH),
        center=(LINK1_LENGTH - 0.017, EAR_CENTER_Y, 0.0),
        material=zinc_steel,
        name="link_1_fork_upper",
    )
    add_box_visual(
        link1,
        size=(FORK_LENGTH, EAR_THICKNESS, LINK_WIDTH),
        center=(LINK1_LENGTH - 0.017, -EAR_CENTER_Y, 0.0),
        material=zinc_steel,
        name="link_1_fork_lower",
    )
    link1.inertial = Inertial.from_geometry(
        Box((LINK1_LENGTH, PIN_SPAN, LINK_BAR_WIDTH)),
        mass=0.12,
        origin=Origin(xyz=(LINK1_LENGTH / 2.0, 0.0, 0.0)),
    )

    link2 = model.part("link_2")
    add_box_visual(
        link2,
        size=(LINK2_LENGTH, LINK_THICKNESS, LINK_BAR_WIDTH),
        center=(LINK2_LENGTH / 2.0, 0.0, 0.0),
        material=zinc_steel,
        name="link_2_bar",
    )
    add_box_visual(
        link2,
        size=(FORK_LENGTH, EAR_THICKNESS, LINK_WIDTH),
        center=(LINK2_LENGTH - 0.017, EAR_CENTER_Y, 0.0),
        material=zinc_steel,
        name="link_2_fork_upper",
    )
    add_box_visual(
        link2,
        size=(FORK_LENGTH, EAR_THICKNESS, LINK_WIDTH),
        center=(LINK2_LENGTH - 0.017, -EAR_CENTER_Y, 0.0),
        material=zinc_steel,
        name="link_2_fork_lower",
    )
    link2.inertial = Inertial.from_geometry(
        Box((LINK2_LENGTH, PIN_SPAN, LINK_BAR_WIDTH)),
        mass=0.10,
        origin=Origin(xyz=(LINK2_LENGTH / 2.0, 0.0, 0.0)),
    )

    link3 = model.part("link_3")
    add_box_visual(
        link3,
        size=(LINK3_BAR_LENGTH, LINK_THICKNESS, LINK_BAR_WIDTH),
        center=(LINK3_BAR_LENGTH / 2.0, 0.0, 0.0),
        material=zinc_steel,
        name="link_3_bar",
    )
    add_box_visual(
        link3,
        size=(0.010, LINK_THICKNESS, LINK_BAR_WIDTH * 0.78),
        center=(LINK3_BAR_LENGTH + 0.005, 0.0, 0.0),
        material=zinc_steel,
        name="link_3_neck",
    )
    add_box_visual(
        link3,
        size=(PAD_LENGTH, LINK_THICKNESS, PAD_HEIGHT),
        center=(LINK3_BAR_LENGTH + 0.010 + PAD_LENGTH / 2.0, 0.0, 0.0),
        material=zinc_steel,
        name="attachment_pad",
    )
    link3.inertial = Inertial.from_geometry(
        Box((LINK3_BAR_LENGTH + PAD_LENGTH + 0.010, LINK_THICKNESS, PAD_HEIGHT)),
        mass=0.09,
        origin=Origin(xyz=((LINK3_BAR_LENGTH + PAD_LENGTH + 0.010) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "bracket_to_link_1",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=2.15),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(LINK2_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.2, lower=0.0, upper=2.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("base_bracket")
    link1 = object_model.get_part("link_1")
    link2 = object_model.get_part("link_2")
    link3 = object_model.get_part("link_3")
    joint_1 = object_model.get_articulation("bracket_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")
    joint_3 = object_model.get_articulation("link_2_to_link_3")

    ctx.allow_overlap(
        bracket,
        link1,
        reason="Simplified hinge knuckle envelope at the base joint; the flat-bar link is modeled as occupying the bracket fork's idealized pivot zone.",
    )
    ctx.allow_overlap(
        link1,
        link2,
        reason="Simplified hinge knuckle envelope at the middle joint; nested fork cheeks and the next link share an idealized pivot volume.",
    )
    ctx.allow_overlap(
        link2,
        link3,
        reason="Simplified hinge knuckle envelope at the terminal joint; the final pad link occupies the idealized revolute pivot zone.",
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
        "all_joint_axes_parallel_y",
        _axis_is_parallel_to_y(joint_1.axis)
        and _axis_is_parallel_to_y(joint_2.axis)
        and _axis_is_parallel_to_y(joint_3.axis),
        details=f"axes were {joint_1.axis}, {joint_2.axis}, {joint_3.axis}",
    )
    ctx.check(
        "joint_limits_support_fold_and_extension",
        (
            joint_1.motion_limits is not None
            and joint_2.motion_limits is not None
            and joint_3.motion_limits is not None
            and joint_1.motion_limits.lower is not None
            and joint_1.motion_limits.upper is not None
            and joint_2.motion_limits.lower is not None
            and joint_2.motion_limits.upper is not None
            and joint_3.motion_limits.lower is not None
            and joint_3.motion_limits.upper is not None
            and joint_1.motion_limits.lower == 0.0
            and joint_2.motion_limits.lower == 0.0
            and joint_3.motion_limits.lower == 0.0
            and joint_1.motion_limits.upper > 1.2
            and joint_2.motion_limits.upper > 1.6
            and joint_3.motion_limits.upper > 1.5
        ),
        details="The stay should rest in an extended pose at 0 rad and fold substantially in one consistent direction.",
    )

    ctx.expect_contact(bracket, link1, name="bracket_pin_contacts_link_1")
    ctx.expect_contact(link1, link2, name="link_1_pin_contacts_link_2")
    ctx.expect_contact(link2, link3, name="link_2_pin_contacts_link_3")

    default_span = _combined_x_span(ctx, [bracket, link1, link2, link3])
    ctx.check(
        "extended_pose_reads_as_long_support_arm",
        default_span is not None and (default_span[1] - default_span[0]) > 0.34,
        details=f"default x span was {None if default_span is None else default_span[1] - default_span[0]:.4f}",
    )

    articulated_pairs = (
        ("joint_1", joint_1, bracket, link1),
        ("joint_2", joint_2, link1, link2),
        ("joint_3", joint_3, link2, link3),
    )
    for label, joint, parent, child in articulated_pairs:
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_lower_no_floating")
                ctx.expect_contact(parent, child, name=f"{label}_lower_parent_child_contact")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_upper_no_floating")
                ctx.expect_contact(parent, child, name=f"{label}_upper_parent_child_contact")

    folded_pose = {joint_1: 1.30, joint_2: 1.90, joint_3: 1.70}
    with ctx.pose(folded_pose):
        folded_span = _combined_x_span(ctx, [bracket, link1, link2, link3])
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="folded_pose_no_floating")
        ctx.expect_contact(bracket, link1, name="folded_pose_bracket_link_1_contact")
        ctx.expect_contact(link1, link2, name="folded_pose_link_1_link_2_contact")
        ctx.expect_contact(link2, link3, name="folded_pose_link_2_link_3_contact")
        ctx.check(
            "folded_pose_reduces_reach",
            default_span is not None
            and folded_span is not None
            and (folded_span[1] - folded_span[0]) < (default_span[1] - default_span[0]) - 0.12,
            details=(
                f"default span={None if default_span is None else default_span[1] - default_span[0]:.4f}, "
                f"folded span={None if folded_span is None else folded_span[1] - folded_span[0]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
