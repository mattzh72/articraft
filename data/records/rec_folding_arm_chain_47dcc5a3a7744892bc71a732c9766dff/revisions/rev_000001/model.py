from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BAR_WIDTH = 0.020
BAR_THICKNESS = 0.0030
BRACKET_THICKNESS = 0.0035
SHOE_THICKNESS = 0.0032

PIN_RADIUS = 0.0028
HOLE_RADIUS = 0.00325
WASHER_RADIUS = 0.0063
PIN_HEAD_RADIUS = 0.0046
PIN_HEAD_HEIGHT = 0.0010

LINK_A_LENGTH = 0.165
LINK_B_LENGTH = 0.125
LINK_C_LENGTH = 0.095
STACK_OFFSET = 0.0042

FOLDED_POSE = {
    "root_to_link_a": 2.72,
    "link_a_to_link_b": -2.78,
    "link_b_to_link_c": 2.74,
}


def _through_holes(points: list[tuple[float, float]], radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .pushPoints(points)
        .circle(radius)
        .extrude(0.03, both=True)
    )


def _pin_feature(
    *,
    parent_top: float,
    child_mid_z: float,
    child_thickness: float,
) -> cq.Workplane:
    child_bottom = child_mid_z - child_thickness / 2.0
    child_top = child_mid_z + child_thickness / 2.0

    shank = (
        cq.Workplane("XY")
        .circle(PIN_RADIUS)
        .extrude(child_top + PIN_HEAD_HEIGHT - parent_top)
        .translate((0.0, 0.0, parent_top))
    )
    head = (
        cq.Workplane("XY")
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_HEIGHT)
        .translate((0.0, 0.0, child_top))
    )
    pin = shank.union(head)

    washer_height = child_bottom - parent_top
    if washer_height > 1e-6:
        washer = (
            cq.Workplane("XY")
            .circle(WASHER_RADIUS)
            .extrude(washer_height)
            .translate((0.0, 0.0, parent_top))
        )
        pin = washer.union(pin)

    return pin


def _root_bracket() -> cq.Workplane:
    plate_center_z = -(BAR_THICKNESS / 2.0 + BRACKET_THICKNESS / 2.0)
    plate = (
        cq.Workplane("XY")
        .center(-0.023, 0.0)
        .slot2D(0.070, 0.024)
        .extrude(BRACKET_THICKNESS)
        .translate((0.0, 0.0, plate_center_z))
    )

    mount_slots = (
        cq.Workplane("XY")
        .pushPoints([(-0.018, 0.0), (-0.034, 0.0)])
        .slot2D(0.010, 0.005)
        .extrude(0.03, both=True)
    )
    plate = plate.cut(mount_slots)

    root_pin = _pin_feature(
        parent_top=-BAR_THICKNESS / 2.0,
        child_mid_z=0.0,
        child_thickness=BAR_THICKNESS,
    )
    return plate.union(root_pin)


def _bar_link_with_distal_pin(length: float) -> cq.Workplane:
    bar = (
        cq.Workplane("XY")
        .center(length / 2.0, 0.0)
        .slot2D(length + BAR_WIDTH, BAR_WIDTH)
        .extrude(BAR_THICKNESS)
        .translate((0.0, 0.0, -BAR_THICKNESS / 2.0))
    )
    bar = bar.cut(_through_holes([(0.0, 0.0)], HOLE_RADIUS))

    distal_pin = _pin_feature(
        parent_top=BAR_THICKNESS / 2.0,
        child_mid_z=STACK_OFFSET,
        child_thickness=BAR_THICKNESS,
    ).translate((length, 0.0, 0.0))
    return bar.union(distal_pin)


def _terminal_bar_link(length: float) -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .center(length / 2.0, 0.0)
        .rect(length, BAR_WIDTH)
        .extrude(BAR_THICKNESS)
        .translate((0.0, 0.0, -BAR_THICKNESS / 2.0))
    )
    eye = (
        cq.Workplane("XY")
        .circle(BAR_WIDTH / 2.0)
        .extrude(BAR_THICKNESS)
        .translate((0.0, 0.0, -BAR_THICKNESS / 2.0))
    )
    link = body.union(eye)
    return link.cut(_through_holes([(0.0, 0.0)], HOLE_RADIUS))


def _end_shoe() -> cq.Workplane:
    points = [
        (0.0, -0.009),
        (0.016, -0.009),
        (0.034, -0.017),
        (0.050, 0.0),
        (0.034, 0.017),
        (0.016, 0.009),
        (0.0, 0.009),
    ]
    shoe = (
        cq.Workplane("XY")
        .polyline(points)
        .close()
        .extrude(SHOE_THICKNESS)
        .translate((0.0, 0.0, -SHOE_THICKNESS / 2.0))
    )
    hole_cutters = _through_holes([(0.020, -0.007), (0.020, 0.007)], 0.0025)
    return shoe.cut(hole_cutters)


def _axis_is_z(axis: tuple[float, float, float] | None) -> bool:
    if axis is None:
        return False
    return all(abs(a - b) < 1e-9 for a, b in zip(axis, (0.0, 0.0, 1.0)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_support_stay")

    zinc = model.material("zinc_plated", rgba=(0.73, 0.75, 0.78, 1.0))
    steel = model.material("tempered_steel", rgba=(0.46, 0.48, 0.51, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_root_bracket(), "root_bracket"),
        material=zinc,
        name="bracket_shell",
    )

    link_a = model.part("link_a")
    link_a.visual(
        mesh_from_cadquery(_bar_link_with_distal_pin(LINK_A_LENGTH), "link_a"),
        material=steel,
        name="link_a_shell",
    )

    link_b = model.part("link_b")
    link_b.visual(
        mesh_from_cadquery(_bar_link_with_distal_pin(LINK_B_LENGTH), "link_b"),
        material=steel,
        name="link_b_shell",
    )

    link_c = model.part("link_c")
    link_c.visual(
        mesh_from_cadquery(_terminal_bar_link(LINK_C_LENGTH), "link_c"),
        material=steel,
        name="link_c_shell",
    )

    end_shoe = model.part("end_shoe")
    end_shoe.visual(
        mesh_from_cadquery(_end_shoe(), "end_shoe"),
        material=zinc,
        name="shoe_shell",
    )

    model.articulation(
        "root_to_link_a",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=link_a,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.5,
            lower=0.0,
            upper=2.95,
        ),
    )
    model.articulation(
        "link_a_to_link_b",
        ArticulationType.REVOLUTE,
        parent=link_a,
        child=link_b,
        origin=Origin(xyz=(LINK_A_LENGTH, 0.0, STACK_OFFSET)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.5,
            lower=-2.95,
            upper=0.0,
        ),
    )
    model.articulation(
        "link_b_to_link_c",
        ArticulationType.REVOLUTE,
        parent=link_b,
        child=link_c,
        origin=Origin(xyz=(LINK_B_LENGTH, 0.0, STACK_OFFSET)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.5,
            lower=0.0,
            upper=2.95,
        ),
    )
    model.articulation(
        "link_c_to_end_shoe",
        ArticulationType.FIXED,
        parent=link_c,
        child=end_shoe,
        origin=Origin(xyz=(LINK_C_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    expected_parts = {
        "root_bracket",
        "link_a",
        "link_b",
        "link_c",
        "end_shoe",
    }
    expected_joints = {
        "root_to_link_a",
        "link_a_to_link_b",
        "link_b_to_link_c",
        "link_c_to_end_shoe",
    }

    root_bracket = object_model.get_part("root_bracket")
    link_a = object_model.get_part("link_a")
    link_b = object_model.get_part("link_b")
    link_c = object_model.get_part("link_c")
    end_shoe = object_model.get_part("end_shoe")

    root_to_link_a = object_model.get_articulation("root_to_link_a")
    link_a_to_link_b = object_model.get_articulation("link_a_to_link_b")
    link_b_to_link_c = object_model.get_articulation("link_b_to_link_c")
    link_c_to_end_shoe = object_model.get_articulation("link_c_to_end_shoe")

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
        "expected_parts_present",
        {part.name for part in object_model.parts} == expected_parts,
        details=f"found parts: {[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "expected_joints_present",
        {joint.name for joint in object_model.articulations} == expected_joints,
        details=f"found joints: {[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "single_root_bracket_root",
        {part.name for part in object_model.root_parts()} == {"root_bracket"},
        details=f"root parts: {[part.name for part in object_model.root_parts()]}",
    )
    ctx.check(
        "revolute_axes_are_coplanar_z",
        _axis_is_z(root_to_link_a.axis)
        and _axis_is_z(link_a_to_link_b.axis)
        and _axis_is_z(link_b_to_link_c.axis),
        details=(
            f"axes: {root_to_link_a.axis}, "
            f"{link_a_to_link_b.axis}, {link_b_to_link_c.axis}"
        ),
    )

    ctx.expect_contact(root_bracket, link_a, name="root_bracket_contacts_link_a")
    ctx.expect_contact(link_a, link_b, name="link_a_contacts_link_b")
    ctx.expect_contact(link_b, link_c, name="link_b_contacts_link_c")
    ctx.expect_contact(link_c, end_shoe, name="link_c_contacts_end_shoe")

    ctx.expect_origin_gap(
        end_shoe,
        root_bracket,
        axis="x",
        min_gap=0.36,
        name="extended_arm_reaches_outward",
    )

    with ctx.pose(FOLDED_POSE):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_pose")
        ctx.expect_overlap(
            link_a,
            link_b,
            axes="xy",
            min_overlap=0.015,
            name="folded_link_a_and_link_b_overlap_in_plan",
        )
        ctx.expect_overlap(
            link_b,
            link_c,
            axes="xy",
            min_overlap=0.015,
            name="folded_link_b_and_link_c_overlap_in_plan",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
