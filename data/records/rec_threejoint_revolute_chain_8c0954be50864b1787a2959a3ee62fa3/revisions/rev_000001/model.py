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


ROOT_PIVOT_Z = 0.095

PLATE_T = 0.008
CLEVIS_GAP = 0.026
TONGUE_W = 0.018
PLATE_OFFSET = CLEVIS_GAP / 2.0 + PLATE_T / 2.0
OUTER_WIDTH = CLEVIS_GAP + 2.0 * PLATE_T

PLATE_H = 0.040
TONGUE_H = 0.024
BRIDGE_H = 0.018
PROX_TONGUE_LEN = 0.030
SIDE_PLATE_START = 0.046
CLEVIS_LEN = 0.020
BOLT_HEAD_R = 0.010
JOINT_GAP = 0.0

LINK1_LEN = 0.210
LINK2_LEN = 0.180
TIP_LEN = 0.082


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    x, y, z = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((x, y, z))


def _y_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    x, y, z = center
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((x, y - length / 2.0, z))
    )


def _make_root_bracket_shape() -> cq.Workplane:
    base = _box((0.118, 0.086, 0.012), (-0.060, 0.0, 0.006))
    spine = _box((0.030, 0.058, ROOT_PIVOT_Z - 0.020), (-0.057, 0.0, 0.044))
    shoulder = _box((0.050, OUTER_WIDTH, 0.020), (-0.025, 0.0, ROOT_PIVOT_Z - 0.016))
    gusset = _box((0.026, OUTER_WIDTH, 0.016), (-0.037, 0.0, ROOT_PIVOT_Z - 0.032))

    shape = base.union(spine).union(shoulder).union(gusset)

    for sign in (-1.0, 1.0):
        y_pos = sign * PLATE_OFFSET
        shape = shape.union(_box((0.024, PLATE_T, PLATE_H), (-0.012, y_pos, ROOT_PIVOT_Z)))

    return shape.clean()


def _make_long_link_shape(length: float) -> cq.Workplane:
    start_x = JOINT_GAP
    end_x = length
    plate_span = end_x - start_x
    plate_center_x = start_x + plate_span / 2.0

    shape = _box((0.024, TONGUE_W, TONGUE_H), (start_x + 0.012, 0.0, 0.0))
    shape = shape.union(_box((0.026, OUTER_WIDTH, BRIDGE_H), (start_x + 0.028, 0.0, 0.0)))

    for sign in (-1.0, 1.0):
        y_pos = sign * PLATE_OFFSET
        shape = shape.union(_box((plate_span, PLATE_T, PLATE_H), (plate_center_x, y_pos, 0.0)))

    shape = shape.union(_box((0.012, OUTER_WIDTH, 0.014), (length * 0.36, 0.0, -0.006)))
    shape = shape.union(_box((0.010, OUTER_WIDTH, 0.012), (length * 0.66, 0.0, 0.006)))
    shape = shape.union(_box((CLEVIS_LEN, OUTER_WIDTH, BRIDGE_H), (end_x - CLEVIS_LEN / 2.0, 0.0, -0.010)))
    shape = shape.union(_box((0.016, OUTER_WIDTH, 0.016), (end_x - 0.010, 0.0, 0.0)))
    return shape.clean()


def _make_tip_shape() -> cq.Workplane:
    start_x = JOINT_GAP
    nose_x = TIP_LEN - 0.010
    shape = _box((0.024, TONGUE_W, TONGUE_H), (start_x + 0.012, 0.0, 0.0))
    shape = shape.union(_box((0.022, OUTER_WIDTH, BRIDGE_H), (start_x + 0.026, 0.0, 0.0)))
    shape = shape.union(_box((0.050, 0.026, 0.020), (0.056, 0.0, 0.0)))
    shape = shape.union(_y_cylinder(0.010, 0.024, (nose_x, 0.0, 0.0)))
    shape = shape.cut(_y_cylinder(0.0045, 0.030, (nose_x, 0.0, 0.0)))
    return shape.clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_three_joint_chain")

    model.material("powder_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("satin_aluminum", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("tool_black", rgba=(0.12, 0.13, 0.14, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_make_root_bracket_shape(), "root_bracket_chain_v2"),
        material="powder_steel",
        name="bracket_shell",
    )
    root_bracket.inertial = Inertial.from_geometry(
        Box((0.112, 0.082, 0.106)),
        mass=1.6,
        origin=Origin(xyz=(-0.030, 0.0, 0.053)),
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_make_long_link_shape(LINK1_LEN), "link_1_chain_v2"),
        material="satin_aluminum",
        name="link_1_shell",
    )
    link_1.inertial = Inertial.from_geometry(
        Box((LINK1_LEN, OUTER_WIDTH, PLATE_H)),
        mass=0.68,
        origin=Origin(xyz=(LINK1_LEN / 2.0, 0.0, 0.0)),
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_make_long_link_shape(LINK2_LEN), "link_2_chain_v2"),
        material="satin_aluminum",
        name="link_2_shell",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((LINK2_LEN, OUTER_WIDTH, PLATE_H)),
        mass=0.54,
        origin=Origin(xyz=(LINK2_LEN / 2.0, 0.0, 0.0)),
    )

    tip_tab = model.part("tip_tab")
    tip_tab.visual(
        mesh_from_cadquery(_make_tip_shape(), "tip_tab_chain_v2"),
        material="tool_black",
        name="tip_shell",
    )
    tip_tab.inertial = Inertial.from_geometry(
        Box((TIP_LEN, OUTER_WIDTH, 0.028)),
        mass=0.20,
        origin=Origin(xyz=(TIP_LEN / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "root_to_link_1",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, ROOT_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=24.0, velocity=1.6),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK1_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=18.0, velocity=1.8),
    )
    model.articulation(
        "link_2_to_tip_tab",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=tip_tab,
        origin=Origin(xyz=(LINK2_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=10.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    tip_tab = object_model.get_part("tip_tab")

    root_to_link_1 = object_model.get_articulation("root_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_tip_tab = object_model.get_articulation("link_2_to_tip_tab")

    tip_shell = tip_tab.get_visual("tip_shell")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=2e-4)
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
        "three serial revolute joints use parallel axes",
        root_to_link_1.axis == link_1_to_link_2.axis == link_2_to_tip_tab.axis == (0.0, -1.0, 0.0),
        (
            f"axes: {root_to_link_1.axis}, {link_1_to_link_2.axis}, "
            f"{link_2_to_tip_tab.axis}"
        ),
    )

    with ctx.pose({root_to_link_1: 0.0, link_1_to_link_2: 0.0, link_2_to_tip_tab: 0.0}):
        ctx.expect_contact(
            root_bracket,
            link_1,
            contact_tol=2e-4,
            name="root bracket supports first link in rest pose",
        )
        ctx.expect_contact(
            link_1,
            link_2,
            contact_tol=2e-4,
            name="first and second links stay seated at elbow in rest pose",
        )
        ctx.expect_contact(
            link_2,
            tip_tab,
            contact_tol=2e-4,
            name="second link supports compact tip tab in rest pose",
        )

    def _z_center_in_pose(joint_positions: dict[object, float]) -> float | None:
        with ctx.pose(joint_positions):
            aabb = ctx.part_element_world_aabb(tip_tab, elem=tip_shell)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_tip_z = _z_center_in_pose(
        {root_to_link_1: 0.0, link_1_to_link_2: 0.0, link_2_to_tip_tab: 0.0}
    )
    shoulder_tip_z = _z_center_in_pose(
        {root_to_link_1: 0.70, link_1_to_link_2: 0.0, link_2_to_tip_tab: 0.0}
    )
    elbow_tip_z = _z_center_in_pose(
        {root_to_link_1: 0.0, link_1_to_link_2: 0.70, link_2_to_tip_tab: 0.0}
    )
    wrist_tip_z = _z_center_in_pose(
        {root_to_link_1: 0.0, link_1_to_link_2: 0.0, link_2_to_tip_tab: 0.70}
    )

    ctx.check(
        "positive root joint raises distal chain",
        rest_tip_z is not None and shoulder_tip_z is not None and shoulder_tip_z > rest_tip_z + 0.12,
        f"rest_tip_z={rest_tip_z}, shoulder_tip_z={shoulder_tip_z}",
    )
    ctx.check(
        "positive middle joint raises tip",
        rest_tip_z is not None and elbow_tip_z is not None and elbow_tip_z > rest_tip_z + 0.07,
        f"rest_tip_z={rest_tip_z}, elbow_tip_z={elbow_tip_z}",
    )
    ctx.check(
        "positive wrist joint pitches compact tip upward",
        rest_tip_z is not None and wrist_tip_z is not None and wrist_tip_z > rest_tip_z + 0.02,
        f"rest_tip_z={rest_tip_z}, wrist_tip_z={wrist_tip_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
