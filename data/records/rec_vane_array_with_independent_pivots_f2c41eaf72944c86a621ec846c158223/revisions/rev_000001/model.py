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


FRAME_OUTER_WIDTH = 1.04
FRAME_OUTER_HEIGHT = 0.78
FRAME_OPENING_WIDTH = 0.90
FRAME_OPENING_HEIGHT = 0.66
FRAME_DEPTH = 0.028
FRAME_CENTER_Y = -0.026

SLAT_COUNT = 7
SLAT_PITCH = 0.092
SLAT_SPAN = FRAME_OPENING_WIDTH

JOURNAL_RADIUS = 0.006
JOURNAL_LENGTH = 0.056
SLAT_BLADE_MARGIN = 0.026
SLAT_BLADE_LENGTH = SLAT_SPAN - 2.0 * SLAT_BLADE_MARGIN
SLAT_BLADE_DEPTH = 0.024
SLAT_BLADE_HEIGHT = 0.064
SLAT_BLADE_CENTER_Y = 0.008

BRACKET_WIDTH = 0.030
BRACKET_DEPTH = 0.038
BRACKET_HEIGHT = 0.040
BRACKET_CENTER_Y = -0.010
SUPPORT_MOUNT_LENGTH = 0.018
SUPPORT_BOSS_LENGTH = 0.010
SUPPORT_BOSS_RADIUS = 0.016
SUPPORT_RIB_DEPTH = 0.016
SUPPORT_RIB_HEIGHT = 0.010
SUPPORT_RIB_Z_OFFSET = 0.013


def slat_axis_z(index: int) -> float:
    return (SLAT_COUNT - 1) * SLAT_PITCH * 0.5 - index * SLAT_PITCH


def make_frame_ring() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .box(FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)
        .translate((0.0, FRAME_CENTER_Y, 0.0))
    )
    opening = (
        cq.Workplane("XY")
        .box(FRAME_OPENING_WIDTH, FRAME_DEPTH + 0.010, FRAME_OPENING_HEIGHT)
        .translate((0.0, FRAME_CENTER_Y, 0.0))
    )
    return ring.cut(opening)


def make_support_bracket(axis_x: float, axis_z: float, side: str) -> cq.Workplane:
    direction = -1.0 if side == "left" else 1.0
    frame_outer_x = direction * (0.5 * FRAME_OUTER_WIDTH)
    contact_x = axis_x + direction * (0.5 * JOURNAL_LENGTH)
    mount_center_x = frame_outer_x - direction * (0.5 * SUPPORT_MOUNT_LENGTH)
    boss_start_x = contact_x
    rib_end_x = contact_x + direction * SUPPORT_BOSS_LENGTH
    rib_start_x = frame_outer_x - direction * SUPPORT_MOUNT_LENGTH
    rib_center_x = 0.5 * (rib_start_x + rib_end_x)
    rib_length = abs(rib_end_x - rib_start_x)

    mount = (
        cq.Workplane("XY")
        .box(SUPPORT_MOUNT_LENGTH, BRACKET_DEPTH, BRACKET_HEIGHT)
        .translate((mount_center_x, BRACKET_CENTER_Y, axis_z))
    )
    top_rib = (
        cq.Workplane("XY")
        .box(rib_length, SUPPORT_RIB_DEPTH, SUPPORT_RIB_HEIGHT)
        .translate((rib_center_x, BRACKET_CENTER_Y, axis_z + SUPPORT_RIB_Z_OFFSET))
    )
    bottom_rib = (
        cq.Workplane("XY")
        .box(rib_length, SUPPORT_RIB_DEPTH, SUPPORT_RIB_HEIGHT)
        .translate((rib_center_x, BRACKET_CENTER_Y, axis_z - SUPPORT_RIB_Z_OFFSET))
    )
    boss = cq.Workplane(
        obj=cq.Solid.makeCylinder(
            SUPPORT_BOSS_RADIUS,
            SUPPORT_BOSS_LENGTH,
            cq.Vector(boss_start_x, BRACKET_CENTER_Y, axis_z),
            cq.Vector(direction, 0.0, 0.0),
        )
    )
    return mount.union(top_rib).union(bottom_rib).union(boss)


def make_slat() -> cq.Workplane:
    blade = (
        cq.Workplane("YZ")
        .center(SLAT_BLADE_CENTER_Y, 0.0)
        .ellipse(SLAT_BLADE_DEPTH * 0.5, SLAT_BLADE_HEIGHT * 0.5)
        .extrude(SLAT_BLADE_LENGTH)
        .translate((SLAT_BLADE_MARGIN, 0.0, 0.0))
    )
    left_journal = (
        cq.Workplane("YZ")
        .circle(JOURNAL_RADIUS)
        .extrude(JOURNAL_LENGTH)
        .translate((-0.5 * JOURNAL_LENGTH, 0.0, 0.0))
    )
    right_journal = (
        cq.Workplane("YZ")
        .circle(JOURNAL_RADIUS)
        .extrude(JOURNAL_LENGTH)
        .translate((SLAT_SPAN - 0.5 * JOURNAL_LENGTH, 0.0, 0.0))
    )
    return blade.union(left_journal).union(right_journal)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_louver_array")

    frame_dark = model.material("frame_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    support_dark = model.material("support_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    slat_aluminum = model.material("slat_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_frame_ring(), "frame_ring"),
        material=frame_dark,
        name="frame_ring",
    )

    left_axis_x = -0.5 * SLAT_SPAN
    right_axis_x = 0.5 * SLAT_SPAN

    for index in range(SLAT_COUNT):
        axis_z = slat_axis_z(index)
        frame.visual(
            mesh_from_cadquery(
                make_support_bracket(left_axis_x, axis_z, "left"),
                f"left_support_{index + 1}",
            ),
            material=support_dark,
            name=f"left_support_{index + 1}",
        )
        frame.visual(
            mesh_from_cadquery(
                make_support_bracket(right_axis_x, axis_z, "right"),
                f"right_support_{index + 1}",
            ),
            material=support_dark,
            name=f"right_support_{index + 1}",
        )

    for index in range(SLAT_COUNT):
        slat = model.part(f"slat_{index + 1}")
        slat.visual(
            mesh_from_cadquery(make_slat(), f"slat_{index + 1}"),
            material=slat_aluminum,
            name="slat_body",
        )
        model.articulation(
            f"frame_to_slat_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(-0.5 * SLAT_SPAN, 0.0, slat_axis_z(index))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.5,
                lower=-0.7,
                upper=0.7,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    frame_ring = frame.get_visual("frame_ring")
    slats = [object_model.get_part(f"slat_{index + 1}") for index in range(SLAT_COUNT)]
    joints = [
        object_model.get_articulation(f"frame_to_slat_{index + 1}")
        for index in range(SLAT_COUNT)
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

    ctx.check(
        "frame plus independent slat count matches prompt",
        len(object_model.parts) == SLAT_COUNT + 1 and len(object_model.articulations) == SLAT_COUNT,
        details=f"parts={len(object_model.parts)}, articulations={len(object_model.articulations)}",
    )

    for index, (slat, joint) in enumerate(zip(slats, joints), start=1):
        slat_body = slat.get_visual("slat_body")
        left_support = frame.get_visual(f"left_support_{index}")
        right_support = frame.get_visual(f"right_support_{index}")

        ctx.expect_contact(
            slat,
            frame,
            elem_a=slat_body,
            elem_b=left_support,
            name=f"slat {index} contacts left fork support",
        )
        ctx.expect_contact(
            slat,
            frame,
            elem_a=slat_body,
            elem_b=right_support,
            name=f"slat {index} contacts right fork support",
        )
        ctx.expect_gap(
            slat,
            frame,
            axis="y",
            positive_elem=slat_body,
            negative_elem=frame_ring,
            min_gap=0.006,
            name=f"slat {index} stays forward of rear frame ring",
        )
        limits = joint.motion_limits
        ctx.check(
            f"slat {index} uses a revolute span axis",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.axis == (1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    for upper, lower, pair_index in zip(slats[:-1], slats[1:], range(1, SLAT_COUNT)):
        ctx.expect_origin_gap(
            upper,
            lower,
            axis="z",
            min_gap=SLAT_PITCH - 0.0005,
            max_gap=SLAT_PITCH + 0.0005,
            name=f"slat pair {pair_index}-{pair_index + 1} keeps regular stack pitch",
        )

    mid_index = SLAT_COUNT // 2
    mid_slat = slats[mid_index]
    mid_joint = joints[mid_index]
    neighbor_slat = slats[mid_index - 1]

    rest_mid_aabb = ctx.part_world_aabb(mid_slat)
    rest_neighbor_aabb = ctx.part_world_aabb(neighbor_slat)
    with ctx.pose({mid_joint: 0.55}):
        opened_mid_aabb = ctx.part_world_aabb(mid_slat)
        opened_neighbor_aabb = ctx.part_world_aabb(neighbor_slat)
        ctx.expect_gap(
            neighbor_slat,
            mid_slat,
            axis="z",
            min_gap=0.004,
            name="opened middle slat still clears the slat above",
        )

    mid_opens_upward = (
        rest_mid_aabb is not None
        and opened_mid_aabb is not None
        and opened_mid_aabb[1][2] > rest_mid_aabb[1][2] + 0.004
    )
    ctx.check(
        "positive slat rotation lifts the visible blade edge",
        mid_opens_upward,
        details=f"rest={rest_mid_aabb}, opened={opened_mid_aabb}",
    )

    neighbor_unchanged = (
        rest_neighbor_aabb is not None
        and opened_neighbor_aabb is not None
        and all(
            abs(a - b) <= 1e-9
            for a, b in zip(
                rest_neighbor_aabb[0] + rest_neighbor_aabb[1],
                opened_neighbor_aabb[0] + opened_neighbor_aabb[1],
            )
        )
    )
    ctx.check(
        "opening one slat leaves adjacent slat in place",
        neighbor_unchanged,
        details=f"rest={rest_neighbor_aabb}, opened={opened_neighbor_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
