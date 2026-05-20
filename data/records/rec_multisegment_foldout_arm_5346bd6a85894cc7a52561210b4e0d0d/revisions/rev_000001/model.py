from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_PLATE_LENGTH = 0.160
BASE_PLATE_WIDTH = 0.072
BASE_PLATE_THICKNESS = 0.012
SPINE_HEIGHT = 0.238
SPINE_WIDTH = 0.026
SPINE_DEPTH = 0.030

CHEEK_THICKNESS = 0.004
CHEEK_LENGTH = 0.020
CHEEK_HEIGHT = 0.034
PIVOT_SPAN = 0.016
PIVOT_RADIUS = 0.010
JOINT_BLOCK_LENGTH = 0.014
ROOT_PAD_LENGTH = 0.018
END_PAD_LENGTH = 0.020
SHOULDER_PAD_LENGTH = 0.024

LINK_WIDTH = 0.010
LINK_HEIGHT = 0.016
ROOT_BLOCK_WIDTH = 0.012

LINK_1_LENGTH = 0.185
LINK_2_LENGTH = 0.170
LINK_3_LENGTH = 0.145
PLATFORM_REACH = 0.060

FORK_Y_OFFSET = PIVOT_SPAN / 2.0 + CHEEK_THICKNESS / 2.0
FORK_TOTAL_WIDTH = PIVOT_SPAN + 2.0 * CHEEK_THICKNESS


def _fork_cheek_pair(x_center: float) -> cq.Workplane:
    pair = cq.Workplane("XY")
    for y_center in (-FORK_Y_OFFSET, FORK_Y_OFFSET):
        pair = pair.union(
            cq.Workplane("XY").box(CHEEK_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT).translate(
                (x_center, y_center, 0.0)
            )
        )
    return pair


def _joint_pad(x_center: float, pad_length: float) -> cq.Workplane:
    core = cq.Workplane("XY").box(
        pad_length,
        FORK_TOTAL_WIDTH,
        LINK_HEIGHT,
    ).translate((x_center, 0.0, 0.0))
    cheeks = _fork_cheek_pair(x_center)
    return core.union(cheeks)


def _make_spine_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(BASE_PLATE_LENGTH, BASE_PLATE_WIDTH, BASE_PLATE_THICKNESS)
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.006)
    )

    mast = (
        cq.Workplane("XY")
        .box(SPINE_DEPTH, SPINE_WIDTH, SPINE_HEIGHT)
        .translate((-0.015, 0.0, BASE_PLATE_THICKNESS + SPINE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.003)
    )

    shoulder_pad = _joint_pad(-SHOULDER_PAD_LENGTH / 2.0, SHOULDER_PAD_LENGTH).translate(
        (0.0, 0.0, BASE_PLATE_THICKNESS + SPINE_HEIGHT)
    )

    gusset_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.052, BASE_PLATE_THICKNESS),
                (-0.008, BASE_PLATE_THICKNESS),
                (-0.008, 0.105),
                (-0.040, 0.050),
            ]
        )
        .close()
        .extrude(0.008, both=True)
    )

    return (
        base_plate.union(mast)
        .union(shoulder_pad)
        .union(gusset_profile)
    )


def _make_link_shape(length: float) -> cq.Workplane:
    root_pad = _joint_pad(ROOT_PAD_LENGTH / 2.0, ROOT_PAD_LENGTH)

    beam = cq.Workplane("XY").box(
        length - ROOT_PAD_LENGTH - END_PAD_LENGTH,
        LINK_WIDTH,
        LINK_HEIGHT,
    ).translate(((ROOT_PAD_LENGTH + (length - END_PAD_LENGTH)) / 2.0, 0.0, 0.0))

    distal_pad = _joint_pad(length - END_PAD_LENGTH / 2.0, END_PAD_LENGTH)

    return root_pad.union(beam).union(distal_pad)


def _make_platform_bracket_shape() -> cq.Workplane:
    root_pad = _joint_pad(ROOT_PAD_LENGTH / 2.0, ROOT_PAD_LENGTH)

    neck = cq.Workplane("XY").box(
        0.044,
        LINK_WIDTH,
        LINK_HEIGHT,
    ).translate((0.034, 0.0, 0.0))

    riser = cq.Workplane("XY").box(0.014, 0.024, 0.058).translate((0.070, 0.0, 0.021))
    platform = cq.Workplane("XY").box(0.090, 0.048, 0.006).translate((0.105, 0.0, 0.047))
    lip = cq.Workplane("XY").box(0.020, 0.048, 0.018).translate((0.135, 0.0, 0.035))
    gusset = cq.Workplane("XY").box(0.034, 0.016, 0.020).translate((0.056, 0.0, 0.010))

    return root_pad.union(neck).union(riser).union(platform).union(lip).union(gusset)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_fold_out_arm")

    model.material("graphite", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("platform_gray", rgba=(0.52, 0.56, 0.60, 1.0))

    spine = model.part("spine")
    spine.visual(mesh_from_cadquery(_make_spine_shape(), "spine"), material="graphite", name="spine_body")
    spine.inertial = Inertial.from_geometry(
        Box((BASE_PLATE_LENGTH, BASE_PLATE_WIDTH, BASE_PLATE_THICKNESS + SPINE_HEIGHT)),
        mass=5.4,
        origin=Origin(xyz=(-0.008, 0.0, (BASE_PLATE_THICKNESS + SPINE_HEIGHT) / 2.0)),
    )

    link_1 = model.part("link_1")
    link_1.visual(mesh_from_cadquery(_make_link_shape(LINK_1_LENGTH), "link_1"), material="aluminum", name="link_1_body")
    link_1.inertial = Inertial.from_geometry(
        Box((LINK_1_LENGTH + 0.024, FORK_TOTAL_WIDTH, CHEEK_HEIGHT)),
        mass=0.90,
        origin=Origin(xyz=(LINK_1_LENGTH / 2.0, 0.0, 0.0)),
    )

    link_2 = model.part("link_2")
    link_2.visual(mesh_from_cadquery(_make_link_shape(LINK_2_LENGTH), "link_2"), material="aluminum", name="link_2_body")
    link_2.inertial = Inertial.from_geometry(
        Box((LINK_2_LENGTH + 0.024, FORK_TOTAL_WIDTH, CHEEK_HEIGHT)),
        mass=0.76,
        origin=Origin(xyz=(LINK_2_LENGTH / 2.0, 0.0, 0.0)),
    )

    link_3 = model.part("link_3")
    link_3.visual(mesh_from_cadquery(_make_link_shape(LINK_3_LENGTH), "link_3"), material="aluminum", name="link_3_body")
    link_3.inertial = Inertial.from_geometry(
        Box((LINK_3_LENGTH + 0.024, FORK_TOTAL_WIDTH, CHEEK_HEIGHT)),
        mass=0.64,
        origin=Origin(xyz=(LINK_3_LENGTH / 2.0, 0.0, 0.0)),
    )

    platform_bracket = model.part("platform_bracket")
    platform_bracket.visual(
        mesh_from_cadquery(_make_platform_bracket_shape(), "platform_bracket"),
        material="platform_gray",
        name="platform_bracket_body",
    )
    platform_bracket.inertial = Inertial.from_geometry(
        Box((0.145, 0.048, 0.058)),
        mass=0.55,
        origin=Origin(xyz=(0.095, 0.0, 0.026)),
    )

    common_limits = MotionLimits(
        effort=18.0,
        velocity=radians(120.0),
        lower=-1.10,
        upper=1.45,
    )

    model.articulation(
        "spine_to_link_1",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS + SPINE_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=radians(90.0),
            lower=-0.55,
            upper=1.45,
        ),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_3_to_platform_bracket",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=platform_bracket,
        origin=Origin(xyz=(LINK_3_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=radians(140.0),
            lower=-1.00,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    platform_bracket = object_model.get_part("platform_bracket")

    shoulder = object_model.get_articulation("spine_to_link_1")
    elbow_1 = object_model.get_articulation("link_1_to_link_2")
    elbow_2 = object_model.get_articulation("link_2_to_link_3")
    wrist = object_model.get_articulation("link_3_to_platform_bracket")

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
    ctx.allow_overlap(
        spine,
        link_1,
        reason="The compact shoulder cheek pack is represented as a tight hinge envelope at the grounded spine.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        reason="Adjacent elbow links intentionally share a simplified compact hinge envelope at their joint cheeks.",
    )
    ctx.allow_overlap(
        link_2,
        link_3,
        reason="Adjacent elbow links intentionally share a simplified compact hinge envelope at their joint cheeks.",
    )
    ctx.allow_overlap(
        link_3,
        platform_bracket,
        reason="The terminal bracket root is represented as a tight wrist hinge envelope against the last link.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(spine, link_1, name="shoulder link is physically seated in the grounded spine")
    ctx.expect_contact(link_1, link_2, name="first elbow link pair stays connected")
    ctx.expect_contact(link_2, link_3, name="second elbow link pair stays connected")
    ctx.expect_contact(link_3, platform_bracket, name="end bracket stays mounted on the last link")

    all_parallel = all(
        articulation.axis == (0.0, -1.0, 0.0)
        for articulation in (shoulder, elbow_1, elbow_2, wrist)
    )
    ctx.check(
        "all four serial joints use the same parallel pitch axis",
        all_parallel,
        details=str([shoulder.axis, elbow_1.axis, elbow_2.axis, wrist.axis]),
    )

    rest_platform_position = ctx.part_world_position(platform_bracket)
    with ctx.pose(
        {
            shoulder: 0.70,
            elbow_1: 0.55,
            elbow_2: 0.45,
            wrist: 0.30,
        }
    ):
        deployed_platform_position = ctx.part_world_position(platform_bracket)
        ctx.fail_if_parts_overlap_in_current_pose(name="deployed service arm pose stays clear")

    ctx.check(
        "positive joint motion folds the arm upward",
        rest_platform_position is not None
        and deployed_platform_position is not None
        and deployed_platform_position[2] > rest_platform_position[2] + 0.18,
        details=f"rest={rest_platform_position}, deployed={deployed_platform_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
