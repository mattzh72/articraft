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


PLATE_THICKNESS = 0.004
HINGE_HOLE_RADIUS = 0.0044

FOOT_LENGTH = 0.112
FOOT_WIDTH = 0.036
FOOT_EYE_RADIUS = 0.016

PRIMARY_LINK_LENGTH = 0.152
PRIMARY_LINK_EYE_RADIUS = 0.015
PRIMARY_LINK_BODY_WIDTH = 0.018

SECONDARY_LINK_LENGTH = 0.144
SECONDARY_LINK_EYE_RADIUS = 0.014
SECONDARY_LINK_BODY_WIDTH = 0.017

NOSE_LINK_LENGTH = 0.076
NOSE_LINK_EYE_RADIUS = 0.013
NOSE_LINK_BODY_WIDTH = 0.015
NOSE_LINK_PAD_RADIUS = 0.008

HOOK_TAB_LENGTH = 0.050
HOOK_TAB_WIDTH = 0.014
HOOK_OUTER_RADIUS = 0.0115
HOOK_INNER_RADIUS = 0.0062

PACKED_POSE = {
    "foot_to_primary_link": 2.58,
    "primary_to_secondary_link": -2.44,
    "secondary_to_nose_link": 1.92,
}

EXTENDED_POSE = {
    "foot_to_primary_link": 0.30,
    "primary_to_secondary_link": -0.24,
    "secondary_to_nose_link": 0.18,
}


def _rect_plate(x0: float, x1: float, width: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center((x0 + x1) * 0.5, 0.0)
        .rect(x1 - x0, width)
        .extrude(thickness)
    )


def _disc(x: float, radius: float, thickness: float, y: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").center(x, y).circle(radius).extrude(thickness)


def _through_holes(
    points: list[tuple[float, float]], radius: float, thickness: float
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .pushPoints(points)
        .circle(radius)
        .extrude(thickness + 0.002)
        .translate((0.0, 0.0, -0.001))
    )


def make_grounded_foot() -> cq.Workplane:
    rear_cap_center = -FOOT_LENGTH + FOOT_WIDTH * 0.5
    body = _rect_plate(rear_cap_center, 0.0, FOOT_WIDTH, PLATE_THICKNESS)
    foot = body.union(_disc(0.0, FOOT_EYE_RADIUS, PLATE_THICKNESS)).union(
        _disc(rear_cap_center, FOOT_WIDTH * 0.5, PLATE_THICKNESS)
    )
    holes = _through_holes(
        [
            (0.0, 0.0),
            (-0.038, 0.0),
            (-0.079, 0.0),
        ],
        HINGE_HOLE_RADIUS,
        PLATE_THICKNESS,
    )
    return foot.cut(holes)


def make_long_link(
    length: float,
    eye_radius: float,
    body_width: float,
    thickness: float,
    slot_ratio: float,
) -> cq.Workplane:
    link = (
        _disc(0.0, eye_radius, thickness)
        .union(_disc(length, eye_radius, thickness))
        .union(_rect_plate(0.0, length, body_width, thickness))
    )
    slot = (
        cq.Workplane("XY")
        .center(length * 0.5, 0.0)
        .slot2D(length * slot_ratio, body_width * 0.55)
        .extrude(thickness + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    return link.cut(_through_holes([(0.0, 0.0), (length, 0.0)], HINGE_HOLE_RADIUS, thickness)).cut(slot)


def make_nose_link() -> cq.Workplane:
    distal_cap_center = NOSE_LINK_LENGTH - NOSE_LINK_PAD_RADIUS
    nose = (
        _disc(0.0, NOSE_LINK_EYE_RADIUS, PLATE_THICKNESS)
        .union(_rect_plate(0.0, distal_cap_center, NOSE_LINK_BODY_WIDTH, PLATE_THICKNESS))
        .union(_disc(distal_cap_center, NOSE_LINK_PAD_RADIUS, PLATE_THICKNESS))
    )
    slot = (
        cq.Workplane("XY")
        .center(NOSE_LINK_LENGTH * 0.43, 0.0)
        .slot2D(NOSE_LINK_LENGTH * 0.28, NOSE_LINK_BODY_WIDTH * 0.48)
        .extrude(PLATE_THICKNESS + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    return nose.cut(_through_holes([(0.0, 0.0)], HINGE_HOLE_RADIUS, PLATE_THICKNESS)).cut(slot)


def make_hook_tab() -> cq.Workplane:
    hook_center_x = HOOK_TAB_LENGTH - HOOK_OUTER_RADIUS
    shank = _rect_plate(0.0, hook_center_x, HOOK_TAB_WIDTH, PLATE_THICKNESS)
    outer_hook = _disc(hook_center_x, HOOK_OUTER_RADIUS, PLATE_THICKNESS)
    hook = shank.union(outer_hook)

    inner_relief = _disc(
        hook_center_x + 0.0015,
        HOOK_INNER_RADIUS,
        PLATE_THICKNESS + 0.002,
        y=0.0015,
    ).translate((0.0, 0.0, -0.001))
    throat_cut = (
        cq.Workplane("XY")
        .center(hook_center_x + 0.006, HOOK_OUTER_RADIUS * 0.55)
        .rect(HOOK_OUTER_RADIUS * 1.25, HOOK_OUTER_RADIUS * 0.95)
        .extrude(PLATE_THICKNESS + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    heel_trim = (
        cq.Workplane("XY")
        .center(hook_center_x - 0.002, -HOOK_OUTER_RADIUS * 0.72)
        .circle(HOOK_OUTER_RADIUS * 0.55)
        .extrude(PLATE_THICKNESS + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    return hook.cut(inner_relief).cut(throat_cut).cut(heel_trim)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_folding_bracket_chain")

    black_oxide = model.material("black_oxide", rgba=(0.19, 0.19, 0.20, 1.0))
    plated_steel = model.material("plated_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    grounded_foot = model.part("grounded_foot")
    grounded_foot.visual(
        mesh_from_cadquery(make_grounded_foot(), "grounded_foot_mesh"),
        material=black_oxide,
        name="foot_shell",
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        mesh_from_cadquery(
            make_long_link(
                PRIMARY_LINK_LENGTH,
                PRIMARY_LINK_EYE_RADIUS,
                PRIMARY_LINK_BODY_WIDTH,
                PLATE_THICKNESS,
                slot_ratio=0.44,
            ),
            "primary_link_mesh",
        ),
        material=plated_steel,
        name="primary_link_shell",
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        mesh_from_cadquery(
            make_long_link(
                SECONDARY_LINK_LENGTH,
                SECONDARY_LINK_EYE_RADIUS,
                SECONDARY_LINK_BODY_WIDTH,
                PLATE_THICKNESS,
                slot_ratio=0.42,
            ),
            "secondary_link_mesh",
        ),
        material=bright_steel,
        name="secondary_link_shell",
    )

    nose_link = model.part("nose_link")
    nose_link.visual(
        mesh_from_cadquery(make_nose_link(), "nose_link_mesh"),
        material=plated_steel,
        name="nose_link_shell",
    )

    hook_end_tab = model.part("hook_end_tab")
    hook_end_tab.visual(
        mesh_from_cadquery(make_hook_tab(), "hook_end_tab_mesh"),
        material=black_oxide,
        name="hook_tab_shell",
    )

    model.articulation(
        "foot_to_primary_link",
        ArticulationType.REVOLUTE,
        parent=grounded_foot,
        child=primary_link,
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=2.5,
            lower=-0.60,
            upper=2.78,
        ),
    )
    model.articulation(
        "primary_to_secondary_link",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(PRIMARY_LINK_LENGTH, 0.0, PLATE_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.8,
            lower=-2.70,
            upper=0.85,
        ),
    )
    model.articulation(
        "secondary_to_nose_link",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=nose_link,
        origin=Origin(xyz=(SECONDARY_LINK_LENGTH, 0.0, PLATE_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=3.0,
            lower=-0.45,
            upper=2.30,
        ),
    )
    model.articulation(
        "nose_to_hook_end_tab",
        ArticulationType.FIXED,
        parent=nose_link,
        child=hook_end_tab,
        origin=Origin(xyz=(NOSE_LINK_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    grounded_foot = object_model.get_part("grounded_foot")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    nose_link = object_model.get_part("nose_link")
    hook_end_tab = object_model.get_part("hook_end_tab")

    foot_to_primary = object_model.get_articulation("foot_to_primary_link")
    primary_to_secondary = object_model.get_articulation("primary_to_secondary_link")
    secondary_to_nose = object_model.get_articulation("secondary_to_nose_link")
    nose_to_hook = object_model.get_articulation("nose_to_hook_end_tab")

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
        "all expected parts present",
        all(
            part is not None
            for part in (
                grounded_foot,
                primary_link,
                secondary_link,
                nose_link,
                hook_end_tab,
            )
        ),
        "Missing one or more authored bracket parts.",
    )
    ctx.check(
        "three parallel revolute joints",
        (
            foot_to_primary.articulation_type == ArticulationType.REVOLUTE
            and primary_to_secondary.articulation_type == ArticulationType.REVOLUTE
            and secondary_to_nose.articulation_type == ArticulationType.REVOLUTE
            and nose_to_hook.articulation_type == ArticulationType.FIXED
            and foot_to_primary.axis == primary_to_secondary.axis == secondary_to_nose.axis
            == (0.0, 0.0, 1.0)
        ),
        "Primary hinges should be parallel revolute joints about +Z.",
    )

    ctx.expect_contact(grounded_foot, primary_link, name="foot touches primary link")
    ctx.expect_contact(primary_link, secondary_link, name="primary touches secondary")
    ctx.expect_contact(secondary_link, nose_link, name="secondary touches nose")
    ctx.expect_contact(nose_link, hook_end_tab, name="nose touches hook tab")

    ctx.expect_overlap(
        grounded_foot,
        primary_link,
        axes="xy",
        min_overlap=0.020,
        name="foot and primary hinge footprints align",
    )
    ctx.expect_overlap(
        primary_link,
        secondary_link,
        axes="xy",
        min_overlap=0.020,
        name="primary and secondary hinge footprints align",
    )
    ctx.expect_overlap(
        secondary_link,
        nose_link,
        axes="xy",
        min_overlap=0.018,
        name="secondary and nose hinge footprints align",
    )
    ctx.expect_overlap(
        nose_link,
        hook_end_tab,
        axes="y",
        min_overlap=0.010,
        name="hook tab matches nose link width",
    )

    ctx.expect_gap(
        primary_link,
        grounded_foot,
        axis="z",
        max_gap=0.0002,
        max_penetration=0.0,
        name="primary link sits directly on top of foot layer",
    )
    ctx.expect_gap(
        secondary_link,
        primary_link,
        axis="z",
        max_gap=0.0002,
        max_penetration=0.0,
        name="secondary link sits directly on top of primary layer",
    )
    ctx.expect_gap(
        nose_link,
        secondary_link,
        axis="z",
        max_gap=0.0002,
        max_penetration=0.0,
        name="nose link sits directly on top of secondary layer",
    )

    with ctx.pose(PACKED_POSE):
        ctx.expect_origin_distance(
            hook_end_tab,
            grounded_foot,
            axes="xy",
            max_dist=0.180,
            name="packed pose keeps hook tab close to foot",
        )
        ctx.expect_overlap(
            grounded_foot,
            primary_link,
            axes="xy",
            min_overlap=0.022,
            name="packed primary link folds over foot footprint",
        )

    with ctx.pose(EXTENDED_POSE):
        ctx.expect_origin_gap(
            hook_end_tab,
            grounded_foot,
            axis="x",
            min_gap=0.300,
            name="extended pose reaches outward from foot",
        )
        ctx.expect_origin_distance(
            hook_end_tab,
            grounded_foot,
            axes="y",
            max_dist=0.110,
            name="extended pose remains a shallow arc",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
