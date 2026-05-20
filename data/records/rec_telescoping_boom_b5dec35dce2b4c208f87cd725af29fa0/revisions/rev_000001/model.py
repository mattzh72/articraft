from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


EPS = 2.0e-4

ROOT_LEN = 0.42
ROOT_REAR_W = 0.20
ROOT_REAR_H = 0.18
ROOT_FRONT_W = 0.146
ROOT_FRONT_H = 0.130
ROOT_GUIDE_W = 0.110
ROOT_GUIDE_H = 0.094

MED_LEN = 0.56
MED_BODY_W = 0.098
MED_BODY_H = 0.082
MED_INNER_W = 0.066
MED_INNER_H = 0.054
MED_PAD_W = ROOT_GUIDE_W
MED_PAD_H = ROOT_GUIDE_H
MED_GUIDE_W = 0.058
MED_GUIDE_H = 0.046

TERM_MAIN_LEN = 0.34
TERM_NOSE_LEN = 0.12
TERM_LEN = TERM_MAIN_LEN + TERM_NOSE_LEN
TERM_BODY_W = 0.050
TERM_BODY_H = 0.038
TERM_INNER_W = 0.032
TERM_INNER_H = 0.022
TERM_PAD_W = MED_GUIDE_W
TERM_PAD_H = MED_GUIDE_H
TERM_NOSE_W = 0.038
TERM_NOSE_H = 0.030

PLATE_W = 0.080
PLATE_H = 0.065
PLATE_T = 0.008

ROOT_TO_MEDIUM_X = ROOT_LEN
MEDIUM_TO_TERMINAL_X = MED_LEN
MEDIUM_TRAVEL = 0.26
TERMINAL_TRAVEL = 0.24


def _rect_tube(length: float, outer_w: float, outer_h: float, inner_w: float, inner_h: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(length, outer_w, outer_h, centered=(False, True, True))
    inner = (
        cq.Workplane("XY")
        .box(length + 2.0 * EPS, inner_w, inner_h, centered=(False, True, True))
        .translate((-EPS, 0.0, 0.0))
    )
    return outer.cut(inner)


def _pad(
    x0: float,
    length: float,
    width: float,
    height: float,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(False, True, True)).translate((x0, y, z))


def make_root_housing() -> cq.Workplane:
    side_thickness = 0.012
    top_thickness = 0.012

    rear_frame = _rect_tube(0.032, ROOT_REAR_W, ROOT_REAR_H, 0.144, 0.124)
    front_collar = _rect_tube(0.30, ROOT_FRONT_W, ROOT_FRONT_H, ROOT_GUIDE_W, ROOT_GUIDE_H).translate((0.12, 0.0, 0.0))

    side_outline = [
        (0.02, -0.082),
        (0.02, 0.082),
        (0.14, 0.074),
        (0.30, 0.066),
        (0.41, 0.062),
        (0.41, -0.062),
        (0.30, -0.066),
        (0.14, -0.074),
    ]
    side_plate = cq.Workplane("XZ").polyline(side_outline).close().extrude(side_thickness, both=True)
    side_window_lower = (
        cq.Workplane("XZ").polyline([(0.085, -0.058), (0.205, -0.006), (0.335, -0.058)]).close().extrude(side_thickness + 2.0 * EPS, both=True)
    )
    side_window_upper = (
        cq.Workplane("XZ").polyline([(0.115, 0.058), (0.235, 0.006), (0.365, 0.058)]).close().extrude(side_thickness + 2.0 * EPS, both=True)
    )
    side_plate = side_plate.cut(side_window_lower.union(side_window_upper))
    left_side = side_plate.translate((0.0, ROOT_REAR_W / 2.0 - side_thickness / 2.0, 0.0))
    right_side = side_plate.translate((0.0, -(ROOT_REAR_W / 2.0 - side_thickness / 2.0), 0.0))

    top_outline = [
        (0.035, -0.076),
        (0.035, 0.076),
        (0.16, 0.062),
        (0.31, 0.055),
        (0.41, 0.050),
        (0.41, -0.050),
        (0.31, -0.055),
        (0.16, -0.062),
    ]
    top_plate = cq.Workplane("XY").polyline(top_outline).close().extrude(top_thickness, both=True)
    top_window = (
        cq.Workplane("XY")
        .polyline([(0.11, -0.022), (0.31, -0.016), (0.31, 0.016), (0.11, 0.022)])
        .close()
        .extrude(top_thickness + 2.0 * EPS, both=True)
    )
    top_plate = top_plate.cut(top_window)
    upper_truss = top_plate.translate((0.0, 0.0, ROOT_REAR_H / 2.0 - top_thickness / 2.0))
    lower_truss = top_plate.translate((0.0, 0.0, -(ROOT_REAR_H / 2.0 - top_thickness / 2.0)))

    mount_block = (
        cq.Workplane("XY")
        .box(0.12, 0.14, 0.05, centered=(False, True, True))
        .translate((-0.06, 0.0, -0.055))
    )
    mount_bore = (
        cq.Workplane("YZ")
        .center(0.0, -0.055)
        .circle(0.016)
        .extrude(0.14, both=True)
        .translate((-0.01, 0.0, 0.0))
    )
    mount_block = mount_block.cut(mount_bore)

    return rear_frame.union(front_collar).union(left_side).union(right_side).union(upper_truss).union(lower_truss).union(mount_block)


def make_medium_stage() -> cq.Workplane:
    rear_sleeve = _rect_tube(0.30, MED_PAD_W, MED_PAD_H, MED_BODY_W, MED_BODY_H)
    body_tube = _rect_tube(0.28, MED_BODY_W, MED_BODY_H, MED_INNER_W, MED_INNER_H).translate((0.18, 0.0, 0.0))
    front_collar = _rect_tube(0.32, 0.078, 0.066, MED_GUIDE_W, MED_GUIDE_H).translate((0.24, 0.0, 0.0))

    upper_spine = _pad(0.18, 0.18, 0.046, 0.0035, z=MED_BODY_H / 2.0 + 0.00175)
    lower_spine = _pad(0.18, 0.18, 0.046, 0.0035, z=-(MED_BODY_H / 2.0 + 0.00175))
    left_runner = _pad(0.22, 0.14, 0.0035, 0.036, y=MED_BODY_W / 2.0 + 0.00175)
    right_runner = _pad(0.22, 0.14, 0.0035, 0.036, y=-(MED_BODY_W / 2.0 + 0.00175))

    front_nose = (
        cq.Workplane("YZ")
        .workplane(offset=0.44)
        .rect(0.078, 0.066)
        .workplane(offset=0.12)
        .rect(0.070, 0.058)
        .loft(combine=True)
    )
    front_nose = front_nose.cut(
        cq.Workplane("XY")
        .box(0.122, MED_GUIDE_W, MED_GUIDE_H, centered=(False, True, True))
        .translate((0.44 - EPS, 0.0, 0.0))
    )

    return rear_sleeve.union(body_tube).union(front_collar).union(upper_spine).union(lower_spine).union(left_runner).union(right_runner).union(front_nose)


def make_terminal_stage() -> cq.Workplane:
    rear_sleeve = _rect_tube(0.28, TERM_PAD_W, TERM_PAD_H, TERM_BODY_W, TERM_BODY_H)
    body_tube = _rect_tube(0.20, TERM_BODY_W, TERM_BODY_H, TERM_INNER_W, TERM_INNER_H).translate((0.18, 0.0, 0.0))

    top_rib = _pad(0.18, 0.10, 0.022, 0.003, z=TERM_BODY_H / 2.0 + 0.0015)
    bottom_rib = _pad(0.18, 0.10, 0.022, 0.003, z=-(TERM_BODY_H / 2.0 + 0.0015))
    side_rib_l = _pad(0.20, 0.09, 0.003, 0.020, y=TERM_BODY_W / 2.0 + 0.0015)
    side_rib_r = _pad(0.20, 0.09, 0.003, 0.020, y=-(TERM_BODY_W / 2.0 + 0.0015))

    nose = (
        cq.Workplane("YZ")
        .workplane(offset=0.34)
        .rect(TERM_BODY_W, TERM_BODY_H)
        .workplane(offset=TERM_NOSE_LEN)
        .rect(TERM_NOSE_W, TERM_NOSE_H)
        .loft(combine=True)
    )

    return rear_sleeve.union(body_tube).union(top_rib).union(bottom_rib).union(side_rib_l).union(side_rib_r).union(nose)


def make_face_plate() -> cq.Workplane:
    plate = cq.Workplane("YZ").rect(PLATE_W, PLATE_H).extrude(PLATE_T)
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(((-0.022, -0.018), (-0.022, 0.018), (0.022, -0.018), (0.022, 0.018)))
        .hole(0.0055)
    )
    boss = cq.Workplane("YZ").circle(0.011).extrude(0.006).translate((PLATE_T - EPS, 0.0, 0.0))
    return plate.union(boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_support_boom")

    root_color = model.material("root_color", rgba=(0.16, 0.17, 0.18, 1.0))
    medium_color = model.material("medium_color", rgba=(0.27, 0.28, 0.30, 1.0))
    terminal_color = model.material("terminal_color", rgba=(0.48, 0.50, 0.53, 1.0))
    plate_color = model.material("plate_color", rgba=(0.10, 0.10, 0.11, 1.0))

    root_housing = model.part("root_housing")
    root_housing.visual(
        mesh_from_cadquery(make_root_housing(), "root_housing_mesh"),
        name="housing",
        material=root_color,
    )
    root_housing.inertial = Inertial.from_geometry(
        Box((0.48, 0.20, 0.20)),
        mass=7.2,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
    )

    medium_stage = model.part("medium_stage")
    medium_stage.visual(
        mesh_from_cadquery(make_medium_stage(), "medium_stage_mesh"),
        name="stage",
        material=medium_color,
    )
    medium_stage.inertial = Inertial.from_geometry(
        Box((MED_LEN, 0.114, 0.098)),
        mass=3.1,
        origin=Origin(xyz=(MED_LEN / 2.0, 0.0, 0.0)),
    )

    terminal_stage = model.part("terminal_stage")
    terminal_stage.visual(
        mesh_from_cadquery(make_terminal_stage(), "terminal_stage_mesh"),
        name="stage",
        material=terminal_color,
    )
    terminal_stage.inertial = Inertial.from_geometry(
        Box((TERM_LEN, 0.062, 0.052)),
        mass=1.45,
        origin=Origin(xyz=(TERM_LEN / 2.0, 0.0, 0.0)),
    )

    face_plate = model.part("face_plate")
    face_plate.visual(
        mesh_from_cadquery(make_face_plate(), "face_plate_mesh"),
        name="plate",
        material=plate_color,
    )
    face_plate.inertial = Inertial.from_geometry(
        Box((0.014, PLATE_W, PLATE_H)),
        mass=0.24,
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
    )

    model.articulation(
        "root_to_medium",
        ArticulationType.PRISMATIC,
        parent=root_housing,
        child=medium_stage,
        origin=Origin(xyz=(ROOT_TO_MEDIUM_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.35,
            lower=0.0,
            upper=MEDIUM_TRAVEL,
        ),
    )

    model.articulation(
        "medium_to_terminal",
        ArticulationType.PRISMATIC,
        parent=medium_stage,
        child=terminal_stage,
        origin=Origin(xyz=(MEDIUM_TO_TERMINAL_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=TERMINAL_TRAVEL,
        ),
    )

    model.articulation(
        "terminal_to_face_plate",
        ArticulationType.FIXED,
        parent=terminal_stage,
        child=face_plate,
        origin=Origin(xyz=(TERM_LEN, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_housing = object_model.get_part("root_housing")
    medium_stage = object_model.get_part("medium_stage")
    terminal_stage = object_model.get_part("terminal_stage")
    face_plate = object_model.get_part("face_plate")

    root_to_medium = object_model.get_articulation("root_to_medium")
    medium_to_terminal = object_model.get_articulation("medium_to_terminal")
    terminal_to_face_plate = object_model.get_articulation("terminal_to_face_plate")

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
        "parts_present",
        all(
            part is not None
            for part in (root_housing, medium_stage, terminal_stage, face_plate)
        ),
        "Expected root housing, medium stage, terminal stage, and face plate parts.",
    )

    ctx.check(
        "serial_prismatic_axes",
        root_to_medium.axis == (1.0, 0.0, 0.0)
        and medium_to_terminal.axis == (1.0, 0.0, 0.0)
        and terminal_to_face_plate.articulation_type == ArticulationType.FIXED,
        "Boom stages should slide serially along +X and terminate in a fixed face plate.",
    )

    ctx.expect_contact(root_housing, medium_stage, name="root_supports_medium_stage")
    ctx.expect_contact(medium_stage, terminal_stage, name="medium_supports_terminal_stage")
    ctx.expect_contact(terminal_stage, face_plate, name="terminal_supports_face_plate")

    ctx.expect_within(
        medium_stage,
        root_housing,
        axes="yz",
        outer_elem="housing",
        margin=0.003,
        name="medium_nested_inside_root_cross_section",
    )
    ctx.expect_within(
        terminal_stage,
        medium_stage,
        axes="yz",
        outer_elem="stage",
        margin=0.003,
        name="terminal_nested_inside_medium_cross_section",
    )
    ctx.expect_overlap(
        face_plate,
        terminal_stage,
        axes="yz",
        min_overlap=0.028,
        name="face_plate_centered_on_terminal",
    )
    ctx.expect_gap(
        face_plate,
        terminal_stage,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="face_plate_seated_flush_to_terminal",
    )

    housing_aabb = ctx.part_element_world_aabb(root_housing, elem="housing")
    medium_aabb = ctx.part_element_world_aabb(medium_stage, elem="stage")
    terminal_aabb = ctx.part_element_world_aabb(terminal_stage, elem="stage")
    taper_ok = False
    if housing_aabb and medium_aabb and terminal_aabb:
        housing_w = housing_aabb[1][1] - housing_aabb[0][1]
        housing_h = housing_aabb[1][2] - housing_aabb[0][2]
        medium_w = medium_aabb[1][1] - medium_aabb[0][1]
        medium_h = medium_aabb[1][2] - medium_aabb[0][2]
        terminal_w = terminal_aabb[1][1] - terminal_aabb[0][1]
        terminal_h = terminal_aabb[1][2] - terminal_aabb[0][2]
        taper_ok = housing_w > medium_w > terminal_w and housing_h > medium_h > terminal_h
    ctx.check(
        "silhouette_tapers_from_root_to_tip",
        taper_ok,
        "Expected decreasing cross-sections from root housing to medium stage to terminal stage.",
    )

    medium_rest = ctx.part_world_position(medium_stage)
    terminal_rest = ctx.part_world_position(terminal_stage)
    medium_motion_ok = False
    terminal_motion_ok = False

    if medium_rest is not None:
        with ctx.pose({root_to_medium: MEDIUM_TRAVEL}):
            medium_extended = ctx.part_world_position(medium_stage)
        if medium_extended is not None:
            medium_motion_ok = (
                medium_extended[0] > medium_rest[0] + 0.20
                and isclose(medium_extended[1], medium_rest[1], abs_tol=1.0e-6)
                and isclose(medium_extended[2], medium_rest[2], abs_tol=1.0e-6)
            )
    ctx.check(
        "medium_stage_translates_along_boom_axis",
        medium_motion_ok,
        "Medium stage should extend substantially along +X without drifting off-axis.",
    )

    if terminal_rest is not None:
        with ctx.pose({root_to_medium: MEDIUM_TRAVEL * 0.5, medium_to_terminal: TERMINAL_TRAVEL}):
            terminal_extended = ctx.part_world_position(terminal_stage)
        if terminal_extended is not None:
            terminal_motion_ok = (
                terminal_extended[0] > terminal_rest[0] + 0.18
                and isclose(terminal_extended[1], terminal_rest[1], abs_tol=1.0e-6)
                and isclose(terminal_extended[2], terminal_rest[2], abs_tol=1.0e-6)
            )
    ctx.check(
        "terminal_stage_translates_along_boom_axis",
        terminal_motion_ok,
        "Terminal stage should extend further along +X without lateral drift.",
    )

    forward_reach_ok = False
    with ctx.pose({root_to_medium: MEDIUM_TRAVEL, medium_to_terminal: TERMINAL_TRAVEL}):
        root_aabb = ctx.part_element_world_aabb(root_housing, elem="housing")
        plate_aabb = ctx.part_element_world_aabb(face_plate, elem="plate")
        if root_aabb and plate_aabb:
            forward_reach_ok = (plate_aabb[1][0] - root_aabb[1][0]) > 0.75
    ctx.check(
        "extended_boom_reaches_forward",
        forward_reach_ok,
        "Fully extended face plate should project well ahead of the root housing.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
