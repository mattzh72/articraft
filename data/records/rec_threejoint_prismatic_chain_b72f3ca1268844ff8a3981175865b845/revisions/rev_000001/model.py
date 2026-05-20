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


BASE_L = 0.72
BASE_W = 0.24
BASE_PLATE_H = 0.028
BASE_BED_L = 0.62
BASE_BED_W = 0.15
BASE_BED_H = 0.022
BASE_RAIL_W = 0.016
BASE_RAIL_H = 0.012
BASE_RAIL_Y = 0.051
BASE_GUIDE_Z = BASE_PLATE_H + BASE_BED_H

FIRST_STAGE_TRAVEL = 0.20

FIRST_SHOE_L = 0.17
FIRST_SHOE_W = 0.070
FIRST_SHOE_PAD_W = 0.028
FIRST_SHOE_H = 0.020
FIRST_BODY_L = 0.20
FIRST_BODY_W = 0.18
FIRST_BODY_H = 0.036
FIRST_DECK_X = 0.14
FIRST_DECK_Y = 0.28
FIRST_DECK_H = 0.018
FIRST_Y_RAIL_W = 0.014
FIRST_Y_RAIL_H = 0.012
FIRST_Y_RAIL_X = 0.034
FIRST_Y_RAIL_Y = 0.26
SECOND_STAGE_GUIDE_Z = FIRST_SHOE_H + FIRST_BODY_H + FIRST_DECK_H

SECOND_STAGE_TRAVEL = 0.07

SECOND_SHOE_X = 0.050
SECOND_SHOE_Y = 0.12
SECOND_SHOE_H = 0.018
SECOND_SHOE_PAD_X = 0.028
SECOND_BODY_X = 0.10
SECOND_BODY_Y = 0.14
SECOND_BODY_H = 0.028
SECOND_BEAM_X = 0.14
SECOND_BEAM_Y = 0.055
SECOND_BEAM_H = 0.035
SECOND_BEAM_CENTER_X = 0.09
SECOND_BEAM_Z = SECOND_SHOE_H + SECOND_BODY_H
SECOND_MAST_T = 0.022
SECOND_MAST_W = 0.090
SECOND_MAST_H = 0.24
SECOND_MAST_FRONT_X = 0.160
SECOND_MAST_CENTER_X = SECOND_MAST_FRONT_X - (SECOND_MAST_T / 2.0)
SECOND_MAST_BOTTOM_Z = 0.03
SECOND_RIB_X = 0.05
SECOND_RIB_Y = 0.012
SECOND_RIB_H = 0.10
SECOND_RIB_CENTER_X = 0.120
SECOND_RIB_CENTER_Y = 0.032
SECOND_RIB_BOTTOM_Z = SECOND_BEAM_Z

THIRD_STAGE_TRAVEL = 0.11

THIRD_PLATE_T = 0.016
THIRD_PLATE_Y = 0.086
THIRD_PLATE_Z = 0.10
THIRD_BODY_X = 0.064
THIRD_BODY_Y = 0.050
THIRD_BODY_Z = 0.050
THIRD_BODY_CENTER_X = 0.039
THIRD_BODY_BOTTOM_Z = 0.020
THIRD_NOSE_X = 0.028
THIRD_NOSE_Y = 0.028
THIRD_NOSE_Z = 0.070
THIRD_NOSE_CENTER_X = 0.083
THIRD_NOSE_BOTTOM_Z = 0.010


def _box(size: tuple[float, float, float], *, center_xy: tuple[float, float] = (0.0, 0.0), z0: float = 0.0) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy = center_xy
    return cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, False)).translate((cx, cy, z0))


def _combine(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _add_mesh(part, shape: cq.Workplane, mesh_name: str, material: str, visual_name: str) -> None:
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name=visual_name)


def _base_shape() -> cq.Workplane:
    plate = _box((BASE_L, BASE_W, BASE_PLATE_H), z0=0.0)
    bed = _box((BASE_BED_L, BASE_BED_W, BASE_BED_H), z0=BASE_PLATE_H)
    rail_left = _box(
        (BASE_BED_L, BASE_RAIL_W, BASE_RAIL_H),
        center_xy=(0.0, -BASE_RAIL_Y),
        z0=BASE_GUIDE_Z,
    )
    rail_right = _box(
        (BASE_BED_L, BASE_RAIL_W, BASE_RAIL_H),
        center_xy=(0.0, BASE_RAIL_Y),
        z0=BASE_GUIDE_Z,
    )
    return _combine(plate, bed, rail_left, rail_right)


def _first_carriage_shape() -> cq.Workplane:
    shoe_left = _box(
        (FIRST_SHOE_L, FIRST_SHOE_PAD_W, FIRST_SHOE_H),
        center_xy=(0.0, -BASE_RAIL_Y),
        z0=0.0,
    )
    shoe_right = _box(
        (FIRST_SHOE_L, FIRST_SHOE_PAD_W, FIRST_SHOE_H),
        center_xy=(0.0, BASE_RAIL_Y),
        z0=0.0,
    )
    body = _box((FIRST_BODY_L, FIRST_BODY_W, FIRST_BODY_H), z0=FIRST_SHOE_H)
    deck = _box((FIRST_DECK_X, FIRST_DECK_Y, FIRST_DECK_H), z0=FIRST_SHOE_H + FIRST_BODY_H)
    rail_left = _box(
        (FIRST_Y_RAIL_W, FIRST_Y_RAIL_Y, FIRST_Y_RAIL_H),
        center_xy=(-FIRST_Y_RAIL_X, 0.0),
        z0=SECOND_STAGE_GUIDE_Z,
    )
    rail_right = _box(
        (FIRST_Y_RAIL_W, FIRST_Y_RAIL_Y, FIRST_Y_RAIL_H),
        center_xy=(FIRST_Y_RAIL_X, 0.0),
        z0=SECOND_STAGE_GUIDE_Z,
    )
    return _combine(shoe_left, shoe_right, body, deck, rail_left, rail_right)


def _second_stage_shape() -> cq.Workplane:
    shoe_left = _box(
        (SECOND_SHOE_PAD_X, SECOND_SHOE_Y, SECOND_SHOE_H),
        center_xy=(-FIRST_Y_RAIL_X, 0.0),
        z0=0.0,
    )
    shoe_right = _box(
        (SECOND_SHOE_PAD_X, SECOND_SHOE_Y, SECOND_SHOE_H),
        center_xy=(FIRST_Y_RAIL_X, 0.0),
        z0=0.0,
    )
    body = _box((SECOND_BODY_X, SECOND_BODY_Y, SECOND_BODY_H), z0=SECOND_SHOE_H)
    beam = _box(
        (SECOND_BEAM_X, SECOND_BEAM_Y, SECOND_BEAM_H),
        center_xy=(SECOND_BEAM_CENTER_X, 0.0),
        z0=SECOND_BEAM_Z,
    )
    mast = _box(
        (SECOND_MAST_T, SECOND_MAST_W, SECOND_MAST_H),
        center_xy=(SECOND_MAST_CENTER_X, 0.0),
        z0=SECOND_MAST_BOTTOM_Z,
    )
    rib_left = _box(
        (SECOND_RIB_X, SECOND_RIB_Y, SECOND_RIB_H),
        center_xy=(SECOND_RIB_CENTER_X, -SECOND_RIB_CENTER_Y),
        z0=SECOND_RIB_BOTTOM_Z,
    )
    rib_right = _box(
        (SECOND_RIB_X, SECOND_RIB_Y, SECOND_RIB_H),
        center_xy=(SECOND_RIB_CENTER_X, SECOND_RIB_CENTER_Y),
        z0=SECOND_RIB_BOTTOM_Z,
    )
    return _combine(shoe_left, shoe_right, body, beam, mast, rib_left, rib_right)


def _third_stage_shape() -> cq.Workplane:
    back_plate = _box((THIRD_PLATE_T, THIRD_PLATE_Y, THIRD_PLATE_Z), z0=0.0)
    front_body = _box(
        (THIRD_BODY_X, THIRD_BODY_Y, THIRD_BODY_Z),
        center_xy=(THIRD_BODY_CENTER_X, 0.0),
        z0=THIRD_BODY_BOTTOM_Z,
    )
    nose = _box(
        (THIRD_NOSE_X, THIRD_NOSE_Y, THIRD_NOSE_Z),
        center_xy=(THIRD_NOSE_CENTER_X, 0.0),
        z0=THIRD_NOSE_BOTTOM_Z,
    )
    return _combine(back_plate, front_body, nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_prismatic_chain")

    model.material("base_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("light_aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    model.material("tool_orange", rgba=(0.86, 0.42, 0.12, 1.0))

    base = model.part("base_guide")
    _add_mesh(base, _base_shape(), "base_guide", "base_black", "base_shell")
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_GUIDE_Z + BASE_RAIL_H)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_GUIDE_Z + BASE_RAIL_H) / 2.0)),
    )

    first_carriage = model.part("first_carriage")
    _add_mesh(first_carriage, _first_carriage_shape(), "first_carriage", "machined_aluminum", "carriage_shell")
    first_carriage.inertial = Inertial.from_geometry(
        Box((FIRST_BODY_L, FIRST_DECK_Y, SECOND_STAGE_GUIDE_Z + FIRST_Y_RAIL_H)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, (SECOND_STAGE_GUIDE_Z + FIRST_Y_RAIL_H) / 2.0)),
    )

    second_stage = model.part("second_stage")
    _add_mesh(second_stage, _second_stage_shape(), "second_stage", "light_aluminum", "stage_shell")
    second_stage.inertial = Inertial.from_geometry(
        Box((SECOND_MAST_FRONT_X + 0.01, SECOND_BODY_Y, SECOND_MAST_H)),
        mass=4.0,
        origin=Origin(xyz=(0.08, 0.0, SECOND_MAST_H / 2.0)),
    )

    third_stage = model.part("third_stage")
    _add_mesh(third_stage, _third_stage_shape(), "third_stage", "tool_orange", "tip_shell")
    third_stage.inertial = Inertial.from_geometry(
        Box((0.11, THIRD_PLATE_Y, THIRD_PLATE_Z)),
        mass=1.5,
        origin=Origin(xyz=(0.05, 0.0, THIRD_PLATE_Z / 2.0)),
    )

    model.articulation(
        "base_to_first",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_GUIDE_Z + BASE_RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-FIRST_STAGE_TRAVEL,
            upper=FIRST_STAGE_TRAVEL,
            effort=800.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=second_stage,
        origin=Origin(xyz=(0.0, 0.0, SECOND_STAGE_GUIDE_Z + FIRST_Y_RAIL_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SECOND_STAGE_TRAVEL,
            upper=SECOND_STAGE_TRAVEL,
            effort=500.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "second_to_third",
        ArticulationType.PRISMATIC,
        parent=second_stage,
        child=third_stage,
        origin=Origin(xyz=(SECOND_MAST_FRONT_X + (THIRD_PLATE_T / 2.0), 0.0, SECOND_MAST_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=THIRD_STAGE_TRAVEL,
            effort=220.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    first_carriage = object_model.get_part("first_carriage")
    second_stage = object_model.get_part("second_stage")
    third_stage = object_model.get_part("third_stage")
    base_to_first = object_model.get_articulation("base_to_first")
    first_to_second = object_model.get_articulation("first_to_second")
    second_to_third = object_model.get_articulation("second_to_third")

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
        "all_joints_are_prismatic",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            for joint in (base_to_first, first_to_second, second_to_third)
        ),
        "Every stage in the chain should be a prismatic articulation.",
    )
    ctx.check(
        "joint_axes_match_stage_guides",
        base_to_first.axis == (1.0, 0.0, 0.0)
        and first_to_second.axis == (0.0, 1.0, 0.0)
        and second_to_third.axis == (0.0, 0.0, 1.0),
        "Expected serial X, Y, and Z guide axes for the three supported stages.",
    )

    ctx.expect_contact(first_carriage, base, contact_tol=1e-4, name="first_carriage_supported_on_base_guide")
    ctx.expect_overlap(first_carriage, base, axes="xy", min_overlap=0.07, name="first_carriage_has_supported_footprint")
    ctx.expect_contact(second_stage, first_carriage, contact_tol=1e-4, name="second_stage_supported_on_first_carriage")
    ctx.expect_overlap(second_stage, first_carriage, axes="xy", min_overlap=0.05, name="second_stage_has_supported_footprint")
    ctx.expect_contact(third_stage, second_stage, contact_tol=1e-4, name="third_stage_supported_on_tip_mast")
    ctx.expect_overlap(third_stage, second_stage, axes="yz", min_overlap=0.06, name="third_stage_overlaps_tip_guide_face")

    with ctx.pose({base_to_first: FIRST_STAGE_TRAVEL}):
        ctx.expect_origin_gap(
            first_carriage,
            base,
            axis="x",
            min_gap=0.19,
            name="first_stage_translates_along_base_x",
        )
        ctx.expect_overlap(
            first_carriage,
            base,
            axes="xy",
            min_overlap=0.07,
            name="first_stage_remains_supported_at_positive_travel",
        )

    with ctx.pose({first_to_second: SECOND_STAGE_TRAVEL}):
        ctx.expect_origin_gap(
            second_stage,
            first_carriage,
            axis="y",
            min_gap=0.06,
            name="second_stage_translates_along_carriage_y",
        )
        ctx.expect_overlap(
            second_stage,
            first_carriage,
            axes="xy",
            min_overlap=0.05,
            name="second_stage_remains_supported_at_positive_travel",
        )

    with ctx.pose({second_to_third: THIRD_STAGE_TRAVEL}):
        ctx.expect_origin_gap(
            third_stage,
            second_stage,
            axis="z",
            min_gap=0.13,
            name="third_stage_translates_along_tip_z",
        )
        ctx.expect_overlap(
            third_stage,
            second_stage,
            axes="yz",
            min_overlap=0.06,
            name="third_stage_remains_on_tip_guide_at_positive_travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
