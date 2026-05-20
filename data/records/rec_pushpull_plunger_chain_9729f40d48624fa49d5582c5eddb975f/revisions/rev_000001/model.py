from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


PLATE_L = 0.18
PLATE_H = 0.11
PLATE_T = 0.006

GUIDE_L = 0.11
GUIDE_D = 0.028
GUIDE_H = 0.034
GUIDE_X = -0.026
GUIDE_Y = (PLATE_T / 2.0) + (GUIDE_D / 2.0)

BACK_WALL = 0.006
FRONT_WALL = 0.010
CAVITY_L = GUIDE_L - BACK_WALL - FRONT_WALL
CAVITY_D = 0.018
CAVITY_H = 0.022
CAVITY_X = GUIDE_X - ((FRONT_WALL - BACK_WALL) / 2.0)

NOSE_L = 0.032
NOSE_D = GUIDE_D
NOSE_H = 0.024
NOSE_X = 0.041
NOSE_Z = 0.004
NOSE_BRIDGE_L = 0.020
NOSE_BRIDGE_X = 0.035

STEM_TUNNEL_L = 0.048
STEM_TUNNEL_D = 0.014
STEM_TUNNEL_H = 0.014
STEM_TUNNEL_X = 0.035

HINGE_X = 0.056
HINGE_Z = 0.012
EAR_L = 0.016
EAR_T = 0.007
EAR_H = 0.028
EAR_GAP = 0.012
EAR_Y_OFFSET = (EAR_GAP / 2.0) + (EAR_T / 2.0)

CLEVIS_RELIEF_L = 0.028
CLEVIS_RELIEF_H = 0.034
CLEVIS_RELIEF_X = 0.052
CLEVIS_RELIEF_Z = 0.014

PLUNGER_ORIGIN_X = GUIDE_X - (GUIDE_L / 2.0) + BACK_WALL
PLUNGER_STROKE = 0.022
PLUNGER_COLLAR_L = 0.016
PLUNGER_COLLAR_W = 0.016
PLUNGER_COLLAR_H = 0.018
PLUNGER_STEM_L = 0.094
PLUNGER_STEM_W = 0.010
PLUNGER_STEM_H = 0.012
PLUNGER_TIP_L = 0.010
PLUNGER_TIP_W = 0.008
PLUNGER_TIP_H = 0.010
PLUNGER_TIP_Z = 0.001

FLAP_T = EAR_GAP
FLAP_BARREL_R = 0.0075
FLAP_OPEN = 0.78

GUIDE_RAIL_T = (GUIDE_H - CAVITY_H) / 2.0
GUIDE_SIDE_WALL_T = (GUIDE_D - CAVITY_D) / 2.0
GUIDE_INNER_WALL_Y = (PLATE_T / 2.0) + (GUIDE_SIDE_WALL_T / 2.0)
GUIDE_OUTER_WALL_Y = GUIDE_Y + (CAVITY_D / 2.0) + (GUIDE_SIDE_WALL_T / 2.0)
GUIDE_TOP_Z = (CAVITY_H / 2.0) + (GUIDE_RAIL_T / 2.0)
NOSE_BLOCK_T = (NOSE_H - STEM_TUNNEL_H) / 2.0
NOSE_TOP_Z = (STEM_TUNNEL_H / 2.0) + (NOSE_BLOCK_T / 2.0)


def _slot_cutters() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .pushPoints([(-0.055, 0.0), (0.055, 0.0)])
        .slot2D(0.022, 0.008, angle=90)
        .extrude(PLATE_T * 4.0, both=True)
    )


def _make_side_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_L, PLATE_T, PLATE_H)
    guide = cq.Workplane("XY").box(GUIDE_L, GUIDE_D, GUIDE_H).translate((GUIDE_X, GUIDE_Y, 0.0))
    nose = cq.Workplane("XY").box(NOSE_L, NOSE_D, NOSE_H).translate((NOSE_X, GUIDE_Y, NOSE_Z))

    ear_inner = (
        cq.Workplane("XY")
        .box(EAR_L, EAR_T, EAR_H)
        .translate((HINGE_X, GUIDE_Y - EAR_Y_OFFSET, HINGE_Z))
    )
    ear_outer = (
        cq.Workplane("XY")
        .box(EAR_L, EAR_T, EAR_H)
        .translate((HINGE_X, GUIDE_Y + EAR_Y_OFFSET, HINGE_Z))
    )

    cavity = cq.Workplane("XY").box(CAVITY_L, CAVITY_D, CAVITY_H).translate((CAVITY_X, GUIDE_Y, 0.0))
    stem_tunnel = (
        cq.Workplane("XY")
        .box(STEM_TUNNEL_L, STEM_TUNNEL_D, STEM_TUNNEL_H)
        .translate((STEM_TUNNEL_X, GUIDE_Y, 0.0))
    )
    clevis_relief = (
        cq.Workplane("XY")
        .box(CLEVIS_RELIEF_L, EAR_GAP, CLEVIS_RELIEF_H)
        .translate((CLEVIS_RELIEF_X, GUIDE_Y, CLEVIS_RELIEF_Z))
    )

    shape = plate.union(guide).union(nose).union(ear_inner).union(ear_outer)
    shape = shape.cut(_slot_cutters())
    shape = shape.cut(cavity).cut(stem_tunnel).cut(clevis_relief)
    return shape


def _make_plunger_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .box(PLUNGER_COLLAR_L, PLUNGER_COLLAR_W, PLUNGER_COLLAR_H)
        .translate((PLUNGER_COLLAR_L / 2.0, 0.0, 0.0))
    )
    stem = (
        cq.Workplane("XY")
        .box(PLUNGER_STEM_L, PLUNGER_STEM_W, PLUNGER_STEM_H)
        .translate((0.012 + (PLUNGER_STEM_L / 2.0), 0.0, 0.0))
    )
    tip = (
        cq.Workplane("XY")
        .box(PLUNGER_TIP_L, PLUNGER_TIP_W, PLUNGER_TIP_H)
        .translate((0.106 + (PLUNGER_TIP_L / 2.0), 0.0, PLUNGER_TIP_Z))
    )
    return collar.union(stem).union(tip)


def _make_flap_shape() -> cq.Workplane:
    barrel = cq.Workplane("XZ").circle(FLAP_BARREL_R).extrude(FLAP_T / 2.0, both=True)
    lever_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.015, -0.010),
                (-0.004, -0.010),
                (0.010, -0.004),
                (0.044, 0.002),
                (0.044, 0.014),
                (0.006, 0.010),
                (-0.014, 0.004),
            ]
        )
        .close()
        .extrude(FLAP_T / 2.0, both=True)
    )
    return barrel.union(lever_profile)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_plunger_linkage")

    model.material("painted_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("machined_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("black_oxide", rgba=(0.16, 0.17, 0.18, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((PLATE_L, PLATE_T, PLATE_H)),
        material="painted_steel",
        name="side_plate_shell",
    )
    side_plate.visual(
        Box((BACK_WALL, GUIDE_D, GUIDE_H)),
        origin=Origin(xyz=(GUIDE_X - (GUIDE_L / 2.0) + (BACK_WALL / 2.0), GUIDE_Y, 0.0)),
        material="painted_steel",
        name="rear_stop",
    )
    side_plate.visual(
        Box((CAVITY_L, GUIDE_D, GUIDE_RAIL_T)),
        origin=Origin(xyz=(CAVITY_X, GUIDE_Y, GUIDE_TOP_Z)),
        material="painted_steel",
        name="guide_top",
    )
    side_plate.visual(
        Box((CAVITY_L, GUIDE_D, GUIDE_RAIL_T)),
        origin=Origin(xyz=(CAVITY_X, GUIDE_Y, -GUIDE_TOP_Z)),
        material="painted_steel",
        name="guide_bottom",
    )
    side_plate.visual(
        Box((CAVITY_L, GUIDE_SIDE_WALL_T, CAVITY_H)),
        origin=Origin(xyz=(CAVITY_X, GUIDE_INNER_WALL_Y, 0.0)),
        material="painted_steel",
        name="guide_inner_wall",
    )
    side_plate.visual(
        Box((CAVITY_L, GUIDE_SIDE_WALL_T, CAVITY_H)),
        origin=Origin(xyz=(CAVITY_X, GUIDE_OUTER_WALL_Y, 0.0)),
        material="painted_steel",
        name="guide_outer_wall",
    )
    side_plate.visual(
        Box((NOSE_BRIDGE_L, GUIDE_D, NOSE_BLOCK_T)),
        origin=Origin(xyz=(NOSE_BRIDGE_X, GUIDE_Y, NOSE_TOP_Z)),
        material="painted_steel",
        name="nose_top",
    )
    side_plate.visual(
        Box((NOSE_BRIDGE_L, GUIDE_D, NOSE_BLOCK_T)),
        origin=Origin(xyz=(NOSE_BRIDGE_X, GUIDE_Y, -NOSE_TOP_Z)),
        material="painted_steel",
        name="nose_bottom",
    )
    side_plate.visual(
        Box((NOSE_L, GUIDE_SIDE_WALL_T, STEM_TUNNEL_H)),
        origin=Origin(xyz=(NOSE_X, GUIDE_INNER_WALL_Y, 0.0)),
        material="painted_steel",
        name="nose_inner_cheek",
    )
    side_plate.visual(
        Box((NOSE_L, GUIDE_SIDE_WALL_T, STEM_TUNNEL_H)),
        origin=Origin(xyz=(NOSE_X, GUIDE_OUTER_WALL_Y, 0.0)),
        material="painted_steel",
        name="nose_outer_cheek",
    )
    side_plate.visual(
        Box((EAR_L, EAR_T, EAR_H)),
        origin=Origin(xyz=(HINGE_X, GUIDE_Y - EAR_Y_OFFSET, HINGE_Z)),
        material="painted_steel",
        name="inner_ear",
    )
    side_plate.visual(
        Box((EAR_L, EAR_T, EAR_H)),
        origin=Origin(xyz=(HINGE_X, GUIDE_Y + EAR_Y_OFFSET, HINGE_Z)),
        material="painted_steel",
        name="outer_ear",
    )
    side_plate.inertial = Inertial.from_geometry(
        Box((PLATE_L, GUIDE_D + PLATE_T, PLATE_H)),
        mass=2.4,
        origin=Origin(xyz=(0.0, GUIDE_Y * 0.7, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Box((PLUNGER_COLLAR_L, PLUNGER_COLLAR_W, PLUNGER_COLLAR_H)),
        origin=Origin(xyz=(PLUNGER_COLLAR_L / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="plunger_shell",
    )
    plunger.visual(
        Box((PLUNGER_STEM_L, PLUNGER_STEM_W, PLUNGER_STEM_H)),
        origin=Origin(xyz=(0.012 + (PLUNGER_STEM_L / 2.0), 0.0, 0.0)),
        material="machined_steel",
        name="plunger_stem",
    )
    plunger.visual(
        Box((PLUNGER_TIP_L, PLUNGER_TIP_W, PLUNGER_TIP_H)),
        origin=Origin(xyz=(0.105 + (PLUNGER_TIP_L / 2.0), 0.0, PLUNGER_TIP_Z)),
        material="machined_steel",
        name="plunger_tip",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.116, PLUNGER_COLLAR_W, PLUNGER_COLLAR_H)),
        mass=0.24,
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
    )

    output_flap = model.part("output_flap")
    output_flap.visual(
        Cylinder(radius=FLAP_BARREL_R, length=FLAP_T),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="black_oxide",
        name="output_flap_shell",
    )
    output_flap.visual(
        Box((0.040, FLAP_T, 0.012)),
        origin=Origin(xyz=(0.022, 0.0, 0.005)),
        material="black_oxide",
        name="flap_paddle",
    )
    output_flap.visual(
        Box((0.010, FLAP_T, 0.010)),
        origin=Origin(xyz=(-0.011, 0.0, -0.006)),
        material="black_oxide",
        name="flap_heel",
    )
    output_flap.inertial = Inertial.from_geometry(
        Box((0.060, FLAP_T, 0.024)),
        mass=0.12,
        origin=Origin(xyz=(0.012, 0.0, 0.002)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=plunger,
        origin=Origin(xyz=(PLUNGER_ORIGIN_X, GUIDE_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PLUNGER_STROKE,
            effort=120.0,
            velocity=0.12,
        ),
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=output_flap,
        origin=Origin(xyz=(HINGE_X, GUIDE_Y, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FLAP_OPEN,
            effort=18.0,
            velocity=1.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    plunger = object_model.get_part("plunger")
    output_flap = object_model.get_part("output_flap")
    plunger_slide = object_model.get_articulation("plunger_slide")
    flap_hinge = object_model.get_articulation("flap_hinge")

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
        "parts present",
        all(part is not None for part in (side_plate, plunger, output_flap)),
        "Expected side_plate, plunger, and output_flap parts.",
    )
    ctx.check(
        "plunger joint axis and stroke",
        tuple(plunger_slide.axis) == (1.0, 0.0, 0.0)
        and plunger_slide.motion_limits is not None
        and plunger_slide.motion_limits.lower == 0.0
        and plunger_slide.motion_limits.upper == PLUNGER_STROKE,
        "Plunger should slide forward along +X over the intended short stroke.",
    )
    ctx.check(
        "flap hinge axis and limit",
        tuple(flap_hinge.axis) == (0.0, -1.0, 0.0)
        and flap_hinge.motion_limits is not None
        and flap_hinge.motion_limits.lower == 0.0
        and flap_hinge.motion_limits.upper == FLAP_OPEN,
        "Flap should hinge upward about -Y from the guide-body end.",
    )

    with ctx.pose({plunger_slide: 0.0, flap_hinge: 0.0}):
        ctx.expect_contact(
            plunger,
            side_plate,
            contact_tol=6e-4,
            name="plunger seated in guide stop",
        )
        ctx.expect_contact(
            output_flap,
            side_plate,
            contact_tol=6e-4,
            name="flap barrel supported by clevis ears",
        )
        ctx.expect_contact(
            plunger,
            output_flap,
            contact_tol=0.001,
            name="plunger meets flap heel in rest pose",
        )

    with ctx.pose({plunger_slide: 0.0}):
        retracted_aabb = ctx.part_world_aabb(plunger)
    with ctx.pose({plunger_slide: PLUNGER_STROKE}):
        extended_aabb = ctx.part_world_aabb(plunger)
    ctx.check(
        "plunger extends toward output end",
        retracted_aabb is not None
        and extended_aabb is not None
        and extended_aabb[1][0] > retracted_aabb[1][0] + 0.015,
        "Plunger upper stroke should move its nose forward along +X.",
    )

    with ctx.pose({flap_hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(output_flap)
    with ctx.pose({flap_hinge: FLAP_OPEN}):
        open_aabb = ctx.part_world_aabb(output_flap)
    ctx.check(
        "flap opens upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.02,
        "Positive hinge motion should lift the flap tip upward.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
