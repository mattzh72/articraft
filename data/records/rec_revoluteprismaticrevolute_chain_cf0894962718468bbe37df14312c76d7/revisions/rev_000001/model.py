from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BRACKET_PLATE_THICK = 0.006
BRACKET_WIDTH = 0.086
BRACKET_HEIGHT = 0.150
BRACKET_PLATE_X = -0.030
BRACKET_CHEEK_THICK = 0.010
BRACKET_CHEEK_CENTER_Y = 0.026
BRACKET_CHEEK_LENGTH = 0.032
BRACKET_CHEEK_HEIGHT = 0.060

ROOT_AXIS = (0.0, -1.0, 0.0)
ROOT_LOWER = 0.0
ROOT_UPPER = 1.05

BOOM_ROOT_BARREL_RADIUS = 0.014
BOOM_ROOT_BARREL_LENGTH = 0.042
BOOM_ROOT_BLOCK_LEN = 0.060
BOOM_ROOT_BLOCK_WIDTH = 0.040
BOOM_ROOT_BLOCK_HEIGHT = 0.038
BOOM_HOUSING_START_X = 0.090
BOOM_HOUSING_LEN = 0.240
BOOM_HOUSING_WIDTH = 0.058
BOOM_HOUSING_HEIGHT = 0.048
BOOM_CAVITY_LEN = 0.236
BOOM_CAVITY_WIDTH = 0.044
BOOM_CAVITY_HEIGHT = 0.034
SLIDE_ORIGIN_X = 0.102
SLIDE_TRAVEL = 0.145

STAGE_CARRIAGE_LEN = 0.070
STAGE_CARRIAGE_WIDTH = 0.038
STAGE_CARRIAGE_HEIGHT = 0.024
STAGE_RAIL_LEN = 0.190
STAGE_RAIL_WIDTH = 0.020
STAGE_RAIL_HEIGHT = 0.014
HEAD_JOINT_X = 0.287

HEAD_AXIS = (0.0, -1.0, 0.0)
HEAD_LOWER = -0.45
HEAD_UPPER = 0.65
HEAD_HUB_RADIUS = 0.010
HEAD_HUB_LENGTH = 0.022
HEAD_PLATE_THICK = 0.008
HEAD_PLATE_WIDTH = 0.082
HEAD_PLATE_HEIGHT = 0.060


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> Box:
    return Box(size=size), Origin(xyz=center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0))


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0))


def _mount_hole_points() -> list[tuple[float, float]]:
    return [(-0.026, -0.050), (-0.026, 0.050), (0.026, -0.050), (0.026, 0.050)]


def _head_hole_points() -> list[tuple[float, float]]:
    return [(-0.024, -0.018), (-0.024, 0.018), (0.024, -0.018), (0.024, 0.018)]


def _make_bracket_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(BRACKET_PLATE_THICK, BRACKET_WIDTH, BRACKET_HEIGHT).translate(
        (BRACKET_PLATE_X, 0.0, 0.0)
    )
    pedestal = cq.Workplane("XY").box(0.040, 0.050, 0.060).translate((-0.010, 0.0, 0.0))
    lower_web = cq.Workplane("XY").box(0.022, 0.050, 0.016).translate((-0.004, 0.0, -0.026))
    upper_web = cq.Workplane("XY").box(0.018, 0.046, 0.014).translate((-0.006, 0.0, 0.026))
    left_cheek = cq.Workplane("XY").box(
        BRACKET_CHEEK_LENGTH, BRACKET_CHEEK_THICK, BRACKET_CHEEK_HEIGHT
    ).translate((0.010, BRACKET_CHEEK_CENTER_Y, 0.0))
    right_cheek = cq.Workplane("XY").box(
        BRACKET_CHEEK_LENGTH, BRACKET_CHEEK_THICK, BRACKET_CHEEK_HEIGHT
    ).translate((0.010, -BRACKET_CHEEK_CENTER_Y, 0.0))

    body = plate.union(pedestal).union(lower_web).union(upper_web).union(left_cheek).union(right_cheek)
    pivot_relief = cq.Workplane("XZ").circle(0.0152).extrude(0.060, both=True)
    body = body.cut(pivot_relief)
    body = body.faces("<X").workplane().pushPoints(_mount_hole_points()).hole(0.007)
    return body


def _make_boom_shape() -> cq.Workplane:
    root_neck = cq.Workplane("XY").box(0.038, 0.016, 0.022).translate((0.031, 0.0, 0.0))
    sleeve = cq.Workplane("XY").box(
        0.270, BOOM_HOUSING_WIDTH, BOOM_HOUSING_HEIGHT
    ).translate((0.050 + 0.270 / 2.0, 0.0, 0.0))
    cavity = cq.Workplane("XY").box(
        0.258, BOOM_CAVITY_WIDTH, BOOM_CAVITY_HEIGHT
    ).translate((0.056 + 0.258 / 2.0, 0.0, 0.0))
    sleeve = sleeve.cut(cavity)

    front_ring = cq.Workplane("XY").box(0.012, 0.050, 0.040).translate((0.325, 0.0, 0.0))
    front_opening = cq.Workplane("XY").box(0.014, 0.032, 0.022).translate((0.325, 0.0, 0.0))
    front_ring = front_ring.cut(front_opening)

    top_cover = cq.Workplane("XY").box(0.090, BOOM_HOUSING_WIDTH, 0.003).translate((0.238, 0.0, 0.0255))
    bottom_cover = cq.Workplane("XY").box(0.090, BOOM_HOUSING_WIDTH, 0.003).translate((0.238, 0.0, -0.0255))

    return root_neck.union(sleeve).union(front_ring).union(top_cover).union(bottom_cover)


def _make_stage_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(
        STAGE_CARRIAGE_LEN, STAGE_CARRIAGE_WIDTH, STAGE_CARRIAGE_HEIGHT
    ).translate((STAGE_CARRIAGE_LEN / 2.0, 0.0, 0.0))
    rail = cq.Workplane("XY").box(0.190, STAGE_RAIL_WIDTH, STAGE_RAIL_HEIGHT).translate((0.165, 0.0, 0.0))
    nose_block = cq.Workplane("XY").box(0.014, 0.020, 0.020).translate((0.255, 0.0, 0.0))
    return carriage.union(rail).union(nose_block)


def _make_head_shape() -> cq.Workplane:
    plate = cq.Workplane("YZ").rect(HEAD_PLATE_WIDTH, HEAD_PLATE_HEIGHT).extrude(HEAD_PLATE_THICK).translate(
        (0.018, 0.0, 0.0)
    )
    plate = plate.faces(">X").workplane().pushPoints(_head_hole_points()).hole(0.005)
    rib = cq.Workplane("XY").box(0.020, 0.018, 0.040).translate((0.010, 0.0, 0.0))
    return plate.union(rib)


def _vec_close(actual: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_boom")

    model.material("powder_coat", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("machine_gray", rgba=(0.61, 0.64, 0.68, 1.0))
    model.material("anodized_charcoal", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("zinc", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("matte_black", rgba=(0.12, 0.13, 0.14, 1.0))

    bracket = model.part("mounting_bracket")
    geom, origin = _box((0.006, 0.090, 0.150), (-0.033, 0.0, 0.0))
    bracket.visual(geom, origin=origin, material="powder_coat", name="bracket_body")
    geom, origin = _box((0.024, 0.050, 0.020), (-0.021, 0.0, -0.022))
    bracket.visual(geom, origin=origin, material="powder_coat", name="lower_spine")
    geom, origin = _box((0.024, 0.046, 0.014), (-0.021, 0.0, 0.022))
    bracket.visual(geom, origin=origin, material="powder_coat", name="upper_spine")
    geom, origin = _box((0.010, 0.050, 0.010), (-0.010, 0.0, -0.018))
    bracket.visual(geom, origin=origin, material="powder_coat", name="lower_bridge")
    geom, origin = _box((0.010, 0.044, 0.010), (-0.010, 0.0, 0.020))
    bracket.visual(geom, origin=origin, material="powder_coat", name="upper_bridge")
    geom, origin = _box((0.026, 0.008, 0.056), (0.005, 0.024, 0.0))
    bracket.visual(geom, origin=origin, material="powder_coat", name="left_cheek")
    geom, origin = _box((0.026, 0.008, 0.056), (0.005, -0.024, 0.0))
    bracket.visual(geom, origin=origin, material="powder_coat", name="right_cheek")
    geom, origin = _box((0.020, 0.004, 0.028), (0.005, 0.022, 0.0))
    bracket.visual(geom, origin=origin, material="powder_coat", name="left_cheek_pad")
    geom, origin = _box((0.020, 0.004, 0.028), (0.005, -0.022, 0.0))
    bracket.visual(geom, origin=origin, material="powder_coat", name="right_cheek_pad")
    geom, origin = _box((0.010, 0.010, 0.010), (0.019, 0.018, -0.012))
    bracket.visual(geom, origin=origin, material="powder_coat", name="lower_root_stop")
    geom, origin = _box((0.010, 0.010, 0.010), (-0.015, -0.018, 0.015))
    bracket.visual(geom, origin=origin, material="powder_coat", name="upper_root_stop")
    for y_pos in (-0.042, 0.042):
        for z_pos in (-0.050, 0.050):
            geom, origin = _x_cylinder(0.0045, 0.007, (-0.031, y_pos, z_pos))
            bracket.visual(
                geom,
                origin=origin,
                material="zinc",
                name=f"mount_bolt_{'n' if y_pos < 0 else 'p'}_{'l' if z_pos < 0 else 'u'}",
            )
    geom, origin = _y_cylinder(0.012, 0.007, (0.005, 0.0295, 0.0))
    bracket.visual(geom, origin=origin, material="zinc", name="left_root_cap")
    geom, origin = _y_cylinder(0.012, 0.007, (0.005, -0.0295, 0.0))
    bracket.visual(geom, origin=origin, material="zinc", name="right_root_cap")

    boom = model.part("boom_outer")
    geom, origin = _y_cylinder(BOOM_ROOT_BARREL_RADIUS, 0.040, (0.0, 0.0, 0.0))
    boom.visual(geom, origin=origin, material="machine_gray", name="root_barrel")
    geom, origin = _box((0.040, 0.016, 0.020), (0.020, 0.0, 0.0))
    boom.visual(geom, origin=origin, material="machine_gray", name="root_neck")
    geom, origin = _box((0.024, 0.056, 0.028), (0.048, 0.0, 0.0))
    boom.visual(geom, origin=origin, material="machine_gray", name="rear_collar")
    geom, origin = _box((0.246, 0.056, 0.004), (0.173, 0.0, 0.022))
    boom.visual(geom, origin=origin, material="machine_gray", name="top_wall")
    geom, origin = _box((0.246, 0.056, 0.004), (0.173, 0.0, -0.022))
    boom.visual(geom, origin=origin, material="machine_gray", name="bottom_wall")
    geom, origin = _box((0.246, 0.004, 0.044), (0.173, 0.026, 0.0))
    boom.visual(geom, origin=origin, material="machine_gray", name="left_wall")
    geom, origin = _box((0.246, 0.004, 0.044), (0.173, -0.026, 0.0))
    boom.visual(geom, origin=origin, material="machine_gray", name="right_wall")
    geom, origin = _box((0.220, 0.003, 0.020), (0.170, 0.0205, 0.0))
    boom.visual(geom, origin=origin, material="machine_gray", name="left_inner_guide")
    geom, origin = _box((0.220, 0.003, 0.020), (0.170, -0.0205, 0.0))
    boom.visual(geom, origin=origin, material="machine_gray", name="right_inner_guide")
    geom, origin = _box((0.100, 0.056, 0.002), (0.240, 0.0, 0.025))
    boom.visual(geom, origin=origin, material="machine_gray", name="slider_cover")
    geom, origin = _box((0.006, 0.008, 0.018), (0.296, 0.020, 0.0))
    boom.visual(geom, origin=origin, material="machine_gray", name="front_stop_face")
    geom, origin = _box((0.006, 0.008, 0.018), (0.296, -0.020, 0.0))
    boom.visual(geom, origin=origin, material="machine_gray", name="right_front_stop")
    geom, origin = _box((0.008, 0.006, 0.010), (0.010, 0.018, -0.012))
    boom.visual(geom, origin=origin, material="machine_gray", name="lower_root_stop_pip")
    geom, origin = _box((0.008, 0.006, 0.010), (0.010, -0.018, 0.012))
    boom.visual(geom, origin=origin, material="machine_gray", name="upper_root_stop_pip")

    stage = model.part("center_stage")
    geom, origin = _box((STAGE_CARRIAGE_LEN, STAGE_CARRIAGE_WIDTH, STAGE_CARRIAGE_HEIGHT), (0.035, 0.0, 0.0))
    stage.visual(geom, origin=origin, material="anodized_charcoal", name="stage_body")
    geom, origin = _box((0.006, 0.044, 0.016), (0.043, 0.0, 0.0))
    stage.visual(geom, origin=origin, material="anodized_charcoal", name="extension_stop")
    geom, origin = _box((STAGE_RAIL_LEN, STAGE_RAIL_WIDTH, STAGE_RAIL_HEIGHT), (0.165, 0.0, 0.0))
    stage.visual(geom, origin=origin, material="anodized_charcoal", name="rail")
    geom, origin = _box((0.020, 0.022, 0.014), (0.266, 0.0, 0.0))
    stage.visual(geom, origin=origin, material="anodized_charcoal", name="bridge")
    geom, origin = _box((0.026, 0.006, 0.032), (HEAD_JOINT_X, 0.014, 0.0))
    stage.visual(geom, origin=origin, material="anodized_charcoal", name="left_wrist_ear")
    geom, origin = _box((0.026, 0.006, 0.032), (HEAD_JOINT_X, -0.014, 0.0))
    stage.visual(geom, origin=origin, material="anodized_charcoal", name="right_wrist_ear")
    geom, origin = _box((0.010, 0.012, 0.006), (0.294, 0.012, -0.018))
    stage.visual(geom, origin=origin, material="anodized_charcoal", name="lower_tip_stop")
    geom, origin = _box((0.010, 0.012, 0.006), (0.293, -0.012, 0.018))
    stage.visual(geom, origin=origin, material="anodized_charcoal", name="upper_tip_stop")
    geom, origin = _y_cylinder(0.010, 0.007, (HEAD_JOINT_X, 0.0205, 0.0))
    stage.visual(geom, origin=origin, material="zinc", name="left_wrist_cap")
    geom, origin = _y_cylinder(0.010, 0.007, (HEAD_JOINT_X, -0.0205, 0.0))
    stage.visual(geom, origin=origin, material="zinc", name="right_wrist_cap")

    head = model.part("head_plate")
    geom, origin = _y_cylinder(HEAD_HUB_RADIUS, HEAD_HUB_LENGTH, (0.0, 0.0, 0.0))
    head.visual(geom, origin=origin, material="matte_black", name="hub")
    geom, origin = _box((0.024, 0.024, 0.032), (0.012, 0.0, 0.0))
    head.visual(geom, origin=origin, material="matte_black", name="rib")
    geom, origin = _box((HEAD_PLATE_THICK, HEAD_PLATE_WIDTH, HEAD_PLATE_HEIGHT), (0.026, 0.0, 0.0))
    head.visual(geom, origin=origin, material="matte_black", name="plate_body")
    for y_pos, z_pos in _head_hole_points():
        geom, origin = _x_cylinder(0.003, 0.006, (0.028, y_pos, z_pos))
        head.visual(
            geom,
            origin=origin,
            material="zinc",
            name=f"plate_fastener_{'n' if y_pos < 0 else 'p'}_{'l' if z_pos < 0 else 'u'}",
        )
    geom, origin = _box((0.008, 0.010, 0.006), (0.010, 0.010, -0.015))
    head.visual(geom, origin=origin, material="matte_black", name="lower_tip_stop_pip")
    geom, origin = _box((0.008, 0.010, 0.006), (0.012, -0.010, 0.014))
    head.visual(geom, origin=origin, material="matte_black", name="upper_tip_stop_pip")

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=boom,
        origin=Origin(),
        axis=ROOT_AXIS,
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=ROOT_LOWER, upper=ROOT_UPPER),
    )
    model.articulation(
        "center_extension",
        ArticulationType.PRISMATIC,
        parent=boom,
        child=stage,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.28, lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=stage,
        child=head,
        origin=Origin(xyz=(HEAD_JOINT_X, 0.0, 0.0)),
        axis=HEAD_AXIS,
        motion_limits=MotionLimits(effort=9.0, velocity=1.8, lower=HEAD_LOWER, upper=HEAD_UPPER),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("mounting_bracket")
    boom = object_model.get_part("boom_outer")
    stage = object_model.get_part("center_stage")
    head = object_model.get_part("head_plate")
    root = object_model.get_articulation("root_pivot")
    slide = object_model.get_articulation("center_extension")
    wrist = object_model.get_articulation("head_tilt")

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
        "boom articulation layout",
        _vec_close(root.axis, ROOT_AXIS)
        and _vec_close(slide.axis, (1.0, 0.0, 0.0))
        and _vec_close(wrist.axis, HEAD_AXIS)
        and root.motion_limits is not None
        and slide.motion_limits is not None
        and wrist.motion_limits is not None
        and abs((root.motion_limits.lower or 0.0) - ROOT_LOWER) <= 1e-6
        and abs((root.motion_limits.upper or 0.0) - ROOT_UPPER) <= 1e-6
        and abs((slide.motion_limits.upper or 0.0) - SLIDE_TRAVEL) <= 1e-6
        and abs((wrist.motion_limits.lower or 0.0) - HEAD_LOWER) <= 1e-6
        and abs((wrist.motion_limits.upper or 0.0) - HEAD_UPPER) <= 1e-6,
        details=(
            f"root_axis={root.axis}, slide_axis={slide.axis}, wrist_axis={wrist.axis}, "
            f"root_limits={root.motion_limits}, slide_limits={slide.motion_limits}, wrist_limits={wrist.motion_limits}"
        ),
    )

    ctx.expect_contact(boom, bracket, elem_a="root_barrel", elem_b="left_cheek_pad", contact_tol=1e-5, name="root barrel is carried by bracket cheek")
    ctx.expect_contact(
        stage,
        boom,
        elem_a="stage_body",
        elem_b="left_inner_guide",
        contact_tol=1e-5,
        name="stage carriage rides the guide rail",
    )
    ctx.expect_contact(head, stage, elem_a="hub", elem_b="left_wrist_ear", contact_tol=1e-5, name="head hub is captured in the wrist yoke")

    with ctx.pose({root: ROOT_LOWER}):
        ctx.expect_contact(
            boom,
            bracket,
            elem_a="lower_root_stop_pip",
            elem_b="lower_root_stop",
            contact_tol=8e-4,
            name="root lower hard stop seats at rest",
        )

    with ctx.pose({root: ROOT_UPPER}):
        ctx.expect_contact(
            boom,
            bracket,
            elem_a="upper_root_stop_pip",
            elem_b="upper_root_stop",
            contact_tol=1.2e-3,
            name="root upper hard stop seats at full lift",
        )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            boom,
            stage,
            elem_a="front_stop_face",
            elem_b="extension_stop",
            contact_tol=1e-5,
            name="center stage seats against the front extension stop",
        )

    with ctx.pose({wrist: HEAD_LOWER}):
        ctx.expect_contact(
            head,
            stage,
            elem_a="lower_tip_stop_pip",
            elem_b="lower_tip_stop",
            contact_tol=8e-4,
            name="tip lower hard stop seats",
        )

    with ctx.pose({wrist: HEAD_UPPER}):
        ctx.expect_contact(
            head,
            stage,
            elem_a="upper_tip_stop_pip",
            elem_b="upper_tip_stop",
            contact_tol=8e-4,
            name="tip upper hard stop seats",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
