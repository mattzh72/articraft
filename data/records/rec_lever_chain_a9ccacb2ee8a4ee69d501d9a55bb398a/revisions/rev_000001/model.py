from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, degrees, hypot

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


PLATE_T = 0.008
BASE_T = 0.010
INPUT_Z0 = 0.014
COUPLER_Z0 = 0.028
OUTPUT_Z0 = 0.014

JOINT_HOLE_R = 0.011
MOUNT_HOLE_R = 0.0055
TAB_HOLE_R = 0.0045
COLLAR_T = 0.002
SPACER_R = 0.0165
TOP_BOSS_R = 0.0175

INPUT_END = (0.165, 0.065)
COUPLER_END = (0.110, -0.018)
COUPLER_LOCAL_END = (COUPLER_END[0] - INPUT_END[0], COUPLER_END[1] - INPUT_END[1])


def cylinder_at(x: float, y: float, radius: float, z0: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z0))


def box_at(
    x: float,
    y: float,
    sx: float,
    sy: float,
    z0: float,
    height: float,
    angle_deg: float = 0.0,
) -> cq.Workplane:
    body = cq.Workplane("XY").rect(sx, sy).extrude(height)
    if angle_deg:
        body = body.rotate((0, 0, 0), (0, 0, 1), angle_deg)
    return body.translate((x, y, z0))


def bar_between(
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    z0: float,
    height: float,
) -> cq.Workplane:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = hypot(dx, dy)
    angle_deg = degrees(atan2(dy, dx))
    mx = 0.5 * (start[0] + end[0])
    my = 0.5 * (start[1] + end[1])
    return box_at(mx, my, length, width, z0, height, angle_deg=angle_deg)


def union_all(solids: list[cq.Workplane]) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def make_base_lug() -> cq.Workplane:
    body = union_all(
        [
            cylinder_at(0.000, 0.000, 0.030, 0.000, BASE_T),
            cylinder_at(-0.060, 0.034, 0.020, 0.000, BASE_T),
            cylinder_at(-0.060, -0.034, 0.020, 0.000, BASE_T),
            cylinder_at(-0.022, 0.000, 0.026, 0.000, BASE_T),
            bar_between((-0.060, 0.034), (-0.010, 0.014), 0.024, 0.000, BASE_T),
            bar_between((-0.060, -0.034), (-0.010, -0.014), 0.024, 0.000, BASE_T),
            box_at(-0.041, 0.000, 0.055, 0.034, 0.000, BASE_T),
            cylinder_at(0.000, 0.000, SPACER_R, BASE_T, INPUT_Z0 - BASE_T),
        ]
    )
    body = body.cut(cylinder_at(-0.060, 0.034, MOUNT_HOLE_R, -0.001, BASE_T + 0.002))
    body = body.cut(cylinder_at(-0.060, -0.034, MOUNT_HOLE_R, -0.001, BASE_T + 0.002))
    return body


def make_input_lever() -> cq.Workplane:
    end_x, end_y = INPUT_END
    body = union_all(
        [
            cylinder_at(0.000, 0.000, 0.030, INPUT_Z0, PLATE_T),
            cylinder_at(0.090, 0.050, 0.018, INPUT_Z0, PLATE_T),
            cylinder_at(end_x, end_y, 0.021, INPUT_Z0, PLATE_T),
            cylinder_at(0.000, 0.000, TOP_BOSS_R, INPUT_Z0 + PLATE_T, COLLAR_T),
            bar_between((0.000, 0.000), (0.090, 0.050), 0.024, INPUT_Z0, PLATE_T),
            bar_between((0.090, 0.050), INPUT_END, 0.020, INPUT_Z0, PLATE_T),
            bar_between((0.010, -0.001), INPUT_END, 0.016, INPUT_Z0, PLATE_T),
            cylinder_at(end_x, end_y, SPACER_R, INPUT_Z0 + PLATE_T, COUPLER_Z0 - (INPUT_Z0 + PLATE_T)),
        ]
    )
    body = body.cut(cylinder_at(0.075, -0.070, 0.070, INPUT_Z0 - 0.001, PLATE_T + 0.002))
    body = body.cut(cylinder_at(0.000, 0.000, JOINT_HOLE_R, INPUT_Z0 - 0.001, PLATE_T + 0.002))
    body = body.cut(cylinder_at(end_x, end_y, JOINT_HOLE_R, INPUT_Z0 - 0.001, PLATE_T + 0.002))
    return body


def make_coupler() -> cq.Workplane:
    end_x, end_y = COUPLER_LOCAL_END
    body = union_all(
        [
            cylinder_at(0.000, 0.000, 0.019, COUPLER_Z0, PLATE_T),
            cylinder_at(-0.024, -0.030, 0.012, COUPLER_Z0, PLATE_T),
            cylinder_at(end_x, end_y, 0.018, COUPLER_Z0, PLATE_T),
            cylinder_at(0.000, 0.000, TOP_BOSS_R, COUPLER_Z0 + PLATE_T, COLLAR_T),
            cylinder_at(end_x, end_y, TOP_BOSS_R, COUPLER_Z0 + PLATE_T, COLLAR_T),
            bar_between((0.000, 0.000), (-0.024, -0.030), 0.020, COUPLER_Z0, PLATE_T),
            bar_between((-0.024, -0.030), COUPLER_LOCAL_END, 0.018, COUPLER_Z0, PLATE_T),
        ]
    )
    body = body.cut(cylinder_at(-0.004, 0.032, 0.036, COUPLER_Z0 - 0.001, PLATE_T + 0.002))
    body = body.cut(cylinder_at(0.000, 0.000, JOINT_HOLE_R, COUPLER_Z0 - 0.001, PLATE_T + 0.002))
    body = body.cut(cylinder_at(end_x, end_y, JOINT_HOLE_R, COUPLER_Z0 - 0.001, PLATE_T + 0.002))
    return body


def make_output_lever() -> cq.Workplane:
    body = union_all(
        [
            cylinder_at(0.000, 0.000, 0.024, OUTPUT_Z0, PLATE_T),
            cylinder_at(0.132, 0.010, 0.015, OUTPUT_Z0, PLATE_T),
            cylinder_at(0.262, 0.028, 0.018, OUTPUT_Z0, PLATE_T),
            cylinder_at(0.000, 0.000, SPACER_R, OUTPUT_Z0 + PLATE_T, COUPLER_Z0 - (OUTPUT_Z0 + PLATE_T)),
            bar_between((0.000, 0.000), (0.132, 0.010), 0.024, OUTPUT_Z0, PLATE_T),
            bar_between((0.132, 0.010), (0.220, 0.022), 0.022, OUTPUT_Z0, PLATE_T),
            box_at(0.240, 0.028, 0.050, 0.036, OUTPUT_Z0, PLATE_T),
        ]
    )
    body = body.cut(cylinder_at(0.102, -0.045, 0.050, OUTPUT_Z0 - 0.001, PLATE_T + 0.002))
    body = body.cut(cylinder_at(0.000, 0.000, JOINT_HOLE_R, OUTPUT_Z0 - 0.001, PLATE_T + 0.002))
    body = body.cut(cylinder_at(0.242, 0.028, TAB_HOLE_R, OUTPUT_Z0 - 0.001, PLATE_T + 0.002))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_lever_linkage")

    base_mat = model.material("base_mat", color=(0.22, 0.22, 0.24, 1.0))
    input_mat = model.material("input_mat", color=(0.36, 0.45, 0.56, 1.0))
    coupler_mat = model.material("coupler_mat", color=(0.64, 0.65, 0.67, 1.0))
    output_mat = model.material("output_mat", color=(0.53, 0.54, 0.57, 1.0))

    base = model.part("base_lug")
    base.visual(
        mesh_from_cadquery(make_base_lug(), "base_lug"),
        origin=Origin(),
        material=base_mat,
        name="base_lug_body",
    )

    input_lever = model.part("input_lever")
    input_lever.visual(
        mesh_from_cadquery(make_input_lever(), "input_lever"),
        origin=Origin(),
        material=input_mat,
        name="input_body",
    )

    coupler = model.part("coupler_lever")
    coupler.visual(
        mesh_from_cadquery(make_coupler(), "coupler_lever"),
        origin=Origin(),
        material=coupler_mat,
        name="coupler_body",
    )

    output_lever = model.part("output_lever")
    output_lever.visual(
        mesh_from_cadquery(make_output_lever(), "output_lever"),
        origin=Origin(),
        material=output_mat,
        name="output_body",
    )

    base_to_input = model.articulation(
        "base_to_input",
        ArticulationType.REVOLUTE,
        parent=base,
        child=input_lever,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.2, upper=1.1),
    )
    input_to_coupler = model.articulation(
        "input_to_coupler",
        ArticulationType.REVOLUTE,
        parent=input_lever,
        child=coupler,
        origin=Origin(xyz=(INPUT_END[0], INPUT_END[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9.0, velocity=3.0, lower=-1.5, upper=1.5),
    )
    model.articulation(
        "coupler_to_output",
        ArticulationType.REVOLUTE,
        parent=coupler,
        child=output_lever,
        origin=Origin(xyz=(COUPLER_LOCAL_END[0], COUPLER_LOCAL_END[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-1.4, upper=1.4),
    )

    model.meta["primary_articulations"] = (base_to_input.name, input_to_coupler.name, "coupler_to_output")
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_lug")
    input_lever = object_model.get_part("input_lever")
    coupler = object_model.get_part("coupler_lever")
    output_lever = object_model.get_part("output_lever")

    base_to_input = object_model.get_articulation("base_to_input")
    input_to_coupler = object_model.get_articulation("input_to_coupler")
    coupler_to_output = object_model.get_articulation("coupler_to_output")

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
        "all joints are revolute about +Z",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE and joint.axis == (0.0, 0.0, 1.0)
            for joint in (base_to_input, input_to_coupler, coupler_to_output)
        ),
        details="Expected three parallel in-plane revolute joints with +Z axes.",
    )
    ctx.check(
        "joint origins stay on common z plane",
        all(joint.origin.xyz[2] == 0.0 for joint in (base_to_input, input_to_coupler, coupler_to_output)),
        details="All three articulation origins should lie on the same axis plane.",
    )

    ctx.expect_contact(base, input_lever, contact_tol=5e-4, name="base lug clamps input lever")
    ctx.expect_contact(input_lever, coupler, contact_tol=5e-4, name="input lever clamps coupler")
    ctx.expect_contact(coupler, output_lever, contact_tol=5e-4, name="coupler clamps output lever")

    with ctx.pose({base_to_input: 0.55}):
        ctx.expect_contact(base, input_lever, contact_tol=5e-4, name="base-input pin stays engaged when driven")
    with ctx.pose({base_to_input: 0.30, input_to_coupler: -0.60}):
        ctx.expect_contact(input_lever, coupler, contact_tol=5e-4, name="coupler pin stays engaged in motion")
    with ctx.pose({base_to_input: 0.20, input_to_coupler: -0.40, coupler_to_output: 0.60}):
        ctx.expect_contact(coupler, output_lever, contact_tol=5e-4, name="output pin stays engaged in motion")

    output_aabb = ctx.part_element_world_aabb(output_lever, elem="output_body")
    output_origin = ctx.part_world_position(output_lever)
    tab_reach = None
    if output_aabb is not None and output_origin is not None:
        tab_reach = output_aabb[1][0] - output_origin[0]
    ctx.check(
        "output lever projects a long tab beyond its pivot",
        tab_reach is not None and tab_reach >= 0.20,
        details=f"Output lever reach was {tab_reach!r}; expected at least 0.20 m beyond the pivot.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
