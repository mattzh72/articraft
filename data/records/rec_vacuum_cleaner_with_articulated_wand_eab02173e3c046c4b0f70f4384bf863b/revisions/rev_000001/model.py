from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    """Add a cylinder whose local Z axis runs between two points in the XZ plane."""

    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # All authored wand/neck tubes lie in the part XZ plane, so one Y rotation
    # is enough to align the cylinder's local +Z axis to the requested vector.
    angle_y = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, angle_y, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_captured_barrel(part, *, radius: float, length: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_distal_fork(
    part,
    end: tuple[float, float, float],
    *,
    material,
    prefix: str,
    side_y: float = 0.055,
    inner_clearance: float = 0.040,
) -> None:
    """Add a simple clevis/fork behind a revolute hinge without occupying its center."""

    ex, _, ez = end
    for sign, side_name in ((1.0, "upper"), (-1.0, "lower")):
        part.visual(
            Box((0.070, 0.024, 0.070)),
            origin=Origin(xyz=(ex - 0.020, sign * side_y, ez)),
            material=material,
            name=f"{prefix}_{side_name}_fork_ear",
        )
    part.visual(
        Box((0.034, side_y * 2.0 + 0.024, 0.034)),
        origin=Origin(xyz=(ex - 0.066, 0.0, ez + 0.018)),
        material=material,
        name=f"{prefix}_fork_bridge",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    shell = model.material("warm_white_shell", rgba=(0.86, 0.88, 0.86, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.08, 0.09, 0.10, 1.0))
    dark = model.material("dark_rubber", rgba=(0.025, 0.025, 0.028, 1.0))
    metal = model.material("brushed_metal", rgba=(0.62, 0.65, 0.66, 1.0))
    clear_bin = model.material("smoked_clear_bin", rgba=(0.45, 0.62, 0.68, 0.42))
    accent = model.material("blue_accent", rgba=(0.08, 0.32, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(CapsuleGeometry(radius=0.125, length=0.42, radial_segments=36), "motor_pod"),
        origin=Origin(xyz=(-0.015, 0.0, 0.660)),
        material=shell,
        name="motor_pod",
    )
    body.visual(
        Cylinder(radius=0.078, length=0.330),
        origin=Origin(xyz=(0.095, 0.0, 0.650)),
        material=clear_bin,
        name="dust_bin",
    )
    body.visual(
        Box((0.210, 0.160, 0.125)),
        origin=Origin(xyz=(-0.020, 0.0, 0.345)),
        material=charcoal,
        name="battery_base",
    )
    body.visual(
        Box((0.030, 0.170, 0.155)),
        origin=Origin(xyz=(0.080, 0.0, 0.405)),
        material=accent,
        name="bin_release_band",
    )
    body.visual(
        Box((0.045, 0.070, 0.345)),
        origin=Origin(xyz=(-0.250, 0.0, 0.655)),
        material=charcoal,
        name="rear_grip",
    )
    body.visual(
        Box((0.230, 0.070, 0.045)),
        origin=Origin(xyz=(-0.145, 0.0, 0.840)),
        material=charcoal,
        name="top_handle_bridge",
    )
    body.visual(
        Box((0.205, 0.070, 0.040)),
        origin=Origin(xyz=(-0.150, 0.0, 0.495)),
        material=charcoal,
        name="lower_handle_bridge",
    )
    body.visual(
        Cylinder(radius=0.053, length=0.200),
        origin=Origin(xyz=(0.220, 0.0, 0.620), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="front_socket",
    )
    for sign, side_name in ((1.0, "upper"), (-1.0, "lower")):
        body.visual(
            Box((0.058, 0.025, 0.086)),
            origin=Origin(xyz=(0.346, sign * 0.055, 0.620)),
            material=charcoal,
            name=f"body_{side_name}_fork_ear",
        )

    upper_wand = model.part("upper_wand")
    _add_captured_barrel(
        upper_wand,
        radius=0.029,
        length=0.085,
        material=metal,
        name="proximal_barrel",
    )
    _cylinder_between(
        upper_wand,
        (0.014, 0.0, -0.005),
        (0.430, 0.0, -0.153),
        radius=0.020,
        material=metal,
        name="upper_tube",
    )
    upper_wand.visual(
        Cylinder(radius=0.027, length=0.070),
        origin=Origin(xyz=(0.080, 0.0, -0.028), rpy=(0.0, math.atan2(0.48, -0.17), 0.0)),
        material=charcoal,
        name="upper_grip_sleeve",
    )
    _add_distal_fork(
        upper_wand,
        (0.480, 0.0, -0.170),
        material=charcoal,
        prefix="middle",
        side_y=0.055,
    )

    lower_wand = model.part("lower_wand")
    _add_captured_barrel(
        lower_wand,
        radius=0.028,
        length=0.086,
        material=metal,
        name="middle_barrel",
    )
    _cylinder_between(
        lower_wand,
        (0.014, 0.0, -0.008),
        (0.500, 0.0, -0.268),
        radius=0.018,
        material=metal,
        name="lower_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(xyz=(0.160, 0.0, -0.086), rpy=(0.0, math.atan2(0.56, -0.30), 0.0)),
        material=charcoal,
        name="lower_sleeve",
    )
    _add_distal_fork(
        lower_wand,
        (0.560, 0.0, -0.300),
        material=charcoal,
        prefix="nozzle",
        side_y=0.075,
    )

    floor_nozzle = model.part("floor_nozzle")
    _add_captured_barrel(
        floor_nozzle,
        radius=0.034,
        length=0.126,
        material=charcoal,
        name="nozzle_barrel",
    )
    _cylinder_between(
        floor_nozzle,
        (0.020, 0.0, -0.015),
        (0.115, 0.0, -0.073),
        radius=0.025,
        material=charcoal,
        name="swivel_neck",
    )
    floor_nozzle.visual(
        Box((0.420, 0.185, 0.060)),
        origin=Origin(xyz=(0.165, 0.0, -0.115)),
        material=charcoal,
        name="nozzle_body",
    )
    floor_nozzle.visual(
        Box((0.440, 0.030, 0.034)),
        origin=Origin(xyz=(0.165, 0.100, -0.102)),
        material=dark,
        name="front_bumper",
    )
    floor_nozzle.visual(
        Box((0.360, 0.032, 0.012)),
        origin=Origin(xyz=(0.165, -0.075, -0.146)),
        material=dark,
        name="brush_strip",
    )
    floor_nozzle.visual(
        Box((0.260, 0.020, 0.006)),
        origin=Origin(xyz=(0.165, 0.000, -0.147)),
        material=accent,
        name="suction_mouth",
    )

    model.articulation(
        "body_elbow",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(xyz=(0.350, 0.0, 0.620)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-0.65, upper=0.75),
    )
    model.articulation(
        "wand_elbow",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.480, 0.0, -0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.85, upper=0.95),
    )
    model.articulation(
        "nozzle_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.560, 0.0, -0.300)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.75, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    upper = object_model.get_part("upper_wand")
    lower = object_model.get_part("lower_wand")
    nozzle = object_model.get_part("floor_nozzle")
    body_elbow = object_model.get_articulation("body_elbow")
    wand_elbow = object_model.get_articulation("wand_elbow")
    nozzle_pitch = object_model.get_articulation("nozzle_pitch")

    for link in (body, upper, lower, nozzle):
        ctx.check(f"{link.name}_present", link is not None, "Expected vacuum link.")
    for joint in (body_elbow, wand_elbow, nozzle_pitch):
        ctx.check(f"{joint.name}_present", joint is not None, "Expected revolute joint.")

    ctx.expect_origin_gap(nozzle, body, axis="x", min_gap=1.0, name="nozzle extends forward from body")
    ctx.expect_within(upper, body, axes="y", margin=0.05, name="upper wand stays on midline")
    ctx.expect_within(lower, body, axes="y", margin=0.05, name="lower wand stays on midline")
    ctx.expect_within(nozzle, body, axes="y", margin=0.12, name="nozzle centered near body midline")

    rest_nozzle = ctx.part_world_position(nozzle)
    rest_brush_aabb = ctx.part_element_world_aabb(nozzle, elem="brush_strip")
    with ctx.pose({body_elbow: 0.45, wand_elbow: -0.45, nozzle_pitch: 0.35}):
        flexed_nozzle = ctx.part_world_position(nozzle)
        ctx.expect_within(upper, body, axes="y", margin=0.05, name="flexed upper remains centered")
        ctx.expect_within(lower, body, axes="y", margin=0.05, name="flexed lower remains centered")
    with ctx.pose({nozzle_pitch: 0.45}):
        pitched_brush_aabb = ctx.part_element_world_aabb(nozzle, elem="brush_strip")

    ctx.check(
        "wand joints bend the chain",
        rest_nozzle is not None
        and flexed_nozzle is not None
        and abs(flexed_nozzle[0] - rest_nozzle[0]) > 0.05,
        details=f"rest={rest_nozzle}, flexed={flexed_nozzle}",
    )
    rest_brush_z = None if rest_brush_aabb is None else (rest_brush_aabb[0][2] + rest_brush_aabb[1][2]) * 0.5
    pitched_brush_z = (
        None if pitched_brush_aabb is None else (pitched_brush_aabb[0][2] + pitched_brush_aabb[1][2]) * 0.5
    )
    ctx.check(
        "nozzle hinge pitches floor head",
        rest_brush_z is not None and pitched_brush_z is not None and abs(pitched_brush_z - rest_brush_z) > 0.025,
        details=f"rest_brush_z={rest_brush_z}, pitched_brush_z={pitched_brush_z}",
    )

    return ctx.report()


object_model = build_object_model()
