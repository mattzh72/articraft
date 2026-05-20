from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A soft rectangular cushion-like solid centered on the local origin."""
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges().fillet(radius)


def _rot_y_point(x: float, z: float, angle: float) -> tuple[float, float]:
    """Rotate a local X/Z offset by a pitch angle around local Y."""
    return (
        x * math.cos(angle) + z * math.sin(angle),
        -x * math.sin(angle) + z * math.cos(angle),
    )


def _aabb_center_axis(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][axis_index] + aabb[1][axis_index]) * 0.5


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="padded_chaise_recliner")

    upholstery = model.material("warm_taupe_upholstery", rgba=(0.63, 0.52, 0.42, 1.0))
    seam = model.material("slightly_darker_piping", rgba=(0.35, 0.28, 0.22, 1.0))
    frame_mat = model.material("matte_black_frame", rgba=(0.055, 0.052, 0.048, 1.0))
    metal = model.material("brushed_steel_pivots", rgba=(0.62, 0.60, 0.56, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_rounded_box((0.86, 0.72, 0.14), 0.045), "seat_cushion_mesh"),
        origin=Origin(xyz=(0.10, 0.0, 0.43)),
        material=upholstery,
        name="seat_cushion",
    )
    for y in (-0.46, 0.46):
        suffix = 0 if y < 0 else 1
        base.visual(
            mesh_from_cadquery(_rounded_box((1.06, 0.22, 0.31), 0.045), f"side_arm_mesh_{suffix}"),
            origin=Origin(xyz=(0.10, y, 0.49)),
            material=upholstery,
            name=f"side_arm_{suffix}",
        )
        base.visual(
            Box((0.74, 0.018, 0.030)),
            origin=Origin(xyz=(0.13, y * 0.985, 0.655)),
            material=seam,
            name=f"arm_top_piping_{suffix}",
        )

    base.visual(
        Box((0.96, 0.88, 0.11)),
        origin=Origin(xyz=(0.10, 0.0, 0.285)),
        material=frame_mat,
        name="low_plinth",
    )
    for x in (-0.28, 0.48):
        for y in (-0.34, 0.34):
            base.visual(
                Cylinder(radius=0.050, length=0.24),
                origin=Origin(xyz=(x, y, 0.125)),
                material=frame_mat,
                name=f"short_foot_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )

    # Fixed hinge bosses protrude from the inside of the broad arms so the moving
    # backrest and footrest read as mounted between those arms.
    for y in (-0.345, 0.345):
        suffix = 0 if y < 0 else 1
        base.visual(
            Cylinder(radius=0.043, length=0.070),
            origin=Origin(xyz=(-0.34, y, 0.555), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"back_pivot_boss_{suffix}",
        )
        base.visual(
            Cylinder(radius=0.040, length=0.070),
            origin=Origin(xyz=(0.580, y, 0.435), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"foot_pivot_boss_{suffix}",
        )

    back = model.part("back_panel")
    back_angle = math.radians(-12.0)
    back_half = 0.41
    back_x, back_z = _rot_y_point(0.0, back_half, back_angle)
    back.visual(
        mesh_from_cadquery(_rounded_box((0.135, 0.58, 0.82), 0.045), "back_cushion_mesh"),
        origin=Origin(xyz=(back_x, 0.0, back_z), rpy=(0.0, back_angle, 0.0)),
        material=upholstery,
        name="back_cushion",
    )
    head_x, head_z = _rot_y_point(0.076, 0.68, back_angle)
    back.visual(
        mesh_from_cadquery(_rounded_box((0.060, 0.52, 0.17), 0.020), "head_pillow_mesh"),
        origin=Origin(xyz=(head_x, 0.0, head_z), rpy=(0.0, back_angle, 0.0)),
        material=upholstery,
        name="head_pillow",
    )
    lumbar_x, lumbar_z = _rot_y_point(0.060, 0.34, back_angle)
    back.visual(
        Box((0.035, 0.54, 0.022)),
        origin=Origin(xyz=(lumbar_x, 0.0, lumbar_z), rpy=(0.0, back_angle, 0.0)),
        material=seam,
        name="lumbar_seam",
    )
    back.visual(
        Cylinder(radius=0.030, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="back_hinge_barrel",
    )

    footrest = model.part("footrest_panel")
    footrest.visual(
        mesh_from_cadquery(_rounded_box((0.92, 0.64, 0.12), 0.045), "foot_panel_mesh"),
        origin=Origin(xyz=(0.49, 0.0, 0.0)),
        material=upholstery,
        name="foot_panel",
    )
    footrest.visual(
        Cylinder(radius=0.034, length=0.60),
        origin=Origin(xyz=(0.92, 0.0, 0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seam,
        name="front_roll",
    )
    footrest.visual(
        Box((0.022, 0.58, 0.016)),
        origin=Origin(xyz=(0.24, 0.0, 0.063)),
        material=seam,
        name="footrest_seam",
    )
    footrest.visual(
        Cylinder(radius=0.030, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="foot_hinge_barrel",
    )

    model.articulation(
        "base_to_back",
        ArticulationType.REVOLUTE,
        parent=base,
        child=back,
        origin=Origin(xyz=(-0.34, 0.0, 0.555)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.8, lower=0.0, upper=math.radians(35.0)),
    )
    model.articulation(
        "base_to_footrest",
        ArticulationType.REVOLUTE,
        parent=base,
        child=footrest,
        origin=Origin(xyz=(0.580, 0.0, 0.435)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=math.radians(-70.0), upper=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    back = object_model.get_part("back_panel")
    footrest = object_model.get_part("footrest_panel")
    back_joint = object_model.get_articulation("base_to_back")
    foot_joint = object_model.get_articulation("base_to_footrest")

    ctx.check(
        "backrest has about 35 degrees of recline",
        abs((back_joint.motion_limits.upper - back_joint.motion_limits.lower) - math.radians(35.0)) < math.radians(1.0),
    )
    ctx.check(
        "footrest has about 70 degrees of travel",
        abs((foot_joint.motion_limits.upper - foot_joint.motion_limits.lower) - math.radians(70.0)) < math.radians(1.0),
    )
    ctx.expect_within(
        back,
        base,
        axes="y",
        inner_elem="back_cushion",
        outer_elem="seat_cushion",
        margin=0.045,
        name="backrest fits between the side arms",
    )
    ctx.expect_within(
        footrest,
        base,
        axes="y",
        inner_elem="foot_panel",
        outer_elem="seat_cushion",
        margin=0.045,
        name="footrest fits between the side arms",
    )
    ctx.expect_gap(
        footrest,
        base,
        axis="x",
        positive_elem="foot_panel",
        negative_elem="seat_cushion",
        min_gap=0.010,
        max_gap=0.080,
        name="footrest is mounted just ahead of the seat front",
    )

    back_rest_x = _aabb_center_axis(ctx.part_element_world_aabb(back, elem="back_cushion"), 0)
    foot_extended_z = _aabb_center_axis(ctx.part_element_world_aabb(footrest, elem="foot_panel"), 2)
    with ctx.pose({back_joint: math.radians(35.0), foot_joint: math.radians(-70.0)}):
        back_reclined_x = _aabb_center_axis(ctx.part_element_world_aabb(back, elem="back_cushion"), 0)
        foot_stowed_z = _aabb_center_axis(ctx.part_element_world_aabb(footrest, elem="foot_panel"), 2)

    ctx.check(
        "backrest joint reclines rearward",
        back_rest_x is not None
        and back_reclined_x is not None
        and back_reclined_x < back_rest_x - 0.10,
        details=f"rest_x={back_rest_x}, reclined_x={back_reclined_x}",
    )
    ctx.check(
        "footrest joint swings the panel downward",
        foot_extended_z is not None
        and foot_stowed_z is not None
        and foot_stowed_z < foot_extended_z - 0.25,
        details=f"extended_z={foot_extended_z}, stowed_z={foot_stowed_z}",
    )

    return ctx.report()


object_model = build_object_model()
