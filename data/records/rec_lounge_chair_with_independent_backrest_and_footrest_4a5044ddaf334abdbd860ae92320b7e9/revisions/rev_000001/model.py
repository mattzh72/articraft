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


def _rounded_box_mesh(name: str, size: tuple[float, float, float], radius: float):
    """Small upholstered cushion block with softened manufactured edges."""
    sx, sy, sz = size
    model = cq.Workplane("XY").box(sx, sy, sz).edges().fillet(radius)
    return mesh_from_cadquery(model, name, tolerance=0.002, angular_tolerance=0.18)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reclining_lounge_chair")

    fabric = model.material("warm_taupe_fabric", rgba=(0.56, 0.47, 0.38, 1.0))
    seam_fabric = model.material("darker_seam_fabric", rgba=(0.34, 0.28, 0.23, 1.0))
    frame_metal = model.material("satin_black_metal", rgba=(0.06, 0.06, 0.06, 1.0))
    hinge_metal = model.material("brushed_hinge_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    underside = model.material("dark_underside_panel", rgba=(0.16, 0.12, 0.10, 1.0))

    seat_base = model.part("seat_base")
    seat_base.visual(
        Box((0.94, 0.76, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.4075)),
        material=underside,
        name="seat_pan",
    )
    seat_base.visual(
        _rounded_box_mesh("seat_cushion_rounded", (0.86, 0.70, 0.105), 0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.485)),
        material=fabric,
        name="seat_cushion",
    )
    # Subtle upholstered channels sunk slightly into the cushion top.
    for index, y in enumerate((-0.18, 0.18)):
        seat_base.visual(
            Box((0.76, 0.012, 0.009)),
            origin=Origin(xyz=(0.03, y, 0.539)),
            material=seam_fabric,
            name=f"seat_seam_{index}",
        )

    # Low welded support frame and legs.
    for x in (-0.36, 0.36):
        for y in (-0.30, 0.30):
            seat_base.visual(
                Box((0.055, 0.055, 0.365)),
                origin=Origin(xyz=(x, y, 0.20)),
                material=frame_metal,
                name=f"leg_{'front' if x > 0 else 'rear'}_{'pos' if y > 0 else 'neg'}",
            )
    for y in (-0.30, 0.30):
        seat_base.visual(
            Box((0.78, 0.045, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.215)),
            material=frame_metal,
            name=f"side_rail_{'pos' if y > 0 else 'neg'}",
        )
    for x in (-0.36, 0.36):
        seat_base.visual(
            Box((0.055, 0.64, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.215)),
            material=frame_metal,
            name=f"cross_rail_{'front' if x > 0 else 'rear'}",
        )

    # Fixed low arm rests and side posts, integrated into the base.
    for y in (-0.43, 0.43):
        side = "pos" if y > 0 else "neg"
        post_y = 0.39 if y > 0 else -0.39
        seat_base.visual(
            _rounded_box_mesh(f"armrest_{side}_rounded", (0.84, 0.08, 0.08), 0.018),
            origin=Origin(xyz=(0.04, y, 0.62)),
            material=fabric,
            name=f"armrest_{side}",
        )
        for x in (-0.32, 0.32):
            seat_base.visual(
                Box((0.055, 0.070, 0.245)),
                origin=Origin(xyz=(x, post_y, 0.51)),
                material=frame_metal,
                name=f"arm_post_{'front' if x > 0 else 'rear'}_{side}",
            )

    # Rear side pivot brackets for the backrest hinge.
    for y in (-0.400, 0.400):
        side = "pos" if y > 0 else "neg"
        seat_base.visual(
            Box((0.085, 0.045, 0.18)),
            origin=Origin(xyz=(-0.465, y, 0.505)),
            material=frame_metal,
            name=f"rear_pivot_bracket_{side}",
        )
        cap_y = y + (0.0285 if y > 0 else -0.0285)
        seat_base.visual(
            Cylinder(radius=0.038, length=0.012),
            origin=Origin(xyz=(-0.465, cap_y, 0.505), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"rear_pivot_cap_{side}",
        )

    # Front clevis plates for the independent, linkage-free footrest hinge.
    for y in (-0.39, 0.39):
        side = "pos" if y > 0 else "neg"
        seat_base.visual(
            Box((0.080, 0.040, 0.120)),
            origin=Origin(xyz=(0.485, y, 0.470)),
            material=frame_metal,
            name=f"front_hinge_plate_{side}",
        )
        seat_base.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(0.485, y * 1.01, 0.490), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"front_hinge_cap_{side}",
        )

    backrest = model.part("backrest")
    back_angle = math.radians(-5.0)
    back_dir = (math.sin(back_angle), 0.0, math.cos(back_angle))
    back_normal = (math.cos(back_angle), 0.0, -math.sin(back_angle))

    def _back_point(length_along: float, normal_offset: float = 0.0) -> tuple[float, float, float]:
        return (
            back_dir[0] * length_along + back_normal[0] * normal_offset,
            0.0,
            back_dir[2] * length_along + back_normal[2] * normal_offset,
        )

    backrest.visual(
        _rounded_box_mesh("back_cushion_rounded", (0.10, 0.66, 0.74), 0.030),
        origin=Origin(xyz=_back_point(0.37, -0.035), rpy=(0.0, back_angle, 0.0)),
        material=fabric,
        name="back_cushion",
    )
    backrest.visual(
        Box((0.028, 0.70, 0.77)),
        origin=Origin(xyz=_back_point(0.385, -0.090), rpy=(0.0, back_angle, 0.0)),
        material=underside,
        name="back_shell",
    )
    for y in (-0.355, 0.355):
        side = "pos" if y > 0 else "neg"
        p = _back_point(0.375, -0.075)
        backrest.visual(
            Box((0.050, 0.045, 0.76)),
            origin=Origin(xyz=(p[0], y, p[2]), rpy=(0.0, back_angle, 0.0)),
            material=frame_metal,
            name=f"back_side_rail_{side}",
        )
    for y in (-0.16, 0.16):
        p = _back_point(0.39, 0.020)
        backrest.visual(
            Box((0.010, 0.012, 0.64)),
            origin=Origin(xyz=(p[0], y, p[2]), rpy=(0.0, back_angle, 0.0)),
            material=seam_fabric,
            name=f"back_seam_{'pos' if y > 0 else 'neg'}",
        )
    backrest.visual(
        Cylinder(radius=0.027, length=0.755),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="rear_hinge_barrel",
    )

    footrest = model.part("footrest")
    footrest.visual(
        Box((0.065, 0.64, 0.48)),
        origin=Origin(xyz=(0.030, 0.0, -0.240)),
        material=underside,
        name="footrest_board",
    )
    footrest.visual(
        _rounded_box_mesh("foot_pad_rounded", (0.035, 0.60, 0.42), 0.018),
        origin=Origin(xyz=(0.075, 0.0, -0.265)),
        material=fabric,
        name="foot_pad",
    )
    for y in (-0.18, 0.18):
        footrest.visual(
            Box((0.010, 0.012, 0.36)),
            origin=Origin(xyz=(0.095, y, -0.265)),
            material=seam_fabric,
            name=f"foot_seam_{'pos' if y > 0 else 'neg'}",
        )
    footrest.visual(
        Cylinder(radius=0.025, length=0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="front_hinge_barrel",
    )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=seat_base,
        child=backrest,
        origin=Origin(xyz=(-0.465, 0.0, 0.505)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.8,
            lower=0.0,
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "footrest_hinge",
        ArticulationType.REVOLUTE,
        parent=seat_base,
        child=footrest,
        origin=Origin(xyz=(0.485, 0.0, 0.490)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    seat_base = object_model.get_part("seat_base")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")
    back_hinge = object_model.get_articulation("back_hinge")
    footrest_hinge = object_model.get_articulation("footrest_hinge")

    ctx.expect_gap(
        seat_base,
        backrest,
        axis="x",
        positive_elem="seat_cushion",
        negative_elem="back_cushion",
        min_gap=0.005,
        name="upright back cushion clears rear of seat",
    )
    ctx.expect_gap(
        footrest,
        seat_base,
        axis="x",
        positive_elem="footrest_board",
        negative_elem="seat_pan",
        min_gap=0.004,
        name="folded footrest is just ahead of seat pan",
    )

    back_limits = back_hinge.motion_limits
    foot_limits = footrest_hinge.motion_limits
    ctx.check(
        "backrest reclines from 95 to 130 degree seat angle",
        back_limits is not None
        and abs((back_limits.upper or 0.0) - math.radians(35.0)) < math.radians(1.0),
        details=f"limits={back_limits!r}",
    )
    ctx.check(
        "footrest rotates upward about 80 degrees",
        foot_limits is not None
        and abs((foot_limits.upper or 0.0) - math.radians(80.0)) < math.radians(1.0),
        details=f"limits={foot_limits!r}",
    )

    rest_back_aabb = ctx.part_element_world_aabb(backrest, elem="back_cushion")
    rest_foot_aabb = ctx.part_element_world_aabb(footrest, elem="foot_pad")
    with ctx.pose({back_hinge: math.radians(35.0), footrest_hinge: math.radians(80.0)}):
        reclined_back_aabb = ctx.part_element_world_aabb(backrest, elem="back_cushion")
        raised_foot_aabb = ctx.part_element_world_aabb(footrest, elem="foot_pad")
        ctx.expect_gap(
            footrest,
            seat_base,
            axis="x",
            positive_elem="foot_pad",
            negative_elem="seat_cushion",
            min_gap=0.010,
            name="raised footrest projects forward of fixed seat",
        )

    ctx.check(
        "backrest upper pose moves rearward",
        rest_back_aabb is not None
        and reclined_back_aabb is not None
        and float(reclined_back_aabb[0][0]) < float(rest_back_aabb[0][0]) - 0.18,
        details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
    )
    ctx.check(
        "footrest upper pose lifts and extends",
        rest_foot_aabb is not None
        and raised_foot_aabb is not None
        and float(raised_foot_aabb[1][0]) > float(rest_foot_aabb[1][0]) + 0.30
        and float(raised_foot_aabb[0][2]) > float(rest_foot_aabb[0][2]) + 0.25,
        details=f"rest={rest_foot_aabb}, raised={raised_foot_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
