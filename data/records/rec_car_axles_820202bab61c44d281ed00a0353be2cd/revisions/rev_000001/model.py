from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


Point = tuple[float, float, float]


def _v(p: Point) -> cq.Vector:
    return cq.Vector(p[0], p[1], p[2])


def _cyl_between(a: Point, b: Point, radius: float) -> cq.Solid:
    va = _v(a)
    direction = _v((b[0] - a[0], b[1] - a[1], b[2] - a[2]))
    length = direction.Length
    return cq.Solid.makeCylinder(radius, length, va, direction.normalized())


def _box(center: Point, size: Point) -> cq.Solid:
    p = (
        center[0] - size[0] / 2.0,
        center[1] - size[1] / 2.0,
        center[2] - size[2] / 2.0,
    )
    return cq.Solid.makeBox(size[0], size[1], size[2], _v(p))


def _sphere(center: Point, radius: float) -> cq.Solid:
    return cq.Solid.makeSphere(radius, _v(center))


def _union(solids: list[cq.Solid]) -> cq.Workplane:
    body = cq.Workplane("XY").newObject([solids[0]])
    for solid in solids[1:]:
        body = body.union(cq.Workplane("XY").newObject([solid]), clean=False)
    return body


def _wishbone_mesh(side: float, upper: bool) -> cq.Workplane:
    if upper:
        inner_y = 0.155
        outer_x = side * 0.305
        outer_z = -0.030
        tube_r = 0.016
        eye_r = 0.028
        ball_r = 0.030
    else:
        inner_y = 0.205
        outer_x = side * 0.360
        outer_z = 0.020
        tube_r = 0.021
        eye_r = 0.036
        ball_r = 0.038

    front = (0.0, inner_y, 0.0)
    rear = (0.0, -inner_y, 0.0)
    outer = (outer_x, 0.0, outer_z)
    brace = (side * 0.075, 0.0, 0.0)

    solids = [
        _cyl_between(front, outer, tube_r),
        _cyl_between(rear, outer, tube_r),
        _cyl_between(front, rear, tube_r * 0.85),
        _cyl_between(front, brace, tube_r * 0.70),
        _cyl_between(rear, brace, tube_r * 0.70),
        _cyl_between((0.0, inner_y - 0.040, 0.0), (0.0, inner_y + 0.040, 0.0), eye_r),
        _cyl_between((0.0, -inner_y - 0.040, 0.0), (0.0, -inner_y + 0.040, 0.0), eye_r),
        _sphere(outer, ball_r),
        _cyl_between(
            (outer_x - side * 0.020, 0.0, outer_z),
            (outer_x + side * 0.035, 0.0, outer_z),
            ball_r * 0.62,
        ),
    ]
    return _union(solids)


def _knuckle_mesh(side: float) -> cq.Workplane:
    lower = (0.0, 0.0, 0.0)
    upper = (-side * 0.130, 0.0, 0.230)
    hub = (side * 0.075, 0.0, 0.118)
    tab = (side * 0.025, 0.145, 0.105)

    solids = [
        _sphere(lower, 0.050),
        _sphere(upper, 0.044),
        _cyl_between((side * 0.010, 0.0, 0.015), (upper[0], 0.0, upper[2] - 0.010), 0.031),
        _cyl_between((side * 0.015, 0.0, 0.035), (hub[0] - side * 0.015, 0.0, hub[2]), 0.052),
        _cyl_between((hub[0] - side * 0.070, 0.0, hub[2]), (hub[0] + side * 0.020, 0.0, hub[2]), 0.073),
        _cyl_between((hub[0] - side * 0.010, 0.0, hub[2]), (hub[0] + side * 0.125, 0.0, hub[2]), 0.026),
        _box(tab, (0.125, 0.220, 0.030)),
        _cyl_between(
            (tab[0], tab[1] - 0.015, tab[2]),
            (tab[0], tab[1] + 0.095, tab[2]),
            0.026,
        ),
        _cyl_between(
            (tab[0], tab[1] + 0.070, tab[2] - 0.018),
            (tab[0], tab[1] + 0.070, tab[2] + 0.018),
            0.024,
        ),
    ]
    return _union(solids)


def _hub_mesh(side: float) -> cq.Workplane:
    solids: list[cq.Solid] = [
        _cyl_between((-side * 0.040, 0.0, 0.0), (-side * 0.005, 0.0, 0.0), 0.158),
        _cyl_between((-side * 0.020, 0.0, 0.0), (side * 0.030, 0.0, 0.0), 0.113),
        _cyl_between((side * 0.000, 0.0, 0.0), (side * 0.085, 0.0, 0.0), 0.065),
        _cyl_between((side * 0.070, 0.0, 0.0), (side * 0.105, 0.0, 0.0), 0.085),
        _cyl_between((side * 0.102, 0.0, 0.0), (side * 0.122, 0.0, 0.0), 0.036),
    ]
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0 + math.pi / 2.0
        y = 0.075 * math.cos(angle)
        z = 0.075 * math.sin(angle)
        solids.append(_cyl_between((side * 0.102, y, z), (side * 0.142, y, z), 0.0085))
        solids.append(_cyl_between((side * 0.123, y, z), (side * 0.137, y, z), 0.0140))
    return _union(solids)


def _subframe_mesh() -> cq.Workplane:
    solids: list[cq.Solid] = [
        _cyl_between((-0.470, 0.0, 0.240), (0.470, 0.0, 0.240), 0.044),
        _cyl_between((-0.420, 0.310, 0.025), (0.420, 0.310, 0.025), 0.030),
        _cyl_between((-0.420, -0.310, 0.025), (0.420, -0.310, 0.025), 0.030),
        _cyl_between((-0.315, 0.0, 0.455), (0.315, 0.0, 0.455), 0.030),
    ]

    for side in (-1.0, 1.0):
        x_low = side * 0.320
        x_up = side * 0.250
        solids.extend(
            [
                _cyl_between((side * 0.180, -0.315, 0.025), (side * 0.180, -0.035, 0.455), 0.026),
                _cyl_between((side * 0.180, 0.315, 0.025), (side * 0.180, 0.035, 0.455), 0.026),
                _box((side * 0.180, 0.0, 0.340), (0.045, 0.070, 0.240)),
            ]
        )
        for y in (-0.205, 0.205):
            y_sign = 1.0 if y > 0.0 else -1.0
            solids.extend(
                [
                    _box((x_low - side * 0.055, y, 0.100), (0.014, 0.092, 0.090)),
                    _box((x_low + side * 0.055, y, 0.100), (0.014, 0.092, 0.090)),
                    _box((x_low, y, 0.054), (0.120, 0.095, 0.022)),
                    _box((x_low, y + y_sign * 0.055, 0.054), (0.105, 0.090, 0.020)),
                    _cyl_between((x_low - side * 0.070, y, 0.145), (x_low + side * 0.070, y, 0.145), 0.010),
                ]
            )
        for y in (-0.155, 0.155):
            solids.extend(
                [
                    _box((x_up - side * 0.042, y, 0.380), (0.012, 0.075, 0.070)),
                    _box((x_up + side * 0.042, y, 0.380), (0.012, 0.075, 0.070)),
                    _box((x_up, y, 0.340), (0.095, 0.080, 0.020)),
                    _cyl_between((x_up - side * 0.052, y, 0.414), (x_up + side * 0.052, y, 0.414), 0.008),
                ]
            )
    return _union(solids)


def _norm(p: Point) -> Point:
    length = math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])
    return (p[0] / length, p[1] / length, p[2] / length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="independent_front_suspension_axle")

    model.material("powder_coat_black", rgba=(0.015, 0.016, 0.015, 1.0))
    model.material("satin_black_steel", rgba=(0.035, 0.037, 0.035, 1.0))
    model.material("forged_dark_iron", rgba=(0.11, 0.105, 0.095, 1.0))
    model.material("brake_steel", rgba=(0.42, 0.40, 0.37, 1.0))

    subframe = model.part("subframe")
    subframe.visual(
        mesh_from_cadquery(_subframe_mesh(), "subframe_weldment", tolerance=0.004),
        material="powder_coat_black",
        name="subframe_weldment",
    )

    side_data = [
        ("0", 1.0),
        ("1", -1.0),
    ]
    for suffix, side in side_data:
        lower = model.part(f"lower_wishbone_{suffix}")
        lower.visual(
            mesh_from_cadquery(_wishbone_mesh(side, upper=False), f"lower_wishbone_{suffix}", tolerance=0.003),
            material="satin_black_steel",
            name="lower_wishbone_forging",
        )

        upper = model.part(f"upper_wishbone_{suffix}")
        upper.visual(
            mesh_from_cadquery(_wishbone_mesh(side, upper=True), f"upper_wishbone_{suffix}", tolerance=0.003),
            material="satin_black_steel",
            name="upper_wishbone_forging",
        )

        knuckle = model.part(f"knuckle_{suffix}")
        knuckle.visual(
            mesh_from_cadquery(_knuckle_mesh(side), f"knuckle_{suffix}", tolerance=0.003),
            material="forged_dark_iron",
            name="knuckle_forging",
        )

        hub = model.part(f"hub_{suffix}")
        hub.visual(
            mesh_from_cadquery(_hub_mesh(side), f"hub_{suffix}", tolerance=0.0025),
            material="brake_steel",
            name="hub_rotor_lugs",
        )

        model.articulation(
            f"subframe_to_lower_{suffix}",
            ArticulationType.REVOLUTE,
            parent=subframe,
            child=lower,
            origin=Origin(xyz=(side * 0.320, 0.0, 0.100)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1600.0, velocity=1.5, lower=-0.18, upper=0.18),
        )
        model.articulation(
            f"subframe_to_upper_{suffix}",
            ArticulationType.REVOLUTE,
            parent=subframe,
            child=upper,
            origin=Origin(xyz=(side * 0.250, 0.0, 0.380)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=900.0, velocity=1.5, lower=-0.15, upper=0.15),
        )

        steering_axis = _norm((-side * 0.130, 0.0, 0.230))
        model.articulation(
            f"lower_to_knuckle_{suffix}",
            ArticulationType.REVOLUTE,
            parent=lower,
            child=knuckle,
            origin=Origin(xyz=(side * 0.360, 0.0, 0.020)),
            axis=steering_axis,
            motion_limits=MotionLimits(effort=450.0, velocity=2.0, lower=-0.55, upper=0.55),
        )
        model.articulation(
            f"knuckle_to_hub_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=knuckle,
            child=hub,
            origin=Origin(xyz=(side * 0.075, 0.0, 0.118)),
            axis=(side, 0.0, 0.0),
            motion_limits=MotionLimits(effort=300.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for suffix in ("0", "1"):
        subframe = object_model.get_part("subframe")
        lower = object_model.get_part(f"lower_wishbone_{suffix}")
        upper = object_model.get_part(f"upper_wishbone_{suffix}")
        knuckle = object_model.get_part(f"knuckle_{suffix}")
        hub = object_model.get_part(f"hub_{suffix}")

        ctx.allow_overlap(
            subframe,
            lower,
            elem_a="subframe_weldment",
            elem_b="lower_wishbone_forging",
            reason="The inboard lower-arm bushing sleeves are intentionally captured in the subframe clevis pockets.",
        )
        ctx.allow_overlap(
            subframe,
            upper,
            elem_a="subframe_weldment",
            elem_b="upper_wishbone_forging",
            reason="The inboard upper-arm bushing sleeves are intentionally captured in the subframe clevis pockets.",
        )
        ctx.allow_overlap(
            lower,
            knuckle,
            elem_a="lower_wishbone_forging",
            elem_b="knuckle_forging",
            reason="The lower ball is intentionally seated inside the forged knuckle socket.",
        )
        ctx.allow_overlap(
            upper,
            knuckle,
            elem_a="upper_wishbone_forging",
            elem_b="knuckle_forging",
            reason="The upper ball is intentionally seated inside the forged knuckle socket.",
        )
        ctx.allow_overlap(
            knuckle,
            hub,
            elem_a="knuckle_forging",
            elem_b="hub_rotor_lugs",
            reason="The hub bearing body is intentionally modeled around the spindle.",
        )

        ctx.expect_overlap(
            subframe,
            lower,
            axes="xyz",
            min_overlap=0.030,
            elem_a="subframe_weldment",
            elem_b="lower_wishbone_forging",
            name=f"lower bushing captured by subframe {suffix}",
        )
        ctx.expect_overlap(
            subframe,
            upper,
            axes="xyz",
            min_overlap=0.025,
            elem_a="subframe_weldment",
            elem_b="upper_wishbone_forging",
            name=f"upper bushing captured by subframe {suffix}",
        )
        ctx.expect_overlap(
            lower,
            knuckle,
            axes="xyz",
            min_overlap=0.025,
            elem_a="lower_wishbone_forging",
            elem_b="knuckle_forging",
            name=f"lower ball retained in socket {suffix}",
        )
        ctx.expect_overlap(
            upper,
            knuckle,
            axes="xyz",
            min_overlap=0.015,
            elem_a="upper_wishbone_forging",
            elem_b="knuckle_forging",
            name=f"upper ball retained in socket {suffix}",
        )
        ctx.expect_overlap(
            knuckle,
            hub,
            axes="x",
            min_overlap=0.020,
            elem_a="knuckle_forging",
            elem_b="hub_rotor_lugs",
            name=f"hub bearing overlaps spindle {suffix}",
        )
        ctx.expect_origin_distance(
            knuckle,
            hub,
            axes="y",
            max_dist=0.001,
            name=f"hub and spindle share center plane {suffix}",
        )

        steer = object_model.get_articulation(f"lower_to_knuckle_{suffix}")
        rest = ctx.part_world_position(hub)
        with ctx.pose({steer: 0.45}):
            steered = ctx.part_world_position(hub)
        ctx.check(
            f"steering swings hub {suffix}",
            rest is not None and steered is not None and abs(steered[1] - rest[1]) > 0.020,
            details=f"rest={rest}, steered={steered}",
        )

    return ctx.report()


object_model = build_object_model()
