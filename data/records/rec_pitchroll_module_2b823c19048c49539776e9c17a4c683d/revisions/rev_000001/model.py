from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder(
    axis: str,
    length: float,
    radius: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    """Centered CadQuery cylinder along a principal axis."""
    cyl = cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -length / 2.0))
    if axis == "x":
        cyl = cyl.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    elif axis == "y":
        cyl = cyl.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    elif axis != "z":
        raise ValueError(f"unsupported axis {axis!r}")
    return cyl.translate(center)


def _ring(
    axis: str,
    length: float,
    outer_radius: float,
    inner_radius: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    body = _cylinder(axis, length, outer_radius, center)
    bore = _cylinder(axis, length + 0.012, inner_radius, center)
    return body.cut(bore)


def _outer_frame_body() -> cq.Workplane:
    outer = 0.210
    bar = 0.026
    depth = 0.052
    half = outer / 2.0

    body = _box((depth, outer, bar), (0.0, 0.0, half - bar / 2.0))
    body = body.union(_box((depth, outer, bar), (0.0, 0.0, -half + bar / 2.0)))
    body = body.union(_box((depth, bar, outer), (0.0, half - bar / 2.0, 0.0)))
    body = body.union(_box((depth, bar, outer), (0.0, -half + bar / 2.0, 0.0)))

    # Angled corner ribs are set into the square frame, leaving an open central window.
    rib_len = 0.052
    rib_w = 0.010
    rib_centers = [
        (0.0, 0.068, 0.068, 45.0),
        (0.0, -0.068, 0.068, -45.0),
        (0.0, 0.068, -0.068, -45.0),
        (0.0, -0.068, -0.068, 45.0),
    ]
    for x, y, z, angle in rib_centers:
        rib = _box((depth + 0.004, rib_w, rib_len), (x, y, z)).rotate(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle
        )
        body = body.union(rib)

    # Pitch bearing bosses on the two side rails.
    for y in (-0.092, 0.092):
        body = body.union(_ring("y", 0.034, 0.030, 0.015, (0.0, y, 0.0)))

    # Rear roll journal: a large clear-bore cartridge ring supported from the back of the square.
    body = body.union(_ring("x", 0.030, 0.060, 0.047, (-0.054, 0.0, 0.0)))
    # A thin thrust shoulder kisses the fixed bearing face so the roll frame is visibly captured
    # while the journal itself still has radial clearance inside the bearing bore.
    body = body.union(_ring("x", 0.006, 0.076, 0.054, (-0.066, 0.0, 0.0)))
    for y, z, sy, sz in [
        (0.0, 0.070, 0.016, 0.030),
        (0.0, -0.070, 0.016, 0.030),
        (0.070, 0.0, 0.030, 0.016),
        (-0.070, 0.0, 0.030, 0.016),
    ]:
        body = body.union(_box((0.024, sy, sz), (-0.036, y, z)))

    # Bore through the pitch bearing line after all side boss material is in place.
    body = body.cut(_cylinder("y", 0.250, 0.015, (0.0, 0.0, 0.0)))
    return body


def _roll_bearing_body() -> cq.Workplane:
    ring = _ring("x", 0.034, 0.078, 0.063, (-0.086, 0.0, 0.0))
    base = _box((0.086, 0.066, 0.018), (-0.112, 0.0, -0.146))
    post = _box((0.028, 0.038, 0.074), (-0.088, 0.0, -0.112))
    gusset_a = _box((0.026, 0.012, 0.072), (-0.108, 0.032, -0.112)).rotate(
        (-0.108, 0.0, -0.112), (-0.108, 1.0, -0.112), 14.0
    )
    gusset_b = _box((0.026, 0.012, 0.072), (-0.108, -0.032, -0.112)).rotate(
        (-0.108, 0.0, -0.112), (-0.108, 1.0, -0.112), -14.0
    )
    return ring.union(base).union(post).union(gusset_a).union(gusset_b)


def _inner_barrel_body() -> cq.Workplane:
    body = _ring("x", 0.060, 0.044, 0.027, (0.0, 0.0, 0.0))
    body = body.union(_ring("x", 0.008, 0.050, 0.030, (0.033, 0.0, 0.0)))
    body = body.union(_ring("x", 0.008, 0.048, 0.030, (-0.033, 0.0, 0.0)))

    # Trunnion tube along the pitch axis.  It is continuous through the cradle wall.
    body = body.union(_cylinder("y", 0.158, 0.010, (0.0, 0.0, 0.0)))
    for y in (-0.063, 0.063):
        body = body.union(_cylinder("y", 0.018, 0.014, (0.0, y, 0.0)))

    # Open lightening pockets on the top and bottom of the barrel, away from the trunnions.
    pocket = _box((0.076, 0.030, 0.022), (0.0, 0.0, 0.044))
    body = body.cut(pocket).cut(_box((0.076, 0.030, 0.022), (0.0, 0.0, -0.044)))
    return body


def _center_flange_body() -> cq.Workplane:
    flange = _box((0.010, 0.064, 0.064), (0.039, 0.0, 0.0))
    flange = flange.cut(_cylinder("x", 0.018, 0.010, (0.039, 0.0, 0.0)))
    for y in (-0.022, 0.022):
        for z in (-0.022, 0.022):
            flange = flange.cut(_cylinder("x", 0.018, 0.0042, (0.039, y, z)))
    return flange


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_gimbal_cube")

    dark = model.material("black_hard_anodized", rgba=(0.025, 0.027, 0.030, 1.0))
    graphite = model.material("graphite_bearing", rgba=(0.18, 0.18, 0.17, 1.0))
    satin = model.material("satin_aluminum", rgba=(0.72, 0.70, 0.64, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    amber = model.material("bronze_retainer", rgba=(0.74, 0.46, 0.18, 1.0))

    roll_carrier = model.part("roll_carrier")
    roll_carrier.visual(
        mesh_from_cadquery(_roll_bearing_body(), "roll_carrier_body", tolerance=0.0008),
        material=graphite,
        name="bearing_saddle",
    )
    roll_carrier.visual(
        Cylinder(radius=0.0045, length=0.142),
        origin=Origin(xyz=(-0.072, 0.0, -0.112), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="base_dowel",
    )

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        mesh_from_cadquery(_outer_frame_body(), "outer_frame_body", tolerance=0.0008),
        material=dark,
        name="frame_body",
    )
    outer_frame.visual(
        mesh_from_cadquery(_ring("x", 0.006, 0.062, 0.048, (-0.036, 0.0, 0.0)), "roll_retainer"),
        material=amber,
        name="roll_retainer",
    )
    for y, name in [(-0.092, "pitch_bushing_0"), (0.092, "pitch_bushing_1")]:
        outer_frame.visual(
            mesh_from_cadquery(_ring("y", 0.010, 0.022, 0.015, (0.0, y, 0.0)), name),
            material=steel,
            name=name,
        )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        mesh_from_cadquery(_inner_barrel_body(), "inner_barrel_body", tolerance=0.0007),
        material=satin,
        name="barrel_cradle",
    )
    inner_cradle.visual(
        mesh_from_cadquery(_center_flange_body(), "center_flange", tolerance=0.0007),
        material=steel,
        name="center_flange",
    )
    inner_cradle.visual(
        Cylinder(radius=0.004, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=amber,
        name="trunnion_pin",
    )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=roll_carrier,
        child=outer_frame,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=inner_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-0.52, upper=0.52),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roll_carrier = object_model.get_part("roll_carrier")
    outer_frame = object_model.get_part("outer_frame")
    inner_cradle = object_model.get_part("inner_cradle")
    roll_axis = object_model.get_articulation("roll_axis")
    pitch_axis = object_model.get_articulation("pitch_axis")

    ctx.check("roll joint uses main shaft axis", tuple(roll_axis.axis) == (1.0, 0.0, 0.0))
    ctx.check("pitch joint uses perpendicular cross-axis", tuple(pitch_axis.axis) == (0.0, 1.0, 0.0))

    ctx.expect_contact(
        outer_frame,
        roll_carrier,
        elem_a="frame_body",
        elem_b="bearing_saddle",
        contact_tol=0.0015,
        name="rear thrust shoulder seats on bearing saddle",
    )
    ctx.expect_within(
        inner_cradle,
        outer_frame,
        axes="yz",
        margin=0.004,
        name="barrel cradle sits inside the square frame envelope",
    )
    ctx.expect_overlap(
        inner_cradle,
        outer_frame,
        axes="y",
        elem_a="trunnion_pin",
        elem_b="frame_body",
        min_overlap=0.050,
        name="pitch trunnion passes through both side bearings",
    )

    rest_flange = ctx.part_element_world_aabb(inner_cradle, elem="center_flange")
    with ctx.pose({pitch_axis: pitch_axis.motion_limits.upper}):
        ctx.expect_within(
            inner_cradle,
            outer_frame,
            axes="yz",
            margin=0.004,
            name="positive pitch clears the frame opening",
        )
        upper_barrel = ctx.part_element_world_aabb(inner_cradle, elem="barrel_cradle")
        pitched_flange = ctx.part_element_world_aabb(inner_cradle, elem="center_flange")
    with ctx.pose({pitch_axis: pitch_axis.motion_limits.lower}):
        ctx.expect_within(
            inner_cradle,
            outer_frame,
            axes="yz",
            margin=0.004,
            name="negative pitch clears the frame opening",
        )
        lower_barrel = ctx.part_element_world_aabb(inner_cradle, elem="barrel_cradle")
    for value, name in [
        (roll_axis.motion_limits.upper, "positive roll keeps thrust bearing seated"),
        (roll_axis.motion_limits.lower, "negative roll keeps thrust bearing seated"),
    ]:
        with ctx.pose({roll_axis: value}):
            ctx.expect_contact(
                outer_frame,
                roll_carrier,
                elem_a="frame_body",
                elem_b="bearing_saddle",
                contact_tol=0.0015,
                name=name,
            )

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[2] + hi[2]) / 2.0

    rest_z = _aabb_center_z(rest_flange)
    pitch_z = _aabb_center_z(pitched_flange)
    ctx.check(
        "center flange visibly pitches with cradle",
        rest_z is not None and pitch_z is not None and abs(pitch_z - rest_z) > 0.015,
        details=f"rest_z={rest_z}, pitched_z={pitch_z}",
    )

    def _within_vertical_opening(aabb):
        if aabb is None:
            return False
        lo, hi = aabb
        return lo[2] > -0.066 and hi[2] < 0.066

    ctx.check(
        "barrel clears top and bottom rails at pitch limits",
        _within_vertical_opening(upper_barrel) and _within_vertical_opening(lower_barrel),
        details=f"upper={upper_barrel}, lower={lower_barrel}",
    )

    return ctx.report()


object_model = build_object_model()
