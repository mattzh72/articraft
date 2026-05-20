from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _helix(radius: float, radial_offset: float, pitch: float, height: float, z_taper: float, frac: float = 0.08):
    """CadQuery parametric helix used for the Edison-style thread ribs."""

    def func(t: float) -> tuple[float, float, float]:
        if frac < t < 1.0 - frac:
            z = height * t + z_taper
            r = radius + radial_offset
        elif t <= frac:
            z = height * t + z_taper * math.sin(math.pi / 2.0 * t / frac)
            r = radius + radial_offset * math.sin(math.pi / 2.0 * t / frac)
        else:
            z = height * t - z_taper * math.sin(2.0 * math.pi - math.pi / 2.0 * (1.0 - t) / frac)
            r = radius - radial_offset * math.sin(2.0 * math.pi - math.pi / 2.0 * (1.0 - t) / frac)

        turns = height / pitch
        x = r * math.sin(-2.0 * math.pi * turns * t)
        y = r * math.cos(2.0 * math.pi * turns * t)
        return x, y, z

    return func


def _thread_rib(
    *,
    radius: float,
    pitch: float,
    height: float,
    z0: float,
    radial_depth: float,
) -> cq.Workplane:
    """A shallow helical rib; positive depth protrudes outward, negative inward."""

    half_pitch = pitch / 4.0
    e1_bottom = cq.Workplane("XY").parametricCurve(_helix(radius, 0.0, pitch, height, -half_pitch)).val()
    e1_top = cq.Workplane("XY").parametricCurve(_helix(radius, 0.0, pitch, height, half_pitch)).val()
    e2_bottom = (
        cq.Workplane("XY")
        .parametricCurve(_helix(radius, radial_depth, pitch, height, -half_pitch / 8.0))
        .val()
    )
    e2_top = (
        cq.Workplane("XY")
        .parametricCurve(_helix(radius, radial_depth, pitch, height, half_pitch / 8.0))
        .val()
    )

    faces = [
        cq.Face.makeRuledSurface(e1_bottom, e1_top),
        cq.Face.makeRuledSurface(e2_bottom, e2_top),
        cq.Face.makeRuledSurface(e1_bottom, e2_bottom),
        cq.Face.makeRuledSurface(e1_top, e2_top),
    ]
    solid = cq.Solid.makeSolid(cq.Shell.makeShell(faces))
    return cq.Workplane("XY").add(solid).translate((0.0, 0.0, z0))


def _hollow_cylinder(outer_radius: float, inner_radius: float, height: float, z0: float) -> cq.Workplane:
    outer = cq.Workplane("XY", origin=(0.0, 0.0, z0)).circle(outer_radius).extrude(height)
    cutter = cq.Workplane("XY", origin=(0.0, 0.0, z0 - 0.001)).circle(inner_radius).extrude(height + 0.002)
    return outer.cut(cutter)


def _socket_body_shape() -> cq.Workplane:
    """Cutaway ceramic socket cup with a bottom flange and a visible side window."""

    flange = cq.Workplane("XY").circle(0.040).extrude(0.008)
    cup = cq.Workplane("XY", origin=(0.0, 0.0, 0.008)).circle(0.031).extrude(0.058)
    body = flange.union(cup)

    bore = cq.Workplane("XY", origin=(0.0, 0.0, 0.014)).circle(0.0186).extrude(0.060)
    window = cq.Workplane("XY").box(0.032, 0.050, 0.044).translate((0.0, -0.027, 0.046))
    return body.cut(bore).cut(window)


def _socket_collar_shape() -> cq.Workplane:
    """Stationary metal threaded collar captured in the ceramic socket."""

    collar = _hollow_cylinder(0.0190, 0.0152, 0.040, 0.024)
    top_lip = _hollow_cylinder(0.0210, 0.0150, 0.004, 0.064)
    lower_lip = _hollow_cylinder(0.0194, 0.0151, 0.004, 0.024)
    return collar.union(top_lip).union(lower_lip)


def _threaded_bulb_base_shape() -> cq.Workplane:
    """Rotating Edison screw shell with raised helical thread and rolled lip."""

    core = cq.Workplane("XY", origin=(0.0, 0.0, -0.035)).circle(0.0120).extrude(0.039)
    base = core
    # Narrow raised crests approximate the Edison screw thread while remaining a
    # single robust metal shell for tessellation and collision checks.
    for z in (-0.033, -0.028, -0.023, -0.018, -0.013, -0.008, -0.003):
        crest = cq.Workplane("XY", origin=(0.0, 0.0, z)).circle(0.01335).extrude(0.0021)
        base = base.union(crest)
    top_band = cq.Workplane("XY", origin=(0.0, 0.0, 0.004)).circle(0.0142).extrude(0.006)
    bottom_band = cq.Workplane("XY", origin=(0.0, 0.0, -0.039)).circle(0.0130).extrude(0.004)
    return base.union(top_band).union(bottom_band)


def _glass_envelope_shape() -> cq.Workplane:
    """Thin hollow pear-shaped A19-style glass envelope."""

    outer_sections = [
        (0.018, 0.0105),
        (0.028, 0.0120),
        (0.043, 0.0220),
        (0.064, 0.0305),
        (0.090, 0.0290),
        (0.113, 0.0170),
        (0.124, 0.0030),
    ]
    outer = cq.Workplane("XY").workplane(offset=outer_sections[0][0]).circle(outer_sections[0][1])
    last_z = outer_sections[0][0]
    for z, r in outer_sections[1:]:
        outer = outer.workplane(offset=z - last_z).circle(r)
        last_z = z
    return outer.loft(combine=True)


def _mesh(shape: cq.Workplane, name: str, *, tolerance: float, angular_tolerance: float):
    try:
        return mesh_from_cadquery(shape, name, tolerance=tolerance, angular_tolerance=angular_tolerance)
    except Exception as exc:  # pragma: no cover - makes compile diagnostics name the failed mesh.
        raise RuntimeError(f"failed to tessellate {name}: {exc}") from exc

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb_socket")

    ceramic = model.material("warm_white_ceramic", rgba=(0.92, 0.88, 0.76, 1.0))
    brushed_metal = model.material("brushed_silver", rgba=(0.72, 0.70, 0.66, 1.0))
    brass = model.material("brass_contact", rgba=(0.95, 0.68, 0.24, 1.0))
    black = model.material("black_insulator", rgba=(0.02, 0.02, 0.018, 1.0))
    glass = model.material("frosted_clear_glass", rgba=(0.78, 0.92, 1.0, 0.34))
    red = model.material("red_axis_mark", rgba=(0.9, 0.05, 0.03, 1.0))

    socket = model.part("socket")
    socket.visual(
        _mesh(_socket_body_shape(), "socket_ceramic", tolerance=0.0007, angular_tolerance=0.08),
        material=ceramic,
        name="ceramic_body",
    )
    socket.visual(
        _mesh(_socket_collar_shape(), "socket_threaded_collar", tolerance=0.00045, angular_tolerance=0.06),
        material=brushed_metal,
        name="threaded_collar",
    )
    for i, z in enumerate((0.032, 0.038, 0.044, 0.050, 0.056)):
        socket.visual(
            _mesh(
                _hollow_cylinder(0.0193, 0.0148, 0.0008, z),
                f"socket_collar_thread_{i}",
                tolerance=0.0005,
                angular_tolerance=0.08,
            ),
            material=black,
            name=f"collar_thread_{i}",
        )
    socket.visual(
        Cylinder(radius=0.0135, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0170)),
        material=black,
        name="bottom_insulator",
    )
    socket.visual(
        Cylinder(radius=0.0060, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=brass,
        name="center_contact",
    )
    bulb = model.part("bulb")
    bulb.visual(
        _mesh(_threaded_bulb_base_shape(), "bulb_threaded_base", tolerance=0.00045, angular_tolerance=0.06),
        material=brushed_metal,
        name="threaded_base",
    )
    bulb.visual(
        Cylinder(radius=0.0048, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=brass,
        name="bottom_contact",
    )
    bulb.visual(
        Cylinder(radius=0.0108, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=black,
        name="seal_band",
    )
    bulb.visual(
        _mesh(_glass_envelope_shape(), "bulb_glass_envelope", tolerance=0.00055, angular_tolerance=0.08),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0010, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=brass,
        name="support_stem",
    )
    bulb.visual(
        Box((0.0022, 0.0040, 0.022)),
        origin=Origin(xyz=(0.0141, 0.0, -0.013)),
        material=red,
        name="axis_mark",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("socket_to_bulb")

    ctx.check(
        "bulb has continuous screw rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="threaded_base",
        outer_elem="threaded_collar",
        margin=0.0,
        name="bulb screw shell sits inside socket collar radius",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_base",
        elem_b="threaded_collar",
        min_overlap=0.030,
        name="screw shell is seated down inside the socket collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0002,
        positive_elem="bottom_contact",
        negative_elem="center_contact",
        name="bulb bottom contact rests on socket center contact",
    )

    marker0 = ctx.part_element_world_aabb(bulb, elem="axis_mark")
    with ctx.pose({spin: math.pi / 2.0}):
        marker90 = ctx.part_element_world_aabb(bulb, elem="axis_mark")

    def _xy_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0)

    m0 = _xy_center(marker0)
    m90 = _xy_center(marker90)
    ctx.check(
        "painted index mark visibly orbits the shared socket axis",
        m0 is not None and m90 is not None and m0[0] > 0.010 and abs(m0[1]) < 0.004 and m90[1] > 0.010,
        details=f"rest_marker_xy={m0}, quarter_turn_marker_xy={m90}",
    )

    return ctx.report()


object_model = build_object_model()
