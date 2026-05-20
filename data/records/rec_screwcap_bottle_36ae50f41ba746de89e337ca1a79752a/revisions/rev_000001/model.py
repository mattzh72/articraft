from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_HEIGHT = 0.210
NECK_BOTTOM_Z = 0.205
NECK_TOP_Z = 0.295
CAP_BOTTOM_Z = 0.245
CAP_HEIGHT = 0.085


def _circular_loft(sections: list[tuple[float, float]]) -> cq.Workplane:
    """Loft circular sections described as (z, radius) in meters."""
    first_z, first_r = sections[0]
    wp = cq.Workplane("XY").workplane(offset=first_z).circle(first_r)
    last_z = first_z
    for z, radius in sections[1:]:
        wp = wp.workplane(offset=z - last_z).circle(radius)
        last_z = z
    return wp.loft(combine=True)


def _make_body_shell() -> cq.Workplane:
    """Transparent molded bottle body with a thick base and sloping shoulders."""
    outer = _circular_loft(
        [
            (0.000, 0.035),
            (0.010, 0.044),
            (0.030, 0.047),
            (0.150, 0.047),
            (0.178, 0.043),
            (0.198, 0.030),
            (BODY_HEIGHT, 0.017),
        ]
    )
    return outer


def _make_neck_tube() -> cq.Workplane:
    height = NECK_TOP_Z - NECK_BOTTOM_Z
    tube = (
        cq.Workplane("XY")
        .circle(0.0140)
        .circle(0.0095)
        .extrude(height)
        .translate((0, 0, NECK_BOTTOM_Z))
    )
    lower_bead = (
        cq.Workplane("XY")
        .circle(0.0180)
        .circle(0.0095)
        .extrude(0.006)
        .translate((0, 0, NECK_BOTTOM_Z + 0.006))
    )
    mouth_bead = (
        cq.Workplane("XY")
        .circle(0.0158)
        .circle(0.0095)
        .extrude(0.006)
        .translate((0, 0, NECK_TOP_Z - 0.006))
    )
    neck = tube.union(lower_bead).union(mouth_bead)
    for z in (0.222, 0.231, 0.240, 0.249, 0.258, 0.267):
        thread_ring = (
            cq.Workplane("XY")
            .circle(0.0155)
            .circle(0.0136)
            .extrude(0.0022)
            .translate((0, 0, z))
        )
        neck = neck.union(thread_ring)
    return neck


def _helix(radius: float, radial_offset: float, pitch: float, height: float, z_offset: float, frac: float = 0.08):
    turns = height / pitch

    def _point(t: float):
        if frac < t < 1.0 - frac:
            z = height * t + z_offset
            r = radius + radial_offset
        elif t <= frac:
            z = height * t + z_offset * math.sin(math.pi * 0.5 * t / frac)
            r = radius + radial_offset * math.sin(math.pi * 0.5 * t / frac)
        else:
            z = height * t - z_offset * math.sin(2 * math.pi - math.pi * 0.5 * (1.0 - t) / frac)
            r = radius - radial_offset * math.sin(2 * math.pi - math.pi * 0.5 * (1.0 - t) / frac)

        theta = 2.0 * math.pi * turns * t
        return (r * math.cos(theta), r * math.sin(theta), z)

    return _point


def _make_thread_ridge() -> cq.Workplane:
    """External helical thread ridge around the bottle neck."""
    radius = 0.0140
    pitch = 0.0070
    height = 0.052
    crest = 0.00125
    half_width = pitch * 0.18

    e_root_low = cq.Workplane("XY").parametricCurve(_helix(radius, 0.0, pitch, height, -half_width)).val()
    e_root_high = cq.Workplane("XY").parametricCurve(_helix(radius, 0.0, pitch, height, half_width)).val()
    e_crest_low = cq.Workplane("XY").parametricCurve(_helix(radius, crest, pitch, height, -half_width * 0.35)).val()
    e_crest_high = cq.Workplane("XY").parametricCurve(_helix(radius, crest, pitch, height, half_width * 0.35)).val()

    faces = [
        cq.Face.makeRuledSurface(e_root_low, e_root_high),
        cq.Face.makeRuledSurface(e_crest_low, e_crest_high),
        cq.Face.makeRuledSurface(e_root_low, e_crest_low),
        cq.Face.makeRuledSurface(e_root_high, e_crest_high),
    ]
    solid = cq.Solid.makeSolid(cq.Shell.makeShell(faces))
    return cq.Workplane("XY").add(solid).translate((0, 0, 0.222))


def _make_label_band() -> cq.Workplane:
    """A thin paper/plastic wrap seated on the cylindrical body."""
    return (
        cq.Workplane("XY")
        .circle(0.0478)
        .circle(0.0464)
        .extrude(0.058)
        .translate((0, 0, 0.082))
    )


def _make_cap_shell() -> cq.Workplane:
    """Tall, narrow, hollow screw closure with long grip ribs."""
    cap_radius = 0.0220
    inner_radius = 0.0186
    top_thickness = 0.007
    cap = cq.Workplane("XY").circle(cap_radius).extrude(CAP_HEIGHT)

    rib_count = 32
    rib_height = CAP_HEIGHT - 0.014
    rib_radial = 0.0022
    rib_tangential = 0.0019
    rib_z = 0.007 + rib_height * 0.5
    for i in range(rib_count):
        angle = 360.0 * i / rib_count
        rib = (
            cq.Workplane("XY")
            .box(rib_radial, rib_tangential, rib_height)
            .translate((cap_radius + rib_radial * 0.5 - 0.0005, 0.0, rib_z))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        cap = cap.union(rib)

    inner_cavity = cq.Workplane("XY").circle(inner_radius).extrude(CAP_HEIGHT - top_thickness)
    cap = cap.cut(inner_cavity)

    # Hidden inside the closure: an internal thread/retainer land.  It is
    # intentionally very shallow and close to the bottle finish bead so the cap
    # reads as screwed on rather than hovering around a clearanced neck.
    internal_land = (
        cq.Workplane("XY")
        .circle(inner_radius + 0.0006)
        .circle(0.0155)
        .extrude(0.004)
        .translate((0, 0, NECK_TOP_Z - CAP_BOTTOM_Z - 0.006))
    )
    return cap.union(internal_land)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    clear_pet = Material("clear_pet", rgba=(0.76, 0.94, 1.0, 0.42))
    pale_thread = Material("pale_thread", rgba=(0.88, 0.97, 1.0, 0.72))
    label_mat = Material("white_label", rgba=(0.95, 0.96, 0.92, 1.0))
    cap_blue = Material("ribbed_blue_cap", rgba=(0.03, 0.20, 0.78, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_make_body_shell(), "bottle_body_shell", tolerance=0.0007, angular_tolerance=0.08),
        material=clear_pet,
        name="body_shell",
    )
    bottle.visual(
        mesh_from_cadquery(_make_neck_tube(), "threaded_neck_tube", tolerance=0.0005, angular_tolerance=0.06),
        material=pale_thread,
        name="neck_tube",
    )
    bottle.visual(
        mesh_from_cadquery(_make_label_band(), "label_band", tolerance=0.0008, angular_tolerance=0.08),
        material=label_mat,
        name="label_band",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_make_cap_shell(), "long_ribbed_screw_cap", tolerance=0.00055, angular_tolerance=0.06),
        material=cap_blue,
        name="cap_shell",
    )

    model.articulation(
        "neck_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    joint = object_model.get_articulation("neck_to_cap")

    ctx.allow_overlap(
        bottle,
        cap,
        elem_a="neck_tube",
        elem_b="cap_shell",
        reason=(
            "The hollow screw cap is represented as a sleeve proxy around the threaded neck; "
            "the hidden internal land captures the neck finish."
        ),
    )
    ctx.check(
        "cap has continuous screw rotation",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="neck_tube",
        outer_elem="cap_shell",
        margin=0.001,
        name="cap is coaxial around the neck",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        elem_a="cap_shell",
        elem_b="neck_tube",
        min_overlap=0.035,
        name="cap sleeve covers a useful length of neck",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="body_shell",
        min_gap=0.012,
        name="cap clears the shoulder below",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({joint: math.tau * 1.25}):
        turned_pos = ctx.part_world_position(cap)
        ctx.expect_within(
            bottle,
            cap,
            axes="xy",
            inner_elem="neck_tube",
            outer_elem="cap_shell",
            margin=0.001,
            name="cap stays coaxial after more than one turn",
        )
    ctx.check(
        "continuous rotation keeps cap on the neck axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
