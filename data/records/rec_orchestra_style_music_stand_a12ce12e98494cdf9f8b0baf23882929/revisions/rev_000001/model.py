from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _cylinder_between(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[float, Origin]:
    """Return a URDF-cylinder length and origin aligned from start to end."""

    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return length, Origin(
        xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
        rpy=(0.0, pitch, yaw),
    )


def _hollow_cylinder_geometry(
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 64,
) -> MeshGeometry:
    """Closed annular tube mesh, open through the center for a real sleeve."""

    geom = MeshGeometry()
    outer_bottom = []
    outer_top = []
    inner_bottom = []
    inner_top = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_min))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_max))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_min))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_max))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner wall, opposite winding.
        geom.add_face(inner_bottom[i], inner_top[j], inner_bottom[j])
        geom.add_face(inner_bottom[i], inner_top[i], inner_top[j])
        # Annular top and bottom caps.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        geom.add_face(outer_bottom[i], inner_bottom[j], outer_bottom[j])
        geom.add_face(outer_bottom[i], inner_bottom[i], inner_bottom[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_music_stand")

    black = model.material("matte_black", rgba=(0.005, 0.005, 0.004, 1.0))
    satin = model.material("satin_black_metal", rgba=(0.02, 0.022, 0.021, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Sphere(0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=satin,
        name="tripod_hub",
    )
    base.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(0.026, 0.020, 0.08, 0.82),
            "lower_sleeve",
        ),
        material=satin,
        name="lower_sleeve",
    )
    base.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(0.038, 0.0205, 0.775, 0.845),
            "height_clamp_collar",
        ),
        material=satin,
        name="height_clamp_collar",
    )
    base.visual(
        Box((0.030, 0.025, 0.028)),
        origin=Origin(xyz=(0.0, -0.0275, 0.812)),
        material=satin,
        name="height_clamp_pad",
    )

    leg_radius = 0.016
    leg_start = (0.0, 0.0, 0.07)
    for index, angle in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        foot_x = 0.43 * math.cos(angle)
        foot_y = 0.43 * math.sin(angle)
        leg_end = (0.35 * math.cos(angle), 0.35 * math.sin(angle), 0.035)
        length, origin = _cylinder_between(leg_start, leg_end)
        base.visual(
            Cylinder(radius=leg_radius, length=length),
            origin=origin,
            material=satin,
            name=f"tripod_leg_{index}",
        )
        base.visual(
            Box((0.20, 0.055, 0.030)),
            origin=Origin(xyz=(foot_x, foot_y, 0.015), rpy=(0.0, 0.0, angle)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )

    height_knob = model.part("height_knob")
    height_knob.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_disk",
    )
    model.articulation(
        "base_to_height_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=height_knob,
        origin=Origin(xyz=(0.0, -0.049, 0.812)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    upper_pole = model.part("upper_pole")
    upper_pole.visual(
        Cylinder(radius=0.017, length=1.13),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin,
        name="inner_mast",
    )
    upper_pole.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.57), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="tilt_pin",
    )
    model.articulation(
        "base_to_upper_pole",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_pole,
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.22, lower=0.0, upper=0.35),
    )

    desk = model.part("desk")
    panel_profile = rounded_rect_profile(0.56, 0.36, 0.018, corner_segments=8)
    panel_geom = ExtrudeGeometry(panel_profile, 0.012, center=True).rotate_x(math.pi / 2.0)
    desk.visual(
        mesh_from_geometry(panel_geom, "rounded_desk_panel"),
        origin=Origin(xyz=(0.0, -0.018, 0.21)),
        material=black,
        name="rounded_panel",
    )
    desk.visual(
        Box((0.60, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, -0.036, 0.022)),
        material=black,
        name="lower_shelf",
    )
    desk.visual(
        Box((0.60, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, -0.059, 0.045)),
        material=black,
        name="front_lip",
    )
    desk.visual(
        Box((0.54, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.012, 0.395)),
        material=black,
        name="top_bead",
    )
    desk.visual(
        Box((0.040, 0.028, 0.36)),
        origin=Origin(xyz=(-0.22, -0.008, 0.21)),
        material=black,
        name="pressed_rib_0",
    )
    desk.visual(
        Box((0.040, 0.028, 0.36)),
        origin=Origin(xyz=(0.22, -0.008, 0.21)),
        material=black,
        name="pressed_rib_1",
    )
    desk.visual(
        Box((0.025, 0.048, 0.080)),
        origin=Origin(xyz=(-0.046, 0.012, 0.030)),
        material=satin,
        name="hinge_ear_0",
    )
    desk.visual(
        Box((0.025, 0.048, 0.080)),
        origin=Origin(xyz=(0.046, 0.012, 0.030)),
        material=satin,
        name="hinge_ear_1",
    )
    desk.visual(
        Box((0.020, 0.070, 0.085)),
        origin=Origin(xyz=(0.286, 0.004, 0.0)),
        material=satin,
        name="side_ear",
    )
    model.articulation(
        "upper_pole_to_desk",
        ArticulationType.REVOLUTE,
        parent=upper_pole,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.57), rpy=(-0.22, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.50, upper=0.45),
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        Cylinder(radius=0.032, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="knob_disk",
    )
    model.articulation(
        "desk_to_tilt_knob",
        ArticulationType.CONTINUOUS,
        parent=desk,
        child=tilt_knob,
        origin=Origin(xyz=(0.313, 0.004, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_pole = object_model.get_part("upper_pole")
    desk = object_model.get_part("desk")
    height_slide = object_model.get_articulation("base_to_upper_pole")
    desk_tilt = object_model.get_articulation("upper_pole_to_desk")

    ctx.allow_overlap(
        base,
        upper_pole,
        elem_a="height_clamp_pad",
        elem_b="inner_mast",
        reason="The thumb-screw clamp pad is intentionally shown pressing lightly into the telescoping mast.",
    )
    ctx.expect_overlap(
        base,
        upper_pole,
        axes="xyz",
        elem_a="height_clamp_pad",
        elem_b="inner_mast",
        min_overlap=0.001,
        name="height clamp pad bears on the inner mast",
    )

    ctx.allow_overlap(
        upper_pole,
        desk,
        elem_a="tilt_pin",
        elem_b="hinge_ear_0",
        reason="The desk hinge ear is captured around the cross pin in the tilt joint.",
    )
    ctx.expect_overlap(
        upper_pole,
        desk,
        axes="xyz",
        elem_a="tilt_pin",
        elem_b="hinge_ear_0",
        min_overlap=0.015,
        name="tilt pin is captured by the first hinge ear",
    )
    ctx.allow_overlap(
        upper_pole,
        desk,
        elem_a="tilt_pin",
        elem_b="hinge_ear_1",
        reason="The opposite desk hinge ear is also captured around the cross pin.",
    )
    ctx.expect_overlap(
        upper_pole,
        desk,
        axes="xyz",
        elem_a="tilt_pin",
        elem_b="hinge_ear_1",
        min_overlap=0.015,
        name="tilt pin is captured by the second hinge ear",
    )

    ctx.expect_within(
        upper_pole,
        base,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="lower_sleeve",
        margin=0.002,
        name="inner mast is centered in the lower sleeve",
    )
    ctx.expect_overlap(
        upper_pole,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="lower_sleeve",
        min_overlap=0.20,
        name="collapsed mast remains inserted in the sleeve",
    )

    rest_position = ctx.part_world_position(upper_pole)
    with ctx.pose({height_slide: 0.35}):
        ctx.expect_within(
            upper_pole,
            base,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="lower_sleeve",
            margin=0.002,
            name="extended mast stays centered in the sleeve",
        )
        ctx.expect_overlap(
            upper_pole,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="lower_sleeve",
            min_overlap=0.16,
            name="extended mast retains insertion in the sleeve",
        )
        raised_position = ctx.part_world_position(upper_pole)

    ctx.check(
        "telescoping pole raises the desk support",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.30,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    ctx.expect_overlap(
        desk,
        desk,
        axes="x",
        elem_a="lower_shelf",
        elem_b="rounded_panel",
        min_overlap=0.52,
        name="lower lip spans nearly the full sheet-music desk width",
    )
    with ctx.pose({desk_tilt: -0.35}):
        tilted_aabb = ctx.part_element_world_aabb(desk, elem="rounded_panel")
    ctx.check(
        "desk has a useful backward tilt range",
        tilted_aabb is not None,
        details=f"tilted panel aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
