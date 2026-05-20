from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_alignment_turntable")

    model.material("black_anodized", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("matte_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    model.material("bearing_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("white_engraving", rgba=(0.92, 0.94, 0.90, 1.0))
    model.material("datum_blue", rgba=(0.12, 0.24, 0.55, 1.0))
    model.material("cartridge_red", rgba=(0.70, 0.08, 0.045, 1.0))

    # The plinth is the one grounded reference part.  Small engraved marks,
    # datum rails, bearing collars, and knob collars are intentionally authored
    # into this same part so the stationary mechanical stack is connected.
    plinth = model.part("plinth")
    plinth.visual(
        Box((0.58, 0.42, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="black_anodized",
        name="plinth_core",
    )
    plinth.visual(
        Box((0.54, 0.38, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material="brushed_aluminum",
        name="datum_top_plate",
    )

    platter_xy = (-0.09, 0.03)
    platter_joint_z = 0.065
    plinth.visual(
        Cylinder(radius=0.158, length=0.004),
        origin=Origin(xyz=(platter_xy[0], platter_xy[1], 0.0570)),
        material="matte_graphite",
        name="platter_shadow_land",
    )
    plinth.visual(
        Cylinder(radius=0.090, length=0.009),
        origin=Origin(xyz=(platter_xy[0], platter_xy[1], 0.0605)),
        material="bearing_steel",
        name="platter_bearing_collar",
    )
    plinth.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(platter_xy[0], platter_xy[1], 0.0610)),
        material="bearing_steel",
        name="platter_bearing_boss",
    )

    # A square-friendly L datum and straight rails sit proud of the top plate.
    plinth.visual(
        Box((0.230, 0.012, 0.006)),
        origin=Origin(xyz=(-0.045, -0.178, 0.0578)),
        material="datum_blue",
        name="x_datum_rail",
    )
    plinth.visual(
        Box((0.012, 0.165, 0.006)),
        origin=Origin(xyz=(-0.154, -0.101, 0.0578)),
        material="datum_blue",
        name="y_datum_rail",
    )
    plinth.visual(
        Box((0.060, 0.004, 0.003)),
        origin=Origin(xyz=(-0.018, -0.178, 0.0610)),
        material="white_engraving",
        name="datum_zero_line",
    )

    # Fixed index ticks around the platter pocket; the rotating platter carries
    # its own fine ticks, while this stationary scale supplies the datum.
    for i in range(32):
        angle = 2.0 * math.pi * i / 32.0
        r = 0.166
        tick_len = 0.018 if i % 4 == 0 else 0.010
        plinth.visual(
            Box((tick_len, 0.0020, 0.0025)),
            origin=Origin(
                xyz=(platter_xy[0] + r * math.cos(angle), platter_xy[1] + r * math.sin(angle), 0.0558),
                rpy=(0.0, 0.0, angle),
            ),
            material="white_engraving",
            name=f"plinth_index_{i}",
        )
    plinth.visual(
        Box((0.034, 0.006, 0.006)),
        origin=Origin(xyz=(platter_xy[0] + 0.169, platter_xy[1], 0.0610)),
        material="white_engraving",
        name="platter_zero_pointer",
    )

    # Stationary tonearm bearing tower and post.  The rotating stage is a
    # hollow sleeve around this post with a visible radial clearance.
    tonearm_xy = (0.200, -0.130)
    plinth.visual(
        Cylinder(radius=0.037, length=0.045),
        origin=Origin(xyz=(tonearm_xy[0], tonearm_xy[1], 0.0774)),
        material="bearing_steel",
        name="pivot_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(tonearm_xy[0], tonearm_xy[1], 0.1000)),
        material="bearing_steel",
        name="pivot_post",
    )
    for i, angle in enumerate((-0.55, -0.35, -0.15, 0.05, 0.25, 0.45)):
        r = 0.055
        plinth.visual(
            Box((0.012, 0.0020, 0.0025)),
            origin=Origin(
                xyz=(tonearm_xy[0] + r * math.cos(angle), tonearm_xy[1] + r * math.sin(angle), 0.0558),
                rpy=(0.0, 0.0, angle),
            ),
            material="white_engraving",
            name=f"arm_arc_tick_{i}",
        )

    # Two rotating adjustment dials with stationary collars: a platter pitch
    # trim and a tonearm height micrometer cap.
    pitch_xy = (0.205, 0.145)
    plinth.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(pitch_xy[0], pitch_xy[1], 0.0570)),
        material="bearing_steel",
        name="pitch_collar",
    )
    plinth.visual(
        Box((0.045, 0.0025, 0.0025)),
        origin=Origin(xyz=(pitch_xy[0], pitch_xy[1] + 0.038, 0.0562)),
        material="white_engraving",
        name="pitch_zero_mark",
    )
    vta_xy = (0.258, -0.130)
    plinth.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(vta_xy[0], vta_xy[1], 0.0570)),
        material="bearing_steel",
        name="vta_collar",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.150, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="brushed_aluminum",
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.132, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0295)),
        material="matte_graphite",
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material="bearing_steel",
        name="spindle",
    )
    for i in range(48):
        angle = 2.0 * math.pi * i / 48.0
        tick_len = 0.014 if i % 6 == 0 else 0.008
        platter.visual(
            Box((tick_len, 0.0018, 0.0020)),
            origin=Origin(
                xyz=(0.144 * math.cos(angle), 0.144 * math.sin(angle), 0.0286),
                rpy=(0.0, 0.0, angle),
            ),
            material="white_engraving",
            name=f"platter_tick_{i}",
        )

    tonearm = model.part("tonearm")
    sleeve_geom = ExtrudeWithHolesGeometry(
        _circle_profile(0.024, 64),
        [_circle_profile(0.012, 64)],
        0.028,
        center=True,
    )
    tonearm.visual(
        mesh_from_geometry(sleeve_geom, "tonearm_pivot_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="brushed_aluminum",
        name="pivot_sleeve",
    )
    tonearm.visual(
        Box((0.278, 0.012, 0.010)),
        origin=Origin(xyz=(0.158, 0.0, 0.023)),
        material="brushed_aluminum",
        name="arm_wand",
    )
    tonearm.visual(
        Box((0.055, 0.028, 0.008)),
        origin=Origin(xyz=(0.315, 0.0, 0.022)),
        material="black_anodized",
        name="headshell",
    )
    tonearm.visual(
        Box((0.024, 0.015, 0.018)),
        origin=Origin(xyz=(0.331, 0.0, 0.010)),
        material="cartridge_red",
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.020, length=0.045),
        origin=Origin(xyz=(-0.045, 0.0, 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="matte_graphite",
        name="counterweight",
    )
    tonearm.visual(
        Box((0.060, 0.004, 0.003)),
        origin=Origin(xyz=(0.030, 0.000, 0.0285)),
        material="white_engraving",
        name="arm_datum_line",
    )

    pitch_knob = model.part("pitch_knob")
    pitch_knob.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material="matte_graphite",
        name="knob_body",
    )
    pitch_knob.visual(
        Box((0.032, 0.0035, 0.0020)),
        origin=Origin(xyz=(0.008, 0.0, 0.0141)),
        material="white_engraving",
        name="knob_pointer",
    )

    vta_knob = model.part("vta_knob")
    vta_knob.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="matte_graphite",
        name="knob_body",
    )
    vta_knob.visual(
        Box((0.026, 0.003, 0.0018)),
        origin=Origin(xyz=(0.006, 0.0, 0.0081)),
        material="white_engraving",
        name="knob_pointer",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_xy[0], platter_xy[1], platter_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0),
    )

    target = (platter_xy[0], platter_xy[1])
    # The q=0 alignment pose parks the stylus on a repeatable outer-groove
    # target rather than directly on the spindle.
    arm_yaw = math.atan2(target[1] - tonearm_xy[1], target[0] - tonearm_xy[0]) + 0.22
    model.articulation(
        "tonearm_pivot",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(tonearm_xy[0], tonearm_xy[1], 0.102), rpy=(0.0, 0.0, arm_yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.45, effort=1.5, velocity=1.2),
    )
    model.articulation(
        "pitch_trim",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=pitch_knob,
        origin=Origin(xyz=(pitch_xy[0], pitch_xy[1], 0.0590)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.5, upper=2.5, effort=0.3, velocity=3.0),
    )
    model.articulation(
        "vta_trim",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=vta_knob,
        origin=Origin(xyz=(vta_xy[0], vta_xy[1], 0.0590)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-3.14, upper=3.14, effort=0.2, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    pitch_knob = object_model.get_part("pitch_knob")
    vta_knob = object_model.get_part("vta_knob")

    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_pivot = object_model.get_articulation("tonearm_pivot")
    pitch_trim = object_model.get_articulation("pitch_trim")
    vta_trim = object_model.get_articulation("vta_trim")

    # The platter is deliberately supported by a visible bearing collar while
    # maintaining a controlled shadow gap to the fixed plinth.
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disc",
        negative_elem="platter_bearing_collar",
        min_gap=0.0,
        max_gap=0.0002,
        name="platter bearing is seated on the precision collar",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_disc",
        elem_b="platter_bearing_collar",
        min_overlap=0.15,
        name="platter is centered over its bearing collar",
    )

    # The tonearm sleeve surrounds the stationary post with no volume overlap;
    # these projection checks prove the post is retained and concentric inside
    # the rotating support stack.
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="pivot_sleeve",
        negative_elem="pivot_pedestal",
        min_gap=0.001,
        max_gap=0.004,
        name="tonearm sleeve clears the fixed pedestal",
    )
    ctx.expect_within(
        plinth,
        tonearm,
        axes="xy",
        inner_elem="pivot_post",
        outer_elem="pivot_sleeve",
        margin=0.001,
        name="tonearm post is concentric within the sleeve envelope",
    )
    ctx.expect_overlap(
        tonearm,
        plinth,
        axes="xy",
        elem_a="pivot_sleeve",
        elem_b="pivot_pedestal",
        min_overlap=0.045,
        name="tonearm sleeve is centered over the pedestal footprint",
    )

    ctx.expect_contact(
        pitch_knob,
        plinth,
        elem_a="knob_body",
        elem_b="pitch_collar",
        contact_tol=0.0002,
        name="pitch trim dial is seated on its collar",
    )
    ctx.expect_contact(
        vta_knob,
        plinth,
        elem_a="knob_body",
        elem_b="vta_collar",
        contact_tol=0.0002,
        name="tonearm trim dial is seated on its collar",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_platter_origin = ctx.part_world_position(platter)
    with ctx.pose({platter_spin: 1.35}):
        spun_platter_origin = ctx.part_world_position(platter)
    ctx.check(
        "platter spin is a pure centered rotation",
        rest_platter_origin is not None
        and spun_platter_origin is not None
        and math.dist(rest_platter_origin, spun_platter_origin) < 1e-6,
        details=f"rest={rest_platter_origin}, spun={spun_platter_origin}",
    )

    rest_cartridge = _aabb_center(ctx.part_element_world_aabb(tonearm, elem="cartridge"))
    with ctx.pose({tonearm_pivot: 0.40}):
        swept_cartridge = _aabb_center(ctx.part_element_world_aabb(tonearm, elem="cartridge"))
    ctx.check(
        "tonearm pivot visibly sweeps the cartridge",
        rest_cartridge is not None
        and swept_cartridge is not None
        and math.hypot(rest_cartridge[0] - swept_cartridge[0], rest_cartridge[1] - swept_cartridge[1]) > 0.09,
        details=f"rest={rest_cartridge}, swept={swept_cartridge}",
    )

    rest_pitch = ctx.part_world_position(pitch_knob)
    with ctx.pose({pitch_trim: 1.0, vta_trim: -1.0}):
        moved_pitch = ctx.part_world_position(pitch_knob)
        moved_vta = ctx.part_world_position(vta_knob)
    ctx.check(
        "adjustment dials rotate in place on supported axes",
        rest_pitch is not None
        and moved_pitch is not None
        and moved_vta is not None
        and math.dist(rest_pitch, moved_pitch) < 1e-6,
        details=f"pitch_rest={rest_pitch}, pitch_moved={moved_pitch}, vta_moved={moved_vta}",
    )

    return ctx.report()


object_model = build_object_model()
