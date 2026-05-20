from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel_mesh(width: float, height: float, thickness: float, radius: float, name: str):
    """A thin rounded rectangle whose local axes are X width, Y thickness, Z height."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        thickness,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _tube_mesh(
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    name: str,
):
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=48,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conductors_music_stand")

    black = model.material("satin_black_powdercoat", rgba=(0.005, 0.005, 0.004, 1.0))
    panel_mat = model.material("matte_black_desk_panel", rgba=(0.025, 0.026, 0.024, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.009, 0.008, 1.0))
    pin_metal = model.material("dark_steel_pins", rgba=(0.16, 0.16, 0.15, 1.0))

    # Floor base and fixed lower sleeve.  The broad tripod footprint keeps the
    # conductor-scale top visually supported.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.095, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=black,
        name="weighted_hub",
    )
    for i, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        base.visual(
            Box((0.70, 0.058, 0.034)),
            origin=Origin(
                xyz=(0.275 * math.cos(yaw), 0.275 * math.sin(yaw), 0.025),
                rpy=(0.0, 0.0, yaw),
            ),
            material=black,
            name=f"tripod_foot_{i}",
        )
        base.visual(
            Box((0.085, 0.066, 0.018)),
            origin=Origin(
                xyz=(0.585 * math.cos(yaw), 0.585 * math.sin(yaw), 0.013),
                rpy=(0.0, 0.0, yaw),
            ),
            material=rubber,
            name=f"rubber_pad_{i}",
        )

    base.visual(
        Cylinder(radius=0.034, length=0.7675),
        origin=Origin(xyz=(0.0, 0.0, 0.44625)),
        material=black,
        name="outer_sleeve",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.830)),
        material=black,
        name="clamp_collar",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=black,
        name="lower_collar",
    )

    # Sliding upper mast.  It remains inserted in the hollow sleeve at full
    # extension but carries the tilt yoke at conductor height.
    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.018, length=0.986),
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
        material=black,
        name="inner_tube",
    )
    mast.visual(
        Box((0.230, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.605)),
        material=black,
        name="yoke_crossbar",
    )
    mast.visual(
        Box((0.020, 0.078, 0.118)),
        origin=Origin(xyz=(-0.105, 0.0, 0.675)),
        material=black,
        name="head_ear_0",
    )
    mast.visual(
        Box((0.020, 0.078, 0.118)),
        origin=Origin(xyz=(0.105, 0.0, 0.675)),
        material=black,
        name="head_ear_1",
    )
    mast.visual(
        Cylinder(radius=0.014, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.675), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_metal,
        name="head_pin",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.830)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.320),
    )

    # Central desk frame.  The part frame is the head hinge axis; the broad
    # panel rises from it and carries the side hinge lines for the score wings.
    desk = model.part("desk")
    desk.visual(
        Cylinder(radius=0.025, length=0.190),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_metal,
        name="tilt_boss",
    )
    desk.visual(
        _rounded_panel_mesh(0.720, 0.455, 0.022, 0.025, "central_panel"),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=panel_mat,
        name="central_panel",
    )
    desk.visual(
        Box((0.090, 0.018, 0.095)),
        origin=Origin(xyz=(0.0, 0.020, 0.060)),
        material=black,
        name="center_bracket",
    )
    desk.visual(
        Box((0.250, 0.050, 0.035)),
        origin=Origin(xyz=(-0.255, 0.036, 0.036)),
        material=black,
        name="music_lip_0",
    )
    desk.visual(
        Box((0.250, 0.050, 0.035)),
        origin=Origin(xyz=(0.255, 0.036, 0.036)),
        material=black,
        name="music_lip_1",
    )
    desk.visual(
        Box((0.770, 0.030, 0.027)),
        origin=Origin(xyz=(0.0, -0.004, 0.535)),
        material=black,
        name="top_rail",
    )
    for x, name in ((-0.386, "side_rail_0"), (0.386, "side_rail_1")):
        desk.visual(
            Box((0.026, 0.030, 0.500)),
            origin=Origin(xyz=(x, -0.004, 0.300)),
            material=black,
            name=name,
        )
        desk.visual(
            Box((0.012, 0.020, 0.430)),
            origin=Origin(xyz=(x, -0.005, 0.280)),
            material=pin_metal,
            name=f"hinge_leaf_{name[-1]}",
        )

    # Resting angle leans the top back like a real conductor's desk.  The
    # revolute joint still gives the horizontal head tilt adjustment.
    model.articulation(
        "head_hinge",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.675), rpy=(math.radians(12.0), 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=math.radians(-18.0), upper=math.radians(28.0)),
    )

    def add_wing(name: str, sign: float):
        wing = model.part(name)
        wing.visual(
            _rounded_panel_mesh(0.305, 0.405, 0.020, 0.020, f"{name}_panel"),
            origin=Origin(xyz=(sign * 0.1655, 0.0, 0.220)),
            material=panel_mat,
            name="panel",
        )
        wing.visual(
            Box((0.330, 0.042, 0.032)),
            origin=Origin(xyz=(sign * 0.178, 0.031, 0.038)),
            material=black,
            name="lower_lip",
        )
        wing.visual(
            Box((0.022, 0.026, 0.390)),
            origin=Origin(xyz=(sign * 0.314, -0.004, 0.225)),
            material=black,
            name="outer_rail",
        )
        wing.visual(
            Cylinder(radius=0.009, length=0.380),
            origin=Origin(xyz=(sign * 0.008, -0.043, 0.230)),
            material=pin_metal,
            name="hinge_barrel",
        )
        wing.visual(
            Box((0.055, 0.030, 0.390)),
            origin=Origin(xyz=(sign * 0.040, -0.025, 0.225)),
            material=pin_metal,
            name="hinge_leaf",
        )
        return wing

    wing_0 = add_wing("wing_0", -1.0)
    wing_1 = add_wing("wing_1", 1.0)

    model.articulation(
        "wing_hinge_0",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=wing_0,
        origin=Origin(xyz=(-0.386, 0.0, 0.015)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.15, upper=1.75),
    )
    model.articulation(
        "wing_hinge_1",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=wing_1,
        origin=Origin(xyz=(0.386, 0.0, 0.015)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-1.75, upper=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    mast_slide = object_model.get_articulation("mast_slide")
    head_hinge = object_model.get_articulation("head_hinge")
    wing_hinge_0 = object_model.get_articulation("wing_hinge_0")
    wing_hinge_1 = object_model.get_articulation("wing_hinge_1")

    ctx.allow_overlap(
        base,
        mast,
        elem_a="outer_sleeve",
        elem_b="inner_tube",
        reason="The height-adjustment tube is intentionally represented as a nested sliding member inside the sleeve proxy.",
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="clamp_collar",
        elem_b="inner_tube",
        reason="The clamp collar surrounds the sliding mast as a simplified solid guide collar.",
    )
    ctx.allow_overlap(
        desk,
        mast,
        elem_a="tilt_boss",
        elem_b="head_pin",
        reason="The horizontal tilt pin is intentionally captured through the desk boss.",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="sliding mast is centered in sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.25,
        name="collapsed mast remains deeply inserted",
    )
    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="clamp_collar",
        margin=0.001,
        name="sliding mast passes through clamp collar",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="clamp_collar",
        min_overlap=0.05,
        name="clamp collar surrounds mast tube",
    )

    rest_mast = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.320}):
        raised_mast = ctx.part_world_position(mast)
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="raised mast remains centered in sleeve",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.045,
            name="raised mast keeps retained insertion",
        )
    ctx.check(
        "mast slide raises the head",
        rest_mast is not None and raised_mast is not None and raised_mast[2] > rest_mast[2] + 0.30,
        details=f"rest={rest_mast}, raised={raised_mast}",
    )

    ctx.expect_contact(
        desk,
        mast,
        elem_a="tilt_boss",
        elem_b="head_ear_0",
        contact_tol=0.006,
        name="tilt boss seats in yoke ear",
    )
    ctx.expect_within(
        mast,
        desk,
        axes="yz",
        inner_elem="head_pin",
        outer_elem="tilt_boss",
        margin=0.001,
        name="tilt pin is centered in boss bore",
    )
    ctx.expect_overlap(
        mast,
        desk,
        axes="x",
        elem_a="head_pin",
        elem_b="tilt_boss",
        min_overlap=0.18,
        name="tilt pin spans the boss",
    )

    def center_from_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    desk_center = center_from_aabb(ctx.part_element_world_aabb(desk, elem="central_panel"))
    with ctx.pose({head_hinge: math.radians(24.0)}):
        tilted_center = center_from_aabb(ctx.part_element_world_aabb(desk, elem="central_panel"))
    ctx.check(
        "head hinge tilts the desk panel",
        desk_center is not None
        and tilted_center is not None
        and abs(tilted_center[1] - desk_center[1]) > 0.06,
        details=f"rest={desk_center}, tilted={tilted_center}",
    )

    ctx.expect_contact(
        wing_0,
        desk,
        elem_a="panel",
        elem_b="side_rail_0",
        contact_tol=0.003,
        name="wing 0 hinges directly on desk frame",
    )
    ctx.expect_contact(
        wing_1,
        desk,
        elem_a="panel",
        elem_b="side_rail_1",
        contact_tol=0.003,
        name="wing 1 hinges directly on desk frame",
    )

    wing_0_rest = center_from_aabb(ctx.part_element_world_aabb(wing_0, elem="panel"))
    wing_1_rest = center_from_aabb(ctx.part_element_world_aabb(wing_1, elem="panel"))
    with ctx.pose({wing_hinge_0: 1.25, wing_hinge_1: -1.25}):
        wing_0_folded = center_from_aabb(ctx.part_element_world_aabb(wing_0, elem="panel"))
        wing_1_folded = center_from_aabb(ctx.part_element_world_aabb(wing_1, elem="panel"))
    ctx.check(
        "score wings rotate on side hinges",
        wing_0_rest is not None
        and wing_0_folded is not None
        and wing_1_rest is not None
        and wing_1_folded is not None
        and abs(wing_0_folded[1] - wing_0_rest[1]) > 0.12
        and abs(wing_1_folded[1] - wing_1_rest[1]) > 0.12,
        details=f"wing0 {wing_0_rest}->{wing_0_folded}; wing1 {wing_1_rest}->{wing_1_folded}",
    )

    return ctx.report()


object_model = build_object_model()
