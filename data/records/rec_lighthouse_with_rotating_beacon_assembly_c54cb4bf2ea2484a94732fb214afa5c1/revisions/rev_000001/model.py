from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="seaside_lighthouse_lantern")

    dark_iron = model.material("salt_dark_iron", rgba=(0.05, 0.055, 0.055, 1.0))
    weathered_brass = model.material("weathered_brass", rgba=(0.72, 0.55, 0.25, 1.0))
    aged_copper = model.material("aged_copper", rgba=(0.35, 0.28, 0.19, 1.0))
    glass = model.material("slightly_green_glass", rgba=(0.62, 0.90, 0.95, 0.33))
    lens_glass = model.material("fresnel_lens_glass", rgba=(0.78, 0.95, 1.0, 0.48))
    lamp_glow = model.material("warm_lamp_glow", rgba=(1.0, 0.75, 0.25, 0.9))

    lantern = model.part("lantern")

    # Heavy masonry/iron sill and cupola roof give the lantern a believable
    # seaside lighthouse silhouette, while the glazed room remains visibly open.
    lantern.visual(
        Cylinder(radius=0.64, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_iron,
        name="round_sill",
    )
    lantern.visual(
        Cylinder(radius=0.56, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=weathered_brass,
        name="lantern_floor",
    )
    lantern.visual(
        Cylinder(radius=0.045, length=1.42),
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        material=weathered_brass,
        name="central_shaft",
    )

    side_radius = 0.53
    facet_width = 0.40
    frame_thickness = 0.045
    glass_z = 0.95
    glass_height = 1.05
    for i in range(8):
        angle = math.radians(i * 45.0)
        x = side_radius * math.cos(angle)
        y = side_radius * math.sin(angle)
        # Horizontal octagonal rails around the glazed room.
        for z, rail_name in ((0.43, "lower_rail"), (1.48, "upper_rail")):
            lantern.visual(
                Box((frame_thickness, facet_width, 0.070)),
                origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, angle)),
                material=dark_iron,
                name=f"{rail_name}_{i}",
            )
        # Seven facets are fixed glass.  The +X facet is reserved for the door.
        if i != 0:
            lantern.visual(
                Box((0.014, facet_width - 0.055, glass_height)),
                origin=Origin(xyz=(x, y, glass_z), rpy=(0.0, 0.0, angle)),
                material=glass,
                name=f"glass_facet_{i}",
            )

    vertex_radius = 0.575
    for i in range(8):
        angle = math.radians(22.5 + i * 45.0)
        lantern.visual(
            Cylinder(radius=0.028, length=1.18),
            origin=Origin(
                xyz=(vertex_radius * math.cos(angle), vertex_radius * math.sin(angle), 0.95)
            ),
            material=dark_iron,
            name=f"corner_mullion_{i}",
        )

    # Narrow door jamb and fixed hinge knuckles on the front facet.
    door_plane_x = side_radius + 0.006
    door_width = 0.32
    hinge_y = -door_width / 2.0
    for y, name in ((hinge_y - 0.025, "hinge_jamb"), (door_width / 2.0 + 0.025, "latch_jamb")):
        lantern.visual(
            Box((0.060, 0.032, 1.08)),
            origin=Origin(xyz=(door_plane_x, y, 0.95)),
            material=dark_iron,
            name=name,
        )
    for z in (0.55, 1.35):
        lantern.visual(
            Cylinder(radius=0.022, length=0.16),
            origin=Origin(xyz=(door_plane_x + 0.064, hinge_y, z)),
            material=dark_iron,
            name=f"fixed_hinge_knuckle_{z:.2f}",
        )
        lantern.visual(
            Box((0.075, 0.022, 0.040)),
            origin=Origin(xyz=(door_plane_x + 0.030, hinge_y - 0.025, z)),
            material=dark_iron,
            name=f"fixed_hinge_leaf_{z:.2f}",
        )

    roof_profile = [
        (0.16, 1.50),
        (0.66, 1.53),
        (0.58, 1.63),
        (0.38, 1.88),
        (0.16, 2.05),
        (0.10, 2.10),
    ]
    lantern.visual(
        mesh_from_geometry(LatheGeometry(roof_profile, segments=72, closed=True), "copper_roof"),
        material=aged_copper,
        name="copper_roof",
    )
    lantern.visual(
        Cylinder(radius=0.075, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 1.82)),
        material=aged_copper,
        name="roof_spindle",
    )
    lantern.visual(
        Cylinder(radius=0.12, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 2.16)),
        material=dark_iron,
        name="vent_cap",
    )

    lens_carriage = model.part("lens_carriage")
    # The carriage origin is on the shaft centerline.  A solid collar is used as
    # a compact proxy for the rotating bearing around the fixed shaft.
    lens_carriage.visual(
        Cylinder(radius=0.070, length=0.96),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=weathered_brass,
        name="shaft_collar",
    )
    lens_carriage.visual(
        mesh_from_geometry(TorusGeometry(radius=0.082, tube=0.016, radial_segments=24, tubular_segments=48), "top_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=weathered_brass,
        name="top_bearing_ring",
    )
    lens_carriage.visual(
        mesh_from_geometry(TorusGeometry(radius=0.082, tube=0.016, radial_segments=24, tubular_segments=48), "lower_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.45)),
        material=weathered_brass,
        name="lower_bearing_ring",
    )
    for sign, glow_name in ((1.0, "lamp_glow_0"), (-1.0, "lamp_glow_1")):
        lens_carriage.visual(
            Box((0.085, 0.040, 0.22)),
            origin=Origin(xyz=(sign * 0.105, 0.0, 0.0)),
            material=lamp_glow,
            name=glow_name,
        )
    for sign, label in ((1.0, "0"), (-1.0, "1")):
        lens_carriage.visual(
            Box((0.34, 0.030, 0.045)),
            origin=Origin(xyz=(sign * 0.235, 0.0, 0.23)),
            material=weathered_brass,
            name=f"upper_spoke_{label}",
        )
        lens_carriage.visual(
            Box((0.34, 0.030, 0.045)),
            origin=Origin(xyz=(sign * 0.235, 0.0, -0.23)),
            material=weathered_brass,
            name=f"lower_spoke_{label}",
        )
        lens_carriage.visual(
            Box((0.045, 0.30, 0.58)),
            origin=Origin(xyz=(sign * 0.37, 0.0, 0.0)),
            material=lens_glass,
            name=f"lens_{label}_glass",
        )
        lens_carriage.visual(
            Box((0.060, 0.34, 0.045)),
            origin=Origin(xyz=(sign * 0.37, 0.0, 0.31)),
            material=weathered_brass,
            name=f"lens_{label}_top_frame",
        )
        lens_carriage.visual(
            Box((0.060, 0.34, 0.045)),
            origin=Origin(xyz=(sign * 0.37, 0.0, -0.31)),
            material=weathered_brass,
            name=f"lens_{label}_lower_frame",
        )
        # Subtle horizontal Fresnel bands on each lens face.
        for band in range(5):
            z = -0.20 + 0.10 * band
            lens_carriage.visual(
                Box((0.052, 0.335, 0.010)),
                origin=Origin(xyz=(sign * 0.398, 0.0, z)),
                material=weathered_brass,
                name=f"lens_{label}_fresnel_band_{band}",
            )

    door = model.part("door")
    door.visual(
        Box((0.034, door_width, 0.88)),
        origin=Origin(xyz=(0.0, door_width / 2.0, 0.0)),
        material=glass,
        name="door_leaf_glass",
    )
    door.visual(
        Box((0.046, 0.035, 0.96)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=dark_iron,
        name="hinge_stile",
    )
    door.visual(
        Box((0.046, 0.035, 0.96)),
        origin=Origin(xyz=(0.0, door_width - 0.015, 0.0)),
        material=dark_iron,
        name="latch_stile",
    )
    for z, rail_name in ((0.455, "top_rail"), (-0.455, "bottom_rail")):
        door.visual(
            Box((0.046, door_width, 0.035)),
            origin=Origin(xyz=(0.0, door_width / 2.0, z)),
            material=dark_iron,
            name=rail_name,
        )
    for z in (-0.17, 0.17):
        door.visual(
            Cylinder(radius=0.021, length=0.20),
            origin=Origin(xyz=(0.060, 0.0, z)),
            material=dark_iron,
            name=f"door_hinge_knuckle_{z:.2f}",
        )
        door.visual(
            Box((0.075, 0.020, 0.045)),
            origin=Origin(xyz=(0.030, 0.0, z)),
            material=dark_iron,
            name=f"hinge_leaf_{z:.2f}",
        )
    door.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.035, door_width - 0.055, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_brass,
        name="round_pull",
    )

    model.articulation(
        "lens_spin",
        ArticulationType.CONTINUOUS,
        parent=lantern,
        child=lens_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=lantern,
        child=door,
        origin=Origin(xyz=(door_plane_x + 0.004, hinge_y, 0.95)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lantern = object_model.get_part("lantern")
    lens_carriage = object_model.get_part("lens_carriage")
    door = object_model.get_part("door")
    lens_spin = object_model.get_articulation("lens_spin")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.allow_overlap(
        lantern,
        lens_carriage,
        elem_a="central_shaft",
        elem_b="shaft_collar",
        reason="The rotating bearing collar is intentionally captured around the fixed central shaft proxy.",
    )
    ctx.expect_within(
        lantern,
        lens_carriage,
        axes="xy",
        inner_elem="central_shaft",
        outer_elem="shaft_collar",
        margin=0.002,
        name="shaft centered inside rotating collar",
    )
    ctx.expect_overlap(
        lantern,
        lens_carriage,
        axes="z",
        elem_a="central_shaft",
        elem_b="shaft_collar",
        min_overlap=0.70,
        name="collar rides along central shaft",
    )

    ctx.check(
        "lens carriage uses continuous vertical rotation",
        lens_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(lens_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={lens_spin.articulation_type}, axis={lens_spin.axis}",
    )
    ctx.check(
        "door hinge has narrow outward swing limit",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and 1.2 <= door_hinge.motion_limits.upper <= 1.5
        and tuple(door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"limits={door_hinge.motion_limits}, axis={door_hinge.axis}",
    )

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    lens_rest = ctx.part_element_world_aabb(lens_carriage, elem="lens_0_glass")
    with ctx.pose({lens_spin: math.pi / 2.0}):
        lens_quarter = ctx.part_element_world_aabb(lens_carriage, elem="lens_0_glass")
    if lens_rest is not None and lens_quarter is not None:
        c0 = aabb_center(lens_rest)
        c1 = aabb_center(lens_quarter)
        ctx.check(
            "front lens sweeps around central shaft",
            c0[0] > 0.25 and c1[1] > 0.25 and abs(c1[0]) < 0.08,
            details=f"rest_center={c0}, quarter_turn_center={c1}",
        )
    else:
        ctx.fail("front lens sweeps around central shaft", "missing lens_0_glass AABB")

    door_closed = ctx.part_element_world_aabb(door, elem="door_leaf_glass")
    with ctx.pose({door_hinge: 0.90}):
        door_open = ctx.part_element_world_aabb(door, elem="door_leaf_glass")
    if door_closed is not None and door_open is not None:
        ctx.check(
            "door swings outward from lantern facet",
            door_open[1][0] > door_closed[1][0] + 0.10,
            details=f"closed={door_closed}, open={door_open}",
        )
    else:
        ctx.fail("door swings outward from lantern facet", "missing door_leaf_glass AABB")

    return ctx.report()


object_model = build_object_model()
