from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _curved_panel(
    *,
    radius: float,
    thickness: float,
    height: float,
    start_deg: float,
    end_deg: float,
    z0: float,
    segments: int = 64,
) -> MeshGeometry:
    """Thin cylindrical shell segment, open in the middle of the car."""
    geom = MeshGeometry()
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    outer = radius + thickness * 0.5
    inner = radius - thickness * 0.5
    rings: list[tuple[int, int, int, int]] = []
    for i in range(segments + 1):
        t = start + (end - start) * i / segments
        c = math.cos(t)
        s = math.sin(t)
        rings.append(
            (
                geom.add_vertex(outer * c, outer * s, z0),
                geom.add_vertex(outer * c, outer * s, z0 + height),
                geom.add_vertex(inner * c, inner * s, z0),
                geom.add_vertex(inner * c, inner * s, z0 + height),
            )
        )
    for i in range(segments):
        ob0, ot0, ib0, it0 = rings[i]
        ob1, ot1, ib1, it1 = rings[i + 1]
        # outer and inner skins
        geom.add_face(ob0, ob1, ot1)
        geom.add_face(ob0, ot1, ot0)
        geom.add_face(ib1, ib0, it0)
        geom.add_face(ib1, it0, it1)
        # bottom and top annular edges
        geom.add_face(ib0, ib1, ob1)
        geom.add_face(ib0, ob1, ob0)
        geom.add_face(ot0, ot1, it1)
        geom.add_face(ot0, it1, it0)
    # radial end caps
    for idx in (0, segments):
        ob, ot, ib, it = rings[idx]
        geom.add_face(ob, ot, it)
        geom.add_face(ob, it, ib)
    return geom


def _hollow_cylinder_x(
    *, length: float, inner_radius: float, outer_radius: float, segments: int = 32
) -> MeshGeometry:
    """Hollow pin sleeve with its axis along local X."""
    geom = MeshGeometry()
    xs = (-length * 0.5, length * 0.5)
    rings: list[tuple[int, int, int, int]] = []
    for i in range(segments + 1):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        rings.append(
            (
                geom.add_vertex(xs[0], outer_radius * c, outer_radius * s),
                geom.add_vertex(xs[1], outer_radius * c, outer_radius * s),
                geom.add_vertex(xs[0], inner_radius * c, inner_radius * s),
                geom.add_vertex(xs[1], inner_radius * c, inner_radius * s),
            )
        )
    for i in range(segments):
        o0a, o1a, i0a, i1a = rings[i]
        o0b, o1b, i0b, i1b = rings[i + 1]
        geom.add_face(o0a, o0b, o1b)
        geom.add_face(o0a, o1b, o1a)
        geom.add_face(i0b, i0a, i1a)
        geom.add_face(i0b, i1a, i1b)
        geom.add_face(o0a, i0a, i0b)
        geom.add_face(o0a, i0b, o0b)
        geom.add_face(o1b, i1b, i1a)
        geom.add_face(o1b, i1a, o1a)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_panoramic_elevator")

    glass = model.material("blue_tinted_glass", rgba=(0.45, 0.78, 0.95, 0.32))
    stainless = model.material("brushed_stainless", rgba=(0.74, 0.74, 0.70, 1.0))
    dark = model.material("dark_rubber", rgba=(0.02, 0.022, 0.025, 1.0))
    floor_mat = model.material("speckled_stone_floor", rgba=(0.62, 0.60, 0.55, 1.0))
    rail_mat = model.material("machined_guide_rail", rgba=(0.52, 0.54, 0.56, 1.0))

    # Stationary shaft base and vertical prismatic guide rail.
    guide_rail = model.part("guide_rail")
    guide_rail.visual(
        Box((2.45, 2.00, 0.08)),
        origin=Origin(xyz=(0.0, 0.20, 0.04)),
        material=floor_mat,
        name="landing_slab",
    )
    guide_rail.visual(
        Box((0.24, 0.13, 6.4)),
        origin=Origin(xyz=(0.0, 1.12, 3.22)),
        material=rail_mat,
        name="center_track",
    )
    for x, nm in ((-0.18, "rail_post_0"), (0.18, "rail_post_1")):
        guide_rail.visual(
            Cylinder(radius=0.035, length=6.45),
            origin=Origin(xyz=(x, 1.02, 3.245)),
            material=stainless,
            name=nm,
        )
    guide_rail.visual(
        Box((0.78, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, 1.08, 6.52)),
        material=rail_mat,
        name="top_header",
    )

    # Moving circular glass car, authored as a thin cylindrical shell with an
    # open front bay for the sliding door.
    car = model.part("car")
    car.visual(
        Cylinder(radius=0.92, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=floor_mat,
        name="round_floor",
    )
    car.visual(
        Cylinder(radius=0.94, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 2.46)),
        material=stainless,
        name="ceiling_ring",
    )
    car.visual(
        mesh_from_geometry(
            _curved_panel(
                radius=0.90,
                thickness=0.035,
                height=2.26,
                start_deg=-50,
                end_deg=230,
                z0=0.15,
            ),
            "curved_glass_wall",
        ),
        material=glass,
        name="curved_glass_wall",
    )
    # Stainless door jambs and curved tracks are part of the car shell.
    for x, nm in ((0.58, "door_jamb_0"), (-0.58, "door_jamb_1")):
        y = -0.69
        car.visual(
            Cylinder(radius=0.045, length=2.33),
            origin=Origin(xyz=(x, y, 1.30)),
            material=stainless,
            name=nm,
        )
    for z, nm in ((0.24, "lower_door_track"), (2.32, "upper_door_track")):
        car.visual(
            mesh_from_geometry(
                _curved_panel(
                    radius=0.86,
                    thickness=0.055,
                    height=0.045,
                    start_deg=-128,
                    end_deg=-52,
                    z0=z,
                    segments=24,
                ),
                nm,
            ),
            material=stainless,
            name=nm,
        )
    # Guide shoes and carriage brackets visually capture the vertical rail.
    car.visual(
        Box((0.42, 0.20, 0.22)),
        origin=Origin(xyz=(0.0, 0.84, 0.58)),
        material=stainless,
        name="lower_carriage",
    )
    car.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(-0.18, 0.940, 0.58), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lower_roller_0",
    )
    car.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(0.18, 0.940, 0.58), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lower_roller_1",
    )
    car.visual(
        Box((0.42, 0.20, 0.22)),
        origin=Origin(xyz=(0.0, 0.84, 2.03)),
        material=stainless,
        name="upper_carriage",
    )
    car.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(-0.18, 0.940, 2.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="upper_roller_0",
    )
    car.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(0.18, 0.940, 2.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="upper_roller_1",
    )

    # Clevis-style pin mount for the handrail, attached to the rear glass wall.
    for x, suffix in ((-0.08, "0"), (0.08, "1")):
        car.visual(
            Box((0.026, 0.18, 0.050)),
            origin=Origin(xyz=(x, 0.80, 1.12)),
            material=stainless,
            name=f"handrail_cheek_{suffix}",
        )
        car.visual(
            Box((0.026, 0.040, 0.13)),
            origin=Origin(xyz=(x, 0.72, 1.12)),
            material=stainless,
            name=f"handrail_lug_{suffix}",
        )
    car.visual(
        Cylinder(radius=0.018, length=0.18),
        origin=Origin(xyz=(0.0, 0.72, 1.12), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="handrail_pin",
    )

    model.articulation(
        "rail_to_car",
        ArticulationType.PRISMATIC,
        parent=guide_rail,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=3.35, effort=9000.0, velocity=1.2),
    )

    # One curved glass door panel translates laterally to uncover the opening.
    door = model.part("door")
    door.visual(
        mesh_from_geometry(
            _curved_panel(
                radius=0.82,
                thickness=0.030,
                height=2.02,
                start_deg=-122,
                end_deg=-58,
                z0=0.30,
                segments=28,
            ),
            "curved_door_panel",
        ),
        material=glass,
        name="curved_door_panel",
    )
    for z, nm in ((0.285, "lower_door_shoe"), (2.270, "upper_door_shoe")):
        door.visual(
            mesh_from_geometry(
                _curved_panel(
                    radius=0.82,
                    thickness=0.040,
                    height=0.050,
                    start_deg=-122,
                    end_deg=-58,
                    z0=z,
                    segments=18,
                ),
                nm,
            ),
            material=stainless,
            name=nm,
        )
    door.visual(
        Box((0.04, 0.035, 1.05)),
        origin=Origin(xyz=(0.0, -0.80, 1.26)),
        material=stainless,
        name="vertical_pull_bar",
    )
    model.articulation(
        "car_to_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.44, effort=180.0, velocity=0.45),
    )

    # Stainless interior perimeter handrail on a revolute pin mount.
    handrail = model.part("handrail")
    handrail.visual(
        mesh_from_geometry(
            _hollow_cylinder_x(length=0.108, inner_radius=0.018, outer_radius=0.034),
            "handrail_sleeve",
        ),
        material=stainless,
        name="handrail_sleeve",
    )
    handrail.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.052, -0.006, -0.046),
                    (0.056, -0.040, -0.046),
                    (0.036, -0.060, -0.018),
                    (0.024, -0.040, 0.0),
                ],
                radius=0.012,
                samples_per_segment=8,
                radial_segments=14,
                cap_ends=True,
            ),
            "handrail_arm",
        ),
        material=stainless,
        name="handrail_arm",
    )
    rail_points = []
    for deg in range(-32, 213, 12):
        t = math.radians(deg)
        rail_points.append((0.68 * math.cos(t), 0.68 * math.sin(t) - 0.72, 0.0))
    handrail.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                rail_points,
                radius=0.022,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            ),
            "perimeter_handrail",
        ),
        material=stainless,
        name="perimeter_handrail",
    )
    for deg in (10, 170):
        t = math.radians(deg)
        x = 0.68 * math.cos(t)
        y = 0.68 * math.sin(t) - 0.72
        handrail.visual(
            Cylinder(radius=0.010, length=0.12),
            origin=Origin(xyz=(x, y + 0.055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"pin_standoff_{deg}",
        )
    model.articulation(
        "car_to_handrail",
        ArticulationType.REVOLUTE,
        parent=car,
        child=handrail,
        origin=Origin(xyz=(0.0, 0.72, 1.12)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.18, upper=0.18, effort=20.0, velocity=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    car = object_model.get_part("car")
    guide_rail = object_model.get_part("guide_rail")
    door = object_model.get_part("door")
    handrail = object_model.get_part("handrail")
    car_lift = object_model.get_articulation("rail_to_car")
    door_slide = object_model.get_articulation("car_to_door")
    handrail_pivot = object_model.get_articulation("car_to_handrail")

    ctx.allow_overlap(
        car,
        handrail,
        elem_a="handrail_pin",
        elem_b="handrail_sleeve",
        reason=(
            "The stainless handrail sleeve is intentionally captured around the "
            "revolute pin; the mesh sleeve is a proxy for a bushed pin joint."
        ),
    )
    ctx.expect_overlap(
        car,
        handrail,
        axes="x",
        elem_a="handrail_pin",
        elem_b="handrail_sleeve",
        min_overlap=0.08,
        name="handrail sleeve has retained length on the pin",
    )
    ctx.expect_within(
        car,
        handrail,
        axes="yz",
        inner_elem="handrail_pin",
        outer_elem="handrail_sleeve",
        margin=0.001,
        name="handrail pin is centered inside sleeve outside diameter",
    )
    ctx.expect_overlap(
        door,
        car,
        axes="z",
        elem_a="curved_door_panel",
        elem_b="curved_glass_wall",
        min_overlap=1.8,
        name="door panel is full-height in the car opening",
    )
    ctx.expect_within(
        handrail,
        car,
        axes="xy",
        elem_a="perimeter_handrail",
        elem_b="round_floor",
        margin=0.02,
        name="handrail sits inside circular car footprint",
    )

    rest_car_pos = ctx.part_world_position(car)
    rest_door_pos = ctx.part_world_position(door)
    rest_handrail_aabb = ctx.part_element_world_aabb(handrail, elem="perimeter_handrail")
    with ctx.pose({handrail_pivot: 0.12}):
        tilted_handrail_aabb = ctx.part_element_world_aabb(
            handrail, elem="perimeter_handrail"
        )
    with ctx.pose({car_lift: 1.25, door_slide: 0.36, handrail_pivot: 0.12}):
        raised_car_pos = ctx.part_world_position(car)
        open_door_pos = ctx.part_world_position(door)
        ctx.expect_overlap(
            car,
            guide_rail,
            axes="z",
            elem_a="upper_carriage",
            elem_b="center_track",
            min_overlap=0.20,
            name="raised car remains engaged with vertical guide rail",
        )

    ctx.check(
        "car lift travels upward",
        rest_car_pos is not None
        and raised_car_pos is not None
        and raised_car_pos[2] > rest_car_pos[2] + 1.0,
        details=f"rest={rest_car_pos}, raised={raised_car_pos}",
    )
    ctx.check(
        "door slides laterally",
        rest_door_pos is not None
        and open_door_pos is not None
        and open_door_pos[0] > rest_door_pos[0] + 0.30,
        details=f"rest={rest_door_pos}, open={open_door_pos}",
    )
    ctx.check(
        "handrail has a revolute pin motion",
        rest_handrail_aabb is not None
        and tilted_handrail_aabb is not None
        and abs(
            (tilted_handrail_aabb[0][2] + tilted_handrail_aabb[1][2])
            - (rest_handrail_aabb[0][2] + rest_handrail_aabb[1][2])
        )
        > 0.04,
        details=f"rest={rest_handrail_aabb}, tilted={tilted_handrail_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
