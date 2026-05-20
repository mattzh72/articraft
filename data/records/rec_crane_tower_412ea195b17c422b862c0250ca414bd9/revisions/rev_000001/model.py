from __future__ import annotations

import math

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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _bar_origin_between(p0, p1) -> tuple[float, Origin]:
    """Return a length and transform for a box whose local +X spans p0->p1."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    horizontal = math.sqrt(dx * dx + dy * dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, horizontal)
    return length, Origin(xyz=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), rpy=(0.0, pitch, yaw))


def _add_bar(part, name: str, p0, p1, thickness: float, material: Material) -> None:
    length, origin = _bar_origin_between(p0, p1)
    part.visual(Box((length, thickness, thickness)), origin=origin, material=material, name=name)


def _add_box(part, name: str, size, center, material: Material) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hammerhead_top_slewing_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(1.0, 0.72, 0.06, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    bearing_grey = model.material("bearing_grey", rgba=(0.36, 0.38, 0.40, 1.0))
    counterweight_grey = model.material("counterweight_grey", rgba=(0.42, 0.41, 0.38, 1.0))
    glass_blue = model.material("cab_glass_blue", rgba=(0.20, 0.43, 0.75, 0.70))

    mast = model.part("mast")
    mast_width = 2.2
    post_xy = 0.14
    mast_height = 20.0
    half = mast_width / 2
    # Four heavy corner posts make the square tower section read as a real lattice mast.
    for ix, x in enumerate((-half, half)):
        for iy, y in enumerate((-half, half)):
            _add_box(
                mast,
                f"corner_post_{ix}_{iy}",
                (post_xy, post_xy, mast_height),
                (x, y, mast_height / 2),
                crane_yellow,
            )

    _add_box(mast, "base_footing", (3.4, 3.4, 0.28), (0.0, 0.0, 0.14), bearing_grey)
    _add_box(mast, "top_bearing_plate", (2.55, 2.55, 0.08), (0.0, 0.0, mast_height - 0.04), bearing_grey)

    # Keep the top lattice ring just below the slewing interface so the bearing plates
    # touch cleanly instead of interpenetrating.
    levels = [i * 2.5 for i in range(8)] + [mast_height - 0.14]
    for level_i, z in enumerate(levels):
        _add_bar(mast, f"front_tie_{level_i}", (-half, -half, z), (half, -half, z), 0.10, crane_yellow)
        _add_bar(mast, f"rear_tie_{level_i}", (-half, half, z), (half, half, z), 0.10, crane_yellow)
        _add_bar(mast, f"side_tie_0_{level_i}", (-half, -half, z), (-half, half, z), 0.10, crane_yellow)
        _add_bar(mast, f"side_tie_1_{level_i}", (half, -half, z), (half, half, z), 0.10, crane_yellow)

    for seg_i, z0 in enumerate(levels[:-1]):
        z1 = levels[seg_i + 1]
        if seg_i % 2 == 0:
            front_a, front_b = (-half, -half, z0), (half, -half, z1)
            rear_a, rear_b = (-half, half, z0), (half, half, z1)
            left_a, left_b = (-half, -half, z0), (-half, half, z1)
            right_a, right_b = (half, -half, z0), (half, half, z1)
        else:
            front_a, front_b = (half, -half, z0), (-half, -half, z1)
            rear_a, rear_b = (half, half, z0), (-half, half, z1)
            left_a, left_b = (-half, half, z0), (-half, -half, z1)
            right_a, right_b = (half, half, z0), (half, -half, z1)
        _add_bar(mast, f"front_diag_{seg_i}", front_a, front_b, 0.10, crane_yellow)
        _add_bar(mast, f"rear_diag_{seg_i}", rear_a, rear_b, 0.10, crane_yellow)
        _add_bar(mast, f"side_diag_0_{seg_i}", left_a, left_b, 0.10, crane_yellow)
        _add_bar(mast, f"side_diag_1_{seg_i}", right_a, right_b, 0.10, crane_yellow)

    turntable = model.part("turntable")
    _add_box(turntable, "lower_bearing_plate", (2.65, 2.65, 0.08), (0.0, 0.0, 0.04), bearing_grey)
    _add_box(turntable, "bearing_pedestal", (1.15, 1.15, 0.24), (0.0, 0.0, 0.18), bearing_grey)
    turntable.visual(
        mesh_from_geometry(TorusGeometry(radius=1.18, tube=0.10, radial_segments=18, tubular_segments=64), "slewing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=bearing_grey,
        name="slewing_ring",
    )
    _add_box(turntable, "machinery_deck", (2.7, 2.0, 0.42), (-0.18, 0.0, 0.48), crane_yellow)
    _add_box(turntable, "machinery_house", (1.4, 1.15, 0.72), (-0.75, 0.0, 0.96), crane_yellow)
    _add_box(turntable, "operator_cab", (0.82, 0.78, 0.68), (0.70, -1.18, 0.92), crane_yellow)
    _add_box(turntable, "cab_window", (0.56, 0.035, 0.36), (0.80, -1.585, 0.98), glass_blue)

    # Cathead and pendant ties above the slewing deck keep the long horizontal trusses believable.
    _add_bar(turntable, "cathead_post", (0.0, 0.0, 0.58), (0.0, 0.0, 3.65), 0.18, crane_yellow)
    _add_bar(turntable, "cathead_cross", (0.0, -0.82, 3.55), (0.0, 0.82, 3.55), 0.12, crane_yellow)

    # Main jib: wide double side chords, lower trolley rails, and triangulated web.
    jib_start = 0.15
    jib_end = 24.0
    jib_width = 1.55
    y_left, y_right = -jib_width / 2, jib_width / 2
    bottom_z = 1.02
    top_z = 2.34
    for y, side in ((y_left, "near"), (y_right, "far")):
        _add_bar(turntable, f"main_bottom_chord_{side}", (jib_start, y, bottom_z), (jib_end, y, bottom_z), 0.16, crane_yellow)
        _add_bar(turntable, f"main_top_chord_{side}", (jib_start, y, top_z), (jib_end, y, top_z), 0.13, crane_yellow)
        _add_bar(turntable, f"trolley_rail_{side}", (0.95, y, 0.92), (23.2, y, 0.92), 0.11, dark_steel)

    main_nodes = [0.15, 3.2, 6.4, 9.6, 12.8, 16.0, 19.2, 22.4, 24.0]
    for node_i, x in enumerate(main_nodes):
        _add_bar(turntable, f"main_cross_bottom_{node_i}", (x, y_left, bottom_z), (x, y_right, bottom_z), 0.105, crane_yellow)
        _add_bar(turntable, f"main_cross_top_{node_i}", (x, y_left, top_z), (x, y_right, top_z), 0.095, crane_yellow)
        _add_bar(turntable, f"main_vertical_near_{node_i}", (x, y_left, bottom_z), (x, y_left, top_z), 0.095, crane_yellow)
        _add_bar(turntable, f"main_vertical_far_{node_i}", (x, y_right, bottom_z), (x, y_right, top_z), 0.095, crane_yellow)

    for seg_i in range(len(main_nodes) - 1):
        x0, x1 = main_nodes[seg_i], main_nodes[seg_i + 1]
        for y, side in ((y_left, "near"), (y_right, "far")):
            if seg_i % 2 == 0:
                _add_bar(turntable, f"main_web_{side}_{seg_i}", (x0, y, bottom_z), (x1, y, top_z), 0.085, crane_yellow)
            else:
                _add_bar(turntable, f"main_web_{side}_{seg_i}", (x0, y, top_z), (x1, y, bottom_z), 0.085, crane_yellow)
    _add_bar(turntable, "main_pendant_near", (0.0, -0.52, 3.55), (16.0, y_left, top_z), 0.055, dark_steel)
    _add_bar(turntable, "main_pendant_far", (0.0, 0.52, 3.55), (16.0, y_right, top_z), 0.055, dark_steel)

    # Counter-jib: shorter opposite truss with concrete counterweight stack.
    counter_end = -8.6
    counter_nodes = [0.15, -2.8, -5.6, counter_end]
    for y, side in ((y_left, "near"), (y_right, "far")):
        _add_bar(turntable, f"counter_bottom_chord_{side}", (0.0, y, bottom_z), (counter_end, y, bottom_z), 0.17, crane_yellow)
        _add_bar(turntable, f"counter_top_chord_{side}", (0.0, y, top_z), (counter_end, y, top_z), 0.13, crane_yellow)
    for node_i, x in enumerate(counter_nodes):
        _add_bar(turntable, f"counter_cross_bottom_{node_i}", (x, y_left, bottom_z), (x, y_right, bottom_z), 0.105, crane_yellow)
        _add_bar(turntable, f"counter_cross_top_{node_i}", (x, y_left, top_z), (x, y_right, top_z), 0.095, crane_yellow)
        _add_bar(turntable, f"counter_vertical_near_{node_i}", (x, y_left, bottom_z), (x, y_left, top_z), 0.095, crane_yellow)
        _add_bar(turntable, f"counter_vertical_far_{node_i}", (x, y_right, bottom_z), (x, y_right, top_z), 0.095, crane_yellow)
    for seg_i in range(len(counter_nodes) - 1):
        x0, x1 = counter_nodes[seg_i], counter_nodes[seg_i + 1]
        for y, side in ((y_left, "near"), (y_right, "far")):
            _add_bar(turntable, f"counter_web_{side}_{seg_i}", (x0, y, top_z), (x1, y, bottom_z), 0.085, crane_yellow)
    _add_bar(turntable, "counter_pendant_near", (0.0, -0.52, 3.55), (-6.8, y_left, top_z), 0.055, dark_steel)
    _add_bar(turntable, "counter_pendant_far", (0.0, 0.52, 3.55), (-6.8, y_right, top_z), 0.055, dark_steel)
    for idx, x in enumerate((-7.25, -7.85, -8.45)):
        _add_box(turntable, f"counterweight_{idx}", (0.48, 1.55, 0.92), (x, 0.0, 0.58), counterweight_grey)

    trolley = model.part("trolley")
    _add_box(trolley, "trolley_crossbeam", (0.82, 1.92, 0.16), (0.0, 0.0, -0.08), dark_steel)
    _add_box(trolley, "left_wheel_frame", (0.72, 0.23, 0.18), (0.0, y_left, 0.0), crane_yellow)
    _add_box(trolley, "right_wheel_frame", (0.72, 0.23, 0.18), (0.0, y_right, 0.0), crane_yellow)
    for x in (-0.28, 0.28):
        for y, side in ((y_left, "left"), (y_right, "right")):
            trolley.visual(
                Cylinder(radius=0.10, length=0.13),
                origin=Origin(xyz=(x, y, 0.045), rpy=(math.pi / 2, 0.0, 0.0)),
                material=dark_steel,
                name=f"rail_wheel_{side}_{'front' if x > 0 else 'rear'}",
            )
    trolley.visual(
        Cylinder(radius=0.025, length=3.10),
        origin=Origin(xyz=(0.0, 0.0, -1.59)),
        material=dark_steel,
        name="hoist_cable",
    )
    _add_box(trolley, "hook_block", (0.55, 0.38, 0.48), (0.0, 0.0, -3.38), bearing_grey)
    hook_tube = tube_from_spline_points(
        [
            (0.0, 0.0, -3.58),
            (0.0, 0.0, -3.90),
            (0.18, 0.0, -4.10),
            (0.10, 0.0, -4.34),
            (-0.14, 0.0, -4.26),
            (-0.04, 0.0, -4.08),
        ],
        radius=0.045,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    trolley.visual(mesh_from_geometry(hook_tube, "hook"), material=dark_steel, name="hook")

    model.articulation(
        "mast_to_turntable",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, mast_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.18, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "turntable_to_trolley",
        ArticulationType.PRISMATIC,
        parent=turntable,
        child=trolley,
        origin=Origin(xyz=(2.0, 0.0, 0.72)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.65, lower=0.0, upper=19.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    turntable = object_model.get_part("turntable")
    trolley = object_model.get_part("trolley")
    slew = object_model.get_articulation("mast_to_turntable")
    trolley_slide = object_model.get_articulation("turntable_to_trolley")

    def coord(vec, index: int) -> float:
        attr = "xyz"[index]
        return getattr(vec, attr) if hasattr(vec, attr) else vec[index]

    # The slewing assembly should sit on the mast-top bearing rather than float above it.
    ctx.expect_gap(
        turntable,
        mast,
        axis="z",
        positive_elem="lower_bearing_plate",
        negative_elem="top_bearing_plate",
        min_gap=0.0,
        max_gap=0.001,
        name="slewing bearing seated on mast",
    )
    ctx.expect_overlap(
        turntable,
        mast,
        axes="xy",
        elem_a="lower_bearing_plate",
        elem_b="top_bearing_plate",
        min_overlap=2.0,
        name="bearing plates overlap in plan",
    )

    # The hook trolley wheels are carried by the bottom rail pair of the main jib.
    for side, wheel, rail in (
        ("near", "rail_wheel_left_front", "trolley_rail_near"),
        ("far", "rail_wheel_right_front", "trolley_rail_far"),
    ):
        ctx.expect_gap(
            turntable,
            trolley,
            axis="z",
            positive_elem=rail,
            negative_elem=wheel,
            min_gap=0.0,
            max_gap=0.002,
            name=f"trolley wheel contacts {side} rail underside",
        )
        ctx.expect_overlap(
            trolley,
            turntable,
            axes="xy",
            elem_a=wheel,
            elem_b=rail,
            min_overlap=0.05,
            name=f"trolley wheel remains under {side} rail",
        )

    main_aabb = ctx.part_element_world_aabb(turntable, elem="main_bottom_chord_near")
    counter_aabb = ctx.part_element_world_aabb(turntable, elem="counter_bottom_chord_near")
    ctx.check(
        "main jib is much longer than counter jib",
        main_aabb is not None
        and counter_aabb is not None
        and coord(main_aabb[1], 0) > 23.5
        and coord(counter_aabb[0], 0) < -8.4,
        details=f"main={main_aabb}, counter={counter_aabb}",
    )

    rest_pos = ctx.part_world_position(trolley)
    with ctx.pose({trolley_slide: 19.2}):
        ctx.expect_within(
            trolley,
            turntable,
            axes="x",
            inner_elem="trolley_crossbeam",
            outer_elem="trolley_rail_near",
            margin=0.0,
            name="extended trolley still lies within main rail length",
        )
        extended_pos = ctx.part_world_position(trolley)
    ctx.check(
        "trolley translates outward along the main jib",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 18.5,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({slew: math.pi / 2}):
        rotated_main = ctx.part_element_world_aabb(turntable, elem="main_bottom_chord_near")
    ctx.check(
        "slewing joint rotates the horizontal jib about the mast",
        rotated_main is not None and coord(rotated_main[1], 1) > 23.0,
        details=f"rotated main jib aabb={rotated_main}",
    )

    return ctx.report()


object_model = build_object_model()
