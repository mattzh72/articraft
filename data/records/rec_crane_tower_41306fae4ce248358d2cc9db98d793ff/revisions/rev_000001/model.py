from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _member_origin(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[Origin, float]:
    """Return an Origin that points a local-Z cylinder from start to end."""

    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("lattice member endpoints must be distinct")

    horizontal = math.hypot(dx, dy)
    pitch = math.atan2(horizontal, dz)
    yaw = math.atan2(dy, dx) if horizontal > 1e-9 else 0.0
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str,
    *,
    overlap: float = 0.08,
) -> None:
    """Add a round steel member with slight node penetration for welded joints."""

    origin, length = _member_origin(start, end)
    part.visual(
        Cylinder(radius=radius, length=length + overlap),
        origin=origin,
        material=material,
        name=name,
    )


def _add_box(part, size, xyz, material, name) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hammerhead_saddle_jib_tower_crane")

    yellow = model.material("crane_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    dark = model.material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    rail = model.material("worn_black_rail", rgba=(0.015, 0.015, 0.018, 1.0))
    concrete = model.material("cast_concrete", rgba=(0.55, 0.54, 0.50, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.22, 0.48, 0.75, 0.65))
    red = model.material("warning_red", rgba=(0.85, 0.05, 0.03, 1.0))

    mast = model.part("mast")

    # A real tower crane base is a heavy concrete foundation with steel anchor
    # frame; it anchors the square lattice mast and prevents the tower from
    # reading as a floating truss.
    _add_box(mast, (4.2, 4.2, 0.35), (0.0, 0.0, 0.175), concrete, "foundation")
    _add_box(mast, (2.3, 2.3, 0.18), (0.0, 0.0, 0.44), dark, "anchor_frame")

    mast_width = 1.8
    half = mast_width * 0.5
    mast_bottom = 0.45
    mast_top = 18.0
    levels = [mast_bottom + i * (mast_top - mast_bottom) / 6 for i in range(7)]
    corners = [
        (-half, -half),
        (half, -half),
        (half, half),
        (-half, half),
    ]

    for i, (x, y) in enumerate(corners):
        _add_tube(
            mast,
            (x, y, mast_bottom),
            (x, y, mast_top),
            0.075,
            yellow,
            f"mast_leg_{i}",
            overlap=0.14,
        )

    for li, z in enumerate(levels):
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _add_tube(
                mast,
                (x0, y0, z),
                (x1, y1, z),
                0.045,
                yellow,
                f"mast_ring_{li}_{i}",
                overlap=0.14,
            )

    # X bracing on all four faces, alternating bay by bay.
    for bi in range(len(levels) - 1):
        z0, z1 = levels[bi], levels[bi + 1]
        # Faces at constant Y.
        for face_i, y in enumerate((-half, half)):
            _add_tube(
                mast,
                (-half, y, z0),
                (half, y, z1),
                0.035,
                yellow,
                f"mast_diag_y_{face_i}_{bi}_0",
                overlap=0.16,
            )
            _add_tube(
                mast,
                (half, y, z0),
                (-half, y, z1),
                0.035,
                yellow,
                f"mast_diag_y_{face_i}_{bi}_1",
                overlap=0.16,
            )
        # Faces at constant X.
        for face_i, x in enumerate((-half, half)):
            _add_tube(
                mast,
                (x, -half, z0),
                (x, half, z1),
                0.035,
                yellow,
                f"mast_diag_x_{face_i}_{bi}_0",
                overlap=0.16,
            )
            _add_tube(
                mast,
                (x, half, z0),
                (x, -half, z1),
                0.035,
                yellow,
                f"mast_diag_x_{face_i}_{bi}_1",
                overlap=0.16,
            )

    # Fixed top cap that carries the slewing bearing.
    mast.visual(
        Cylinder(radius=1.05, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 18.10)),
        material=dark,
        name="mast_top_cap",
    )

    saddle_jib = model.part("saddle_jib")

    # The rotating upper works: slewing ring, machinery deck, cab, saddle jib,
    # counter jib, and counterweights are one continuous rotating assembly.
    saddle_jib.visual(
        Cylinder(radius=0.98, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=dark,
        name="slewing_ring",
    )
    _add_box(saddle_jib, (2.9, 2.15, 0.24), (0.0, 0.0, 0.37), yellow, "machinery_deck")
    _add_box(saddle_jib, (1.15, 0.82, 0.78), (1.05, -1.25, 0.88), yellow, "operator_cab")
    _add_box(saddle_jib, (0.03, 0.84, 0.42), (1.64, -1.25, 0.95), glass, "cab_front_window")
    _add_box(saddle_jib, (0.70, 0.03, 0.36), (1.02, -1.675, 0.97), glass, "cab_side_window")

    # Main horizontal hammerhead/saddle jib.
    jib_start = 1.10
    jib_end = 25.0
    bottom_z = 0.78
    top_z = 1.82
    jib_y = 0.58
    main_stations = [jib_start + i * (jib_end - jib_start) / 8 for i in range(9)]

    # Central saddle pedestal ties the turntable deck into both the forward jib
    # and counter-jib roots.
    _add_box(saddle_jib, (1.95, 1.45, 0.34), (0.20, 0.0, 0.63), yellow, "saddle_pedestal")
    for y in (-jib_y, jib_y):
        _add_tube(
            saddle_jib,
            (-0.80, y, bottom_z),
            (jib_start, y, bottom_z),
            0.060,
            yellow,
            f"saddle_bottom_bridge_{0 if y < 0 else 1}",
            overlap=0.16,
        )
        _add_tube(
            saddle_jib,
            (-0.80, y, top_z),
            (jib_start, y, top_z),
            0.050,
            yellow,
            f"saddle_top_bridge_{0 if y < 0 else 1}",
            overlap=0.16,
        )

    for y in (-jib_y, jib_y):
        _add_tube(
            saddle_jib,
            (jib_start, y, bottom_z),
            (jib_end, y, bottom_z),
            0.058,
            yellow,
            f"jib_bottom_chord_{0 if y < 0 else 1}",
            overlap=0.12,
        )
        _add_tube(
            saddle_jib,
            (jib_start, y, top_z),
            (jib_end, y, top_z),
            0.052,
            yellow,
            f"jib_top_chord_{0 if y < 0 else 1}",
            overlap=0.12,
        )

    for si, x in enumerate(main_stations):
        _add_tube(
            saddle_jib,
            (x, -jib_y, bottom_z),
            (x, jib_y, bottom_z),
            0.038,
            yellow,
            f"jib_bottom_cross_{si}",
            overlap=0.12,
        )
        _add_tube(
            saddle_jib,
            (x, -jib_y, top_z),
            (x, jib_y, top_z),
            0.035,
            yellow,
            f"jib_top_cross_{si}",
            overlap=0.12,
        )
        for y in (-jib_y, jib_y):
            _add_tube(
                saddle_jib,
                (x, y, bottom_z),
                (x, y, top_z),
                0.038,
                yellow,
                f"jib_vertical_{si}_{0 if y < 0 else 1}",
                overlap=0.12,
            )

    for bi in range(len(main_stations) - 1):
        x0, x1 = main_stations[bi], main_stations[bi + 1]
        for y in (-jib_y, jib_y):
            side = 0 if y < 0 else 1
            _add_tube(
                saddle_jib,
                (x0, y, bottom_z),
                (x1, y, top_z),
                0.033,
                yellow,
                f"jib_diag_{bi}_{side}_0",
                overlap=0.14,
            )
            _add_tube(
                saddle_jib,
                (x0, y, top_z),
                (x1, y, bottom_z),
                0.033,
                yellow,
                f"jib_diag_{bi}_{side}_1",
                overlap=0.14,
            )

    _add_box(
        saddle_jib,
        (23.4, 0.16, 0.10),
        ((jib_start + jib_end) * 0.5, 0.0, 0.55),
        rail,
        "trolley_rail",
    )
    for hi, x in enumerate(main_stations[1:-1]):
        _add_tube(
            saddle_jib,
            (x, 0.0, 0.56),
            (x, 0.0, bottom_z),
            0.026,
            dark,
            f"rail_hanger_{hi}",
            overlap=0.10,
        )
    _add_tube(
        saddle_jib,
        (jib_end - 0.12, -jib_y, bottom_z),
        (jib_end - 0.12, jib_y, bottom_z),
        0.065,
        red,
        "jib_end_stop",
        overlap=0.10,
    )

    # Short counter jib with suspended concrete counterweight slabs.
    counter_start = -0.65
    counter_end = -8.9
    counter_top_z = 1.45
    counter_stations = [counter_start + i * (counter_end - counter_start) / 4 for i in range(5)]
    for y in (-0.48, 0.48):
        _add_tube(
            saddle_jib,
            (counter_start, y, bottom_z),
            (counter_end, y, bottom_z),
            0.055,
            yellow,
            f"counter_bottom_chord_{0 if y < 0 else 1}",
            overlap=0.12,
        )
        _add_tube(
            saddle_jib,
            (counter_start, y, counter_top_z),
            (counter_end, y, counter_top_z),
            0.048,
            yellow,
            f"counter_top_chord_{0 if y < 0 else 1}",
            overlap=0.12,
        )
    for si, x in enumerate(counter_stations):
        _add_tube(
            saddle_jib,
            (x, -0.48, bottom_z),
            (x, 0.48, bottom_z),
            0.035,
            yellow,
            f"counter_cross_{si}",
            overlap=0.12,
        )
        for y in (-0.48, 0.48):
            _add_tube(
                saddle_jib,
                (x, y, bottom_z),
                (x, y, counter_top_z),
                0.035,
                yellow,
                f"counter_vertical_{si}_{0 if y < 0 else 1}",
                overlap=0.12,
            )
    for bi in range(len(counter_stations) - 1):
        x0, x1 = counter_stations[bi], counter_stations[bi + 1]
        for y in (-0.48, 0.48):
            side = 0 if y < 0 else 1
            _add_tube(
                saddle_jib,
                (x0, y, bottom_z),
                (x1, y, counter_top_z),
                0.032,
                yellow,
                f"counter_diag_{bi}_{side}",
                overlap=0.14,
            )

    for i, x in enumerate((-7.95, -7.35, -6.75)):
        _add_box(saddle_jib, (0.46, 1.55, 1.35), (x, 0.0, 0.24), concrete, f"counterweight_{i}")
        _add_tube(
            saddle_jib,
            (x, -0.64, 0.88),
            (x, -0.64, 1.48),
            0.025,
            dark,
            f"counter_strap_{i}_0",
            overlap=0.05,
        )
        _add_tube(
            saddle_jib,
            (x, 0.64, 0.88),
            (x, 0.64, 1.48),
            0.025,
            dark,
            f"counter_strap_{i}_1",
            overlap=0.05,
        )

    model.articulation(
        "mast_to_saddle",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=saddle_jib,
        origin=Origin(xyz=(0.0, 0.0, 18.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50000.0, velocity=0.35),
    )

    trolley = model.part("trolley")

    _add_box(trolley, (0.84, 0.92, 0.20), (0.0, 0.0, -0.08), yellow, "trolley_frame")
    _add_box(trolley, (0.72, 0.12, 0.20), (0.0, -0.52, -0.16), dark, "trolley_side_0")
    _add_box(trolley, (0.72, 0.12, 0.20), (0.0, 0.52, -0.16), dark, "trolley_side_1")
    for xi, x in enumerate((-0.27, 0.27)):
        _add_tube(
            trolley,
            (x, -0.47, 0.02),
            (x, -0.31, 0.02),
            0.115,
            dark,
            f"wheel_{xi}_0",
            overlap=0.0,
        )
        _add_tube(
            trolley,
            (x, 0.31, 0.02),
            (x, 0.47, 0.02),
            0.115,
            dark,
            f"wheel_{xi}_1",
            overlap=0.0,
        )

    # The prompt fixes the hook line block below the trolley rather than giving
    # it a hoist articulation, so the line, pulley block, and hook move as part
    # of the trolley link.
    _add_tube(
        trolley,
        (0.0, 0.0, -0.18),
        (0.0, 0.0, -6.45),
        0.022,
        rail,
        "hoist_line",
        overlap=0.08,
    )
    _add_box(trolley, (0.52, 0.32, 0.58), (0.0, 0.0, -6.78), yellow, "hook_block")
    _add_tube(
        trolley,
        (0.0, 0.0, -7.07),
        (0.0, 0.0, -7.42),
        0.035,
        dark,
        "hook_shank",
        overlap=0.04,
    )
    _add_tube(
        trolley,
        (0.0, 0.0, -7.42),
        (0.20, 0.0, -7.62),
        0.035,
        dark,
        "hook_curve_0",
        overlap=0.05,
    )
    _add_tube(
        trolley,
        (0.20, 0.0, -7.62),
        (0.05, 0.0, -7.82),
        0.035,
        dark,
        "hook_curve_1",
        overlap=0.05,
    )

    model.articulation(
        "saddle_to_trolley",
        ArticulationType.PRISMATIC,
        parent=saddle_jib,
        child=trolley,
        origin=Origin(xyz=(3.00, 0.0, 0.48)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.75, lower=0.0, upper=19.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    saddle = object_model.get_part("saddle_jib")
    trolley = object_model.get_part("trolley")
    slew = object_model.get_articulation("mast_to_saddle")
    travel = object_model.get_articulation("saddle_to_trolley")

    ctx.check(
        "saddle jib slews continuously",
        slew.articulation_type == ArticulationType.CONTINUOUS and tuple(slew.axis) == (0.0, 0.0, 1.0),
        details=f"type={slew.articulation_type}, axis={slew.axis}",
    )
    ctx.check(
        "trolley travel is prismatic along the jib",
        travel.articulation_type == ArticulationType.PRISMATIC
        and tuple(travel.axis) == (1.0, 0.0, 0.0)
        and travel.motion_limits is not None
        and travel.motion_limits.upper >= 19.0,
        details=f"type={travel.articulation_type}, axis={travel.axis}, limits={travel.motion_limits}",
    )

    # The slewing ring is seated on the mast cap at the vertical rotation axis.
    ctx.expect_contact(
        saddle,
        mast,
        elem_a="slewing_ring",
        elem_b="mast_top_cap",
        contact_tol=0.002,
        name="slewing ring sits on mast cap",
    )

    # At both travel extremes the trolley carriage remains under the jib rail.
    ctx.expect_overlap(
        trolley,
        saddle,
        axes="xy",
        elem_a="trolley_frame",
        elem_b="trolley_rail",
        min_overlap=0.10,
        name="trolley starts under bottom rail",
    )
    with ctx.pose({travel: 19.4}):
        ctx.expect_overlap(
            trolley,
            saddle,
            axes="xy",
            elem_a="trolley_frame",
            elem_b="trolley_rail",
            min_overlap=0.10,
            name="trolley remains under rail at full travel",
        )

    rest_pos = ctx.part_world_position(trolley)
    with ctx.pose({travel: 19.4}):
        extended_pos = ctx.part_world_position(trolley)
    ctx.check(
        "trolley moves outward along jib",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 19.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({slew: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(trolley)
    ctx.check(
        "saddle rotation carries trolley around mast",
        rest_pos is not None
        and rotated_pos is not None
        and abs(rotated_pos[0]) < 0.15
        and rotated_pos[1] > rest_pos[0] - 0.15,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    frame_aabb = ctx.part_element_world_aabb(trolley, elem="trolley_frame")
    block_aabb = ctx.part_element_world_aabb(trolley, elem="hook_block")
    ctx.check(
        "fixed hook block hangs below trolley",
        frame_aabb is not None and block_aabb is not None and block_aabb[1][2] < frame_aabb[0][2] - 5.5,
        details=f"frame={frame_aabb}, block={block_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
