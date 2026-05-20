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
    mesh_from_geometry,
    wire_from_points,
)


STEEL_YELLOW = Material("crane_yellow", rgba=(1.0, 0.72, 0.04, 1.0))
DARK_STEEL = Material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
RUBBER = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
GALVANIZED = Material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
BALLAST = Material("concrete_ballast", rgba=(0.38, 0.40, 0.39, 1.0))
GLASS = Material("blue_tinted_glass", rgba=(0.24, 0.48, 0.70, 0.75))


def _beam_between(part, p0, p1, radius, material, name):
    """Add a cylindrical tube from p0 to p1 in the part frame."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _square_mast(part, *, half_width, z_min, z_max, post_size, tube_radius, prefix, material):
    """Build a connected square truss mast with four corner posts and X bracing."""
    height = z_max - z_min
    corners = [
        (-half_width, -half_width),
        (half_width, -half_width),
        (half_width, half_width),
        (-half_width, half_width),
    ]
    for i, (x, y) in enumerate(corners):
        _box(
            part,
            (post_size, post_size, height),
            (x, y, z_min + height * 0.5),
            material,
            f"{prefix}_post_{i}",
        )

    levels = [z_min + height * i / 4.0 for i in range(5)]
    for j, z in enumerate(levels):
        _box(part, (2.0 * half_width + post_size, tube_radius * 2.2, tube_radius * 2.2), (0.0, -half_width, z), material, f"{prefix}_ring_front_{j}")
        _box(part, (2.0 * half_width + post_size, tube_radius * 2.2, tube_radius * 2.2), (0.0, half_width, z), material, f"{prefix}_ring_rear_{j}")
        _box(part, (tube_radius * 2.2, 2.0 * half_width + post_size, tube_radius * 2.2), (-half_width, 0.0, z), material, f"{prefix}_ring_side_{j}")
        _box(part, (tube_radius * 2.2, 2.0 * half_width + post_size, tube_radius * 2.2), (half_width, 0.0, z), material, f"{prefix}_ring_side_b_{j}")

    for j in range(4):
        z0 = levels[j]
        z1 = levels[j + 1]
        flip = -1.0 if j % 2 else 1.0
        _beam_between(part, (-half_width * flip, half_width, z0), (half_width * flip, half_width, z1), tube_radius, material, f"{prefix}_brace_rear_{j}")
        _beam_between(part, (-half_width * flip, -half_width, z0), (half_width * flip, -half_width, z1), tube_radius, material, f"{prefix}_brace_front_{j}")
        _beam_between(part, (half_width, -half_width * flip, z0), (half_width, half_width * flip, z1), tube_radius, material, f"{prefix}_brace_side_{j}")
        _beam_between(part, (-half_width, -half_width * flip, z0), (-half_width, half_width * flip, z1), tube_radius, material, f"{prefix}_brace_side_b_{j}")


def _add_wheel_pair(part, *, x, axle_z, track_y, radius, width, name):
    """Static caster-style wheels mounted to an axle on the base part."""
    _beam_between(part, (x, -track_y, axle_z), (x, track_y, axle_z), 0.035, DARK_STEEL, f"{name}_axle")
    for side, y in (("neg", -track_y), ("pos", track_y)):
        part.visual(
            Cylinder(radius=radius, length=width),
            origin=Origin(xyz=(x, y, axle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=RUBBER,
            name=f"{name}_tire_{side}",
        )
        part.visual(
            Cylinder(radius=radius * 0.48, length=width * 1.08),
            origin=Origin(xyz=(x, y, axle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=GALVANIZED,
            name=f"{name}_hub_{side}",
        )


def _add_jib_truss(part, *, x0, x1, half_width, lower_z, upper_z):
    """Triangular horizontal jib with two lower trolley rails and an upper chord."""
    _beam_between(part, (x0, -half_width, lower_z), (x1, -half_width, lower_z), 0.035, STEEL_YELLOW, "lower_rail_front")
    _beam_between(part, (x0, half_width, lower_z), (x1, half_width, lower_z), 0.035, STEEL_YELLOW, "lower_rail_rear")
    _beam_between(part, (x0 + 0.05, 0.0, upper_z), (x1, 0.0, upper_z), 0.030, STEEL_YELLOW, "upper_chord")

    panel_count = 8
    for i in range(panel_count + 1):
        x = x0 + (x1 - x0) * i / panel_count
        _beam_between(part, (x, -half_width, lower_z), (x, half_width, lower_z), 0.020, STEEL_YELLOW, f"jib_cross_tie_{i}")

    for i in range(panel_count):
        xa = x0 + (x1 - x0) * i / panel_count
        xb = x0 + (x1 - x0) * (i + 1) / panel_count
        _beam_between(part, (xa, -half_width, lower_z), (xb, 0.0, upper_z), 0.020, STEEL_YELLOW, f"jib_web_front_{i}")
        _beam_between(part, (xa, half_width, lower_z), (xb, 0.0, upper_z), 0.020, STEEL_YELLOW, f"jib_web_rear_{i}")
        if i % 2 == 0:
            _beam_between(part, (xb, -half_width, lower_z), (xa, 0.0, upper_z), 0.016, STEEL_YELLOW, f"jib_counterweb_front_{i}")
            _beam_between(part, (xb, half_width, lower_z), (xa, 0.0, upper_z), 0.016, STEEL_YELLOW, f"jib_counterweb_rear_{i}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_erecting_tower_crane")

    base = model.part("base")
    _box(base, (1.45, 1.05, 0.20), (0.0, 0.0, 0.23), DARK_STEEL, "chassis")
    _box(base, (2.60, 0.13, 0.12), (0.0, 0.0, 0.15), DARK_STEEL, "long_outrigger")
    _box(base, (0.13, 2.30, 0.12), (0.0, 0.0, 0.15), DARK_STEEL, "cross_outrigger")
    for i, (x, y) in enumerate(((-1.25, -1.05), (1.25, -1.05), (1.25, 1.05), (-1.25, 1.05))):
        _box(base, (0.32, 0.22, 0.055), (x, y, 0.045), GALVANIZED, f"leveling_pad_{i}")
        _beam_between(base, (0.0, 0.0, 0.18), (x, y, 0.08), 0.025, DARK_STEEL, f"outrigger_strut_{i}")
    _add_wheel_pair(base, x=-0.48, axle_z=0.17, track_y=0.66, radius=0.18, width=0.12, name="front_wheel")
    _add_wheel_pair(base, x=0.48, axle_z=0.17, track_y=0.66, radius=0.18, width=0.12, name="rear_wheel")
    _square_mast(
        base,
        half_width=0.33,
        z_min=0.32,
        z_max=3.15,
        post_size=0.085,
        tube_radius=0.025,
        prefix="outer_mast",
        material=STEEL_YELLOW,
    )
    _box(base, (0.86, 0.085, 0.08), (0.0, -0.38, 3.13), STEEL_YELLOW, "sleeve_cap_front")
    _box(base, (0.86, 0.085, 0.08), (0.0, 0.38, 3.13), STEEL_YELLOW, "sleeve_cap_rear")
    _box(base, (0.085, 0.86, 0.08), (-0.38, 0.0, 3.13), STEEL_YELLOW, "sleeve_cap_side")
    _box(base, (0.085, 0.86, 0.08), (0.38, 0.0, 3.13), STEEL_YELLOW, "sleeve_cap_side_b")
    for i, (x, y) in enumerate(((-0.269, -0.269), (0.269, -0.269), (0.269, 0.269), (-0.269, 0.269))):
        _box(base, (0.072, 0.072, 0.22), (x, y, 3.03), GALVANIZED, f"mast_guide_{i}")

    mast = model.part("mast")
    _square_mast(
        mast,
        half_width=0.205,
        z_min=-1.50,
        z_max=2.66,
        post_size=0.060,
        tube_radius=0.019,
        prefix="inner_mast",
        material=STEEL_YELLOW,
    )
    _box(mast, (0.54, 0.54, 0.075), (0.0, 0.0, 2.70), STEEL_YELLOW, "slew_bearing_seat")

    jib = model.part("jib")
    jib.visual(
        Cylinder(radius=0.31, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=DARK_STEEL,
        name="slewing_ring",
    )
    _box(jib, (0.94, 0.70, 0.12), (0.17, 0.0, 0.16), STEEL_YELLOW, "turntable_deck")
    _box(jib, (0.34, 0.26, 0.34), (0.23, -0.35, 0.39), STEEL_YELLOW, "operator_cab")
    _box(jib, (0.28, 0.22, 0.22), (0.23, -0.48, 0.40), GLASS, "cab_window")
    _beam_between(jib, (0.05, 0.0, 0.20), (0.45, 0.0, 0.82), 0.038, STEEL_YELLOW, "jib_root_strut")
    _add_jib_truss(jib, x0=0.36, x1=5.45, half_width=0.22, lower_z=0.25, upper_z=0.82)
    _beam_between(jib, (-1.25, -0.18, 0.34), (0.05, -0.18, 0.25), 0.030, STEEL_YELLOW, "counterjib_chord_front")
    _beam_between(jib, (-1.25, 0.18, 0.34), (0.05, 0.18, 0.25), 0.030, STEEL_YELLOW, "counterjib_chord_rear")
    _beam_between(jib, (-1.20, 0.0, 0.34), (0.20, 0.0, 0.78), 0.024, STEEL_YELLOW, "counterjib_tie")
    _box(jib, (0.18, 0.48, 0.13), (0.02, 0.0, 0.255), STEEL_YELLOW, "counterjib_root_socket")
    _box(jib, (0.42, 0.62, 0.42), (-1.36, 0.0, 0.35), BALLAST, "counterweight")
    jib.visual(
        Cylinder(radius=0.12, length=0.10),
        origin=Origin(xyz=(5.52, 0.0, 0.26), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=GALVANIZED,
        name="jib_nose_sheave",
    )

    trolley = model.part("trolley")
    _box(trolley, (0.36, 0.50, 0.055), (0.0, 0.0, -0.075), DARK_STEEL, "trolley_frame")
    _box(trolley, (0.36, 0.035, 0.145), (0.0, -0.255, -0.025), DARK_STEEL, "trolley_side_front")
    _box(trolley, (0.36, 0.035, 0.145), (0.0, 0.255, -0.025), DARK_STEEL, "trolley_side_rear")
    for ix, x in enumerate((-0.13, 0.13)):
        for iy, y in enumerate((-0.22, 0.22)):
            trolley.visual(
                Cylinder(radius=0.045, length=0.055),
                origin=Origin(xyz=(x, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=GALVANIZED,
                name=f"trolley_wheel_{ix}_{iy}",
            )
    _beam_between(trolley, (0.0, 0.0, -0.075), (0.0, 0.0, -1.22), 0.012, DARK_STEEL, "hoist_rope")
    _box(trolley, (0.20, 0.16, 0.14), (0.0, 0.0, -1.22), GALVANIZED, "hook_block")
    hook_mesh = mesh_from_geometry(
        wire_from_points(
            [
                (0.0, 0.0, -1.24),
                (0.0, 0.0, -1.45),
                (0.075, 0.0, -1.58),
                (0.155, 0.0, -1.50),
                (0.125, 0.0, -1.40),
            ],
            radius=0.018,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.050,
        ),
        "load_hook",
    )
    trolley.visual(hook_mesh, material=DARK_STEEL, name="load_hook")

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 3.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80000.0, velocity=0.22, lower=0.0, upper=1.10),
    )
    model.articulation(
        "slew",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=jib,
        origin=Origin(xyz=(0.0, 0.0, 2.735)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60000.0, velocity=0.45, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "trolley_slide",
        ArticulationType.PRISMATIC,
        parent=jib,
        child=trolley,
        origin=Origin(xyz=(0.90, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.55, lower=0.0, upper=4.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    jib = object_model.get_part("jib")
    trolley = object_model.get_part("trolley")
    mast_slide = object_model.get_articulation("mast_slide")
    slew = object_model.get_articulation("slew")
    trolley_slide = object_model.get_articulation("trolley_slide")

    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        min_overlap=1.0,
        name="collapsed telescoping mast remains inserted in lower sleeve",
    )
    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 1.10}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            min_overlap=0.35,
            name="extended telescoping mast keeps retained sleeve insertion",
        )
        extended_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast slide raises the tower",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 1.0,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    rest_trolley_pos = ctx.part_world_position(trolley)
    with ctx.pose({trolley_slide: 4.25}):
        extended_trolley_pos = ctx.part_world_position(trolley)
        ctx.expect_overlap(
            trolley,
            jib,
            axes="xy",
            min_overlap=0.08,
            name="trolley remains under the jib rail envelope at full travel",
        )
    ctx.check(
        "trolley slide carries hook block along the jib",
        rest_trolley_pos is not None
        and extended_trolley_pos is not None
        and extended_trolley_pos[0] > rest_trolley_pos[0] + 4.0,
        details=f"rest={rest_trolley_pos}, extended={extended_trolley_pos}",
    )

    with ctx.pose({slew: math.pi / 2.0}):
        rotated_jib_aabb = ctx.part_world_aabb(jib)
    ctx.check(
        "slewing joint rotates the horizontal jib about the mast top",
        rotated_jib_aabb is not None and rotated_jib_aabb[1][1] > 5.0,
        details=f"rotated_jib_aabb={rotated_jib_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
