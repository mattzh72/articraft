from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _beam_between_xz(part, p0, p1, thickness, depth, *, name, material):
    """Rectangular timber between two points in an XZ plane (constant Y)."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    angle_y = math.atan2(dx, dz)
    part.visual(
        Box((thickness, depth, length)),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, y0, (z0 + z1) * 0.5),
            rpy=(0.0, angle_y, 0.0),
        ),
        material=material,
        name=name,
    )


def _beam_between_yz(part, p0, p1, thickness, depth, *, name, material):
    """Rectangular timber between two points in a YZ plane (constant X)."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dy = y1 - y0
    dz = z1 - z0
    length = math.hypot(dy, dz)
    # local +Z rotated about X becomes (0, -sin(rx), cos(rx)).
    angle_x = -math.atan2(dy, dz)
    part.visual(
        Box((depth, thickness, length)),
        origin=Origin(
            xyz=(x0, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(angle_x, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nordic_stave_bell_tower")

    weathered_wood = model.material("weathered_wood", rgba=(0.34, 0.20, 0.10, 1.0))
    dark_endgrain = model.material("dark_endgrain", rgba=(0.18, 0.10, 0.055, 1.0))
    shingle_mat = model.material("tarred_wood_shingles", rgba=(0.08, 0.075, 0.065, 1.0))
    iron_mat = model.material("blackened_cast_iron", rgba=(0.035, 0.038, 0.04, 1.0))
    worn_iron = model.material("worn_iron_edges", rgba=(0.12, 0.12, 0.115, 1.0))
    stone_mat = model.material("fieldstone", rgba=(0.29, 0.28, 0.25, 1.0))

    tower = model.part("tower")

    # Stone pads and heavy timber sills at the foot of the stave frame.
    for ix, x in enumerate((-0.56, 0.56)):
        for iy, y in enumerate((-0.56, 0.56)):
            tower.visual(
                Box((0.30, 0.30, 0.17)),
                origin=Origin(xyz=(x, y, 0.085)),
                material=stone_mat,
                name=f"stone_pad_{ix}_{iy}",
            )

    tower.visual(Box((1.38, 0.18, 0.18)), origin=Origin(xyz=(0.0, -0.60, 0.26)), material=weathered_wood, name="front_sill")
    tower.visual(Box((1.38, 0.18, 0.18)), origin=Origin(xyz=(0.0, 0.60, 0.26)), material=weathered_wood, name="rear_sill")
    tower.visual(Box((0.18, 1.38, 0.18)), origin=Origin(xyz=(-0.60, 0.0, 0.26)), material=weathered_wood, name="side_sill_0")
    tower.visual(Box((0.18, 1.38, 0.18)), origin=Origin(xyz=(0.60, 0.0, 0.26)), material=weathered_wood, name="side_sill_1")

    # Four continuous corner staves/posts.
    for ix, x in enumerate((-0.58, 0.58)):
        for iy, y in enumerate((-0.58, 0.58)):
            tower.visual(
                Box((0.16, 0.16, 4.28)),
                origin=Origin(xyz=(x, y, 2.38)),
                material=weathered_wood,
                name=f"corner_post_{ix}_{iy}",
            )

    # Boarded lower shaft with timber-frame rails.
    for z, label in ((0.55, "lower"), (1.62, "middle"), (2.76, "upper")):
        tower.visual(Box((1.28, 0.10, 0.12)), origin=Origin(xyz=(0.0, -0.65, z)), material=weathered_wood, name=f"front_{label}_rail")
        tower.visual(Box((1.28, 0.10, 0.12)), origin=Origin(xyz=(0.0, 0.65, z)), material=weathered_wood, name=f"rear_{label}_rail")
        tower.visual(Box((0.10, 1.28, 0.12)), origin=Origin(xyz=(-0.65, 0.0, z)), material=weathered_wood, name=f"side_{label}_rail_0")
        tower.visual(Box((0.10, 1.28, 0.12)), origin=Origin(xyz=(0.65, 0.0, z)), material=weathered_wood, name=f"side_{label}_rail_1")

    board_xs = (-0.40, -0.20, 0.0, 0.20, 0.40)
    for i, x in enumerate(board_xs):
        tower.visual(Box((0.12, 0.035, 2.05)), origin=Origin(xyz=(x, -0.675, 1.65)), material=dark_endgrain, name=f"front_stave_{i}")
        tower.visual(Box((0.12, 0.035, 2.05)), origin=Origin(xyz=(x, 0.675, 1.65)), material=dark_endgrain, name=f"rear_stave_{i}")
        tower.visual(Box((0.035, 0.12, 2.05)), origin=Origin(xyz=(-0.675, x, 1.65)), material=dark_endgrain, name=f"side_stave_0_{i}")
        tower.visual(Box((0.035, 0.12, 2.05)), origin=Origin(xyz=(0.675, x, 1.65)), material=dark_endgrain, name=f"side_stave_1_{i}")

    _beam_between_xz(tower, (-0.47, -0.71, 0.62), (0.47, -0.71, 2.70), 0.09, 0.08, name="front_diagonal_brace", material=weathered_wood)
    _beam_between_xz(tower, (0.47, 0.71, 0.62), (-0.47, 0.71, 2.70), 0.09, 0.08, name="rear_diagonal_brace", material=weathered_wood)
    _beam_between_yz(tower, (-0.71, -0.47, 0.62), (-0.71, 0.47, 2.70), 0.09, 0.08, name="side_diagonal_brace_0", material=weathered_wood)
    _beam_between_yz(tower, (0.71, 0.47, 0.62), (0.71, -0.47, 2.70), 0.09, 0.08, name="side_diagonal_brace_1", material=weathered_wood)

    # Open belfry tier: rails, knee braces, and bearing blocks for the swinging yoke.
    tower.visual(Box((1.30, 0.12, 0.12)), origin=Origin(xyz=(0.0, -0.64, 3.10)), material=weathered_wood, name="belfry_front_lower_rail")
    for z, label in ((3.10, "lower"), (3.78, "middle"), (4.42, "upper")):
        if label == "lower":
            pass
        else:
            tower.visual(Box((1.30, 0.12, 0.12)), origin=Origin(xyz=(0.0, -0.64, z)), material=weathered_wood, name=f"belfry_front_{label}_rail")
        tower.visual(Box((1.30, 0.12, 0.12)), origin=Origin(xyz=(0.0, 0.64, z)), material=weathered_wood, name=f"belfry_rear_{label}_rail")
        tower.visual(Box((0.12, 1.30, 0.12)), origin=Origin(xyz=(-0.64, 0.0, z)), material=weathered_wood, name=f"belfry_side_{label}_rail_0")
        tower.visual(Box((0.12, 1.30, 0.12)), origin=Origin(xyz=(0.64, 0.0, z)), material=weathered_wood, name=f"belfry_side_{label}_rail_1")

    _beam_between_xz(tower, (-0.52, -0.66, 3.14), (0.52, -0.66, 4.32), 0.08, 0.08, name="front_belfry_knee", material=weathered_wood)
    _beam_between_xz(tower, (0.52, 0.66, 3.14), (-0.52, 0.66, 4.32), 0.08, 0.08, name="rear_belfry_knee", material=weathered_wood)
    _beam_between_yz(tower, (-0.66, -0.52, 3.14), (-0.66, 0.52, 4.32), 0.08, 0.08, name="side_belfry_knee_0", material=weathered_wood)
    _beam_between_yz(tower, (0.66, 0.52, 3.14), (0.66, -0.52, 4.32), 0.08, 0.08, name="side_belfry_knee_1", material=weathered_wood)

    tower.visual(Box((0.16, 1.18, 0.14)), origin=Origin(xyz=(0.0, 0.0, 4.47)), material=weathered_wood, name="ridge_timber")
    tower.visual(
        Box((0.24, 0.20, 0.24)),
        origin=Origin(xyz=(0.0, -0.54, 4.25)),
        material=weathered_wood,
        name="bearing_block_0",
    )
    tower.visual(
        Box((0.24, 0.20, 0.24)),
        origin=Origin(xyz=(0.0, 0.54, 4.25)),
        material=weathered_wood,
        name="bearing_block_1",
    )

    # Steep tarred-shingle roof with a central ridge.
    roof_pitch = math.atan2(0.80, 0.95)
    roof_slope = math.hypot(0.95, 0.80)
    for sign, side in ((-1, "left"), (1, "right")):
        theta = sign * roof_pitch
        tower.visual(
            Box((roof_slope, 1.82, 0.08)),
            origin=Origin(xyz=(sign * 0.475, 0.0, 4.98), rpy=(0.0, theta, 0.0)),
            material=shingle_mat,
            name=f"roof_plane_{side}",
        )
        # Raised shingle courses, proud of the roof plane.
        normal_x = sign * math.sin(roof_pitch)
        normal_z = math.cos(roof_pitch)
        for row in range(7):
            t = 0.16 + row * 0.115
            tower.visual(
                Box((0.055, 1.88, 0.030)),
                origin=Origin(
                    xyz=(
                        sign * 0.95 * t + normal_x * 0.035,
                        0.0,
                        5.38 - 0.80 * t + normal_z * 0.035,
                    ),
                    rpy=(0.0, theta, 0.0),
                ),
                material=shingle_mat,
                name=f"shingle_{side}_{row}",
            )
        # Thick barge boards along the front and rear roof edges.
        for y, edge in ((-0.96, "front"), (0.96, "rear")):
            tower.visual(
                Box((roof_slope, 0.08, 0.11)),
                origin=Origin(xyz=(sign * 0.475, y, 4.96), rpy=(0.0, theta, 0.0)),
                material=weathered_wood,
                name=f"barge_{side}_{edge}",
            )

    tower.visual(Box((0.16, 1.92, 0.16)), origin=Origin(xyz=(0.0, 0.0, 5.40)), material=weathered_wood, name="roof_ridge_cap")
    tower.visual(Box((0.12, 0.12, 0.82)), origin=Origin(xyz=(0.0, 0.0, 4.93)), material=weathered_wood, name="king_post")
    tower.visual(Cylinder(radius=0.035, length=0.65), origin=Origin(xyz=(0.0, 0.0, 5.80)), material=weathered_wood, name="ridge_finial")
    tower.visual(Box((0.38, 0.055, 0.055)), origin=Origin(xyz=(0.0, 0.0, 5.92)), material=weathered_wood, name="finial_crossbar")

    yoke_bell = model.part("yoke_bell")

    # The timber A-frame yoke and its iron axle are authored around the swing axis.
    yoke_bell.visual(
        Cylinder(radius=0.055, length=1.28),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_iron,
        name="axle",
    )
    for iy, y in enumerate((-0.24, 0.24)):
        _beam_between_xz(yoke_bell, (0.0, y, 0.0), (-0.34, y, -0.42), 0.095, 0.085, name=f"yoke_leg_{iy}_0", material=weathered_wood)
        _beam_between_xz(yoke_bell, (0.0, y, 0.0), (0.34, y, -0.42), 0.095, 0.085, name=f"yoke_leg_{iy}_1", material=weathered_wood)
        yoke_bell.visual(Box((0.82, 0.095, 0.10)), origin=Origin(xyz=(0.0, y, -0.42)), material=weathered_wood, name=f"yoke_tie_{iy}")
    yoke_bell.visual(Box((0.10, 0.58, 0.10)), origin=Origin(xyz=(-0.34, 0.0, -0.42)), material=weathered_wood, name="yoke_spacer_0")
    yoke_bell.visual(Box((0.10, 0.58, 0.10)), origin=Origin(xyz=(0.34, 0.0, -0.42)), material=weathered_wood, name="yoke_spacer_1")
    yoke_bell.visual(Box((0.50, 0.62, 0.12)), origin=Origin(xyz=(0.0, 0.0, -0.42)), material=weathered_wood, name="headstock")

    # Hollow cast-iron bell: flared skirt, thick rim, narrow crown, and suspension ears.
    outer_profile = [
        (0.135, 0.02),
        (0.170, -0.035),
        (0.235, -0.145),
        (0.245, -0.300),
        (0.285, -0.520),
        (0.360, -0.735),
        (0.392, -0.790),
        (0.365, -0.845),
    ]
    inner_profile = [
        (0.080, -0.030),
        (0.125, -0.110),
        (0.170, -0.280),
        (0.225, -0.520),
        (0.315, -0.755),
        (0.338, -0.830),
    ]
    bell_shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    yoke_bell.visual(
        mesh_from_geometry(bell_shell, "bell_shell"),
        origin=Origin(xyz=(0.0, 0.0, -0.58)),
        material=iron_mat,
        name="bell_shell",
    )
    yoke_bell.visual(Cylinder(radius=0.13, length=0.17), origin=Origin(xyz=(0.0, 0.0, -0.50)), material=iron_mat, name="bell_crown")
    yoke_bell.visual(Box((0.075, 0.13, 0.23)), origin=Origin(xyz=(-0.11, 0.0, -0.43)), material=iron_mat, name="hanger_ear_0")
    yoke_bell.visual(Box((0.075, 0.13, 0.23)), origin=Origin(xyz=(0.11, 0.0, -0.43)), material=iron_mat, name="hanger_ear_1")
    yoke_bell.visual(
        Cylinder(radius=0.028, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, -0.39), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_iron,
        name="hanger_pin",
    )
    yoke_bell.visual(
        Cylinder(radius=0.040, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, -0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_iron,
        name="clapper_pin",
    )
    yoke_bell.visual(Box((0.06, 0.05, 0.20)), origin=Origin(xyz=(0.0, -0.13, -0.635)), material=worn_iron, name="clapper_hanger_0")
    yoke_bell.visual(Box((0.06, 0.05, 0.20)), origin=Origin(xyz=(0.0, 0.13, -0.635)), material=worn_iron, name="clapper_hanger_1")
    yoke_bell.visual(mesh_from_geometry(TorusGeometry(0.385, 0.020, radial_segments=16, tubular_segments=48), "bell_rim_band"), origin=Origin(xyz=(0.0, 0.0, -1.385)), material=worn_iron, name="rim_band")
    yoke_bell.visual(mesh_from_geometry(TorusGeometry(0.245, 0.010, radial_segments=12, tubular_segments=48), "bell_waist_band"), origin=Origin(xyz=(0.0, 0.0, -0.88)), material=worn_iron, name="waist_band")

    clapper = model.part("clapper")
    clapper.visual(
        Cylinder(radius=0.050, length=0.16),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_iron,
        name="pivot_eye",
    )
    clapper.visual(Cylinder(radius=0.018, length=0.55), origin=Origin(xyz=(0.0, 0.0, -0.325)), material=worn_iron, name="rod")
    clapper.visual(Sphere(radius=0.082), origin=Origin(xyz=(0.0, 0.0, -0.64)), material=iron_mat, name="clapper_ball")

    model.articulation(
        "tower_to_yoke",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yoke_bell,
        origin=Origin(xyz=(0.0, 0.0, 4.24)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-0.55, upper=0.55),
    )

    model.articulation(
        "bell_to_clapper",
        ArticulationType.REVOLUTE,
        parent=yoke_bell,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    yoke_bell = object_model.get_part("yoke_bell")
    clapper = object_model.get_part("clapper")
    swing = object_model.get_articulation("tower_to_yoke")
    clapper_pin = object_model.get_articulation("bell_to_clapper")

    # Captured iron axle in timber bearing blocks, not an accidental collision.
    for bearing in ("bearing_block_0", "bearing_block_1"):
        ctx.allow_overlap(
            tower,
            yoke_bell,
            elem_a=bearing,
            elem_b="axle",
            reason="The yoke axle is intentionally captured in the timber bearing block.",
        )
        ctx.expect_overlap(
            tower,
            yoke_bell,
            axes="yz",
            elem_a=bearing,
            elem_b="axle",
            min_overlap=0.04,
            name=f"{bearing} captures axle",
        )

    ctx.allow_overlap(
        yoke_bell,
        clapper,
        elem_a="clapper_pin",
        elem_b="pivot_eye",
        reason="The clapper eye is intentionally bored around the secondary revolute pin.",
    )
    ctx.expect_overlap(
        yoke_bell,
        clapper,
        axes="yz",
        elem_a="clapper_pin",
        elem_b="pivot_eye",
        min_overlap=0.05,
        name="clapper eye is retained on pin",
    )

    with ctx.pose({swing: 0.0, clapper_pin: 0.0}):
        ctx.expect_within(
            clapper,
            yoke_bell,
            axes="xy",
            inner_elem="clapper_ball",
            outer_elem="bell_shell",
            margin=0.01,
            name="clapper hangs inside bell mouth",
        )
        ctx.expect_gap(
            yoke_bell,
            tower,
            axis="z",
            positive_elem="axle",
            negative_elem="belfry_front_lower_rail",
            min_gap=0.75,
            name="bell axle is high in the open belfry",
        )

    rest_pos = ctx.part_world_position(yoke_bell)
    with ctx.pose({swing: 0.45}):
        swung_pos = ctx.part_world_position(yoke_bell)
        ctx.expect_overlap(
            yoke_bell,
            tower,
            axes="y",
            elem_a="axle",
            elem_b="bearing_block_0",
            min_overlap=0.06,
            name="swinging axle stays in bearing",
        )

    with ctx.pose({clapper_pin: 0.55}):
        ctx.expect_overlap(
            clapper,
            yoke_bell,
            axes="y",
            elem_a="pivot_eye",
            elem_b="clapper_pin",
            min_overlap=0.08,
            name="clapper remains on its horizontal pin",
        )

    ctx.check(
        "yoke has a horizontal swing axis",
        tuple(round(v, 3) for v in swing.axis) == (0.0, 1.0, 0.0),
        details=f"axis={swing.axis}",
    )
    ctx.check(
        "yoke frame origin remains at bearing line",
        rest_pos is not None and swung_pos is not None and abs(rest_pos[2] - swung_pos[2]) < 0.001,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
