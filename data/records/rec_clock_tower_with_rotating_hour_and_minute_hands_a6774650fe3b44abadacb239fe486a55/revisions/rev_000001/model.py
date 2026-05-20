from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


SHAFT_HALF = 2.0
CLOCK_Z = 11.25
CLOCK_RADIUS = 0.78
CLOCK_FACE_THICKNESS = 0.08


def _regular_octagon(radius: float, z: float | None = None):
    """CCW octagon with its first point near the upper-right facet."""
    points = []
    for i in range(8):
        angle = math.radians(67.5 + 45.0 * i)
        if z is None:
            points.append((radius * math.cos(angle), radius * math.sin(angle)))
        else:
            points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return points


def _chamfered_square(width: float, chamfer: float, z: float):
    h = width * 0.5
    c = chamfer
    return [
        (h - c, h, z),
        (-(h - c), h, z),
        (-h, h - c, z),
        (-h, -(h - c), z),
        (-(h - c), -h, z),
        (h - c, -h, z),
        (h, -(h - c), z),
        (h, h - c, z),
    ]


def _axis_rpy(axis: str, sign: float) -> tuple[float, float, float]:
    """Rotate a local-Z cylinder/torus axis onto a world face normal."""
    if axis == "x":
        return (0.0, sign * math.pi / 2.0, 0.0)
    if axis == "y":
        return (-sign * math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _normal_xyz(axis: str, sign: float, offset: float) -> tuple[float, float, float]:
    if axis == "x":
        return (sign * offset, 0.0, 0.0)
    return (0.0, sign * offset, 0.0)


def _face_origin(axis: str, sign: float, face_offset: float, z: float) -> tuple[float, float, float]:
    if axis == "x":
        return (sign * face_offset, 0.0, z)
    return (0.0, sign * face_offset, z)


def _hand_stem_origin(
    axis: str,
    sign: float,
    layer: float,
    length: float,
    angle: float,
) -> Origin:
    # The black pointer begins just outside the golden pivot hub so the fixed
    # clock spindle can pass through the hub without intersecting the stem.
    radial_start = 0.050
    half = length * 0.5
    center_distance = radial_start + half
    if axis == "x":
        # A local Z hand rotated about X remains on the YZ clock plane.
        xyz = (sign * layer, -math.sin(angle) * center_distance, math.cos(angle) * center_distance)
        rpy = (angle, 0.0, 0.0)
    else:
        # A local Z hand rotated about Y remains on the XZ clock plane.
        xyz = (math.sin(angle) * center_distance, sign * layer, math.cos(angle) * center_distance)
        rpy = (0.0, angle, 0.0)
    return Origin(xyz=xyz, rpy=rpy)


def _add_clock_marks(
    tower,
    *,
    axis: str,
    sign: float,
    face_offset: float,
    material: Material,
    face_index: int,
) -> None:
    mark_depth = 0.018
    major_size = (mark_depth, 0.035, 0.135)
    minor_size = (mark_depth, 0.022, 0.085)
    if axis == "y":
        major_size = (0.035, mark_depth, 0.135)
        minor_size = (0.022, mark_depth, 0.085)

    for tick in range(12):
        theta = 2.0 * math.pi * tick / 12.0
        radius = CLOCK_RADIUS * 0.76
        if axis == "x":
            xyz = (
                sign * face_offset,
                math.sin(theta) * radius,
                CLOCK_Z + math.cos(theta) * radius,
            )
            rpy = (-theta, 0.0, 0.0)
        else:
            xyz = (
                math.sin(theta) * radius,
                sign * face_offset,
                CLOCK_Z + math.cos(theta) * radius,
            )
            rpy = (0.0, theta, 0.0)
        tower.visual(
            Box(major_size if tick % 3 == 0 else minor_size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=f"clock_mark_{face_index}_{tick}",
        )


def _add_hand_part(
    model: ArticulatedObject,
    tower,
    *,
    name: str,
    joint_name: str,
    axis: str,
    sign: float,
    pivot_offset: float,
    length: float,
    width: float,
    layer: float,
    start_angle: float,
    material: Material,
    hub_material: Material,
):
    part = model.part(name)
    depth = 0.020
    stem_size = (depth, width, length) if axis == "x" else (width, depth, length)
    part.visual(
        Box(stem_size),
        origin=_hand_stem_origin(axis, sign, layer, length, start_angle),
        material=material,
        name="stem",
    )
    part.visual(
        Cylinder(radius=0.070 if length < 0.55 else 0.055, length=depth * 1.3),
        origin=Origin(xyz=_normal_xyz(axis, sign, layer), rpy=_axis_rpy(axis, sign)),
        material=hub_material,
        name="hub",
    )
    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=tower,
        child=part,
        origin=Origin(xyz=_face_origin(axis, sign, pivot_offset, CLOCK_Z)),
        axis=(sign, 0.0, 0.0) if axis == "x" else (0.0, sign, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.8 if length < 0.55 else 3.0,
            lower=0.0,
            upper=2.0 * math.pi,
        ),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="baroque_church_tower")

    limestone = Material("warm_limestone", rgba=(0.74, 0.66, 0.52, 1.0))
    pale_stone = Material("pale_cut_stone", rgba=(0.86, 0.81, 0.68, 1.0))
    shadow = Material("deep_arch_shadow", rgba=(0.06, 0.055, 0.050, 1.0))
    oxidized_copper = Material("oxidized_copper", rgba=(0.18, 0.50, 0.43, 1.0))
    dark_roof = Material("dark_roof_shadow", rgba=(0.08, 0.12, 0.11, 1.0))
    clock_white = Material("aged_clock_enamel", rgba=(0.94, 0.91, 0.80, 1.0))
    clock_black = Material("black_clock_paint", rgba=(0.01, 0.01, 0.01, 1.0))
    gilded = Material("dulled_gilding", rgba=(0.92, 0.68, 0.20, 1.0))

    tower = model.part("tower")

    # Square masonry shaft and stepped Baroque base.
    tower.visual(
        Box((4.0, 4.0, 13.0)),
        origin=Origin(xyz=(0.0, 0.0, 6.70)),
        material=limestone,
        name="square_masonry_shaft",
    )
    tower.visual(
        Box((4.7, 4.7, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=pale_stone,
        name="lower_plinth",
    )
    tower.visual(
        Box((4.35, 4.35, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=pale_stone,
        name="upper_plinth",
    )
    tower.visual(
        Box((4.28, 4.28, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 13.18)),
        material=pale_stone,
        name="shaft_top_cornice",
    )
    tower.visual(
        Box((4.55, 4.55, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 13.34)),
        material=pale_stone,
        name="overhanging_cornice",
    )

    # Alternating quoins read as cut masonry without turning the shaft into a plain block.
    for level, z in enumerate([1.15 + 0.85 * i for i in range(14)]):
        block_h = 0.34
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                offset = 0.09 if level % 2 == 0 else -0.09
                tower.visual(
                    Box((0.46, 0.22, block_h)),
                    origin=Origin(xyz=(sx * (SHAFT_HALF + 0.015), sy * (SHAFT_HALF - 0.18 + offset), z)),
                    material=pale_stone,
                    name=f"quoin_x_{level}_{int(sx > 0)}_{int(sy > 0)}",
                )
                tower.visual(
                    Box((0.22, 0.46, block_h)),
                    origin=Origin(xyz=(sx * (SHAFT_HALF - 0.18 - offset), sy * (SHAFT_HALF + 0.015), z)),
                    material=pale_stone,
                    name=f"quoin_y_{level}_{int(sx > 0)}_{int(sy > 0)}",
                )

    # Bell-chamber shadow panels below the clocks on all four faces.
    for face_index, (axis, sign) in enumerate((("x", 1.0), ("x", -1.0), ("y", 1.0), ("y", -1.0))):
        if axis == "x":
            tower.visual(
                Box((0.035, 0.72, 1.28)),
                origin=Origin(xyz=(sign * (SHAFT_HALF + 0.005), 0.0, 8.55)),
                material=shadow,
                name=f"arched_louver_{face_index}",
            )
            for bar in (-0.20, 0.0, 0.20):
                tower.visual(
                    Box((0.055, 0.045, 1.15)),
                    origin=Origin(xyz=(sign * (SHAFT_HALF + 0.020), bar, 8.55)),
                    material=pale_stone,
                    name=f"louver_mullion_{face_index}_{bar}",
                )
        else:
            tower.visual(
                Box((0.72, 0.035, 1.28)),
                origin=Origin(xyz=(0.0, sign * (SHAFT_HALF + 0.005), 8.55)),
                material=shadow,
                name=f"arched_louver_{face_index}",
            )
            for bar in (-0.20, 0.0, 0.20):
                tower.visual(
                    Box((0.045, 0.055, 1.15)),
                    origin=Origin(xyz=(bar, sign * (SHAFT_HALF + 0.020), 8.55)),
                    material=pale_stone,
                    name=f"louver_mullion_{face_index}_{bar}",
                )

    # Chamfered square-to-octagon stage and the octagonal drum.
    tower.visual(
        mesh_from_geometry(
            LoftGeometry(
                [
                    _chamfered_square(4.45, 0.46, 13.38),
                    _regular_octagon(1.88, 14.05),
                ],
                cap=True,
                closed=True,
            ),
            "chamfered_octagon_base",
        ),
        material=pale_stone,
        name="chamfered_octagon_base",
    )
    tower.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(_regular_octagon(1.74), 2.28, cap=True, closed=True),
            "octagonal_drum",
        ),
        origin=Origin(xyz=(0.0, 0.0, 13.92)),
        material=limestone,
        name="octagonal_drum",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=1.60, tube=0.08, radial_segments=16, tubular_segments=64), "drum_collar"),
        origin=Origin(xyz=(0.0, 0.0, 16.24)),
        material=pale_stone,
        name="drum_collar",
    )

    # Revolved onion dome with bulbous shoulder, pinched neck, and copper-green skin.
    onion_profile = [
        (0.0, 0.00),
        (1.58, 0.00),
        (1.72, 0.24),
        (1.78, 0.76),
        (1.62, 1.22),
        (1.10, 1.85),
        (0.53, 2.28),
        (0.28, 2.56),
        (0.0, 2.62),
    ]
    tower.visual(
        mesh_from_geometry(LatheGeometry(onion_profile, segments=72, closed=True), "onion_dome"),
        origin=Origin(xyz=(0.0, 0.0, 16.20)),
        material=oxidized_copper,
        name="onion_dome",
    )
    tower.visual(
        Cylinder(radius=0.22, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 18.88)),
        material=dark_roof,
        name="dome_neck",
    )
    tower.visual(
        Cylinder(radius=0.055, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 19.34)),
        material=gilded,
        name="finial_spire",
    )
    tower.visual(
        Box((0.70, 0.065, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 19.55)),
        material=gilded,
        name="cross_bar",
    )
    tower.visual(
        Box((0.075, 0.075, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 19.56)),
        material=gilded,
        name="cross_upright",
    )

    # Four clock faces with separate hour and minute hand parts.
    face_center = SHAFT_HALF + 0.040
    face_surface = SHAFT_HALF + CLOCK_FACE_THICKNESS + 0.004
    hand_pivot = SHAFT_HALF + CLOCK_FACE_THICKNESS + 0.028
    for face_index, (axis, sign) in enumerate((("x", 1.0), ("x", -1.0), ("y", 1.0), ("y", -1.0))):
        tower.visual(
            Cylinder(radius=CLOCK_RADIUS, length=CLOCK_FACE_THICKNESS),
            origin=Origin(xyz=_face_origin(axis, sign, face_center, CLOCK_Z), rpy=_axis_rpy(axis, sign)),
            material=clock_white,
            name=f"clock_face_{face_index}",
        )
        tower.visual(
            mesh_from_geometry(
                TorusGeometry(radius=CLOCK_RADIUS + 0.025, tube=0.035, radial_segments=12, tubular_segments=64),
                f"clock_rim_{face_index}",
            ),
            origin=Origin(xyz=_face_origin(axis, sign, face_surface, CLOCK_Z), rpy=_axis_rpy(axis, sign)),
            material=gilded,
            name=f"clock_rim_{face_index}",
        )
        tower.visual(
            Cylinder(radius=0.032, length=0.18),
            origin=Origin(xyz=_face_origin(axis, sign, hand_pivot + 0.020, CLOCK_Z), rpy=_axis_rpy(axis, sign)),
            material=gilded,
            name=f"hand_spindle_{face_index}",
        )
        _add_clock_marks(
            tower,
            axis=axis,
            sign=sign,
            face_offset=SHAFT_HALF + CLOCK_FACE_THICKNESS + 0.005,
            material=clock_black,
            face_index=face_index,
        )
        _add_hand_part(
            model,
            tower,
            name=f"hour_hand_{face_index}",
            joint_name=f"tower_to_hour_{face_index}",
            axis=axis,
            sign=sign,
            pivot_offset=hand_pivot,
            length=0.46,
            width=0.075,
            layer=0.018,
            start_angle=0.65,
            material=clock_black,
            hub_material=gilded,
        )
        _add_hand_part(
            model,
            tower,
            name=f"minute_hand_{face_index}",
            joint_name=f"tower_to_minute_{face_index}",
            axis=axis,
            sign=sign,
            pivot_offset=hand_pivot,
            length=0.66,
            width=0.043,
            layer=0.052,
            start_angle=-0.78,
            material=clock_black,
            hub_material=gilded,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    ctx.check(
        "four clock faces are modeled on the shaft",
        all(tower.get_visual(f"clock_face_{i}") is not None for i in range(4)),
        details="Expected one round clock face on each of the four upper shaft sides.",
    )

    for i in range(4):
        hour = object_model.get_part(f"hour_hand_{i}")
        minute = object_model.get_part(f"minute_hand_{i}")
        hour_joint = object_model.get_articulation(f"tower_to_hour_{i}")
        minute_joint = object_model.get_articulation(f"tower_to_minute_{i}")
        spindle = f"hand_spindle_{i}"
        ctx.allow_overlap(
            tower,
            hour,
            elem_a=spindle,
            elem_b="hub",
            reason="The fixed clock spindle intentionally passes through the hour-hand hub.",
        )
        ctx.allow_overlap(
            tower,
            minute,
            elem_a=spindle,
            elem_b="hub",
            reason="The fixed clock spindle intentionally passes through the minute-hand hub.",
        )
        ctx.expect_overlap(
            hour,
            tower,
            axes="xyz",
            elem_a="hub",
            elem_b=spindle,
            min_overlap=0.010,
            name=f"clock {i} hour hub is captured by spindle",
        )
        ctx.expect_overlap(
            minute,
            tower,
            axes="xyz",
            elem_a="hub",
            elem_b=spindle,
            min_overlap=0.010,
            name=f"clock {i} minute hub is captured by spindle",
        )
        ctx.check(
            f"clock {i} hands use concentric pivots",
            hour_joint.origin.xyz == minute_joint.origin.xyz,
            details=f"hour={hour_joint.origin.xyz}, minute={minute_joint.origin.xyz}",
        )
        ctx.expect_origin_distance(
            hour,
            minute,
            axes="xyz",
            max_dist=0.001,
            name=f"clock {i} hand part origins are concentric",
        )

    # A decisive pose check proves that a hand sweeps around its hub instead of
    # being a static clock graphic.
    hour0 = object_model.get_part("hour_hand_0")
    hour0_joint = object_model.get_articulation("tower_to_hour_0")
    rest_aabb = ctx.part_element_world_aabb(hour0, elem="stem")
    with ctx.pose({hour0_joint: math.pi / 2.0 - 0.65}):
        posed_aabb = ctx.part_element_world_aabb(hour0, elem="stem")
        if rest_aabb is None or posed_aabb is None:
            ctx.fail("posed hour hand has measurable stem", "No AABB returned for hour_hand_0/stem.")
        else:
            rest_center_y = 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])
            rest_center_z = 0.5 * (rest_aabb[0][2] + rest_aabb[1][2])
            posed_center_y = 0.5 * (posed_aabb[0][1] + posed_aabb[1][1])
            posed_center_z = 0.5 * (posed_aabb[0][2] + posed_aabb[1][2])
            ctx.check(
                "posed hour hand sweeps across the clock face",
                rest_center_z > CLOCK_Z + 0.16
                and abs(posed_center_z - CLOCK_Z) < 0.06
                and abs(posed_center_y - rest_center_y) > 0.09,
                details=(
                    f"rest_center=({rest_center_y:.3f},{rest_center_z:.3f}), "
                    f"posed_center=({posed_center_y:.3f},{posed_center_z:.3f})"
                ),
            )

    return ctx.report()


object_model = build_object_model()
