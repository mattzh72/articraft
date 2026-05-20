from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _toothed_sprocket_geometry() -> MeshGeometry:
    teeth = 25
    root_radius = 0.054
    tip_radius = 0.061
    outer: list[tuple[float, float]] = []
    for tooth in range(teeth):
        base = 2.0 * math.pi * tooth / teeth
        pitch = 2.0 * math.pi / teeth
        for fraction, radius in (
            (0.04, root_radius),
            (0.27, tip_radius),
            (0.73, tip_radius),
            (0.96, root_radius),
        ):
            angle = base + fraction * pitch
            outer.append((radius * math.cos(angle), radius * math.sin(angle)))

    # The large center bore clears the bottom bracket spindle; the five round
    # windows make the plate read as a light BMX sprocket instead of a solid disc.
    holes = [_circle_profile(0.014, segments=56)]
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0 + 0.18
        cy = 0.031 * math.sin(angle)
        cz = 0.031 * math.cos(angle)
        holes.append([(cy + y, cz + z) for y, z in _circle_profile(0.0085, segments=28)])

    sprocket = ExtrudeWithHolesGeometry(outer, holes, 0.005, cap=True, center=True)
    sprocket.rotate_y(math.pi / 2.0)
    return sprocket


def _hollow_cylinder_geometry(
    *, outer_radius: float, inner_radius: float, length: float, segments: int = 64
) -> MeshGeometry:
    tube = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        length,
        cap=True,
        center=True,
    )
    tube.rotate_y(math.pi / 2.0)
    return tube


def _crank_arm_geometry(
    *,
    center_x: float,
    start_z: float,
    end_z: float,
    start_half_x: float = 0.010,
    end_half_x: float = 0.007,
    start_half_y: float = 0.014,
    end_half_y: float = 0.011,
) -> MeshGeometry:
    geom = MeshGeometry()
    stations = (
        (center_x, start_z, start_half_x, start_half_y),
        (center_x, end_z, end_half_x, end_half_y),
    )
    ids: list[list[int]] = []
    for x, z, hx, hy in stations:
        ids.append(
            [
                geom.add_vertex(x - hx, -hy, z),
                geom.add_vertex(x + hx, -hy, z),
                geom.add_vertex(x + hx, hy, z),
                geom.add_vertex(x - hx, hy, z),
            ]
        )
    _add_quad(geom, ids[0][0], ids[0][1], ids[0][2], ids[0][3])
    _add_quad(geom, ids[1][0], ids[1][3], ids[1][2], ids[1][1])
    for index in range(4):
        _add_quad(
            geom,
            ids[0][index],
            ids[0][(index + 1) % 4],
            ids[1][(index + 1) % 4],
            ids[1][index],
        )
    return geom


def _x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _add_platform_pedal(part, *, material, rail_material, axle_material) -> None:
    # The platform is intentionally concave: raised perimeter rails and traction
    # pins sit above a recessed center web, like a BMX/flatland pedal.
    part.visual(
        Cylinder(radius=0.0055, length=0.072),
        origin=_x_axis_origin((0.036, 0.0, 0.0)),
        material=axle_material,
        name="axle",
    )
    part.visual(
        Box((0.078, 0.060, 0.008)),
        origin=Origin(xyz=(0.046, 0.0, -0.006)),
        material=material,
        name="center_web",
    )
    part.visual(
        Box((0.086, 0.012, 0.020)),
        origin=Origin(xyz=(0.046, 0.043, 0.0)),
        material=rail_material,
        name="front_rail",
    )
    part.visual(
        Box((0.086, 0.012, 0.020)),
        origin=Origin(xyz=(0.046, -0.043, 0.0)),
        material=rail_material,
        name="rear_rail",
    )
    part.visual(
        Box((0.012, 0.092, 0.020)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=rail_material,
        name="inner_rail",
    )
    part.visual(
        Box((0.012, 0.092, 0.020)),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material=rail_material,
        name="outer_rail",
    )
    for row, y in enumerate((-0.043, 0.043)):
        for column, x in enumerate((0.018, 0.046, 0.074)):
            part.visual(
                Cylinder(radius=0.0022, length=0.006),
                origin=Origin(xyz=(x, y, 0.013)),
                material=axle_material,
                name=f"pin_{row}_{column}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_crankset")

    black = model.material("anodized_black", rgba=(0.02, 0.022, 0.024, 1.0))
    satin_black = model.material("satin_black", rgba=(0.075, 0.080, 0.085, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.82, 0.80, 0.75, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(outer_radius=0.032, inner_radius=0.0145, length=0.074),
            "bottom_bracket_shell",
        ),
        material=dark_graphite,
        name="bb_shell",
    )
    for x, name in ((-0.040, "left_bearing_cup"), (0.040, "right_bearing_cup")):
        bottom_bracket.visual(
            mesh_from_geometry(
                _hollow_cylinder_geometry(
                    outer_radius=0.037, inner_radius=0.0145, length=0.006, segments=56
                ),
                f"{name}_mesh",
            ),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=steel,
            name=name,
        )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0105, length=0.180),
        origin=_x_axis_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="spindle",
    )
    crank.visual(
        Cylinder(radius=0.027, length=0.025),
        origin=_x_axis_origin((0.080, 0.0, 0.0)),
        material=black,
        name="right_hub_boss",
    )
    crank.visual(
        Cylinder(radius=0.027, length=0.025),
        origin=_x_axis_origin((-0.080, 0.0, 0.0)),
        material=black,
        name="left_hub_boss",
    )
    crank.visual(
        mesh_from_geometry(
            _crank_arm_geometry(center_x=0.078, start_z=-0.018, end_z=-0.176),
            "right_crank_arm",
        ),
        material=black,
        name="right_arm",
    )
    crank.visual(
        mesh_from_geometry(
            _crank_arm_geometry(center_x=-0.078, start_z=0.018, end_z=0.176),
            "left_crank_arm",
        ),
        material=black,
        name="left_arm",
    )
    crank.visual(
        Box((0.004, 0.020, 0.118)),
        origin=Origin(xyz=(0.089, 0.0125, -0.095)),
        material=satin_black,
        name="right_arm_recess",
    )
    crank.visual(
        Box((0.004, 0.020, 0.118)),
        origin=Origin(xyz=(-0.089, -0.0125, 0.095)),
        material=satin_black,
        name="left_arm_recess",
    )
    crank.visual(
        Cylinder(radius=0.018, length=0.025),
        origin=_x_axis_origin((0.080, 0.0, -0.176)),
        material=black,
        name="right_pedal_boss",
    )
    crank.visual(
        Cylinder(radius=0.018, length=0.025),
        origin=_x_axis_origin((-0.080, 0.0, 0.176)),
        material=black,
        name="left_pedal_boss",
    )

    crank.visual(
        mesh_from_geometry(_toothed_sprocket_geometry(), "bmx_sprocket_25t"),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=satin_black,
        name="sprocket",
    )
    crank.visual(
        Box((0.018, 0.019, 0.060)),
        origin=Origin(xyz=(0.071, 0.0, -0.032)),
        material=black,
        name="sprocket_tab",
    )
    crank.visual(
        Cylinder(radius=0.0050, length=0.022),
        origin=_x_axis_origin((0.064, 0.0, -0.036)),
        material=bolt_steel,
        name="bolt_spacer",
    )
    crank.visual(
        Cylinder(radius=0.0080, length=0.004),
        origin=_x_axis_origin((0.076, 0.0, -0.036)),
        material=bolt_steel,
        name="sprocket_bolt",
    )
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0 + 0.30
        y = 0.043 * math.sin(angle)
        z = 0.043 * math.cos(angle)
        crank.visual(
            Cylinder(radius=0.0038, length=0.004),
            origin=_x_axis_origin((0.051, y, z)),
            material=bolt_steel,
            name=f"chainring_bolt_{index}",
        )

    right_pedal = model.part("right_pedal")
    _add_platform_pedal(
        right_pedal,
        material=dark_graphite,
        rail_material=black,
        axle_material=bolt_steel,
    )

    left_pedal = model.part("left_pedal")
    _add_platform_pedal(
        left_pedal,
        material=dark_graphite,
        rail_material=black,
        axle_material=bolt_steel,
    )

    model.articulation(
        "bottom_bracket_spin",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=crank,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=12.0),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=right_pedal,
        origin=Origin(xyz=(0.0925, 0.0, -0.176)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=left_pedal,
        origin=Origin(xyz=(-0.0925, 0.0, 0.176), rpy=(0.0, 0.0, math.pi)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    bottom_bracket = object_model.get_part("bottom_bracket")
    crank = object_model.get_part("crank")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")
    bb_joint = object_model.get_articulation("bottom_bracket_spin")
    right_pedal_joint = object_model.get_articulation("right_pedal_spin")
    left_pedal_joint = object_model.get_articulation("left_pedal_spin")

    ctx.allow_overlap(
        bottom_bracket,
        crank,
        elem_a="bb_shell",
        elem_b="spindle",
        reason="The bearing shell visually represents a hollow sleeve around the spindle; the collision proxy conservatively treats the bore as filled.",
    )
    for cup_name in ("left_bearing_cup", "right_bearing_cup"):
        ctx.allow_overlap(
            bottom_bracket,
            crank,
            elem_a=cup_name,
            elem_b="spindle",
            reason="The bearing cup is an annular sleeve around the spindle; its simplified collision proxy fills the bore.",
        )
    ctx.expect_within(
        crank,
        bottom_bracket,
        axes="yz",
        inner_elem="spindle",
        outer_elem="bb_shell",
        margin=0.002,
        name="spindle is centered in bottom bracket shell",
    )
    ctx.expect_overlap(
        crank,
        bottom_bracket,
        axes="x",
        elem_a="spindle",
        elem_b="bb_shell",
        min_overlap=0.070,
        name="spindle passes through bottom bracket shell",
    )
    for cup_name in ("left_bearing_cup", "right_bearing_cup"):
        ctx.expect_overlap(
            crank,
            bottom_bracket,
            axes="x",
            elem_a="spindle",
            elem_b=cup_name,
            min_overlap=0.005,
            name=f"spindle passes through {cup_name}",
        )

    ctx.check(
        "bottom bracket is a continuous revolute joint",
        bb_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(bb_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={bb_joint.articulation_type}, axis={bb_joint.axis}",
    )
    ctx.check(
        "pedals use axle spin joints",
        right_pedal_joint.articulation_type == ArticulationType.CONTINUOUS
        and left_pedal_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(right_pedal_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(left_pedal_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"right={right_pedal_joint.articulation_type}/{right_pedal_joint.axis}, "
            f"left={left_pedal_joint.articulation_type}/{left_pedal_joint.axis}"
        ),
    )
    ctx.expect_gap(
        right_pedal,
        crank,
        axis="x",
        positive_elem="axle",
        negative_elem="right_pedal_boss",
        max_gap=0.002,
        max_penetration=0.0,
        name="right pedal axle seats against crank boss",
    )
    ctx.expect_gap(
        crank,
        left_pedal,
        axis="x",
        positive_elem="left_pedal_boss",
        negative_elem="axle",
        max_gap=0.002,
        max_penetration=0.0,
        name="left pedal axle seats against crank boss",
    )

    crank_visuals = {visual.name for visual in crank.visuals}
    ctx.check(
        "three-piece crank and bolted sprocket details are present",
        {
            "spindle",
            "right_arm",
            "left_arm",
            "sprocket",
            "sprocket_tab",
            "sprocket_bolt",
        }.issubset(crank_visuals),
        details=f"crank visuals={sorted(crank_visuals)}",
    )
    for pedal in (right_pedal, left_pedal):
        names = {visual.name for visual in pedal.visuals}
        ctx.check(
            f"{pedal.name} has a concave platform body",
            {"center_web", "front_rail", "rear_rail", "inner_rail", "outer_rail"}.issubset(names),
            details=f"pedal visuals={sorted(names)}",
        )

    rest_pos = ctx.part_world_position(right_pedal)
    with ctx.pose({bb_joint: math.pi / 2.0}):
        quarter_turn_pos = ctx.part_world_position(right_pedal)
    ctx.check(
        "bottom bracket rotation carries the crank arm through a quarter turn",
        rest_pos is not None
        and quarter_turn_pos is not None
        and rest_pos[2] < -0.15
        and quarter_turn_pos[1] > 0.15
        and abs(quarter_turn_pos[2]) < 0.020,
        details=f"rest={rest_pos}, quarter_turn={quarter_turn_pos}",
    )

    return ctx.report()


object_model = build_object_model()
