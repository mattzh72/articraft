from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rectangular_frustum(
    *,
    bottom_width: float,
    bottom_depth: float,
    top_width: float,
    top_depth: float,
    z_min: float,
    z_max: float,
) -> MeshGeometry:
    """Closed tapered metronome housing body, centered on the root frame."""
    geom = MeshGeometry()
    bw = bottom_width * 0.5
    bd = bottom_depth * 0.5
    tw = top_width * 0.5
    td = top_depth * 0.5
    vertices = [
        (-bw, -bd, z_min),
        (bw, -bd, z_min),
        (bw, bd, z_min),
        (-bw, bd, z_min),
        (-tw, -td, z_max),
        (tw, -td, z_max),
        (tw, td, z_max),
        (-tw, td, z_max),
    ]
    for vertex in vertices:
        geom.add_vertex(*vertex)
    for face in (
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ):
        geom.add_face(*face)
    return geom


def _tube_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 48,
) -> MeshGeometry:
    """A closed annular cylinder, used for the sliding metronome weight."""
    geom = MeshGeometry()
    z0 = -length * 0.5
    z1 = length * 0.5
    for z in (z0, z1):
        for radius in (outer_radius, inner_radius):
            for index in range(segments):
                angle = 2.0 * pi * index / segments
                geom.add_vertex(radius * cos(angle), radius * sin(angle), z)
    outer_bottom = 0
    inner_bottom = segments
    outer_top = 2 * segments
    inner_top = 3 * segments
    for index in range(segments):
        nxt = (index + 1) % segments
        geom.add_face(outer_bottom + index, outer_bottom + nxt, outer_top + nxt)
        geom.add_face(outer_bottom + index, outer_top + nxt, outer_top + index)
        geom.add_face(inner_bottom + index, inner_top + nxt, inner_bottom + nxt)
        geom.add_face(inner_bottom + index, inner_top + index, inner_top + nxt)
        geom.add_face(outer_top + index, outer_top + nxt, inner_top + nxt)
        geom.add_face(outer_top + index, inner_top + nxt, inner_top + index)
        geom.add_face(outer_bottom + index, inner_bottom + nxt, outer_bottom + nxt)
        geom.add_face(outer_bottom + index, inner_bottom + index, inner_bottom + nxt)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pyramid_metronome")

    walnut = model.material("polished_walnut", rgba=(0.42, 0.20, 0.08, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.22, 0.10, 0.04, 1.0))
    cream = model.material("aged_scale_plate", rgba=(0.92, 0.83, 0.62, 1.0))
    black = model.material("black_print", rgba=(0.02, 0.018, 0.014, 1.0))
    brass = model.material("brass", rgba=(0.86, 0.63, 0.20, 1.0))
    steel = model.material("blued_steel", rgba=(0.35, 0.36, 0.38, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.035, 0.032, 0.030, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.30, 0.22, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_walnut,
        name="base_plinth",
    )
    housing.visual(
        mesh_from_geometry(
            _rectangular_frustum(
                bottom_width=0.215,
                bottom_depth=0.150,
                top_width=0.055,
                top_depth=0.040,
                z_min=0.038,
                z_max=0.330,
            ),
            "tapered_housing",
        ),
        material=walnut,
        name="tapered_body",
    )
    housing.visual(
        Box((0.060, 0.006, 0.185)),
        origin=Origin(xyz=(0.0, -0.047, 0.190), rpy=(-0.19, 0.0, 0.0)),
        material=cream,
        name="scale_plate",
    )
    housing.visual(
        Box((0.010, 0.008, 0.165)),
        origin=Origin(xyz=(0.0, -0.049, 0.190), rpy=(-0.19, 0.0, 0.0)),
        material=black,
        name="pendulum_slot",
    )
    for index, z in enumerate((0.126, 0.140, 0.162, 0.184, 0.206, 0.228, 0.250)):
        tick_width = 0.045 if index % 2 == 0 else 0.032
        housing.visual(
            Box((tick_width, 0.016, 0.004)),
            origin=Origin(xyz=(0.0, -0.049, z), rpy=(-0.19, 0.0, 0.0)),
            material=black,
            name=f"tempo_tick_{index}",
        )

    # Front pendulum bearing and rear winding-key boss.
    housing.visual(
        Cylinder(radius=0.014, length=0.056),
        origin=Origin(xyz=(0.0, -0.046, 0.327), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_boss",
    )
    housing.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.0, 0.055, 0.185), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_boss",
    )

    # Two-piece parent hinge brackets at the rear corners.  The child hinge
    # barrels occupy the open central band between the upper and lower knuckles.
    for side, x in enumerate((-0.135, 0.135)):
        housing.visual(
            Box((0.024, 0.008, 0.040)),
            origin=Origin(xyz=(x, 0.1015, 0.020)),
            material=steel,
            name=f"rear_hinge_leaf_{side}",
        )
        for band, z in enumerate((0.0065, 0.0335)):
            housing.visual(
                Box((0.024, 0.015, 0.012)),
                origin=Origin(xyz=(x, 0.1085, z)),
                material=steel,
                name=f"hinge_bridge_{side}_{band}",
            )
            housing.visual(
                Cylinder(radius=0.0065, length=0.012),
                origin=Origin(xyz=(x, 0.112, z)),
                material=steel,
                name=f"hinge_knuckle_{side}_{band}",
            )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0030, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=steel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_hub",
    )
    pendulum.visual(
        Box((0.010, 0.004, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=steel,
        name="lower_pointer",
    )

    sliding_weight = model.part("sliding_weight")
    sliding_weight.visual(
        mesh_from_geometry(
            _tube_mesh(outer_radius=0.026, inner_radius=0.0055, length=0.062),
            "sliding_weight_sleeve",
        ),
        material=brass,
        name="weight_sleeve",
    )
    sliding_weight.visual(
        mesh_from_geometry(
            _tube_mesh(outer_radius=0.028, inner_radius=0.0055, length=0.012),
            "sliding_weight_band",
        ),
        material=brass,
        name="center_band",
    )
    sliding_weight.visual(
        Box((0.023, 0.006, 0.006)),
        origin=Origin(xyz=(0.0145, 0.0, 0.0)),
        material=steel,
        name="set_screw",
    )

    for leg_index, (x, direction, axis) in enumerate(((-0.135, 1.0, (0.0, 0.0, 1.0)), (0.135, -1.0, (0.0, 0.0, -1.0)))):
        leg = model.part(f"rear_leg_{leg_index}")
        leg.visual(
            Cylinder(radius=0.0060, length=0.014),
            origin=Origin(),
            material=steel,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.132, 0.024, 0.008)),
            origin=Origin(xyz=(direction * 0.066, 0.014, -0.003)),
            material=dark_walnut,
            name="flat_leg",
        )
        leg.visual(
            Box((0.044, 0.026, 0.004)),
            origin=Origin(xyz=(direction * 0.122, 0.014, -0.009)),
            material=rubber,
            name="rubber_foot",
        )
        model.articulation(
            f"leg_hinge_{leg_index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=leg,
            origin=Origin(xyz=(x, 0.112, 0.020)),
            axis=axis,
            motion_limits=MotionLimits(effort=1.5, velocity=1.2, lower=0.0, upper=1.10),
        )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="key_shaft",
    )
    winding_key.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_hub",
    )
    winding_key.visual(
        Box((0.074, 0.007, 0.024)),
        origin=Origin(xyz=(0.0, 0.052, 0.0)),
        material=brass,
        name="key_wings",
    )

    model.articulation(
        "pendulum_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, -0.074, 0.327)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.20, velocity=3.0, lower=-0.38, upper=0.38),
    )
    model.articulation(
        "weight_slide",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=sliding_weight,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.20, lower=0.0, upper=0.120),
    )
    model.articulation(
        "key_turn",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.0, 0.070, 0.185)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("sliding_weight")
    leg_0 = object_model.get_part("rear_leg_0")
    leg_1 = object_model.get_part("rear_leg_1")

    pendulum_pivot = object_model.get_articulation("pendulum_pivot")
    weight_slide = object_model.get_articulation("weight_slide")
    leg_hinge_0 = object_model.get_articulation("leg_hinge_0")
    leg_hinge_1 = object_model.get_articulation("leg_hinge_1")
    key_turn = object_model.get_articulation("key_turn")

    ctx.check(
        "metronome mechanisms present",
        pendulum_pivot.articulation_type == ArticulationType.REVOLUTE
        and weight_slide.articulation_type == ArticulationType.PRISMATIC
        and leg_hinge_0.articulation_type == ArticulationType.REVOLUTE
        and leg_hinge_1.articulation_type == ArticulationType.REVOLUTE
        and key_turn.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected revolute pendulum, prismatic weight, two revolute legs, and continuous winding key.",
    )
    ctx.check(
        "weight slide has realistic travel",
        weight_slide.motion_limits is not None
        and weight_slide.motion_limits.lower == 0.0
        and 0.10 <= (weight_slide.motion_limits.upper or 0.0) <= 0.14,
        details=f"limits={weight_slide.motion_limits}",
    )
    ctx.expect_origin_distance(
        weight,
        pendulum,
        axes="xy",
        max_dist=0.001,
        name="sliding weight remains coaxial with pendulum rod",
    )
    ctx.expect_overlap(
        weight,
        pendulum,
        axes="z",
        min_overlap=0.050,
        elem_a="weight_sleeve",
        elem_b="pendulum_rod",
        name="sliding weight surrounds the rod at rest",
    )

    rest_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({weight_slide: weight_slide.motion_limits.upper}):
        raised_weight_pos = ctx.part_world_position(weight)
        ctx.expect_overlap(
            weight,
            pendulum,
            axes="z",
            min_overlap=0.050,
            elem_a="weight_sleeve",
            elem_b="pendulum_rod",
            name="sliding weight still surrounds the rod when raised",
        )
    ctx.check(
        "sliding weight moves upward",
        rest_weight_pos is not None
        and raised_weight_pos is not None
        and raised_weight_pos[2] > rest_weight_pos[2] + 0.10,
        details=f"rest={rest_weight_pos}, raised={raised_weight_pos}",
    )

    closed_leg_0 = ctx.part_world_aabb(leg_0)
    closed_leg_1 = ctx.part_world_aabb(leg_1)
    with ctx.pose({leg_hinge_0: leg_hinge_0.motion_limits.upper, leg_hinge_1: leg_hinge_1.motion_limits.upper}):
        open_leg_0 = ctx.part_world_aabb(leg_0)
        open_leg_1 = ctx.part_world_aabb(leg_1)
    ctx.check(
        "stabilizer legs fold rearward",
        closed_leg_0 is not None
        and closed_leg_1 is not None
        and open_leg_0 is not None
        and open_leg_1 is not None
        and open_leg_0[1][1] > closed_leg_0[1][1] + 0.060
        and open_leg_1[1][1] > closed_leg_1[1][1] + 0.060,
        details=f"closed={closed_leg_0}, {closed_leg_1}; open={open_leg_0}, {open_leg_1}",
    )

    return ctx.report()


object_model = build_object_model()
