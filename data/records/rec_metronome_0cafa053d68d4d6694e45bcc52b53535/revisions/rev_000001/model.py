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
    MeshGeometry,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _tapered_housing_mesh() -> MeshGeometry:
    """Closed truncated pyramid body for the wooden metronome case."""
    mesh = MeshGeometry()
    bottom_x = 0.084
    bottom_y = 0.056
    top_x = 0.026
    top_y = 0.020
    z0 = 0.042
    z1 = 0.315
    verts = [
        (-bottom_x, -bottom_y, z0),
        (bottom_x, -bottom_y, z0),
        (bottom_x, bottom_y, z0),
        (-bottom_x, bottom_y, z0),
        (-top_x, -top_y, z1),
        (top_x, -top_y, z1),
        (top_x, top_y, z1),
        (-top_x, top_y, z1),
    ]
    v = [mesh.add_vertex(*p) for p in verts]
    _add_quad(mesh, v[0], v[1], v[2], v[3])
    _add_quad(mesh, v[4], v[7], v[6], v[5])
    _add_quad(mesh, v[0], v[4], v[5], v[1])
    _add_quad(mesh, v[1], v[5], v[6], v[2])
    _add_quad(mesh, v[2], v[6], v[7], v[3])
    _add_quad(mesh, v[3], v[7], v[4], v[0])
    return mesh


def _material(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_pyramid_metronome")

    dark_wood = _material(model, "dark_wood", (0.28, 0.15, 0.07, 1.0))
    wood_edge = _material(model, "wood_edge", (0.18, 0.09, 0.04, 1.0))
    brass = _material(model, "brass", (0.92, 0.68, 0.26, 1.0))
    aged_brass = _material(model, "aged_brass", (0.72, 0.49, 0.18, 1.0))
    steel = _material(model, "polished_steel", (0.74, 0.75, 0.72, 1.0))
    black = _material(model, "black_marking", (0.02, 0.018, 0.015, 1.0))
    rubber = _material(model, "black_rubber", (0.025, 0.025, 0.023, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.220, 0.160, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=dark_wood,
        name="base_plinth",
    )
    body.visual(
        Box((0.232, 0.172, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=wood_edge,
        name="base_lip",
    )
    body.visual(
        mesh_from_geometry(_tapered_housing_mesh(), "pyramid_housing"),
        material=dark_wood,
        name="tapered_housing",
    )
    body.visual(
        Box((0.060, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.010, 0.322)),
        material=wood_edge,
        name="top_cap",
    )
    body.visual(
        Box((0.010, 0.006, 0.260)),
        origin=Origin(xyz=(0.0, -0.066, 0.172)),
        material=brass,
        name="front_scale",
    )
    for tick_index, z in enumerate((0.075, 0.100, 0.125, 0.150, 0.175, 0.200, 0.225, 0.250, 0.275)):
        tick_width = 0.034 if tick_index % 2 == 0 else 0.024
        body.visual(
            Box((tick_width, 0.004, 0.0025)),
            origin=Origin(xyz=(0.0, -0.070, z)),
            material=black,
            name=f"scale_tick_{tick_index}",
        )
    body.visual(
        Box((0.034, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, -0.079, 0.292)),
        material=aged_brass,
        name="pivot_bridge",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.0, -0.086, 0.292), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.046, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="key_bushing",
    )
    for idx, x in enumerate((-0.102, 0.102)):
        body.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(x, -0.070, 0.006)),
            material=steel,
            name=f"leg_hinge_{idx}",
        )
    for idx, x in enumerate((-0.080, 0.080)):
        for idy, y in enumerate((-0.052, 0.052)):
            body.visual(
                Box((0.034, 0.020, 0.004)),
                origin=Origin(xyz=(x, y, 0.010)),
                material=rubber,
                name=f"rubber_foot_{idx}_{idy}",
            )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0025, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, -0.121)),
        material=steel,
        name="rod",
    )
    pendulum.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.238)),
        material=brass,
        name="lower_bob",
    )
    model.articulation(
        "body_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pendulum,
        origin=Origin(xyz=(0.0, -0.092, 0.292)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=-0.45, upper=0.45),
    )

    weight = model.part("sliding_weight")
    weight.visual(
        Cylinder(radius=0.021, length=0.034),
        origin=Origin(),
        material=aged_brass,
        name="weight_shell",
    )
    weight.visual(
        Box((0.032, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, -0.021, 0.0)),
        material=black,
        name="grip_line",
    )
    model.articulation(
        "pendulum_to_sliding_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.12, lower=0.0, upper=0.130),
    )

    for idx, x_sign in enumerate((-1.0, 1.0)):
        leg = model.part(f"stabilizer_{idx}")
        direction = -x_sign
        leg.visual(
            Box((0.082, 0.012, 0.010)),
            origin=Origin(xyz=(direction * 0.050, 0.0, 0.001)),
            material=aged_brass,
            name="leg_bar",
        )
        leg.visual(
            Box((0.020, 0.020, 0.005)),
            origin=Origin(xyz=(direction * 0.087, 0.0, -0.004)),
            material=rubber,
            name="foot_pad",
        )
        model.articulation(
            f"body_to_stabilizer_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=leg,
            origin=Origin(xyz=(x_sign * 0.102, -0.070, 0.006)),
            axis=(0.0, 0.0, x_sign),
            motion_limits=MotionLimits(effort=1.2, velocity=1.5, lower=0.0, upper=1.35),
        )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.0045, length=0.028),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="key_stem",
    )
    winding_key.visual(
        Cylinder(radius=0.008, length=0.011),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="key_hub",
    )
    for idx, x in enumerate((-0.015, 0.015)):
        winding_key.visual(
            Box((0.026, 0.006, 0.016)),
            origin=Origin(xyz=(x, 0.035, 0.0)),
            material=aged_brass,
            name=f"key_wing_{idx}",
        )
    model.articulation(
        "body_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=winding_key,
        origin=Origin(xyz=(0.0, 0.055, 0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("sliding_weight")
    winding_key = object_model.get_part("winding_key")
    leg_0 = object_model.get_part("stabilizer_0")
    leg_1 = object_model.get_part("stabilizer_1")
    pendulum_joint = object_model.get_articulation("body_to_pendulum")
    weight_slide = object_model.get_articulation("pendulum_to_sliding_weight")
    leg_joint_0 = object_model.get_articulation("body_to_stabilizer_0")
    leg_joint_1 = object_model.get_articulation("body_to_stabilizer_1")
    key_joint = object_model.get_articulation("body_to_winding_key")

    ctx.allow_overlap(
        pendulum,
        weight,
        elem_a="rod",
        elem_b="weight_shell",
        reason="The cylindrical sliding weight is modeled as a tight sleeve captured around the pendulum rod.",
    )
    ctx.expect_within(
        pendulum,
        weight,
        axes="xy",
        inner_elem="rod",
        outer_elem="weight_shell",
        margin=0.0,
        name="rod passes through sliding weight center",
    )
    ctx.expect_overlap(
        pendulum,
        weight,
        axes="z",
        elem_a="rod",
        elem_b="weight_shell",
        min_overlap=0.030,
        name="sliding weight remains captured on rod",
    )
    ctx.expect_contact(
        body,
        pendulum,
        elem_a="pivot_pin",
        elem_b="rod",
        contact_tol=0.001,
        name="pendulum rod seats on top pivot",
    )
    ctx.expect_contact(
        body,
        winding_key,
        elem_a="key_bushing",
        elem_b="key_stem",
        contact_tol=0.001,
        name="winding key stem seats in rear bushing",
    )

    rest_weight = ctx.part_world_position(weight)
    with ctx.pose({weight_slide: 0.130}):
        raised_weight = ctx.part_world_position(weight)
    ctx.check(
        "weight slides upward along rod",
        rest_weight is not None
        and raised_weight is not None
        and raised_weight[2] > rest_weight[2] + 0.11
        and abs(raised_weight[0] - rest_weight[0]) < 0.002
        and abs(raised_weight[1] - rest_weight[1]) < 0.002,
        details=f"rest={rest_weight}, raised={raised_weight}",
    )

    def _elem_center_y(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[1] + upper[1]) * 0.5

    leg0_rest_y = _elem_center_y("stabilizer_0", "leg_bar")
    leg1_rest_y = _elem_center_y("stabilizer_1", "leg_bar")
    with ctx.pose({leg_joint_0: 1.15, leg_joint_1: 1.15}):
        leg0_open_y = _elem_center_y("stabilizer_0", "leg_bar")
        leg1_open_y = _elem_center_y("stabilizer_1", "leg_bar")
    ctx.check(
        "stabilizer legs fold outward from base",
        leg0_rest_y is not None
        and leg1_rest_y is not None
        and leg0_open_y is not None
        and leg1_open_y is not None
        and leg0_open_y < leg0_rest_y - 0.030
        and leg1_open_y < leg1_rest_y - 0.030,
        details=f"rest=({leg0_rest_y}, {leg1_rest_y}), open=({leg0_open_y}, {leg1_open_y})",
    )

    rod_rest = ctx.part_element_world_aabb(pendulum, elem="rod")
    with ctx.pose({pendulum_joint: 0.35}):
        rod_swing = ctx.part_element_world_aabb(pendulum, elem="rod")
    if rod_rest is not None and rod_swing is not None:
        rest_center_x = (rod_rest[0][0] + rod_rest[1][0]) * 0.5
        swing_center_x = (rod_swing[0][0] + rod_swing[1][0]) * 0.5
    else:
        rest_center_x = swing_center_x = None
    ctx.check(
        "pendulum swings side to side",
        rest_center_x is not None and swing_center_x is not None and abs(swing_center_x - rest_center_x) > 0.025,
        details=f"rest_x={rest_center_x}, swing_x={swing_center_x}",
    )
    ctx.check(
        "rear winding key is continuous",
        getattr(key_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"joint_type={getattr(key_joint, 'articulation_type', None)}",
    )

    return ctx.report()


object_model = build_object_model()
