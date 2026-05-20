from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Return a cylinder origin and length for a strut from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("strut endpoints must differ")
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux)
    return Origin(xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0), rpy=(0.0, pitch, yaw)), length


def _add_x_ring(geom: MeshGeometry, x: float, radius: float, segments: int = 48) -> list[int]:
    ids: list[int] = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        ids.append(geom.add_vertex(x, radius * math.cos(angle), radius * math.sin(angle)))
    return ids


def _connect_rings(geom: MeshGeometry, a: list[int], b: list[int], *, reverse: bool = False) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        if reverse:
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])
        else:
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])


def _cap_ring(geom: MeshGeometry, ring: list[int], x: float, *, reverse: bool = False) -> None:
    center = geom.add_vertex(x, 0.0, 0.0)
    count = len(ring)
    for i in range(count):
        j = (i + 1) % count
        if reverse:
            geom.add_face(center, ring[j], ring[i])
        else:
            geom.add_face(center, ring[i], ring[j])


def _lamp_shell_geometry() -> MeshGeometry:
    """A thick, open-front searchlight can with a slightly flared bezel."""
    geom = MeshGeometry()
    outer_rear = _add_x_ring(geom, -0.200, 0.135)
    outer_mid = _add_x_ring(geom, 0.060, 0.148)
    outer_front = _add_x_ring(geom, 0.240, 0.170)
    inner_front = _add_x_ring(geom, 0.228, 0.145)
    inner_mid = _add_x_ring(geom, 0.060, 0.124)
    inner_rear = _add_x_ring(geom, -0.178, 0.090)

    _connect_rings(geom, outer_rear, outer_mid)
    _connect_rings(geom, outer_mid, outer_front)
    _connect_rings(geom, outer_front, inner_front)
    _connect_rings(geom, inner_front, inner_mid, reverse=True)
    _connect_rings(geom, inner_mid, inner_rear, reverse=True)
    _connect_rings(geom, inner_rear, outer_rear)
    return geom


def _reflector_geometry() -> MeshGeometry:
    """Shallow concave reflector lining visible behind the front glass."""
    geom = MeshGeometry()
    rear = _add_x_ring(geom, -0.070, 0.040)
    throat = _add_x_ring(geom, 0.050, 0.088)
    front = _add_x_ring(geom, 0.205, 0.148)
    _connect_rings(geom, rear, throat)
    _connect_rings(geom, throat, front)
    _cap_ring(geom, rear, -0.070, reverse=True)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    model.material("galvanized_steel", rgba=(0.55, 0.58, 0.58, 1.0))
    model.material("dark_powdercoat", rgba=(0.04, 0.045, 0.05, 1.0))
    model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    model.material("polished_reflector", rgba=(0.86, 0.84, 0.78, 1.0))
    model.material("warm_lamp_glow", rgba=(1.0, 0.78, 0.32, 1.0))
    model.material("pale_glass", rgba=(0.70, 0.86, 1.0, 0.42))

    mast = model.part("mast")
    mast.visual(
        Box((0.70, 0.70, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="galvanized_steel",
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.046, length=1.640),
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        material="galvanized_steel",
        name="main_mast",
    )
    mast.visual(
        Cylinder(radius=0.105, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.700)),
        material="dark_powdercoat",
        name="fixed_top_flange",
    )
    mast.visual(
        Cylinder(radius=0.080, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material="dark_powdercoat",
        name="mast_foot_collar",
    )

    # Four splayed tower braces make the support read as a compact searchlight tower.
    for index, (sx, sy) in enumerate(((0.28, 0.28), (-0.28, 0.28), (-0.28, -0.28), (0.28, -0.28))):
        tx = math.copysign(0.032, sx)
        ty = math.copysign(0.032, sy)
        origin, length = _origin_between((sx, sy, 0.045), (tx, ty, 1.455))
        mast.visual(
            Cylinder(radius=0.018, length=length),
            origin=origin,
            material="galvanized_steel",
            name=f"tower_brace_{index}",
        )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.180, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="dark_powdercoat",
        name="rotary_bearing",
    )
    pan_stage.visual(
        Cylinder(radius=0.125, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material="galvanized_steel",
        name="swivel_cap",
    )
    pan_stage.visual(
        Box((0.300, 0.500, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material="dark_powdercoat",
        name="yoke_base",
    )
    pan_stage.visual(
        Box((0.070, 0.035, 0.310)),
        origin=Origin(xyz=(0.0, -0.220, 0.255)),
        material="dark_powdercoat",
        name="yoke_arm_0",
    )
    pan_stage.visual(
        Cylinder(radius=0.060, length=0.006),
        origin=Origin(xyz=(0.0, -0.200, 0.310), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="galvanized_steel",
        name="bearing_face_0",
    )
    pan_stage.visual(
        Box((0.070, 0.035, 0.310)),
        origin=Origin(xyz=(0.0, 0.220, 0.255)),
        material="dark_powdercoat",
        name="yoke_arm_1",
    )
    pan_stage.visual(
        Cylinder(radius=0.060, length=0.006),
        origin=Origin(xyz=(0.0, 0.200, 0.310), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="galvanized_steel",
        name="bearing_face_1",
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        mesh_from_geometry(_lamp_shell_geometry(), "lamp_shell"),
        material="dark_powdercoat",
        name="lamp_shell",
    )
    lamp_head.visual(
        mesh_from_geometry(_reflector_geometry(), "reflector_bowl"),
        material="polished_reflector",
        name="reflector_bowl",
    )
    lamp_head.visual(
        Cylinder(radius=0.158, length=0.012),
        origin=Origin(xyz=(0.238, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="pale_glass",
        name="front_lens",
    )
    lamp_head.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material="warm_lamp_glow",
        name="bulb",
    )
    lamp_head.visual(
        Cylinder(radius=0.014, length=0.225),
        origin=Origin(xyz=(-0.082, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="galvanized_steel",
        name="bulb_stem",
    )
    lamp_head.visual(
        Cylinder(radius=0.046, length=0.054),
        origin=Origin(xyz=(0.0, -0.172, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="galvanized_steel",
        name="trunnion_hub_0",
    )
    lamp_head.visual(
        Cylinder(radius=0.046, length=0.054),
        origin=Origin(xyz=(0.0, 0.172, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="galvanized_steel",
        name="trunnion_hub_1",
    )

    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        # The lamp can points along local +X; -Y makes positive tilt raise the beam.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.65, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    pan_stage = object_model.get_part("pan_stage")
    lamp_head = object_model.get_part("lamp_head")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")

    ctx.allow_overlap(
        lamp_head,
        pan_stage,
        elem_a="trunnion_hub_0",
        elem_b="bearing_face_0",
        reason="The trunnion hub is intentionally captured slightly inside the yoke bearing face as a compact pivot bushing.",
    )
    ctx.allow_overlap(
        lamp_head,
        pan_stage,
        elem_a="trunnion_hub_1",
        elem_b="bearing_face_1",
        reason="The trunnion hub is intentionally captured slightly inside the yoke bearing face as a compact pivot bushing.",
    )
    ctx.expect_contact(
        mast,
        pan_stage,
        elem_a="fixed_top_flange",
        elem_b="rotary_bearing",
        contact_tol=0.001,
        name="pan bearing sits directly on mast flange",
    )
    ctx.expect_within(
        lamp_head,
        pan_stage,
        axes="y",
        inner_elem="lamp_shell",
        outer_elem="yoke_base",
        margin=0.0,
        name="lamp body is between the yoke sides",
    )
    ctx.expect_overlap(
        lamp_head,
        pan_stage,
        axes="z",
        elem_a="trunnion_hub_0",
        elem_b="yoke_arm_0",
        min_overlap=0.050,
        name="trunnion hub aligns with yoke bore height",
    )
    ctx.expect_gap(
        lamp_head,
        pan_stage,
        axis="y",
        positive_elem="trunnion_hub_0",
        negative_elem="bearing_face_0",
        max_penetration=0.003,
        name="lower yoke bushing has only shallow capture",
    )
    ctx.expect_gap(
        pan_stage,
        lamp_head,
        axis="y",
        positive_elem="bearing_face_1",
        negative_elem="trunnion_hub_1",
        max_penetration=0.003,
        name="upper yoke bushing has only shallow capture",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    rest_lens_center = None
    if rest_lens_aabb is not None:
        rest_lens_center = tuple((rest_lens_aabb[0][i] + rest_lens_aabb[1][i]) / 2.0 for i in range(3))

    with ctx.pose({tilt_joint: 0.75}):
        tilted_lens_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
        tilted_lens_center = None
        if tilted_lens_aabb is not None:
            tilted_lens_center = tuple((tilted_lens_aabb[0][i] + tilted_lens_aabb[1][i]) / 2.0 for i in range(3))
        ctx.check(
            "positive tilt raises beam",
            rest_lens_center is not None
            and tilted_lens_center is not None
            and tilted_lens_center[2] > rest_lens_center[2] + 0.12,
            details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
        )

    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_lens_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
        panned_lens_center = None
        if panned_lens_aabb is not None:
            panned_lens_center = tuple((panned_lens_aabb[0][i] + panned_lens_aabb[1][i]) / 2.0 for i in range(3))
        ctx.check(
            "pan joint slews beam around vertical mast",
            rest_lens_center is not None
            and panned_lens_center is not None
            and panned_lens_center[1] > rest_lens_center[1] + 0.18,
            details=f"rest={rest_lens_center}, panned={panned_lens_center}",
        )

    return ctx.report()


object_model = build_object_model()
