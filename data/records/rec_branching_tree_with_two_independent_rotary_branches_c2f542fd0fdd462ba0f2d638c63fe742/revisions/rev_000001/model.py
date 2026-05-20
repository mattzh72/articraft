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
)


POST_RADIUS = 0.032
LOWER_HUB_Z = 0.58
UPPER_HUB_Z = 0.88
ROTATING_HUB_HEIGHT = 0.070
SUPPORT_WASHER_THICKNESS = 0.018
SUPPORT_GAP = 0.0


def _annular_cylinder_geometry(
    *,
    inner_radius: float,
    outer_radius: float,
    height: float,
    segments: int = 64,
) -> MeshGeometry:
    """Build a real open-bore collar mesh centered at the local origin."""

    geom = MeshGeometry()
    z0 = -height / 2.0
    z1 = height / 2.0
    rings: list[tuple[int, int, int, int]] = []

    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        outer_bottom = geom.add_vertex(outer_radius * c, outer_radius * s, z0)
        outer_top = geom.add_vertex(outer_radius * c, outer_radius * s, z1)
        inner_bottom = geom.add_vertex(inner_radius * c, inner_radius * s, z0)
        inner_top = geom.add_vertex(inner_radius * c, inner_radius * s, z1)
        rings.append((outer_bottom, outer_top, inner_bottom, inner_top))

    for i in range(segments):
        j = (i + 1) % segments
        ob_i, ot_i, ib_i, it_i = rings[i]
        ob_j, ot_j, ib_j, it_j = rings[j]

        # Outer cylindrical wall.
        geom.add_face(ob_i, ob_j, ot_j)
        geom.add_face(ob_i, ot_j, ot_i)

        # Inner bore wall.
        geom.add_face(ib_j, ib_i, it_i)
        geom.add_face(ib_j, it_i, it_j)

        # Top and bottom annular faces.
        geom.add_face(ot_i, ot_j, it_j)
        geom.add_face(ot_i, it_j, it_i)
        geom.add_face(ob_j, ob_i, ib_i)
        geom.add_face(ob_j, ib_i, ib_j)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_two_branch_motion_study")

    steel = Material("brushed_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    dark_steel = Material("dark_bearing_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    base_blue = Material("anodized_blue", rgba=(0.08, 0.22, 0.45, 1.0))
    orange = Material("safety_orange", rgba=(0.95, 0.42, 0.10, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    hub_collar_mesh = mesh_from_geometry(
        _annular_cylinder_geometry(
            inner_radius=POST_RADIUS + 0.011,
            outer_radius=0.066,
            height=ROTATING_HUB_HEIGHT,
        ),
        "rotating_hub_collar",
    )

    stand = model.part("stand")
    stand.visual(
        Box((0.56, 0.38, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="ground_plate",
    )
    stand.visual(
        Cylinder(radius=POST_RADIUS, length=1.25),
        origin=Origin(xyz=(0.0, 0.0, 0.035 + 1.25 / 2.0)),
        material=steel,
        name="central_post",
    )
    stand.visual(
        Cylinder(radius=0.075, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=steel,
        name="post_foot_collar",
    )

    for prefix, hub_z in (("lower", LOWER_HUB_Z), ("upper", UPPER_HUB_Z)):
        lower_washer_z = (
            hub_z
            - ROTATING_HUB_HEIGHT / 2.0
            - SUPPORT_GAP
            - SUPPORT_WASHER_THICKNESS / 2.0
        )
        upper_washer_z = (
            hub_z
            + ROTATING_HUB_HEIGHT / 2.0
            + SUPPORT_GAP
            + SUPPORT_WASHER_THICKNESS / 2.0
        )
        stand.visual(
            Cylinder(radius=0.078, length=SUPPORT_WASHER_THICKNESS),
            origin=Origin(xyz=(0.0, 0.0, lower_washer_z)),
            material=dark_steel,
            name=f"{prefix}_bearing_lower",
        )
        stand.visual(
            Cylinder(radius=0.078, length=SUPPORT_WASHER_THICKNESS),
            origin=Origin(xyz=(0.0, 0.0, upper_washer_z)),
            material=dark_steel,
            name=f"{prefix}_bearing_upper",
        )

    pad_arm = model.part("pad_arm")
    pad_arm.visual(
        hub_collar_mesh,
        origin=Origin(),
        material=base_blue,
        name="hub_collar",
    )
    pad_arm.visual(
        Box((0.045, 0.086, 0.030)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=base_blue,
        name="hub_web",
    )
    for idx, y in enumerate((-0.023, 0.023)):
        pad_arm.visual(
            Box((0.350, 0.018, 0.026)),
            origin=Origin(xyz=(0.235, y, 0.0)),
            material=base_blue,
            name=f"rail_{idx}",
        )
    pad_arm.visual(
        Box((0.055, 0.090, 0.024)),
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        material=base_blue,
        name="pad_yoke",
    )
    pad_arm.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.470, 0.0, 0.0)),
        material=rubber,
        name="pad_tip",
    )

    fork_arm = model.part("fork_arm")
    fork_arm.visual(
        hub_collar_mesh,
        origin=Origin(),
        material=orange,
        name="hub_collar",
    )
    fork_arm.visual(
        Box((0.045, 0.092, 0.030)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=orange,
        name="hub_web",
    )
    for idx, y in enumerate((-0.026, 0.026)):
        fork_arm.visual(
            Box((0.300, 0.016, 0.024)),
            origin=Origin(xyz=(0.220, y, 0.0)),
            material=orange,
            name=f"rail_{idx}",
        )
    fork_arm.visual(
        Box((0.040, 0.112, 0.034)),
        origin=Origin(xyz=(0.385, 0.0, 0.0)),
        material=orange,
        name="fork_bridge",
    )
    for idx, y in enumerate((-0.044, 0.044)):
        fork_arm.visual(
            Box((0.125, 0.020, 0.032)),
            origin=Origin(xyz=(0.450, y, 0.0)),
            material=orange,
            name=f"fork_tine_{idx}",
        )

    model.articulation(
        "pad_pivot",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=pad_arm,
        origin=Origin(xyz=(0.0, 0.0, LOWER_HUB_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "fork_pivot",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=fork_arm,
        origin=Origin(xyz=(0.0, 0.0, UPPER_HUB_Z), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.20, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    pad_arm = object_model.get_part("pad_arm")
    fork_arm = object_model.get_part("fork_arm")
    pad_pivot = object_model.get_articulation("pad_pivot")
    fork_pivot = object_model.get_articulation("fork_pivot")

    ctx.check(
        "two independent revolute branches",
        pad_pivot.articulation_type == ArticulationType.REVOLUTE
        and fork_pivot.articulation_type == ArticulationType.REVOLUTE
        and pad_pivot.child == "pad_arm"
        and fork_pivot.child == "fork_arm",
        details=f"pad={pad_pivot}, fork={fork_pivot}",
    )
    ctx.expect_gap(
        fork_arm,
        pad_arm,
        axis="z",
        min_gap=0.20,
        name="branches are vertically separated on individual hub collars",
    )

    for arm, prefix in ((pad_arm, "lower"), (fork_arm, "upper")):
        ctx.expect_within(
            arm,
            stand,
            axes="xy",
            inner_elem="hub_collar",
            outer_elem=f"{prefix}_bearing_lower",
            margin=0.0,
            name=f"{prefix} rotating hub sits inside bearing footprint",
        )
        ctx.expect_gap(
            arm,
            stand,
            axis="z",
            positive_elem="hub_collar",
            negative_elem=f"{prefix}_bearing_lower",
            min_gap=0.0,
            max_gap=0.001,
            name=f"{prefix} lower bearing clearance",
        )
        ctx.expect_gap(
            stand,
            arm,
            axis="z",
            positive_elem=f"{prefix}_bearing_upper",
            negative_elem="hub_collar",
            min_gap=0.0,
            max_gap=0.001,
            name=f"{prefix} upper bearing clearance",
        )

    def elem_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    pad_tip_rest = elem_center(pad_arm, "pad_tip")
    fork_tip_rest = elem_center(fork_arm, "fork_bridge")
    with ctx.pose({pad_pivot: 0.85}):
        pad_tip_swept = elem_center(pad_arm, "pad_tip")
        fork_tip_while_pad_moves = elem_center(fork_arm, "fork_bridge")
    with ctx.pose({fork_pivot: -0.85}):
        fork_tip_swept = elem_center(fork_arm, "fork_bridge")

    ctx.check(
        "pad branch sweeps while fork branch stays put",
        pad_tip_rest is not None
        and pad_tip_swept is not None
        and fork_tip_rest is not None
        and fork_tip_while_pad_moves is not None
        and pad_tip_swept[1] > pad_tip_rest[1] + 0.25
        and abs(pad_tip_swept[2] - pad_tip_rest[2]) < 0.002
        and abs(fork_tip_while_pad_moves[0] - fork_tip_rest[0]) < 0.002
        and abs(fork_tip_while_pad_moves[1] - fork_tip_rest[1]) < 0.002,
        details=f"pad rest={pad_tip_rest}, pad swept={pad_tip_swept}, fork={fork_tip_rest}->{fork_tip_while_pad_moves}",
    )
    ctx.check(
        "fork branch has its own sweep",
        fork_tip_rest is not None
        and fork_tip_swept is not None
        and fork_tip_swept[0] > fork_tip_rest[0] + 0.22
        and abs(fork_tip_swept[2] - fork_tip_rest[2]) < 0.002,
        details=f"fork rest={fork_tip_rest}, fork swept={fork_tip_swept}",
    )

    return ctx.report()


object_model = build_object_model()
