from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _circle_loop(cx: float, cy: float, radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            cx + math.cos(math.tau * index / segments) * radius,
            cy + math.sin(math.tau * index / segments) * radius,
        )
        for index in range(segments)
    ]


def _carriage_cheek_mesh(name: str):
    outer_profile = [
        (-0.96, 0.00),
        (0.98, 0.00),
        (1.04, 0.18),
        (0.60, 0.52),
        (0.22, 0.70),
        (-0.34, 0.66),
        (-0.82, 0.42),
        (-0.96, 0.16),
    ]
    trunnion_clearance = _circle_loop(0.0, 0.60, 0.165, segments=48)
    geometry = ExtrudeWithHolesGeometry(
        outer_profile,
        [list(reversed(trunnion_clearance))],
        0.115,
        cap=True,
        center=True,
    )
    return mesh_from_geometry(geometry, name)


def _barrel_mesh():
    # Lathed around local +Z, then rotated on the visual so local +Z becomes
    # the cannon bore axis (+X).  The profile includes a blind muzzle bore.
    profile = [
        (0.0, -0.90),
        (0.070, -0.895),
        (0.120, -0.855),
        (0.100, -0.805),
        (0.060, -0.775),
        (0.175, -0.748),
        (0.325, -0.690),
        (0.355, -0.610),
        (0.355, -0.525),
        (0.305, -0.475),
        (0.292, -0.235),
        (0.260, -0.205),
        (0.250, 0.170),
        (0.220, 0.225),
        (0.210, 0.780),
        (0.182, 1.250),
        (0.180, 1.540),
        (0.224, 1.600),
        (0.226, 1.750),
        (0.190, 1.830),
        (0.076, 1.830),
        (0.076, 1.570),
        (0.0, 1.570),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=80), "barrel_casting")


def _handwheel_face_mesh():
    face = MeshGeometry()
    face.merge(TorusGeometry(radius=0.155, tube=0.012, radial_segments=16, tubular_segments=72))
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        spoke = CylinderGeometry(radius=0.0065, height=0.330, radial_segments=12)
        spoke.rotate_y(math.pi / 2.0).rotate_z(angle)
        face.merge(spoke)
    face.merge(CylinderGeometry(radius=0.045, height=0.046, radial_segments=32))

    # Off-centre crank pin and grip, merged into the hand-wheel mesh.
    crank_x = 0.108
    crank_y = 0.108
    face.merge(
        CylinderGeometry(radius=0.0065, height=0.080, radial_segments=12).translate(
            crank_x,
            crank_y,
            -0.040,
        )
    )
    face.merge(
        CylinderGeometry(radius=0.019, height=0.070, radial_segments=18).translate(
            crank_x,
            crank_y,
            -0.105,
        )
    )
    return mesh_from_geometry(face, "right_handwheel_face")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="victorian_coastal_garrison_cannon")

    cast_iron = model.material("cast_iron", rgba=(0.055, 0.058, 0.060, 1.0))
    worn_iron = model.material("worn_iron", rgba=(0.105, 0.108, 0.105, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.12, 0.15, 0.14, 1.0))
    oiled_steel = model.material("oiled_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    brass = model.material("aged_brass", rgba=(0.56, 0.43, 0.20, 1.0))
    timber = model.material("dark_oak", rgba=(0.25, 0.16, 0.08, 1.0))

    barrel_mesh = _barrel_mesh()
    handwheel_mesh = _handwheel_face_mesh()

    carriage = model.part("carriage")
    carriage.visual(
        Box((2.05, 1.05, 0.18)),
        origin=Origin(xyz=(0.02, 0.0, 0.420)),
        material=timber,
        name="platform_deck",
    )
    carriage.visual(
        Box((1.90, 0.13, 0.16)),
        origin=Origin(xyz=(0.0, 0.575, 0.590)),
        material=gunmetal,
        name="rail_0",
    )
    carriage.visual(
        Box((1.90, 0.13, 0.16)),
        origin=Origin(xyz=(0.0, -0.575, 0.590)),
        material=gunmetal,
        name="rail_1",
    )
    for cheek_index, cheek_y in enumerate((0.425, -0.425)):
        carriage.visual(
            Box((1.95, 0.115, 0.260)),
            origin=Origin(xyz=(0.0, cheek_y, 0.580)),
            material=gunmetal,
            name=f"cheek_lower_{cheek_index}",
        )
        carriage.visual(
            Box((0.20, 0.115, 0.520)),
            origin=Origin(xyz=(-0.82, cheek_y, 0.820)),
            material=gunmetal,
            name=f"cheek_rear_post_{cheek_index}",
        )
        carriage.visual(
            Box((0.20, 0.115, 0.460)),
            origin=Origin(xyz=(0.80, cheek_y, 0.790)),
            material=gunmetal,
            name=f"cheek_front_post_{cheek_index}",
        )
        carriage.visual(
            Box((0.55, 0.115, 0.120)),
            origin=Origin(xyz=(-0.49, cheek_y, 1.020)),
            material=gunmetal,
            name=f"cheek_rear_top_{cheek_index}",
        )
        carriage.visual(
            Box((0.55, 0.115, 0.120)),
            origin=Origin(xyz=(0.49, cheek_y, 1.020)),
            material=gunmetal,
            name=f"cheek_front_top_{cheek_index}",
        )
    carriage.visual(
        Box((0.22, 1.05, 0.270)),
        origin=Origin(xyz=(-0.78, 0.0, 0.565)),
        material=gunmetal,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.22, 0.32, 0.070)),
        origin=Origin(xyz=(0.0, 0.425, 0.925)),
        material=oiled_steel,
        name="trunnion_saddle_0",
    )
    carriage.visual(
        Box((0.18, 0.115, 0.210)),
        origin=Origin(xyz=(0.0, 0.425, 0.805)),
        material=gunmetal,
        name="saddle_web_0",
    )
    carriage.visual(
        Box((0.22, 0.32, 0.070)),
        origin=Origin(xyz=(0.0, -0.425, 0.925)),
        material=oiled_steel,
        name="trunnion_saddle_1",
    )
    carriage.visual(
        Box((0.18, 0.115, 0.210)),
        origin=Origin(xyz=(0.0, -0.425, 0.805)),
        material=gunmetal,
        name="saddle_web_1",
    )
    carriage.visual(
        Cylinder(radius=0.070, length=1.10),
        origin=Origin(xyz=(-0.46, 0.0, 0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oiled_steel,
        name="wheel_axle",
    )
    carriage.visual(
        Cylinder(radius=0.100, length=0.140),
        origin=Origin(xyz=(-0.46, 0.600, 0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oiled_steel,
        name="wheel_collar_0",
    )
    carriage.visual(
        Cylinder(radius=0.100, length=0.140),
        origin=Origin(xyz=(-0.46, -0.600, 0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oiled_steel,
        name="wheel_collar_1",
    )
    carriage.visual(
        Box((0.30, 0.105, 0.180)),
        origin=Origin(xyz=(0.44, -0.535, 0.820)),
        material=gunmetal,
        name="traverse_gearbox",
    )
    carriage.visual(
        Box((0.34, 0.080, 0.140)),
        origin=Origin(xyz=(0.44, -0.475, 0.740)),
        material=gunmetal,
        name="traverse_mount",
    )
    carriage.visual(
        Cylinder(radius=0.075, length=0.120),
        origin=Origin(xyz=(0.44, -0.590, 0.820), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="traverse_bearing",
    )
    carriage.visual(
        Box((0.90, 0.035, 0.040)),
        origin=Origin(xyz=(0.42, -0.635, 0.675)),
        material=brass,
        name="traverse_rack",
    )
    for tooth_index in range(7):
        carriage.visual(
            Box((0.035, 0.030, 0.030)),
            origin=Origin(xyz=(0.21 + tooth_index * 0.065, -0.655, 0.710)),
            material=brass,
            name=f"rack_tooth_{tooth_index}",
        )

    barrel = model.part("barrel")
    barrel.visual(
        barrel_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="barrel_casting",
    )
    barrel.visual(
        Cylinder(radius=0.090, length=1.045),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_iron,
        name="trunnion_pin",
    )

    for index, y in enumerate((0.750, -0.750)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.420, length=0.160),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_iron,
            name="solid_wheel",
        )
        wheel.visual(
            Cylinder(radius=0.145, length=0.145),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=oiled_steel,
            name="raised_hub",
        )
        wheel.visual(
            Cylinder(radius=0.060, length=0.145),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=cast_iron,
            name="axle_cap",
        )
        model.articulation(
            f"carriage_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel,
            origin=Origin(xyz=(-0.46, y, 0.420)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=900.0, velocity=5.0),
        )

    right_handwheel = model.part("right_handwheel")
    right_handwheel.visual(
        handwheel_mesh,
        material=brass,
        name="rim_spokes",
    )
    right_handwheel.visual(
        Cylinder(radius=0.026, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=oiled_steel,
        name="shaft",
    )

    model.articulation(
        "trunnion_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 1.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.35,
            lower=math.radians(-5.0),
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "right_traverse_handwheel",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=right_handwheel,
        origin=Origin(xyz=(0.44, -0.820, 0.820), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=7.0),
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

    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    handwheel = object_model.get_part("right_handwheel")
    elevation = object_model.get_articulation("trunnion_elevation")
    handwheel_joint = object_model.get_articulation("right_traverse_handwheel")

    ctx.expect_gap(
        barrel,
        carriage,
        axis="z",
        positive_elem="trunnion_pin",
        negative_elem="trunnion_saddle_0",
        max_gap=0.002,
        max_penetration=0.00001,
        name="trunnion rests on first saddle",
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="z",
        positive_elem="trunnion_pin",
        negative_elem="trunnion_saddle_1",
        max_gap=0.002,
        max_penetration=0.00001,
        name="trunnion rests on second saddle",
    )
    ctx.expect_gap(
        wheel_0,
        carriage,
        axis="y",
        positive_elem="solid_wheel",
        negative_elem="wheel_collar_0",
        max_gap=0.006,
        max_penetration=0.0,
        name="first iron wheel seats on axle collar",
    )
    ctx.expect_gap(
        carriage,
        wheel_1,
        axis="y",
        positive_elem="wheel_collar_1",
        negative_elem="solid_wheel",
        max_gap=0.006,
        max_penetration=0.0,
        name="second iron wheel seats on axle collar",
    )
    ctx.expect_gap(
        carriage,
        handwheel,
        axis="y",
        positive_elem="traverse_bearing",
        negative_elem="shaft",
        max_gap=0.003,
        max_penetration=0.0,
        name="traverse handwheel shaft meets cheek bearing",
    )

    rest_aabb = ctx.part_world_aabb(barrel)
    with ctx.pose({elevation: math.radians(20.0), handwheel_joint: math.radians(90.0)}):
        raised_aabb = ctx.part_world_aabb(barrel)
        ctx.check(
            "elevation joint raises the muzzle",
            rest_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > rest_aabb[1][2] + 0.20,
            details=f"rest={rest_aabb}, raised={raised_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
