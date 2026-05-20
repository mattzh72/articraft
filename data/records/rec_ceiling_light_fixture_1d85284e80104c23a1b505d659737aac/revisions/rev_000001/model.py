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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_oyster_ceiling_light")

    brushed_nickel = model.material("brushed_nickel", rgba=(0.62, 0.58, 0.50, 1.0))
    warm_white = model.material("warm_white", rgba=(1.0, 0.86, 0.52, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.86, 0.93, 1.0, 0.42))
    pale_socket = model.material("pale_socket", rgba=(0.92, 0.90, 0.82, 1.0))

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.190, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.0175)),
        material=brushed_nickel,
        name="housing_pan",
    )
    housing.visual(
        mesh_from_geometry(TorusGeometry(radius=0.165, tube=0.007, radial_segments=64), "housing_rolled_rim"),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=brushed_nickel,
        name="rolled_rim",
    )

    # The socket and warm bulb sit inside the oyster shade and remain fixed to the ceiling pan.
    housing.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=pale_socket,
        name="socket",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.087)),
        material=brushed_nickel,
        name="bulb_base",
    )
    housing.visual(
        Sphere(radius=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=warm_white,
        name="bulb_globe",
    )

    # Two compact bracket blocks carry the separate visible hinge pins on either side.
    for i, sx in enumerate((-1.0, 1.0)):
        housing.visual(
            Box((0.030, 0.022, 0.026)),
            origin=Origin(xyz=(sx * 0.101, -0.155, -0.034)),
            material=brushed_nickel,
            name=f"hinge_bracket_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0048, length=0.050),
            origin=Origin(xyz=(sx * 0.122, -0.145, -0.047), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_nickel,
            name=f"pin_{i}",
        )

    diffuser = model.part("diffuser")
    outer_profile = [
        (0.010, -0.137),
        (0.048, -0.132),
        (0.092, -0.107),
        (0.128, -0.070),
        (0.150, -0.025),
        (0.156, 0.000),
    ]
    inner_profile = [
        (0.006, -0.123),
        (0.040, -0.118),
        (0.080, -0.096),
        (0.114, -0.061),
        (0.139, -0.020),
        (0.146, -0.004),
    ]
    glass_bowl = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=80,
        lip_samples=8,
    )
    diffuser.visual(
        mesh_from_geometry(glass_bowl, "glass_bowl"),
        origin=Origin(xyz=(0.0, 0.145, 0.0)),
        material=frosted_glass,
        name="glass_bowl",
    )
    diffuser.visual(
        mesh_from_geometry(TorusGeometry(radius=0.155, tube=0.0055, radial_segments=72), "retaining_ring"),
        origin=Origin(xyz=(0.0, 0.145, -0.002)),
        material=brushed_nickel,
        name="retaining_ring",
    )
    for i, sx in enumerate((-1.0, 1.0)):
        diffuser.visual(
            Box((0.022, 0.070, 0.012)),
            origin=Origin(xyz=(sx * 0.122, 0.030, -0.006)),
            material=brushed_nickel,
            name=f"hinge_ear_{i}",
        )
    diffuser.visual(
        Box((0.040, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.302, -0.006)),
        material=brushed_nickel,
        name="front_catch",
    )

    model.articulation(
        "housing_to_diffuser",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=diffuser,
        origin=Origin(xyz=(0.0, -0.145, -0.047)),
        # The closed diffuser extends forward from this rear hinge line.
        # -X makes positive motion swing the front catch downward for bulb access.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    diffuser = object_model.get_part("diffuser")
    hinge = object_model.get_articulation("housing_to_diffuser")

    for i in range(2):
        ctx.allow_overlap(
            housing,
            diffuser,
            elem_a=f"pin_{i}",
            elem_b=f"hinge_ear_{i}",
            reason="Each metal hinge pin is intentionally captured through its diffuser ear.",
        )
        ctx.expect_overlap(
            housing,
            diffuser,
            axes="xyz",
            elem_a=f"pin_{i}",
            elem_b=f"hinge_ear_{i}",
            min_overlap=0.003,
            name=f"hinge pin {i} passes through diffuser ear",
        )

    ctx.expect_gap(
        housing,
        diffuser,
        axis="z",
        positive_elem="housing_pan",
        negative_elem="retaining_ring",
        min_gap=0.002,
        max_gap=0.020,
        name="closed retaining ring sits just below flat housing",
    )

    closed_catch_aabb = ctx.part_element_world_aabb(diffuser, elem="front_catch")
    with ctx.pose({hinge: math.radians(70.0)}):
        open_catch_aabb = ctx.part_element_world_aabb(diffuser, elem="front_catch")
        ctx.expect_gap(
            housing,
            diffuser,
            axis="z",
            positive_elem="housing_pan",
            negative_elem="front_catch",
            min_gap=0.10,
            name="opened front catch drops well below housing",
        )

    ctx.check(
        "diffuser swings downward on hinge",
        closed_catch_aabb is not None
        and open_catch_aabb is not None
        and open_catch_aabb[0][2] < closed_catch_aabb[0][2] - 0.10,
        details=f"closed={closed_catch_aabb}, open={open_catch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
