from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _ring_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    name: str,
    segments: int = 72,
):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.006,
        radial_segments=segments,
    )
    return mesh_from_geometry(
        boolean_difference(outer, inner).translate(0.0, 0.0, z_center),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_turnstile_gate")

    concrete = model.material("sealed_concrete", rgba=(0.48, 0.50, 0.48, 1.0))
    galvanized = model.material("hot_dip_galvanized", rgba=(0.62, 0.66, 0.66, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.78, 0.79, 0.76, 1.0))
    dark = model.material("dark_bearing_housing", rgba=(0.16, 0.17, 0.17, 1.0))
    rubber = model.material("black_epdm_seal", rgba=(0.02, 0.02, 0.018, 1.0))
    safety_yellow = model.material("yellow_safety_caps", rgba=(0.96, 0.78, 0.10, 1.0))

    post = model.part("central_post")
    post.visual(
        Box((0.86, 0.86, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=concrete,
        name="concrete_pad",
    )
    post.visual(
        Cylinder(radius=0.29, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=galvanized,
        name="base_plate",
    )
    post.visual(
        Cylinder(radius=0.075, length=0.99),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=galvanized,
        name="sealed_post",
    )
    post.visual(
        Cylinder(radius=0.105, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=rubber,
        name="base_boot",
    )
    post.visual(
        Cylinder(radius=0.150, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=galvanized,
        name="welded_flange",
    )
    post.visual(
        Cylinder(radius=0.118, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 1.073)),
        material=dark,
        name="lower_bearing_body",
    )
    post.visual(
        Cylinder(radius=0.132, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 1.122)),
        material=dark,
        name="bearing_core",
    )
    post.visual(
        Cylinder(radius=0.128, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 1.141)),
        material=rubber,
        name="upper_epdm_seal",
    )
    post.visual(
        Cylinder(radius=0.033, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, 1.260)),
        material=stainless,
        name="supported_spindle",
    )
    post.visual(
        Cylinder(radius=0.160, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.050)),
        material=galvanized,
        name="drip_shelf",
    )

    for i, angle in enumerate([0.0, math.pi * 0.5, math.pi, math.pi * 1.5]):
        x = math.cos(angle)
        y = math.sin(angle)
        _add_tube(
            post,
            (0.092 * x, 0.092 * y, 0.190),
            (0.170 * x, 0.170 * y, 0.112),
            radius=0.014,
            material=galvanized,
            name=f"flange_gusset_{i}",
        )

    for i in range(8):
        angle = i * math.tau / 8.0
        x = 0.225 * math.cos(angle)
        y = 0.225 * math.sin(angle)
        post.visual(
            Cylinder(radius=0.019, length=0.008),
            origin=Origin(xyz=(x, y, 0.118)),
            material=stainless,
            name=f"anchor_washer_{i}",
        )
        post.visual(
            Cylinder(radius=0.008, length=0.070),
            origin=Origin(xyz=(x, y, 0.145)),
            material=stainless,
            name=f"anchor_bolt_{i}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        _ring_mesh(
            outer_radius=0.126,
            inner_radius=0.045,
            height=0.026,
            z_center=0.013,
            name="lower_thrust_ring_mesh",
        ),
        material=stainless,
        name="lower_thrust_ring",
    )
    rotor.visual(
        _ring_mesh(
            outer_radius=0.103,
            inner_radius=0.043,
            height=0.320,
            z_center=0.186,
            name="rotating_sleeve_mesh",
        ),
        material=galvanized,
        name="rotating_sleeve",
    )
    rotor.visual(
        _ring_mesh(
            outer_radius=0.109,
            inner_radius=0.046,
            height=0.026,
            z_center=0.041,
            name="rotating_seal_band_mesh",
        ),
        material=rubber,
        name="rotating_seal_band",
    )
    rotor.visual(
        _ring_mesh(
            outer_radius=0.156,
            inner_radius=0.060,
            height=0.036,
            z_center=0.316,
            name="drip_skirt_mesh",
        ),
        material=galvanized,
        name="drip_skirt",
    )
    rotor.visual(
        Cylinder(radius=0.172, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.356)),
        material=galvanized,
        name="rain_cap",
    )
    rotor.visual(
        _ring_mesh(
            outer_radius=0.120,
            inner_radius=0.086,
            height=0.055,
            z_center=-0.220,
            name="lower_hub_ring_mesh",
        ),
        material=galvanized,
        name="lower_hub_ring",
    )

    for i in range(3):
        angle = i * math.tau / 3.0
        x = math.cos(angle)
        y = math.sin(angle)
        inner_upper = (0.092 * x, 0.092 * y, 0.130)
        outer_upper = (0.945 * x, 0.945 * y, 0.130)
        inner_lower = (0.112 * x, 0.112 * y, -0.220)
        outer_lower = (0.945 * x, 0.945 * y, -0.220)
        upright_bottom = (0.920 * x, 0.920 * y, -0.238)
        upright_top = (0.920 * x, 0.920 * y, 0.148)

        _add_tube(
            rotor,
            inner_upper,
            outer_upper,
            radius=0.026,
            material=galvanized,
            name=f"upper_arm_{i}",
        )
        _add_tube(
            rotor,
            inner_lower,
            outer_lower,
            radius=0.024,
            material=galvanized,
            name=f"lower_arm_{i}",
        )
        _add_tube(
            rotor,
            upright_bottom,
            upright_top,
            radius=0.024,
            material=galvanized,
            name=f"end_upright_{i}",
        )
        _add_tube(
            rotor,
            (0.300 * x, 0.300 * y, -0.220),
            (0.720 * x, 0.720 * y, 0.130),
            radius=0.011,
            material=stainless,
            name=f"diagonal_brace_{i}",
        )
        rotor.visual(
            Sphere(radius=0.038),
            origin=Origin(xyz=(0.970 * x, 0.970 * y, 0.130)),
            material=safety_yellow,
            name=f"upper_safety_cap_{i}",
        )
        rotor.visual(
            Sphere(radius=0.034),
            origin=Origin(xyz=(0.970 * x, 0.970 * y, -0.220)),
            material=safety_yellow,
            name=f"lower_safety_cap_{i}",
        )

    post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.30, length=1.50),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.95, length=0.65),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    model.articulation(
        "spindle_rotation",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("central_post")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("spindle_rotation")

    ctx.check("continuous_spindle_joint", joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("spindle_axis_vertical", tuple(joint.axis) == (0.0, 0.0, 1.0))
    ctx.expect_origin_distance(
        post,
        rotor,
        axes="xy",
        max_dist=0.001,
        name="rotor_centered_on_post",
    )
    ctx.expect_contact(
        rotor,
        post,
        elem_a="lower_thrust_ring",
        elem_b="upper_epdm_seal",
        contact_tol=0.002,
        name="bearing_stack_has_visible_support_contact",
    )

    rest_arm = ctx.part_element_world_aabb(rotor, elem="upper_arm_0")
    with ctx.pose({joint: math.pi * 0.5}):
        turned_arm = ctx.part_element_world_aabb(rotor, elem="upper_arm_0")

    if rest_arm is not None and turned_arm is not None:
        rest_min, rest_max = rest_arm
        turned_min, turned_max = turned_arm
        rest_center = (
            (float(rest_min[0]) + float(rest_max[0])) * 0.5,
            (float(rest_min[1]) + float(rest_max[1])) * 0.5,
        )
        turned_center = (
            (float(turned_min[0]) + float(turned_max[0])) * 0.5,
            (float(turned_min[1]) + float(turned_max[1])) * 0.5,
        )
        ctx.check(
            "radial_arm_rotates_about_spindle",
            rest_center[0] > 0.45 and abs(rest_center[1]) < 0.06 and turned_center[1] > 0.45,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )
    else:
        ctx.fail("radial_arm_aabb_available", "Expected upper_arm_0 AABBs in rest and turned poses.")

    return ctx.report()


object_model = build_object_model()
