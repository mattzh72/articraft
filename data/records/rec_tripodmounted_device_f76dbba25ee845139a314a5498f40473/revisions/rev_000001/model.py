from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _axis_rpy(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotation that points a cylinder's local +Z axis along ``direction``."""
    x, y, z = direction
    h = math.hypot(x, y)
    yaw = math.atan2(y, x) if h > 1.0e-9 else 0.0
    pitch = math.atan2(h, z)
    return (0.0, pitch, yaw)


def _polar(radius: float, theta: float, z: float = 0.0) -> tuple[float, float, float]:
    return (radius * math.cos(theta), radius * math.sin(theta), z)


def _tube_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 48,
):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, height)],
        [(inner_radius, height), (inner_radius, 0.0)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _add_radial_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    radius: float,
    theta: float,
    z: float,
    tangent_offset: float = 0.0,
    material=None,
) -> None:
    radial = (math.cos(theta), math.sin(theta), 0.0)
    tangent = (-math.sin(theta), math.cos(theta), 0.0)
    xyz = (
        radius * radial[0] + tangent_offset * tangent[0],
        radius * radial[1] + tangent_offset * tangent[1],
        z,
    )
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=(0.0, 0.0, theta)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_tripod_measuring_device")

    black = model.material("mat_black", rgba=(0.015, 0.017, 0.018, 1.0))
    dark = model.material("mat_dark", rgba=(0.06, 0.065, 0.07, 1.0))
    rubber = model.material("mat_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    aluminum = model.material("mat_aluminum", rgba=(0.66, 0.68, 0.67, 1.0))
    steel = model.material("mat_steel", rgba=(0.34, 0.35, 0.35, 1.0))
    yellow = model.material("mat_survey_yellow", rgba=(0.95, 0.68, 0.10, 1.0))
    glass = model.material("mat_blue_glass", rgba=(0.08, 0.24, 0.34, 0.85))
    screen = model.material("mat_screen", rgba=(0.015, 0.026, 0.032, 1.0))

    # Root tripod crown and the hollow outer sleeve that carries the sliding mast.
    body = model.part("tripod_body")
    body.visual(
        mesh_from_geometry(
            _tube_geometry(outer_radius=0.058, inner_radius=0.038, height=0.55),
            "outer_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=dark,
        name="outer_sleeve",
    )
    body.visual(
        mesh_from_geometry(
            _tube_geometry(outer_radius=0.098, inner_radius=0.056, height=0.125),
            "crown_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.7425)),
        material=black,
        name="crown_collar",
    )
    body.visual(
        mesh_from_geometry(
            _tube_geometry(outer_radius=0.074, inner_radius=0.056, height=0.070),
            "mast_lock_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.110)),
        material=black,
        name="mast_lock_collar",
    )
    body.visual(
        mesh_from_geometry(
            _tube_geometry(outer_radius=0.116, inner_radius=0.056, height=0.030),
            "lower_crown_flange",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        material=black,
        name="lower_crown_flange",
    )

    hinge_radius = 0.145
    hinge_z = 0.805
    for i, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        _add_radial_box(
            body,
            name=f"hinge_web_{i}",
            size=(0.060, 0.064, 0.046),
            radius=0.085,
            theta=theta,
            z=hinge_z,
            material=black,
        )
        for side, offset in (("a", -0.036), ("b", 0.036)):
            _add_radial_box(
                body,
                name=f"hinge_bridge_{i}_{side}",
                size=(0.040, 0.014, 0.046),
                radius=0.124,
                theta=theta,
                z=hinge_z,
                tangent_offset=offset,
                material=black,
            )
        for side, offset in (("a", -0.036), ("b", 0.036)):
            _add_radial_box(
                body,
                name=f"hinge_cheek_{i}_{side}",
                size=(0.046, 0.014, 0.094),
                radius=hinge_radius,
                theta=theta,
                z=hinge_z,
                tangent_offset=offset,
                material=black,
            )
        tangent = (-math.sin(theta), math.cos(theta), 0.0)
        body.visual(
            Cylinder(radius=0.0075, length=0.095),
            origin=Origin(
                xyz=_polar(hinge_radius, theta, hinge_z),
                rpy=_axis_rpy(tangent),
            ),
            material=steel,
            name=f"leg_pin_{i}",
        )

    # Three hinged folding legs.  Each leg frame lives at its crown hinge pin:
    # local +X points radially outward and local +Y lies along the hinge axis.
    leg_length = 1.105
    leg_dx = 0.735
    leg_dz = -0.825
    leg_beta = math.atan2(leg_dx, leg_dz)
    for i, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{i}")
        leg.visual(
            Cylinder(radius=0.024, length=0.054),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hinge_knuckle",
        )
        leg.visual(
            Cylinder(radius=0.020, length=0.76),
            origin=Origin(
                xyz=(leg_dx * 0.375, 0.0, leg_dz * 0.375),
                rpy=(0.0, leg_beta, 0.0),
            ),
            material=aluminum,
            name="upper_tube",
        )
        leg.visual(
            Box((0.075, 0.030, 0.030)),
            origin=Origin(xyz=(0.055, 0.0, -0.006)),
            material=steel,
            name="leg_neck",
        )
        leg.visual(
            Cylinder(radius=0.017, length=0.53),
            origin=Origin(
                xyz=(leg_dx * 0.75, 0.0, leg_dz * 0.75),
                rpy=(0.0, leg_beta, 0.0),
            ),
            material=aluminum,
            name="lower_tube",
        )
        leg.visual(
            Box((0.095, 0.030, 0.018)),
            origin=Origin(
                xyz=(leg_dx + 0.018, 0.0, leg_dz + 0.018),
                rpy=(0.0, 0.0, 0.0),
            ),
            material=rubber,
            name="foot_pad",
        )
        leg.visual(
            Sphere(radius=0.023),
            origin=Origin(xyz=(leg_dx, 0.0, leg_dz)),
            material=rubber,
            name="foot_socket",
        )
        model.articulation(
            f"body_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=leg,
            origin=Origin(xyz=_polar(hinge_radius, theta, hinge_z), rpy=(0.0, 0.0, theta)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(lower=-0.22, upper=0.72, effort=35.0, velocity=1.3),
        )

    # The telescoping mast is a single sliding assembly with retained insertion.
    mast = model.part("center_mast")
    mast.visual(
        Cylinder(radius=0.026, length=1.15),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=aluminum,
        name="inner_mast",
    )
    mast.visual(
        Cylinder(radius=0.032, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=steel,
        name="sliding_bushing",
    )
    mast.visual(
        Cylinder(radius=0.064, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=black,
        name="mast_stop_collar",
    )
    mast.visual(
        Cylinder(radius=0.019, length=0.53),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=steel,
        name="upper_mast",
    )
    mast.visual(
        Box((0.010, 0.004, 0.62)),
        origin=Origin(xyz=(0.0265, 0.0, 0.255)),
        material=steel,
        name="height_scale",
    )
    model.articulation(
        "body_to_mast",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.35, effort=90.0, velocity=0.20),
    )

    # Pan head with a compact yoke for the tilt hinge.
    head = model.part("pan_head")
    head.visual(
        Cylinder(radius=0.075, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=black,
        name="pan_bearing",
    )
    head.visual(
        Cylinder(radius=0.038, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark,
        name="head_post",
    )
    head.visual(
        Box((0.125, 0.255, 0.035)),
        origin=Origin(xyz=(0.045, 0.0, 0.115)),
        material=black,
        name="yoke_base",
    )
    for side, y in (("a", -0.118), ("b", 0.118)):
        head.visual(
            Box((0.056, 0.020, 0.155)),
            origin=Origin(xyz=(0.060, y, 0.207)),
            material=black,
            name=f"tilt_cheek_{side}",
        )
    head.visual(
        Cylinder(radius=0.010, length=0.265),
        origin=Origin(xyz=(0.060, 0.0, 0.215), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_pin",
    )
    model.articulation(
        "mast_to_head",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.970)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=10.0, velocity=1.0),
    )

    # Compact site measuring device: rounded survey body, trunnion, lens, and display.
    device = model.part("device")
    body_shell = (
        cq.Workplane("XY")
        .box(0.270, 0.172, 0.160)
        .edges()
        .fillet(0.018)
        .translate((0.155, 0.0, 0.0))
    )
    device.visual(
        mesh_from_cadquery(body_shell, "device_body_shell", tolerance=0.0015),
        material=yellow,
        name="body_shell",
    )
    device.visual(
        Cylinder(radius=0.027, length=0.216),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_barrel",
    )
    device.visual(
        Cylinder(radius=0.046, length=0.070),
        origin=Origin(xyz=(0.275, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lens_housing",
    )
    device.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.315, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    device.visual(
        Box((0.004, 0.105, 0.070)),
        origin=Origin(xyz=(0.018, 0.0, 0.006)),
        material=screen,
        name="rear_screen",
    )
    device.visual(
        Box((0.170, 0.140, 0.024)),
        origin=Origin(xyz=(0.098, 0.0, 0.092)),
        material=black,
        name="top_handle",
    )
    device.visual(
        Box((0.030, 0.032, 0.060)),
        origin=Origin(xyz=(0.035, -0.052, 0.065)),
        material=black,
        name="handle_post_0",
    )
    device.visual(
        Box((0.030, 0.032, 0.060)),
        origin=Origin(xyz=(0.160, -0.052, 0.065)),
        material=black,
        name="handle_post_1",
    )
    device.visual(
        Box((0.030, 0.032, 0.060)),
        origin=Origin(xyz=(0.035, 0.052, 0.065)),
        material=black,
        name="handle_post_2",
    )
    device.visual(
        Box((0.030, 0.032, 0.060)),
        origin=Origin(xyz=(0.160, 0.052, 0.065)),
        material=black,
        name="handle_post_3",
    )
    model.articulation(
        "head_to_device",
        ArticulationType.REVOLUTE,
        parent=head,
        child=device,
        origin=Origin(xyz=(0.060, 0.0, 0.215)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.55, effort=8.0, velocity=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("tripod_body")
    mast = object_model.get_part("center_mast")
    head = object_model.get_part("pan_head")
    device = object_model.get_part("device")

    mast_slide = object_model.get_articulation("body_to_mast")
    pan_joint = object_model.get_articulation("mast_to_head")
    tilt_joint = object_model.get_articulation("head_to_device")

    ctx.check(
        "primary mechanism count",
        len(object_model.articulations) == 6,
        details=f"expected 3 leg hinges, mast slide, pan, and tilt; got {len(object_model.articulations)}",
    )

    # Captured hinge pins are intentionally represented by a visible steel pin
    # nested through each leg knuckle.
    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            body,
            leg,
            elem_a=f"leg_pin_{i}",
            elem_b="hinge_knuckle",
            reason="The folding-leg hinge pin is intentionally captured inside the leg knuckle.",
        )
        ctx.expect_overlap(
            leg,
            body,
            axes="z",
            elem_a="hinge_knuckle",
            elem_b=f"leg_pin_{i}",
            min_overlap=0.012,
            name=f"leg {i} knuckle surrounds hinge pin",
        )

    ctx.allow_overlap(
        head,
        device,
        elem_a="tilt_pin",
        elem_b="trunnion_barrel",
        reason="The tilt axle is intentionally captured inside the device trunnion barrel.",
    )
    ctx.expect_overlap(
        device,
        head,
        axes="z",
        elem_a="trunnion_barrel",
        elem_b="tilt_pin",
        min_overlap=0.014,
        name="device trunnion surrounds tilt pin",
    )

    ctx.expect_within(
        mast,
        body,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="center mast is centered in tripod sleeve",
    )
    ctx.expect_overlap(
        mast,
        body,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.40,
        name="collapsed mast retains sleeve insertion",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.35}):
        ctx.expect_within(
            mast,
            body,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.0,
            name="extended mast stays centered in tripod sleeve",
        )
        ctx.expect_overlap(
            mast,
            body,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.09,
            name="extended mast keeps retained insertion",
        )
        raised_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast extends upward",
        rest_mast_pos is not None
        and raised_mast_pos is not None
        and raised_mast_pos[2] > rest_mast_pos[2] + 0.30,
        details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
    )

    leg_0 = object_model.get_part("leg_0")
    leg_0_hinge = object_model.get_articulation("body_to_leg_0")
    foot_rest = ctx.part_element_world_aabb(leg_0, elem="foot_pad")
    rest_foot_x = None if foot_rest is None else (foot_rest[0][0] + foot_rest[1][0]) * 0.5
    with ctx.pose({leg_0_hinge: 0.60}):
        foot_folded = ctx.part_element_world_aabb(leg_0, elem="foot_pad")
        folded_foot_x = None if foot_folded is None else (foot_folded[0][0] + foot_folded[1][0]) * 0.5
    ctx.check(
        "folding leg swings toward tripod body",
        rest_foot_x is not None and folded_foot_x is not None and folded_foot_x < rest_foot_x - 0.30,
        details=f"rest_foot_x={rest_foot_x}, folded_foot_x={folded_foot_x}",
    )

    front_rest = ctx.part_element_world_aabb(device, elem="front_glass")
    rest_lens_z = None if front_rest is None else (front_rest[0][2] + front_rest[1][2]) * 0.5
    with ctx.pose({pan_joint: 1.0, tilt_joint: 0.42}):
        front_tilted = ctx.part_element_world_aabb(device, elem="front_glass")
        tilted_lens_z = None if front_tilted is None else (front_tilted[0][2] + front_tilted[1][2]) * 0.5
    ctx.check(
        "device tilts on horizontal head hinge",
        rest_lens_z is not None and tilted_lens_z is not None and tilted_lens_z < rest_lens_z - 0.05,
        details=f"rest_lens_z={rest_lens_z}, tilted_lens_z={tilted_lens_z}",
    )

    return ctx.report()


object_model = build_object_model()
