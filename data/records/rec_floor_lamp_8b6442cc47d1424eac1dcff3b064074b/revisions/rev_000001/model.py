from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


LEG_SPECS: tuple[tuple[str, float], ...] = (
    ("front_leg", math.pi / 2.0),
    ("rear_left_leg", 7.0 * math.pi / 6.0),
    ("rear_right_leg", 11.0 * math.pi / 6.0),
)

LEG_SPLAY = math.radians(24.0)
LEG_WOOD_LENGTH = 0.76
LEG_SHAFT_OFFSET = 0.028
LEG_FERRULE_LENGTH = 0.040
LEG_FERRULE_OFFSET = 0.020
HUB_HINGE_RADIUS = 0.072


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rotate_xy(x_local: float, y_local: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (x_local * c - y_local * s, x_local * s + y_local * c)


def _axis_offset(distance: float, angle_from_vertical: float) -> tuple[float, float, float]:
    return (
        distance * math.sin(angle_from_vertical),
        0.0,
        -distance * math.cos(angle_from_vertical),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def _radial_xy(point) -> float | None:
    if point is None:
        return None
    return math.hypot(point[0], point[1])


def _build_leg_mesh():
    profile = [
        (0.0, 0.0),
        (0.019, 0.0),
        (0.018, 0.14),
        (0.015, 0.42),
        (0.0115, LEG_WOOD_LENGTH),
        (0.0, LEG_WOOD_LENGTH),
    ]
    return _save_mesh("tripod_floor_lamp_leg", LatheGeometry(profile, segments=40))


def _build_shade_shell_mesh():
    outer_profile = [
        (0.226, 0.680),
        (0.214, 0.760),
        (0.199, 0.845),
        (0.185, 0.925),
        (0.172, 0.995),
    ]
    inner_profile = [
        (0.217, 0.684),
        (0.205, 0.760),
        (0.190, 0.844),
        (0.177, 0.922),
        (0.163, 0.991),
    ]
    return _save_mesh(
        "tripod_floor_lamp_shade_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _build_shade_frame_mesh():
    frame = TorusGeometry(
        radius=0.158,
        tube=0.005,
        radial_segments=18,
        tubular_segments=54,
    ).translate(0.0, 0.0, 0.988)
    frame.merge(
        TorusGeometry(
            radius=0.209,
            tube=0.009,
            radial_segments=18,
            tubular_segments=60,
        ).translate(0.0, 0.0, 0.704)
    )

    for _, angle in LEG_SPECS:
        c = math.cos(angle)
        s = math.sin(angle)
        frame.merge(
            tube_from_spline_points(
                [
                    (0.012 * c, 0.012 * s, 0.824),
                    (0.092 * c, 0.092 * s, 0.764),
                    (0.204 * c, 0.204 * s, 0.710),
                    (0.159 * c, 0.159 * s, 0.986),
                ],
                radius=0.004,
                samples_per_segment=12,
                radial_segments=14,
                cap_ends=True,
            )
        )

    return _save_mesh("tripod_floor_lamp_shade_frame", frame)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="midcentury_tripod_floor_lamp")

    walnut = model.material("walnut", rgba=(0.42, 0.28, 0.18, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.64, 0.54, 0.34, 1.0))
    linen = model.material("linen", rgba=(0.93, 0.90, 0.82, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.11, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.98, 0.93, 0.76, 0.42))

    leg_mesh = _build_leg_mesh()
    shade_shell_mesh = _build_shade_shell_mesh()
    shade_frame_mesh = _build_shade_frame_mesh()

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.055, length=0.10),
        material=aged_brass,
        name="hub_core",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=aged_brass,
        name="hub_collar",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
        material=walnut,
        name="column",
    )
    body.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
        material=aged_brass,
        name="socket_collar",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.880)),
        material=aged_brass,
        name="socket_tube",
    )
    body.visual(
        Sphere(radius=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        material=warm_glass,
        name="bulb",
    )
    body.visual(
        shade_shell_mesh,
        material=linen,
        name="shade_shell",
    )
    body.visual(
        shade_frame_mesh,
        material=aged_brass,
        name="shade_frame",
    )

    for index, (_, yaw) in enumerate(LEG_SPECS):
        ear_x, ear_y = _rotate_xy(0.047, -0.006, yaw)
        body.visual(
            Box((0.050, 0.012, 0.028)),
            origin=Origin(xyz=(ear_x, ear_y, 0.0), rpy=(0.0, 0.0, yaw)),
            material=aged_brass,
            name=f"hub_ear_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.62, 0.62, 1.10)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )

    leg_pitch = math.pi - LEG_SPLAY
    wood_origin = _axis_offset(0.036, LEG_SPLAY)
    ferrule_center = _axis_offset(LEG_FERRULE_OFFSET + (LEG_FERRULE_LENGTH * 0.5), LEG_SPLAY)
    foot_center = _axis_offset(0.036 + LEG_WOOD_LENGTH + 0.006, LEG_SPLAY)

    for leg_name, yaw in LEG_SPECS:
        leg = model.part(leg_name)
        leg.visual(
            Box((0.032, 0.010, 0.024)),
            origin=Origin(xyz=(0.016, 0.005, -0.010)),
            material=aged_brass,
            name="hinge_block",
        )
        leg.visual(
            Cylinder(radius=0.0185, length=LEG_FERRULE_LENGTH),
            origin=Origin(xyz=ferrule_center, rpy=(0.0, leg_pitch, 0.0)),
            material=aged_brass,
            name="leg_ferrule",
        )
        leg.visual(
            leg_mesh,
            origin=Origin(xyz=wood_origin, rpy=(0.0, leg_pitch, 0.0)),
            material=walnut,
            name="wood_shaft",
        )
        leg.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=foot_center, rpy=(0.0, leg_pitch, 0.0)),
            material=rubber,
            name="foot_cap",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.060, 0.050, 0.84)),
            mass=0.8,
            origin=Origin(xyz=_axis_offset(0.036 + (LEG_WOOD_LENGTH * 0.5), LEG_SPLAY), rpy=(0.0, leg_pitch, 0.0)),
        )

        model.articulation(
            f"body_to_{leg_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=leg,
            origin=Origin(
                xyz=(HUB_HINGE_RADIUS * math.cos(yaw), HUB_HINGE_RADIUS * math.sin(yaw), 0.0),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=1.0,
                lower=-math.radians(14.0),
                upper=math.radians(10.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    legs = [object_model.get_part(name) for name, _ in LEG_SPECS]
    leg_joints = [object_model.get_articulation(f"body_to_{name}") for name, _ in LEG_SPECS]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for index, leg in enumerate(legs):
        ctx.expect_contact(
            body,
            leg,
            elem_a=f"hub_ear_{index}",
            elem_b="hinge_block",
            name=f"{leg.name} hinge block touches hub bracket",
        )

    hub_position = ctx.part_world_position(body)
    for leg in legs:
        foot_center = _aabb_center(ctx.part_element_world_aabb(leg, elem="foot_cap"))
        radial = _radial_xy(foot_center)
        ctx.check(
            f"{leg.name} foot reaches floor footprint",
            foot_center is not None
            and hub_position is not None
            and foot_center[2] < hub_position[2] - 0.67
            and radial is not None
            and radial > 0.30,
            details=f"hub={hub_position}, foot={foot_center}, radial={radial}",
        )

    shade_aabb = ctx.part_element_world_aabb(body, elem="shade_shell")
    ctx.check(
        "shade sits high above the tripod hub",
        shade_aabb is not None
        and hub_position is not None
        and shade_aabb[0][2] > hub_position[2] + 0.66
        and (shade_aabb[1][2] - shade_aabb[0][2]) > 0.28,
        details=f"hub={hub_position}, shade={shade_aabb}",
    )

    front_leg = object_model.get_part("front_leg")
    front_joint = object_model.get_articulation("body_to_front_leg")
    front_rest_center = _aabb_center(ctx.part_element_world_aabb(front_leg, elem="foot_cap"))
    front_rest_radius = _radial_xy(front_rest_center)
    with ctx.pose({front_joint: front_joint.motion_limits.lower}):
        front_folded_center = _aabb_center(ctx.part_element_world_aabb(front_leg, elem="foot_cap"))
    front_folded_radius = _radial_xy(front_folded_center)
    ctx.check(
        "front leg folds inward on its hinge",
        front_rest_radius is not None
        and front_folded_radius is not None
        and front_folded_radius < front_rest_radius - 0.10,
        details=f"rest={front_rest_center}, folded={front_folded_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
