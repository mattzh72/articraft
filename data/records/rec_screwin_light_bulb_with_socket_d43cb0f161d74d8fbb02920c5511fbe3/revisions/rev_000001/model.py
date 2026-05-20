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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _x_cylinder(radius: float, length: float, center_x: float) -> MeshGeometry:
    return CylinderGeometry(radius=radius, height=length).rotate_y(math.pi / 2.0).translate(
        center_x,
        0.0,
        0.0,
    )


def _helical_path(
    *,
    start_x: float,
    end_x: float,
    radius: float,
    turns: float,
    phase: float = 0.0,
    samples: int = 96,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        theta = phase + (turns * math.tau * t)
        x = start_x + (end_x - start_x) * t
        points.append((x, radius * math.cos(theta), radius * math.sin(theta)))
    return points


def _build_socket_collar_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.0168, -0.022),
            (0.0178, -0.014),
            (0.0184, -0.004),
            (0.0186, 0.000),
        ],
        [
            (0.0142, -0.0215),
            (0.0143, -0.014),
            (0.0145, -0.004),
            (0.0146, 0.000),
        ],
        segments=52,
    )
    shell.rotate_y(math.pi / 2.0)
    inner_thread = tube_from_spline_points(
        _helical_path(
            start_x=-0.019,
            end_x=-0.002,
            radius=0.01405,
            turns=2.45,
            phase=0.35,
            samples=100,
        ),
        radius=0.00022,
        samples_per_segment=2,
        radial_segments=12,
    )
    shell.merge(inner_thread)
    return shell


def _build_socket_body_shell_mesh() -> MeshGeometry:
    body = LatheGeometry.from_shell_profiles(
        [
            (0.0215, -0.007),
            (0.0210, 0.000),
            (0.0198, 0.010),
        ],
        [
            (0.0170, -0.0065),
            (0.0168, 0.000),
            (0.0162, 0.010),
        ],
        segments=52,
    )
    body.rotate_y(math.pi / 2.0)
    return body


def _build_rotor_guide_mesh() -> MeshGeometry:
    ring = LatheGeometry.from_shell_profiles(
        [
            (0.0186, 0.0220),
            (0.0186, 0.0260),
        ],
        [
            (0.0152, 0.0220),
            (0.0152, 0.0260),
        ],
        segments=44,
    )
    ring.rotate_y(math.pi / 2.0)
    return ring


def _build_bulb_screw_mesh() -> MeshGeometry:
    screw_shell = _x_cylinder(radius=0.01235, length=0.024, center_x=0.0130)
    neck_collar = _x_cylinder(radius=0.0152, length=0.010, center_x=0.0290)
    thread = tube_from_spline_points(
        _helical_path(
            start_x=0.003,
            end_x=0.021,
            radius=0.0130,
            turns=2.45,
            phase=0.0,
            samples=100,
        ),
        radius=0.00075,
        samples_per_segment=2,
        radial_segments=12,
    )
    return _merge_geometries(screw_shell, neck_collar, thread)


def _build_bulb_globe_mesh() -> MeshGeometry:
    globe = LatheGeometry.from_shell_profiles(
        [
            (0.0152, 0.000),
            (0.0180, 0.008),
            (0.0215, 0.020),
            (0.0225, 0.033),
            (0.0200, 0.047),
            (0.0105, 0.059),
            (0.0000, 0.065),
        ],
        [
            (0.0133, 0.002),
            (0.0157, 0.009),
            (0.0191, 0.021),
            (0.0200, 0.033),
            (0.0178, 0.046),
            (0.0076, 0.057),
            (0.0000, 0.062),
        ],
        segments=56,
    )
    globe.rotate_y(math.pi / 2.0).translate(0.028, 0.0, 0.0)
    return globe


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_screw_bulb_lamp")

    matte_white = model.material("matte_white", rgba=(0.94, 0.95, 0.96, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.96, 0.94, 0.90, 0.72))
    satin_black = model.material("satin_black", rgba=(0.13, 0.14, 0.15, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.73, 0.75, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.73, 0.63, 0.36, 1.0))
    amber_insulator = model.material("amber_insulator", rgba=(0.82, 0.70, 0.43, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.22, 0.13, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_white,
        name="base_plate",
    )
    base.visual(
        Box((0.065, 0.13, 0.018)),
        origin=Origin(xyz=(-0.072, 0.0, 0.020)),
        material=matte_white,
        name="rear_plinth",
    )
    base.visual(
        Box((0.016, 0.008, 0.032)),
        origin=Origin(xyz=(-0.082, -0.028, 0.040)),
        material=dark_graphite,
        name="left_hinge_cheek",
    )
    base.visual(
        Box((0.016, 0.008, 0.032)),
        origin=Origin(xyz=(-0.082, 0.028, 0.040)),
        material=dark_graphite,
        name="right_hinge_cheek",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.22, 0.13, 0.052)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.009, length=0.048),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="hinge_barrel",
    )
    arm.visual(
        Box((0.072, 0.008, 0.016)),
        origin=Origin(xyz=(0.036, -0.024, 0.017)),
        material=satin_black,
        name="left_rail",
    )
    arm.visual(
        Box((0.072, 0.008, 0.016)),
        origin=Origin(xyz=(0.036, 0.024, 0.017)),
        material=satin_black,
        name="right_rail",
    )
    arm.visual(
        Box((0.020, 0.008, 0.022)),
        origin=Origin(xyz=(0.072, -0.024, 0.027)),
        material=satin_black,
        name="left_fork",
    )
    arm.visual(
        Box((0.020, 0.008, 0.022)),
        origin=Origin(xyz=(0.072, 0.024, 0.027)),
        material=satin_black,
        name="right_fork",
    )
    arm.visual(
        Box((0.026, 0.040, 0.008)),
        origin=Origin(xyz=(0.038, 0.0, 0.025)),
        material=dark_graphite,
        name="cross_bridge",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.096, 0.058, 0.050)),
        mass=0.35,
        origin=Origin(xyz=(0.046, 0.0, 0.026)),
    )

    socket_head = model.part("socket_head")
    socket_head.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="pivot_barrel",
    )
    socket_head.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="rear_cap",
    )
    socket_head.visual(
        mesh_from_geometry(_build_socket_body_shell_mesh(), "socket_body_shell"),
        origin=Origin(),
        material=satin_black,
        name="socket_body",
    )
    socket_head.visual(
        mesh_from_geometry(_build_socket_collar_mesh(), "socket_collar"),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=warm_brass,
        name="socket_collar",
    )
    socket_head.visual(
        Cylinder(radius=0.0052, length=0.004),
        origin=Origin(xyz=(0.0050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber_insulator,
        name="socket_insulator",
    )
    socket_head.visual(
        Cylinder(radius=0.0038, length=0.0016),
        origin=Origin(xyz=(0.0032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="socket_contact",
    )
    socket_head.inertial = Inertial.from_geometry(
        Box((0.055, 0.048, 0.048)),
        mass=0.20,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
    )

    bulb_rotor = model.part("bulb_rotor")
    bulb_rotor.visual(
        mesh_from_geometry(_build_rotor_guide_mesh(), "bulb_rotor_guide"),
        material=brushed_aluminum,
        name="guide_carrier",
    )
    bulb_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.034),
        mass=0.05,
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_geometry(_build_bulb_screw_mesh(), "bulb_screw_base"),
        material=brushed_aluminum,
        name="screw_base",
    )
    bulb.visual(
        mesh_from_geometry(_build_bulb_globe_mesh(), "bulb_globe"),
        material=warm_glass,
        name="glass_globe",
    )
    bulb.visual(
        Cylinder(radius=0.0032, length=0.002),
        origin=Origin(xyz=(0.0010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="base_tip",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0245, length=0.101),
        mass=0.08,
        origin=Origin(xyz=(0.0505, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(-0.082, 0.0, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "arm_to_socket",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=socket_head,
        origin=Origin(xyz=(0.074, 0.0, 0.036)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "socket_to_bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket_head,
        child=bulb_rotor,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    model.articulation(
        "bulb_spin_to_lift",
        ArticulationType.PRISMATIC,
        parent=bulb_rotor,
        child=bulb,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.03,
            lower=0.0,
            upper=0.012,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    socket_head = object_model.get_part("socket_head")
    bulb_rotor = object_model.get_part("bulb_rotor")
    bulb = object_model.get_part("bulb")

    base_to_arm = object_model.get_articulation("base_to_arm")
    arm_to_socket = object_model.get_articulation("arm_to_socket")
    socket_to_bulb_spin = object_model.get_articulation("socket_to_bulb_spin")
    bulb_spin_to_lift = object_model.get_articulation("bulb_spin_to_lift")

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

    ctx.check(
        "joint_axes_match_mechanism",
        base_to_arm.axis == (0.0, -1.0, 0.0)
        and arm_to_socket.axis == (0.0, -1.0, 0.0)
        and socket_to_bulb_spin.axis == (1.0, 0.0, 0.0)
        and bulb_spin_to_lift.axis == (1.0, 0.0, 0.0),
        "Expected fold hinges around -Y and screw/lift motion coaxial on +X.",
    )
    ctx.check(
        "articulation_types_match_design",
        socket_to_bulb_spin.joint_type == ArticulationType.CONTINUOUS
        and bulb_spin_to_lift.joint_type == ArticulationType.PRISMATIC,
        "Bulb must spin continuously in the socket and lift along the same axis.",
    )

    ctx.expect_contact(base, arm, name="base_and_arm_supported_at_hinge")
    ctx.expect_contact(arm, socket_head, name="arm_and_socket_supported_at_pivot")
    ctx.expect_contact(
        socket_head,
        bulb_rotor,
        elem_a="socket_collar",
        elem_b="guide_carrier",
        name="socket_and_rotor_supported_at_lip",
    )
    ctx.expect_contact(
        bulb_rotor,
        bulb,
        elem_a="guide_carrier",
        elem_b="screw_base",
        name="bulb_supported_by_internal_guide",
    )

    with ctx.pose({base_to_arm: 0.0, arm_to_socket: 0.0, bulb_spin_to_lift: 0.0}):
        ctx.expect_within(
            bulb,
            base,
            axes="xy",
            margin=0.004,
            name="stowed_bulb_within_base_footprint",
        )
        ctx.expect_gap(
            bulb,
            base,
            axis="z",
            min_gap=0.020,
            max_gap=0.040,
            negative_elem="base_plate",
            name="stowed_bulb_low_profile_clearance",
        )
        ctx.expect_gap(
            bulb,
            base,
            axis="x",
            min_gap=0.030,
            negative_elem="rear_plinth",
            name="stowed_bulb_clears_rear_plinth",
        )
        ctx.expect_within(
            bulb,
            socket_head,
            axes="yz",
            inner_elem="screw_base",
            outer_elem="socket_collar",
            margin=0.002,
            name="screw_base_coaxially_within_socket_collar",
        )
        ctx.expect_contact(
            bulb,
            socket_head,
            elem_a="base_tip",
            elem_b="socket_contact",
            name="bulb_tip_reaches_socket_contact",
        )

    with ctx.pose(
        {
            base_to_arm: math.radians(60.0),
            arm_to_socket: math.radians(30.0),
            bulb_spin_to_lift: 0.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_has_no_overlaps")
        ctx.expect_gap(
            bulb,
            base,
            axis="z",
            min_gap=0.060,
            negative_elem="base_plate",
            name="open_pose_bulb_clears_base_height",
        )

    with ctx.pose({base_to_arm: 0.0, arm_to_socket: 0.0, bulb_spin_to_lift: 0.0}):
        seated = ctx.part_world_position(bulb)
    with ctx.pose({base_to_arm: 0.0, arm_to_socket: 0.0, bulb_spin_to_lift: 0.010}):
        lifted = ctx.part_world_position(bulb)
    ctx.check(
        "bulb_lift_moves_outward_along_socket_axis",
        seated is not None and lifted is not None and lifted[0] > seated[0] + 0.008,
        f"Expected lifted bulb x-position to increase in stowed pose, got seated={seated}, lifted={lifted}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
