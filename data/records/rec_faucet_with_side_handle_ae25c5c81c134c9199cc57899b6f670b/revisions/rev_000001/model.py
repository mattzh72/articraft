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
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pull_down_kitchen_faucet")

    steel = model.material("brushed_steel", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")

    base_and_spout = CylinderGeometry(radius=0.032, height=0.008).translate(0.0, 0.0, 0.004)
    base_and_spout.merge(CylinderGeometry(radius=0.023, height=0.012).translate(0.0, 0.0, 0.012))
    base_and_spout.merge(CylinderGeometry(radius=0.018, height=0.066).translate(0.0, 0.0, 0.041))
    base_and_spout.merge(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.052),
                (0.0, 0.0, 0.170),
                (0.025, 0.0, 0.300),
                (0.105, 0.0, 0.430),
                (0.195, 0.0, 0.390),
                (0.220, 0.0, 0.275),
                (0.220, 0.0, 0.235),
            ],
            radius=0.0125,
            samples_per_segment=18,
            radial_segments=22,
        )
    )
    body.visual(
        mesh_from_geometry(base_and_spout, "faucet_body_shell"),
        material=steel,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.220, 0.0, 0.220)),
        material=steel,
        name="tip_socket",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.002, 0.026, 0.062), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_mount",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.280, 0.090, 0.450)),
        mass=4.8,
        origin=Origin(xyz=(0.110, 0.0, 0.225)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0075, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_hub",
    )
    lever_bar = tube_from_spline_points(
        [
            (0.0, 0.010, 0.000),
            (0.0, 0.026, 0.006),
            (0.0, 0.046, 0.018),
            (0.0, 0.066, 0.030),
            (0.0, 0.082, 0.035),
        ],
        radius=0.0042,
        samples_per_segment=16,
        radial_segments=18,
    )
    handle.visual(
        mesh_from_geometry(lever_bar, "faucet_handle_lever"),
        material=steel,
        name="handle_shell",
    )
    handle.visual(
        mesh_from_geometry(SphereGeometry(0.0055).translate(0.0, 0.082, 0.035), "faucet_handle_tip"),
        material=steel,
        name="handle_tip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.020, 0.090, 0.045)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.045, 0.018)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.002, 0.034, 0.062)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.20,
            upper=0.78,
        ),
    )

    spray_head = model.part("spray_head")
    spray_head.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=steel,
        name="head_collar",
    )
    spray_head_shell = LatheGeometry(
        [
            (0.0148, -0.010),
            (0.0148, -0.020),
            (0.0138, -0.030),
            (0.0155, -0.038),
            (0.0142, -0.052),
            (0.0123, -0.066),
            (0.0110, -0.078),
            (0.0124, -0.082),
        ],
        segments=40,
    )
    spray_head.visual(
        mesh_from_geometry(spray_head_shell, "spray_head_shell"),
        material=steel,
        name="head_shell",
    )
    spray_head.visual(
        Cylinder(radius=0.0152, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=black_rubber,
        name="grip_ring",
    )
    spray_head.visual(
        Cylinder(radius=0.0128, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        material=black_rubber,
        name="spray_face",
    )
    spray_head.visual(
        Cylinder(radius=0.0092, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.073)),
        material=dark_trim,
        name="nozzle_insert",
    )
    spray_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.085),
        mass=0.52,
        origin=Origin(xyz=(0.0, 0.0, -0.0425)),
    )

    model.articulation(
        "body_to_spray_head",
        ArticulationType.PRISMATIC,
        parent=body,
        child=spray_head,
        origin=Origin(xyz=(0.220, 0.0, 0.205)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=0.45,
            lower=0.0,
            upper=0.160,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    spray_head = object_model.get_part("spray_head")

    handle_joint = object_model.get_articulation("body_to_handle")
    spray_joint = object_model.get_articulation("body_to_spray_head")

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

    ctx.expect_contact(handle, body, elem_a="pivot_hub", elem_b="handle_mount")
    ctx.expect_contact(spray_head, body, elem_a="head_collar", elem_b="tip_socket")
    ctx.expect_overlap(spray_head, body, axes="xy", min_overlap=0.020)
    ctx.expect_origin_distance(handle, body, axes="y", min_dist=0.025, max_dist=0.050)

    ctx.check(
        "handle_joint_axis_is_front_to_back",
        tuple(round(v, 6) for v in handle_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )
    ctx.check(
        "spray_joint_axis_is_vertical_pull_down",
        tuple(round(v, 6) for v in spray_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={spray_joint.axis}",
    )

    handle_rest = ctx.part_element_world_aabb(handle, elem="handle_shell")
    spray_rest = ctx.part_world_position(spray_head)
    assert handle_rest is not None
    assert spray_rest is not None

    with ctx.pose({handle_joint: 0.65}):
        handle_open = ctx.part_element_world_aabb(handle, elem="handle_shell")
        assert handle_open is not None
        ctx.check(
            "handle_lifts_when_opened",
            handle_open[1][2] > handle_rest[1][2] + 0.020,
            details=f"rest_max_z={handle_rest[1][2]:.4f}, open_max_z={handle_open[1][2]:.4f}",
        )

    with ctx.pose({spray_joint: 0.140}):
        spray_extended = ctx.part_world_position(spray_head)
        assert spray_extended is not None
        ctx.check(
            "spray_head_extends_downward",
            spray_extended[2] < spray_rest[2] - 0.120,
            details=f"rest_z={spray_rest[2]:.4f}, extended_z={spray_extended[2]:.4f}",
        )
        ctx.expect_gap(
            body,
            spray_head,
            axis="z",
            positive_elem="tip_socket",
            negative_elem="head_collar",
            min_gap=0.120,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
