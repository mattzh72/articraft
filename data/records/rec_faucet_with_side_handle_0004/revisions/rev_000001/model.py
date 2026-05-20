from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        (lower[0] + upper[0]) * 0.5,
        (lower[1] + upper[1]) * 0.5,
        (lower[2] + upper[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laundry_utility_faucet")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.05, 0.05, 0.06, 1.0))

    faucet_body = model.part("faucet_body")
    faucet_body.inertial = Inertial.from_geometry(
        Box((0.14, 0.24, 0.44)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.10, 0.22)),
    )
    faucet_body.visual(
        Cylinder(radius=0.047, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=gasket_black,
        name="deck_gasket",
    )
    faucet_body.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=chrome,
        name="base_flange",
    )
    faucet_body.visual(
        Cylinder(radius=0.032, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=chrome,
        name="pedestal",
    )
    faucet_body.visual(
        Cylinder(radius=0.026, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=chrome,
        name="riser_body",
    )
    faucet_body.visual(
        _mesh(
            "collar_band",
            TorusGeometry(radius=0.031, tube=0.004),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=satin_steel,
        name="collar_band",
    )
    faucet_body.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.143)),
        material=chrome,
        name="spout_socket",
    )
    faucet_body.visual(
        Cylinder(radius=0.013, length=0.028),
        origin=Origin(xyz=(0.039, 0.0, 0.104), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="handle_boss",
    )
    faucet_body.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.053, 0.0, 0.104), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="handle_collar",
    )

    swan_spout = tube_from_spline_points(
        [
            (0.0, 0.0, 0.146),
            (0.0, 0.012, 0.190),
            (0.0, 0.050, 0.298),
            (0.0, 0.048, 0.392),
            (0.0, 0.108, 0.430),
            (0.0, 0.178, 0.392),
            (0.0, 0.198, 0.350),
            (0.0, 0.198, 0.316),
        ],
        radius=0.016,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    faucet_body.visual(
        _mesh("swan_spout", swan_spout),
        material=chrome,
        name="swan_spout",
    )
    faucet_body.visual(
        Cylinder(radius=0.019, length=0.036),
        origin=Origin(xyz=(0.0, 0.198, 0.303)),
        material=chrome,
        name="nozzle_shell",
    )
    faucet_body.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.0, 0.198, 0.289)),
        material=shadow_black,
        name="aerator_opening",
    )

    side_handle = model.part("side_handle")
    side_handle.inertial = Inertial.from_geometry(
        Box((0.09, 0.08, 0.08)),
        mass=0.22,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
    )
    side_handle.visual(
        Cylinder(radius=0.0075, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="handle_stem",
    )
    side_handle.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="hub_cap",
    )
    side_handle.visual(
        Cylinder(radius=0.0065, length=0.024),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="hub_spindle",
    )
    side_handle.visual(
        _mesh(
            "gate_wheel",
            TorusGeometry(radius=0.031, tube=0.005).rotate_y(pi / 2.0).translate(0.040, 0.0, 0.0),
        ),
        material=chrome,
        name="gate_wheel",
    )
    side_handle.visual(
        Box((0.010, 0.004, 0.058)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=satin_steel,
        name="vertical_spoke",
    )
    side_handle.visual(
        Box((0.010, 0.058, 0.004)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=satin_steel,
        name="horizontal_spoke",
    )
    side_handle.visual(
        Cylinder(radius=0.0048, length=0.018),
        origin=Origin(xyz=(0.048, 0.0, 0.034), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="turn_grip",
    )

    model.articulation(
        "collar_handle_rotation",
        ArticulationType.REVOLUTE,
        parent=faucet_body,
        child=side_handle,
        origin=Origin(xyz=(0.062, 0.0, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=0.0,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    faucet_body = object_model.get_part("faucet_body")
    side_handle = object_model.get_part("side_handle")
    handle_joint = object_model.get_articulation("collar_handle_rotation")

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

    ctx.expect_contact(side_handle, faucet_body, name="handle_contacts_collar")
    ctx.expect_gap(
        side_handle,
        faucet_body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="handle_seats_flush_on_collar",
    )
    ctx.expect_overlap(
        side_handle,
        faucet_body,
        axes="yz",
        min_overlap=0.020,
        name="handle_overlaps_collar_footprint",
    )
    ctx.expect_gap(
        faucet_body,
        faucet_body,
        axis="z",
        max_gap=0.45,
        max_penetration=0.0,
        positive_elem="swan_spout",
        negative_elem="base_flange",
        name="swan_spout_rises_well_above_base",
    )

    limits = handle_joint.motion_limits
    ctx.check(
        "handle_joint_axis_is_sideways",
        tuple(handle_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1,0,0), got {handle_joint.axis}",
    )
    ctx.check(
        "handle_rotation_span_is_180_degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and isclose(limits.lower, 0.0, abs_tol=1e-6)
        and isclose(limits.upper - limits.lower, pi, abs_tol=1e-6),
        details=f"expected 180 degree span, got {limits}",
    )

    rest_grip_aabb = ctx.part_element_world_aabb(side_handle, elem="turn_grip")
    rest_grip_center = _aabb_center(rest_grip_aabb)
    ctx.check(
        "turn_grip_present",
        rest_grip_center is not None,
        details="missing handle grip visual for articulation readback",
    )

    with ctx.pose({handle_joint: pi}):
        ctx.expect_contact(side_handle, faucet_body, name="handle_stays_mounted_at_full_turn")
        ctx.expect_gap(
            side_handle,
            faucet_body,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            name="handle_stays_flush_at_full_turn",
        )
        open_grip_aabb = ctx.part_element_world_aabb(side_handle, elem="turn_grip")
        open_grip_center = _aabb_center(open_grip_aabb)
        ctx.check(
            "turn_grip_moves_half_turn",
            rest_grip_center is not None
            and open_grip_center is not None
            and open_grip_center[2] < rest_grip_center[2] - 0.05,
            details=f"rest={rest_grip_center}, open={open_grip_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
