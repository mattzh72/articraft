from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="basin_faucet")

    chrome = model.material("chrome", rgba=(0.83, 0.85, 0.88, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    dark_aerator = model.material("dark_aerator", rgba=(0.13, 0.13, 0.14, 1.0))

    body = model.part("faucet_body")
    body.inertial = Inertial.from_geometry(
        Box((0.18, 0.08, 0.22)),
        mass=1.6,
        origin=Origin(xyz=(0.06, 0.0, 0.10)),
    )
    body.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=chrome,
        name="deck_flange",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=chrome,
        name="stem_base",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=chrome,
        name="stem_shaft",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=chrome,
        name="upper_neck",
    )

    spout_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.108),
            (0.0, 0.0, 0.148),
            (0.012, 0.0, 0.176),
            (0.055, 0.0, 0.178),
            (0.102, 0.0, 0.165),
            (0.130, 0.0, 0.152),
        ],
        radius=0.0095,
        samples_per_segment=18,
        radial_segments=24,
        cap_ends=True,
    )
    body.visual(
        _save_mesh("faucet_spout", spout_geom),
        material=chrome,
        name="spout_tube",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.014),
        origin=Origin(xyz=(0.132, 0.0, 0.140)),
        material=satin_steel,
        name="outlet_nozzle",
    )
    body.visual(
        Cylinder(radius=0.0082, length=0.004),
        origin=Origin(xyz=(0.132, 0.0, 0.131)),
        material=dark_aerator,
        name="aerator",
    )

    handle = model.part("side_handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.040, 0.030, 0.030)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
    )
    handle.visual(
        Cylinder(radius=0.0125, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_collar",
    )
    handle.visual(
        Cylinder(radius=0.0155, length=0.014),
        origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_cap",
    )
    handle.visual(
        Cylinder(radius=0.0036, length=0.020),
        origin=Origin(xyz=(0.012, 0.020, 0.0)),
        material=chrome,
        name="grip_pin",
    )

    model.articulation(
        "handle_rotation",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.022, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.8, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("faucet_body")
    handle = object_model.get_part("side_handle")
    handle_joint = object_model.get_articulation("handle_rotation")

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

    ctx.expect_contact(handle, body, contact_tol=0.0005, name="handle_is_mounted_to_body")
    ctx.expect_overlap(handle, body, axes="xz", min_overlap=0.015, name="handle_overlaps_body_in_side_projection")

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("faucet_body_has_geometry", "The faucet body produced no measurable world AABB.")
    else:
        (min_x, min_y, min_z), (max_x, max_y, max_z) = body_aabb
        ctx.check(
            "faucet_has_sink_scale_proportions",
            0.12 <= (max_x - min_x) <= 0.18 and 0.15 <= (max_z - min_z) <= 0.22,
            (
                "Expected a basin-faucet-sized body envelope, "
                f"got dx={max_x - min_x:.4f}, dz={max_z - min_z:.4f}."
            ),
        )

    ctx.check(
        "handle_joint_axis_is_lateral",
        tuple(round(value, 6) for value in handle_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected handle axis (0, 1, 0), got {handle_joint.axis}.",
    )

    grip_rest = _aabb_center(ctx.part_element_world_aabb(handle, elem="grip_pin"))
    with ctx.pose({handle_joint: 0.75}):
        ctx.expect_contact(handle, body, contact_tol=0.0005, name="handle_stays_mounted_when_rotated")
        grip_open = _aabb_center(ctx.part_element_world_aabb(handle, elem="grip_pin"))
    if grip_rest is None or grip_open is None:
        ctx.fail("grip_pin_measurable", "Could not measure the handle grip pin AABB.")
    else:
        ctx.check(
            "handle_rotation_moves_grip_pin",
            abs(grip_open[2] - grip_rest[2]) >= 0.005,
            (
                "Expected the off-axis grip pin to move noticeably when the handle rotates, "
                f"got dz={abs(grip_open[2] - grip_rest[2]):.4f}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
