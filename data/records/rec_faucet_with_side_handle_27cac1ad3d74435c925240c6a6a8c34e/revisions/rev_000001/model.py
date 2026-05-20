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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_scrub_sink_wrist_blade_faucet")

    body_width = 0.26
    body_depth = 0.075
    body_height = 0.085

    plate_width = 0.30
    plate_depth = 0.004
    plate_height = 0.11

    bonnet_radius = 0.023
    bonnet_length = 0.024
    bonnet_center_x = (body_width * 0.5) + (bonnet_length * 0.5)

    hub_radius = 0.021
    hub_length = 0.018
    handle_joint_x = (body_width * 0.5) + bonnet_length + (hub_length * 0.5)

    body_metal = model.material("body_metal", rgba=(0.80, 0.82, 0.84, 1.0))
    plate_metal = model.material("plate_metal", rgba=(0.74, 0.76, 0.78, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.86, 0.87, 0.88, 1.0))

    body = model.part("body")

    housing_geom = ExtrudeGeometry.centered(
        rounded_rect_profile(body_width, body_height, radius=0.012),
        body_depth,
    )
    housing_geom.rotate_x(pi / 2.0)
    body.visual(
        mesh_from_geometry(housing_geom, "housing_shell"),
        material=body_metal,
        name="housing",
    )

    plate_geom = ExtrudeGeometry.centered(
        rounded_rect_profile(plate_width, plate_height, radius=0.010),
        plate_depth,
    )
    plate_geom.rotate_x(pi / 2.0)
    body.visual(
        mesh_from_geometry(plate_geom, "wall_plate"),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5) - (plate_depth * 0.5), 0.0)),
        material=plate_metal,
        name="wall_plate",
    )

    body.visual(
        Box((0.020, 0.050, 0.050)),
        origin=Origin(xyz=(-(body_width * 0.5) - 0.008, 0.0, 0.0)),
        material=body_metal,
        name="left_bonnet_bridge",
    )
    body.visual(
        Box((0.020, 0.050, 0.050)),
        origin=Origin(xyz=((body_width * 0.5) + 0.008, 0.0, 0.0)),
        material=body_metal,
        name="right_bonnet_bridge",
    )

    body.visual(
        Cylinder(radius=bonnet_radius, length=bonnet_length),
        origin=Origin(xyz=(-bonnet_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_metal,
        name="left_bonnet",
    )
    body.visual(
        Cylinder(radius=bonnet_radius, length=bonnet_length),
        origin=Origin(xyz=(bonnet_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_metal,
        name="right_bonnet",
    )

    spout_geom = tube_from_spline_points(
        [
            (0.0, 0.012, -0.010),
            (0.0, 0.048, -0.016),
            (0.0, 0.078, -0.056),
            (0.0, 0.078, -0.094),
        ],
        radius=0.011,
        samples_per_segment=20,
        radial_segments=20,
        cap_ends=True,
    )
    body.visual(
        mesh_from_geometry(spout_geom, "spout"),
        material=body_metal,
        name="spout",
    )

    body.inertial = Inertial.from_geometry(
        Box((plate_width, body_depth + plate_depth, 0.17)),
        mass=4.2,
        origin=Origin(xyz=(0.0, -0.001, -0.020)),
    )

    def build_handle(name: str) -> object:
        handle = model.part(name)
        handle.visual(
            Cylinder(radius=hub_radius, length=hub_length),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=handle_metal,
            name="hub",
        )

        neck_geom = ExtrudeGeometry.centered(
            rounded_rect_profile(0.008, 0.020, radius=0.003),
            0.040,
        )
        handle.visual(
            mesh_from_geometry(neck_geom, f"{name}_neck"),
            origin=Origin(xyz=(0.0, 0.011, -0.020)),
            material=handle_metal,
            name="neck",
        )

        blade_geom = ExtrudeGeometry.centered(
            rounded_rect_profile(0.005, 0.044, radius=0.0022),
            0.098,
        )
        handle.visual(
            mesh_from_geometry(blade_geom, f"{name}_blade"),
            origin=Origin(xyz=(0.0, 0.026, -0.055)),
            material=handle_metal,
            name="blade",
        )
        handle.inertial = Inertial.from_geometry(
            Box((0.02, 0.05, 0.12)),
            mass=0.45,
            origin=Origin(xyz=(0.0, 0.018, -0.048)),
        )
        return handle

    left_handle = build_handle("left_handle")
    right_handle = build_handle("right_handle")

    model.articulation(
        "body_to_left_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_handle,
        origin=Origin(xyz=(-handle_joint_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.45,
            upper=0.65,
        ),
    )
    model.articulation(
        "body_to_right_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_handle,
        origin=Origin(xyz=(handle_joint_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.45,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("body_to_left_handle")
    right_joint = object_model.get_articulation("body_to_right_handle")

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

    ctx.expect_contact(
        left_handle,
        body,
        elem_a="hub",
        elem_b="left_bonnet",
        name="left handle hub seats on the left bonnet",
    )
    ctx.expect_contact(
        right_handle,
        body,
        elem_a="hub",
        elem_b="right_bonnet",
        name="right handle hub seats on the right bonnet",
    )
    ctx.expect_origin_gap(
        right_handle,
        left_handle,
        axis="x",
        min_gap=0.31,
        max_gap=0.34,
        name="blade handles span the faucet body",
    )

    def blade_center_z(part_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem="blade")
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[2] + upper[2]) * 0.5

    left_rest_z = blade_center_z("left_handle")
    with ctx.pose({left_joint: left_joint.motion_limits.upper}):
        left_open_z = blade_center_z("left_handle")
    ctx.check(
        "left blade handle lifts upward at positive rotation",
        left_rest_z is not None
        and left_open_z is not None
        and left_open_z > left_rest_z + 0.015,
        details=f"rest_z={left_rest_z}, open_z={left_open_z}",
    )

    right_rest_z = blade_center_z("right_handle")
    with ctx.pose({right_joint: right_joint.motion_limits.upper}):
        right_open_z = blade_center_z("right_handle")
    ctx.check(
        "right blade handle lifts upward at positive rotation",
        right_rest_z is not None
        and right_open_z is not None
        and right_open_z > right_rest_z + 0.015,
        details=f"rest_z={right_rest_z}, open_z={right_open_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
