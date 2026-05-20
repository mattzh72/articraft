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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_spigot_hose_bib")

    wall_steel = model.material("wall_steel", rgba=(0.52, 0.54, 0.56, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.60, 0.28, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.64, 0.50, 0.23, 1.0))
    handle_green = model.material("handle_green", rgba=(0.23, 0.47, 0.20, 1.0))

    wall_bracket = model.part("wall_bracket")
    plate_mesh = _mesh(
        "wall_bracket_plate",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.092, 0.080, 0.010, corner_segments=8),
            [superellipse_profile(0.028, 0.028, exponent=2.0, segments=28)],
            height=0.008,
            center=True,
        ).rotate_y(pi / 2.0),
    )
    wall_bracket.visual(
        plate_mesh,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=wall_steel,
        name="mount_plate",
    )
    for index, (y_pos, z_pos) in enumerate(
        [(-0.025, 0.025), (0.025, 0.025), (-0.025, -0.025), (0.025, -0.025)]
    ):
        wall_bracket.visual(
            Cylinder(radius=0.0055, length=0.003),
            origin=Origin(xyz=(0.0065, y_pos, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
            material=screw_steel,
            name=f"mount_screw_{index}",
        )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.014, 0.080, 0.092)),
        mass=0.45,
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
    )

    valve_body = model.part("valve_body")
    valve_body.visual(
        Box((0.098, 0.048, 0.050)),
        origin=Origin(xyz=(-0.065, 0.0, -0.002)),
        material=aged_brass,
        name="body_block",
    )
    valve_body.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(-0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="packing_collar",
    )
    valve_body.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="bonnet_cap",
    )
    valve_body.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(-0.125, 0.0, -0.001), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="wall_shank",
    )
    valve_body.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(-0.128, 0.0, -0.001), rpy=(0.0, pi / 2.0, 0.0)),
        material=aged_brass,
        name="mounting_collar",
    )
    for index, x_pos in enumerate((-0.121, -0.117, -0.113)):
        valve_body.visual(
            Cylinder(radius=0.014, length=0.002),
            origin=Origin(xyz=(x_pos, 0.0, -0.001), rpy=(0.0, pi / 2.0, 0.0)),
            material=aged_brass,
            name=f"shank_thread_{index}",
        )
    spout_mesh = _mesh(
        "spigot_spout",
        tube_from_spline_points(
            [
                (-0.030, 0.0, -0.018),
                (-0.022, 0.0, -0.038),
                (-0.010, 0.0, -0.055),
                (0.008, 0.0, -0.068),
                (0.028, 0.0, -0.072),
            ],
            radius=0.011,
            samples_per_segment=16,
            radial_segments=20,
            cap_ends=True,
        ),
    )
    valve_body.visual(spout_mesh, material=aged_brass, name="spout_tube")
    valve_body.visual(
        Cylinder(radius=0.0125, length=0.012),
        origin=Origin(xyz=(0.034, 0.0, -0.072), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="spout_shoulder",
    )
    valve_body.inertial = Inertial.from_geometry(
        Box((0.145, 0.052, 0.125)),
        mass=1.10,
        origin=Origin(xyz=(-0.055, 0.0, -0.030)),
    )

    cross_handle = model.part("cross_handle")
    cross_handle.visual(
        Cylinder(radius=0.015, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_green,
        name="handle_hub",
    )
    cross_handle.visual(
        Cylinder(radius=0.006, length=0.082),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=handle_green,
        name="vertical_bar",
    )
    cross_handle.visual(
        Cylinder(radius=0.006, length=0.082),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_green,
        name="horizontal_bar",
    )
    for name, y_pos, z_pos in (
        ("grip_right", 0.041, 0.0),
        ("grip_left", -0.041, 0.0),
        ("grip_top", 0.0, 0.041),
        ("grip_bottom", 0.0, -0.041),
    ):
        cross_handle.visual(
            Sphere(radius=0.009),
            origin=Origin(xyz=(0.011, y_pos, z_pos)),
            material=handle_green,
            name=name,
        )
    cross_handle.inertial = Inertial.from_geometry(
        Box((0.028, 0.100, 0.100)),
        mass=0.25,
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
    )

    hose_connector = model.part("hose_connector")
    connector_shell = _mesh(
        "hose_connector_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0125, 0.000),
                (0.0142, 0.003),
                (0.0134, 0.006),
                (0.0144, 0.009),
                (0.0135, 0.012),
                (0.0145, 0.015),
                (0.0136, 0.018),
                (0.0146, 0.021),
                (0.0137, 0.024),
                (0.0146, 0.027),
                (0.0141, 0.031),
                (0.0138, 0.034),
            ],
            [
                (0.0084, 0.004),
                (0.0088, 0.018),
                (0.0088, 0.034),
            ],
            segments=64,
        ).rotate_y(pi / 2.0),
    )
    hose_connector.visual(
        connector_shell,
        material=brass,
        name="connector_shell",
    )
    hose_connector.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aged_brass,
        name="connector_base_collar",
    )
    hose_connector.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.034),
        mass=0.12,
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "bracket_to_body",
        ArticulationType.FIXED,
        parent=wall_bracket,
        child=valve_body,
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=valve_body,
        child=cross_handle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-5.2,
            upper=5.2,
        ),
    )
    model.articulation(
        "body_to_connector",
        ArticulationType.FIXED,
        parent=valve_body,
        child=hose_connector,
        origin=Origin(xyz=(0.040, 0.0, -0.072)),
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

    wall_bracket = object_model.get_part("wall_bracket")
    valve_body = object_model.get_part("valve_body")
    cross_handle = object_model.get_part("cross_handle")
    hose_connector = object_model.get_part("hose_connector")
    handle_joint = object_model.get_articulation("body_to_handle")

    ctx.expect_contact(
        wall_bracket,
        valve_body,
        elem_a="mount_plate",
        elem_b="mounting_collar",
        name="body collar seats against wall bracket",
    )
    ctx.expect_contact(
        valve_body,
        cross_handle,
        elem_a="bonnet_cap",
        elem_b="handle_hub",
        name="handle hub bears on bonnet cap",
    )
    ctx.expect_contact(
        valve_body,
        hose_connector,
        elem_a="spout_shoulder",
        elem_b="connector_base_collar",
        name="hose connector seats on the spout outlet shoulder",
    )
    ctx.expect_origin_gap(
        valve_body,
        wall_bracket,
        axis="x",
        min_gap=0.13,
        max_gap=0.15,
        name="valve body projects outward from wall bracket",
    )
    ctx.expect_origin_gap(
        hose_connector,
        valve_body,
        axis="x",
        min_gap=0.035,
        max_gap=0.045,
        name="hose connector projects forward from valve body",
    )
    ctx.expect_origin_gap(
        valve_body,
        hose_connector,
        axis="z",
        min_gap=0.064,
        max_gap=0.078,
        name="spout outlet sits below valve spindle axis",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((a + b) * 0.5 for a, b in zip(lower, upper))

    rest_origin = ctx.part_world_position(cross_handle)
    with ctx.pose({handle_joint: pi / 4.0}):
        turned_origin = ctx.part_world_position(cross_handle)
        top_center = _aabb_center(ctx.part_element_world_aabb(cross_handle, elem="grip_top"))
        right_center = _aabb_center(ctx.part_element_world_aabb(cross_handle, elem="grip_right"))
        ctx.expect_contact(
            valve_body,
            cross_handle,
            elem_a="bonnet_cap",
            elem_b="handle_hub",
            name="handle remains seated while turned",
        )

    axis_ok = (
        rest_origin is not None
        and turned_origin is not None
        and top_center is not None
        and right_center is not None
        and abs(rest_origin[0] - turned_origin[0]) < 1e-6
        and abs(rest_origin[1] - turned_origin[1]) < 1e-6
        and abs(rest_origin[2] - turned_origin[2]) < 1e-6
        and top_center[1] < -0.020
        and right_center[1] > 0.020
        and right_center[2] > 0.020
    )
    ctx.check(
        "cross handle rotates about horizontal spindle axis",
        axis_ok,
        details=(
            f"rest_origin={rest_origin}, turned_origin={turned_origin}, "
            f"top_center={top_center}, right_center={right_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
