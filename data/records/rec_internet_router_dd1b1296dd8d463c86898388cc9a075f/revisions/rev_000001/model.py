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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


BODY_WIDTH = 0.240
BODY_DEPTH = 0.166
BASE_HEIGHT = 0.006
POD_WIDTH = 0.015
POD_LENGTH = 0.058
POD_HEIGHT = 0.011
POD_X = 0.1275
ANTENNA_FRONT_Y = 0.015
ANTENNA_REAR_Y = 0.048
ANTENNA_HINGE_Z = 0.0282


def _rounded_section_at_y(
    width: float,
    height: float,
    y: float,
    *,
    corner_radius: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        height,
        corner_radius,
        corner_segments=corner_segments,
    )
    z_offset = height * 0.5
    return [(x, y, z + z_offset) for x, z in profile]


def _rounded_section_at_z(
    width: float,
    thickness: float,
    z: float,
    *,
    corner_radius: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        thickness,
        corner_radius,
        corner_segments=corner_segments,
    )
    return [(x, y, z) for x, y in profile]


def _build_router_shell_mesh():
    sections = [
        _rounded_section_at_y(0.228, 0.021, -0.076, corner_radius=0.008),
        _rounded_section_at_y(0.234, 0.027, -0.012, corner_radius=0.010),
        _rounded_section_at_y(0.236, 0.031, 0.042, corner_radius=0.011),
        _rounded_section_at_y(0.232, 0.029, 0.076, corner_radius=0.009),
    ]
    return mesh_from_geometry(section_loft(sections), "router_top_shell")


def _build_antenna_paddle_mesh():
    sections = [
        _rounded_section_at_z(0.014, 0.0048, 0.022, corner_radius=0.0018),
        _rounded_section_at_z(0.018, 0.0044, 0.080, corner_radius=0.0020),
        _rounded_section_at_z(0.016, 0.0040, 0.122, corner_radius=0.0018),
        _rounded_section_at_z(0.013, 0.0036, 0.144, corner_radius=0.0015),
    ]
    return mesh_from_geometry(section_loft(sections), "router_paddle_antenna")


def _build_antenna_part(model: ArticulatedObject, name: str, paddle_mesh, material) -> None:
    antenna = model.part(name)
    antenna.visual(
        Cylinder(radius=0.0042, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="root_barrel",
    )
    antenna.visual(
        Box((0.007, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=material,
        name="root_block",
    )
    antenna.visual(
        Box((0.008, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=material,
        name="stem",
    )
    antenna.visual(
        paddle_mesh,
        material=material,
        name="paddle",
    )
    antenna.inertial = Inertial.from_geometry(
        Box((0.020, 0.012, 0.146)),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_router")

    housing_dark = model.material("housing_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    shell_black = model.material("shell_black", rgba=(0.12, 0.13, 0.15, 1.0))
    hinge_black = model.material("hinge_black", rgba=(0.08, 0.09, 0.10, 1.0))
    smoked_cover = model.material("smoked_cover", rgba=(0.20, 0.23, 0.25, 0.85))
    led_blue = model.material("led_blue", rgba=(0.22, 0.72, 0.95, 0.90))
    led_green = model.material("led_green", rgba=(0.38, 0.88, 0.50, 0.90))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    router_shell_mesh = _build_router_shell_mesh()
    antenna_paddle_mesh = _build_antenna_paddle_mesh()

    chassis = model.part("chassis")
    chassis.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
        material=housing_dark,
        name="base_plate",
    )
    chassis.visual(
        router_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        material=shell_black,
        name="top_shell",
    )
    chassis.visual(
        Box((0.132, 0.009, 0.006)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH * 0.5) + 0.0045, 0.011)),
        material=housing_dark,
        name="front_bezel",
    )
    chassis.visual(
        Box((0.108, 0.003, 0.0055)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH * 0.5) + 0.0015, 0.011)),
        material=smoked_cover,
        name="light_window",
    )

    led_positions = (-0.036, -0.012, 0.012, 0.036)
    led_materials = (led_blue, led_blue, led_green, led_green)
    for index, (x_pos, led_material) in enumerate(zip(led_positions, led_materials)):
        chassis.visual(
            Box((0.010, 0.0015, 0.0016)),
            origin=Origin(xyz=(x_pos, -(BODY_DEPTH * 0.5) + 0.00075, 0.011)),
            material=led_material,
            name=f"indicator_{index}",
        )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        chassis.visual(
            Box((0.006, POD_LENGTH, 0.016)),
            origin=Origin(xyz=(side_sign * 0.120, 0.034, 0.020)),
            material=hinge_black,
            name=f"{side_name}_pod_bridge",
        )
        chassis.visual(
            Box((POD_WIDTH, POD_LENGTH, POD_HEIGHT)),
            origin=Origin(xyz=(side_sign * POD_X, 0.034, 0.0185)),
            material=hinge_black,
            name=f"{side_name}_pod_main",
        )
        chassis.visual(
            Cylinder(radius=0.006, length=POD_LENGTH),
            origin=Origin(
                xyz=(side_sign * (POD_X + 0.003), 0.034, 0.0195),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_black,
            name=f"{side_name}_pod_outer_rail",
        )

    foot_offsets = (
        (-0.090, -0.058),
        (0.090, -0.058),
        (-0.090, 0.058),
        (0.090, 0.058),
    )
    for index, (x_pos, y_pos) in enumerate(foot_offsets):
        chassis.visual(
            Cylinder(radius=0.010, length=0.002),
            origin=Origin(xyz=(x_pos, y_pos, -0.001)),
            material=rubber,
            name=f"foot_{index}",
        )

    chassis.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, 0.038)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    cover = model.part("status_cover")
    cover.visual(
        Box((0.118, 0.0028, 0.014)),
        origin=Origin(xyz=(0.0, -0.0014, -0.007)),
        material=smoked_cover,
        name="cover_panel",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.118, 0.003, 0.014)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.0015, -0.007)),
    )

    antenna_names = (
        "antenna_front_left",
        "antenna_rear_left",
        "antenna_front_right",
        "antenna_rear_right",
    )
    for antenna_name in antenna_names:
        _build_antenna_part(model, antenna_name, antenna_paddle_mesh, hinge_black)

    model.articulation(
        "status_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=cover,
        origin=Origin(xyz=(0.0, -(BODY_DEPTH * 0.5), 0.0205)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    antenna_specs = (
        ("antenna_front_left", "antenna_front_left_hinge", -1.0, ANTENNA_FRONT_Y),
        ("antenna_rear_left", "antenna_rear_left_hinge", -1.0, ANTENNA_REAR_Y),
        ("antenna_front_right", "antenna_front_right_hinge", 1.0, ANTENNA_FRONT_Y),
        ("antenna_rear_right", "antenna_rear_right_hinge", 1.0, ANTENNA_REAR_Y),
    )
    for child_name, joint_name, side_sign, y_pos in antenna_specs:
        joint_origin = Origin(
            xyz=(side_sign * 0.129, y_pos, ANTENNA_HINGE_Z),
            rpy=(0.0, 0.0, math.pi if side_sign < 0.0 else 0.0),
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=child_name,
            origin=joint_origin,
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=2.2,
                lower=0.0,
                upper=math.radians(68.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    status_cover = object_model.get_part("status_cover")
    antenna_front_left = object_model.get_part("antenna_front_left")
    antenna_rear_left = object_model.get_part("antenna_rear_left")
    antenna_front_right = object_model.get_part("antenna_front_right")
    antenna_rear_right = object_model.get_part("antenna_rear_right")

    status_cover_hinge = object_model.get_articulation("status_cover_hinge")
    antenna_front_left_hinge = object_model.get_articulation("antenna_front_left_hinge")
    antenna_rear_left_hinge = object_model.get_articulation("antenna_rear_left_hinge")
    antenna_front_right_hinge = object_model.get_articulation("antenna_front_right_hinge")
    antenna_rear_right_hinge = object_model.get_articulation("antenna_rear_right_hinge")

    antenna_parts = (
        antenna_front_left,
        antenna_rear_left,
        antenna_front_right,
        antenna_rear_right,
    )
    antenna_joints = (
        antenna_front_left_hinge,
        antenna_rear_left_hinge,
        antenna_front_right_hinge,
        antenna_rear_right_hinge,
    )
    light_window = chassis.get_visual("light_window")

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
        "status_cover_hinge_axis",
        status_cover_hinge.axis == (-1.0, 0.0, 0.0),
        f"Expected status cover hinge axis (-1, 0, 0), got {status_cover_hinge.axis}.",
    )
    for joint in antenna_joints:
        ctx.check(
            f"{joint.name}_axis",
            joint.axis == (0.0, 1.0, 0.0),
            f"Expected {joint.name} axis (0, 1, 0), got {joint.axis}.",
        )

    ctx.expect_origin_gap(
        antenna_rear_left,
        antenna_front_left,
        axis="y",
        min_gap=0.025,
        max_gap=0.040,
        name="left_antenna_pair_spacing",
    )
    ctx.expect_origin_gap(
        antenna_rear_right,
        antenna_front_right,
        axis="y",
        min_gap=0.025,
        max_gap=0.040,
        name="right_antenna_pair_spacing",
    )

    with ctx.pose({status_cover_hinge: 0.0}):
        ctx.expect_gap(
            chassis,
            status_cover,
            axis="y",
            min_gap=0.0,
            max_gap=0.0005,
            name="status_cover_closed_flush",
        )
        ctx.expect_overlap(
            status_cover,
            chassis,
            axes="x",
            min_overlap=0.100,
            elem_b=light_window,
            name="status_cover_spans_light_window_width",
        )
        ctx.expect_overlap(
            status_cover,
            chassis,
            axes="z",
            min_overlap=0.004,
            elem_b=light_window,
            name="status_cover_overlaps_light_window_height",
        )
        for antenna in antenna_parts:
            ctx.expect_contact(
                antenna,
                chassis,
                contact_tol=0.0005,
                name=f"{antenna.name}_root_contacts_pod",
            )

    cover_rest_aabb = ctx.part_world_aabb(status_cover)
    assert cover_rest_aabb is not None
    with ctx.pose({status_cover_hinge: math.radians(82.0)}):
        cover_open_aabb = ctx.part_world_aabb(status_cover)
        assert cover_open_aabb is not None
        assert cover_open_aabb[0][1] < cover_rest_aabb[0][1] - 0.010
        assert cover_open_aabb[0][2] > cover_rest_aabb[0][2] + 0.005

    right_front_rest_aabb = ctx.part_world_aabb(antenna_front_right)
    left_front_rest_aabb = ctx.part_world_aabb(antenna_front_left)
    assert right_front_rest_aabb is not None
    assert left_front_rest_aabb is not None
    with ctx.pose(
        {
            antenna_front_right_hinge: math.radians(60.0),
            antenna_front_left_hinge: math.radians(60.0),
        }
    ):
        right_front_open_aabb = ctx.part_world_aabb(antenna_front_right)
        left_front_open_aabb = ctx.part_world_aabb(antenna_front_left)
        assert right_front_open_aabb is not None
        assert left_front_open_aabb is not None
        assert right_front_open_aabb[1][0] > right_front_rest_aabb[1][0] + 0.030
        assert right_front_open_aabb[1][2] < right_front_rest_aabb[1][2] - 0.020
        assert left_front_open_aabb[0][0] < left_front_rest_aabb[0][0] - 0.030
        assert left_front_open_aabb[1][2] < left_front_rest_aabb[1][2] - 0.020

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
