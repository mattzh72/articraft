from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shift_profile(profile: list[tuple[float, float]], *, dx: float = 0.0, dy: float = 0.0) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    radius: float,
    *,
    segments: int = 24,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos(2.0 * pi * index / segments),
            cy + radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_drive_swivel_cover_workhorse")

    body_graphite = model.material("body_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    overmold_olive = model.material("overmold_olive", rgba=(0.34, 0.38, 0.26, 1.0))
    blasted_steel = model.material("blasted_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.53, 0.55, 0.58, 1.0))
    connector_metal = model.material("connector_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    connector_insulator = model.material("connector_insulator", rgba=(0.12, 0.12, 0.13, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.08, 1.0))

    body_length = 0.054
    body_width = 0.023
    body_height = 0.012

    connector_length = 0.0145
    connector_width = 0.0125
    connector_height = 0.0046

    cover_thickness = 0.0016
    cover_gap = 0.00075
    top_plate_z = body_height * 0.5 + cover_gap + cover_thickness * 0.5
    cover_outer_width = 0.0275
    cover_outer_height = 2.0 * top_plate_z + cover_thickness
    cover_front_center = 0.060
    cover_back = -0.005
    cover_plate_length = cover_front_center - cover_back
    cover_plate_center_x = (cover_front_center + cover_back) * 0.5
    cover_plate_center_y = (cover_outer_width * 0.5 - 0.0028)

    pin_world_x = -0.0155
    pin_world_y = -(body_width * 0.5 - 0.0012)
    pin_radius = 0.00145
    pin_head_radius = 0.00285
    pin_total_length = body_height + 2.0 * cover_gap + 2.0 * cover_thickness + 0.0024

    body = model.part("body")
    body_shell_profile = rounded_rect_profile(body_height, body_width, radius=0.0028, corner_segments=8)
    body_shell_mesh = _save_mesh(
        "usb_body_shell",
        ExtrudeGeometry(body_shell_profile, body_length, center=True).rotate_y(pi / 2.0),
    )
    body.visual(body_shell_mesh, material=body_graphite, name="body_shell")
    body.visual(
        Box((0.008, 0.016, 0.0075)),
        origin=Origin(xyz=(body_length * 0.5 - 0.0035, 0.0, 0.0)),
        material=body_graphite,
        name="front_reinforcement",
    )
    body.visual(
        Box((0.009, 0.007, 0.0135)),
        origin=Origin(xyz=(pin_world_x + 0.0015, pin_world_y + 0.0021, 0.0)),
        material=body_graphite,
        name="pivot_cheek",
    )
    body.visual(
        Cylinder(radius=pin_radius, length=pin_total_length),
        origin=Origin(xyz=(pin_world_x, pin_world_y, 0.0)),
        material=fastener_steel,
        name="pivot_pin_shaft",
    )
    body.visual(
        Cylinder(radius=pin_head_radius, length=0.0012),
        origin=Origin(xyz=(pin_world_x, pin_world_y, pin_total_length * 0.5 - 0.0006)),
        material=fastener_steel,
        name="pivot_pin_head_top",
    )
    body.visual(
        Cylinder(radius=pin_head_radius, length=0.0012),
        origin=Origin(xyz=(pin_world_x, pin_world_y, -(pin_total_length * 0.5 - 0.0006))),
        material=fastener_steel,
        name="pivot_pin_head_bottom",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_height)),
        mass=0.042,
        origin=Origin(),
    )

    connector = model.part("connector")
    connector_wall = 0.00055
    connector_boot_length = 0.0032
    connector_tongue_length = connector_length * 0.58
    connector_shell_geom = BoxGeometry((connector_length, connector_width, connector_wall)).translate(
        connector_length * 0.5,
        0.0,
        connector_height * 0.5 - connector_wall * 0.5,
    )
    connector_shell_geom.merge(
        BoxGeometry((connector_length, connector_width, connector_wall)).translate(
            connector_length * 0.5,
            0.0,
            -(connector_height * 0.5 - connector_wall * 0.5),
        )
    )
    connector_shell_geom.merge(
        BoxGeometry((connector_length, connector_wall, connector_height - 2.0 * connector_wall)).translate(
            connector_length * 0.5,
            connector_width * 0.5 - connector_wall * 0.5,
            0.0,
        )
    )
    connector_shell_geom.merge(
        BoxGeometry((connector_length, connector_wall, connector_height - 2.0 * connector_wall)).translate(
            connector_length * 0.5,
            -(connector_width * 0.5 - connector_wall * 0.5),
            0.0,
        )
    )
    connector_shell_geom.merge(
        BoxGeometry((connector_wall, connector_width, connector_height)).translate(
            connector_wall * 0.5,
            0.0,
            0.0,
        )
    )
    connector.visual(
        _save_mesh("usb_connector_shell", connector_shell_geom),
        material=connector_metal,
        name="connector_shell",
    )
    connector.visual(
        Box((connector_tongue_length, connector_width * 0.72, 0.0011)),
        origin=Origin(xyz=(connector_boot_length + connector_tongue_length * 0.5, 0.0, 0.0)),
        material=connector_insulator,
        name="connector_tongue",
    )
    connector.visual(
        Box((connector_boot_length, connector_width - 2.0 * connector_wall, connector_height - 2.0 * connector_wall)),
        origin=Origin(xyz=(connector_boot_length * 0.5, 0.0, 0.0)),
        material=gasket_black,
        name="connector_boot",
    )
    connector.inertial = Inertial.from_geometry(
        Box((connector_length, connector_width, connector_height)),
        mass=0.006,
        origin=Origin(xyz=(connector_length * 0.5, 0.0, 0.0)),
    )

    left_scale = model.part("left_scale")
    left_scale.visual(
        Box((0.027, 0.0016, 0.0092)),
        origin=Origin(xyz=(0.0055, 0.0, 0.0)),
        material=overmold_olive,
        name="left_scale_body",
    )
    for index, x_pos in enumerate((-0.002, 0.013)):
        left_scale.visual(
            Cylinder(radius=0.00135, length=0.0019),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=fastener_steel,
            name=f"left_scale_bolt_{index}",
        )
    left_scale.inertial = Inertial.from_geometry(
        Box((0.027, 0.0016, 0.0092)),
        mass=0.002,
        origin=Origin(xyz=(0.0055, 0.0, 0.0)),
    )

    right_scale = model.part("right_scale")
    right_scale.visual(
        Box((0.039, 0.0016, 0.0092)),
        origin=Origin(xyz=(0.001, 0.0, 0.0)),
        material=overmold_olive,
        name="right_scale_body",
    )
    for index, x_pos in enumerate((-0.012, 0.012)):
        right_scale.visual(
            Cylinder(radius=0.00135, length=0.0019),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=fastener_steel,
            name=f"right_scale_bolt_{index}",
        )
    right_scale.inertial = Inertial.from_geometry(
        Box((0.039, 0.0016, 0.0092)),
        mass=0.0025,
        origin=Origin(xyz=(0.001, 0.0, 0.0)),
    )

    service_plate = model.part("service_plate")
    service_plate.visual(
        Box((0.034, 0.015, 0.0012)),
        material=body_graphite,
        name="service_plate_panel",
    )
    service_plate.visual(
        Box((0.028, 0.010, 0.0006)),
        origin=Origin(xyz=(0.0, 0.0, -0.0009)),
        material=gasket_black,
        name="service_gasket",
    )
    for index, x_pos in enumerate((-0.0105, 0.0105)):
        service_plate.visual(
            Cylinder(radius=0.00155, length=0.0016),
            origin=Origin(xyz=(x_pos, 0.0, -0.0014)),
            material=fastener_steel,
            name=f"service_screw_{index}",
        )
    service_plate.inertial = Inertial.from_geometry(
        Box((0.034, 0.015, 0.0012)),
        mass=0.002,
    )

    cover = model.part("swivel_cover")
    hole_profile = _circle_profile(pin_radius + 0.00045, segments=28)
    cover_outer_profile = _shift_profile(
        rounded_rect_profile(cover_plate_length, cover_outer_width, radius=0.0042, corner_segments=8),
        dx=cover_plate_center_x,
        dy=cover_plate_center_y,
    )
    cover_plate_mesh = _save_mesh(
        "usb_cover_plate",
        ExtrudeWithHolesGeometry(
            cover_outer_profile,
            [hole_profile],
            height=cover_thickness,
            center=True,
        ),
    )
    cover.visual(cover_plate_mesh, origin=Origin(xyz=(0.0, 0.0, top_plate_z)), material=blasted_steel, name="top_plate")
    cover.visual(cover_plate_mesh, origin=Origin(xyz=(0.0, 0.0, -top_plate_z)), material=blasted_steel, name="bottom_plate")
    cover.visual(
        Box((0.0022, cover_outer_width, cover_outer_height)),
        origin=Origin(xyz=(cover_front_center + 0.0011, cover_plate_center_y, 0.0)),
        material=blasted_steel,
        name="nose_bridge",
    )
    cover.visual(
        Box((0.008, 0.0024, cover_outer_height)),
        origin=Origin(xyz=(-0.001, -0.0016, 0.0)),
        material=blasted_steel,
        name="pivot_bracket",
    )
    cover.visual(
        Box((0.012, 0.010, 0.0014)),
        origin=Origin(xyz=(0.041, cover_plate_center_y + 0.004, top_plate_z + 0.0015)),
        material=blasted_steel,
        name="thumb_pad",
    )
    cover.inertial = Inertial.from_geometry(
        Box((cover_plate_length + 0.004, cover_outer_width, cover_outer_height)),
        mass=0.016,
        origin=Origin(xyz=(cover_plate_center_x, cover_plate_center_y, 0.0)),
    )

    model.articulation(
        "body_to_connector",
        ArticulationType.FIXED,
        parent=body,
        child=connector,
        origin=Origin(xyz=(body_length * 0.5, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_left_scale",
        ArticulationType.FIXED,
        parent=body,
        child=left_scale,
        origin=Origin(xyz=(0.0, -(body_width * 0.5 + 0.0008), 0.0)),
    )
    model.articulation(
        "body_to_right_scale",
        ArticulationType.FIXED,
        parent=body,
        child=right_scale,
        origin=Origin(xyz=(0.0, body_width * 0.5 + 0.0008, 0.0)),
    )
    model.articulation(
        "body_to_service_plate",
        ArticulationType.FIXED,
        parent=body,
        child=service_plate,
        origin=Origin(xyz=(0.0, 0.0, -(body_height * 0.5 + 0.0006))),
    )
    model.articulation(
        "body_to_swivel_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(pin_world_x, pin_world_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=3.0, lower=0.0, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    connector = object_model.get_part("connector")
    left_scale = object_model.get_part("left_scale")
    right_scale = object_model.get_part("right_scale")
    service_plate = object_model.get_part("service_plate")
    cover = object_model.get_part("swivel_cover")
    swivel = object_model.get_articulation("body_to_swivel_cover")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.warn_if_articulation_overlaps(max_pose_samples=16)

    ctx.expect_contact(connector, body, name="connector seated against body")
    ctx.expect_contact(left_scale, body, name="left scale bolted to body")
    ctx.expect_contact(right_scale, body, name="right scale bolted to body")
    ctx.expect_contact(service_plate, body, name="service plate closes underside")

    with ctx.pose({swivel: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="top_plate",
            negative_elem="body_shell",
            min_gap=0.0002,
            max_gap=0.0022,
            name="closed top cover clears body",
        )
        ctx.expect_gap(
            body,
            cover,
            axis="z",
            positive_elem="body_shell",
            negative_elem="bottom_plate",
            min_gap=0.0002,
            max_gap=0.0022,
            name="closed bottom cover clears body",
        )
        ctx.expect_gap(
            cover,
            connector,
            axis="x",
            positive_elem="nose_bridge",
            min_gap=0.0010,
            max_gap=0.0055,
            name="closed cover leaves serviceable nose clearance",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            elem_a="top_plate",
            min_overlap=0.020,
            name="closed cover plate spans body footprint",
        )

    with ctx.pose({swivel: pi}):
        nose_aabb = ctx.part_element_world_aabb(cover, elem="nose_bridge")
        body_aabb = ctx.part_world_aabb(body)
        if nose_aabb is None or body_aabb is None:
            ctx.fail("open cover pose measurable", "missing AABB for cover or body in open pose")
        else:
            nose_max_x = nose_aabb[1][0]
            body_min_x = body_aabb[0][0]
            ctx.check(
                "open cover swings behind body",
                nose_max_x < body_min_x + 0.004,
                f"nose bridge max x={nose_max_x:.4f} should be behind body min x={body_min_x:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
