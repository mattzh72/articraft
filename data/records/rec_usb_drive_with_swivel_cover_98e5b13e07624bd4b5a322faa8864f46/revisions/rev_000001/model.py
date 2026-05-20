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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 24,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _rounded_body_mesh(
    *,
    name: str,
    length: float,
    width: float,
    height: float,
    corner_radius: float,
):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(height, width, corner_radius, corner_segments=8),
        length,
        cap=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _usb_connector_shell_mesh(
    *,
    name: str,
    length: float,
    width: float,
    height: float,
    wall: float,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(height, width, 0.00035, corner_segments=6),
        [
            rounded_rect_profile(
                height - (2.0 * wall),
                width - (2.0 * wall),
                0.0002,
                corner_segments=6,
            )
        ],
        length,
        cap=False,
        center=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_drive_with_swivel_cover")

    plastic_black = model.material("plastic_black", rgba=(0.13, 0.14, 0.15, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    connector_metal = model.material("connector_metal", rgba=(0.84, 0.85, 0.87, 1.0))
    pin_metal = model.material("pin_metal", rgba=(0.64, 0.66, 0.69, 1.0))

    body_length = 0.040
    body_width = 0.018
    body_height = 0.0085
    body_corner_radius = 0.0018

    collar_length = 0.0055
    collar_width = 0.0154
    collar_height = 0.0062

    connector_length = 0.012
    connector_width = 0.0122
    connector_height = 0.0046

    side_clearance = 0.0004
    cover_thickness = 0.0012
    cover_width = 0.0206
    cover_length = 0.058
    cover_corner_radius = 0.0016
    flange_thickness = 0.0009
    flange_drop = 0.0038
    flange_length = 0.020

    hole_to_left_edge = 0.0022
    hole_to_rear_edge = 0.0028
    cover_center_y = (cover_width / 2.0) - hole_to_left_edge
    cover_center_x = (cover_length / 2.0) - hole_to_rear_edge
    pivot_hole_radius = 0.00125

    top_clearance = 0.00035
    support_embed = 0.0002
    pin_cap_overlap = 0.00015
    cover_center_z = (body_height / 2.0) + top_clearance + (cover_thickness / 2.0)
    hinge_origin = Origin(
        xyz=(
            (-body_length / 2.0) + hole_to_rear_edge,
            -cover_center_y,
            cover_center_z,
        )
    )

    left_flange_center_y = cover_center_y - (cover_width / 2.0) + (flange_thickness / 2.0)
    right_flange_center_y = cover_center_y + (cover_width / 2.0) - (flange_thickness / 2.0)
    flange_center_z = -(cover_thickness / 2.0) - (flange_drop / 2.0)

    body_shell_mesh = _rounded_body_mesh(
        name="usb_body_shell",
        length=body_length,
        width=body_width,
        height=body_height,
        corner_radius=body_corner_radius,
    )
    connector_shell_mesh = _usb_connector_shell_mesh(
        name="usb_connector_shell",
        length=connector_length,
        width=connector_width,
        height=connector_height,
        wall=0.0004,
    )

    outer_cover_profile = _translate_profile(
        rounded_rect_profile(
            cover_length,
            cover_width,
            cover_corner_radius,
            corner_segments=8,
        ),
        dx=cover_center_x,
        dy=cover_center_y,
    )
    cover_plate_geom = ExtrudeWithHolesGeometry(
        outer_cover_profile,
        [_circle_profile(pivot_hole_radius, segments=28)],
        cover_thickness,
        cap=True,
        center=True,
        closed=True,
    )
    cover_plate_mesh = mesh_from_geometry(cover_plate_geom, "usb_swivel_cover_plate")

    body = model.part("body")
    body.visual(body_shell_mesh, material=plastic_black, name="shell")
    body.visual(
        Box((collar_length, collar_width, collar_height)),
        origin=Origin(
            xyz=((body_length / 2.0) + (collar_length / 2.0), 0.0, 0.0)
        ),
        material=dark_gray,
        name="connector_collar",
    )
    body.visual(
        connector_shell_mesh,
        origin=Origin(
            xyz=(
                (body_length / 2.0) + collar_length + (connector_length / 2.0),
                0.0,
                0.0,
            )
        ),
        material=connector_metal,
        name="connector",
    )
    body.visual(
        Box((0.0075, 0.0115, top_clearance + support_embed)),
        origin=Origin(
            xyz=(
                (-body_length / 2.0) + 0.0105,
                0.0,
                (body_height / 2.0) + (top_clearance / 2.0) - (support_embed / 2.0),
            )
        ),
        material=dark_gray,
        name="top_stop_rib",
    )

    pin_base_length = 0.0005
    pin_shaft_radius = 0.0009
    pin_head_radius = 0.0017
    pin_head_length = 0.00055
    pin_shaft_bottom = (body_height / 2.0) - support_embed
    pin_shaft_top = cover_center_z + (cover_thickness / 2.0) + pin_cap_overlap
    pin_shaft_length = pin_shaft_top - pin_shaft_bottom
    pin_shaft_center_z = (pin_shaft_top + pin_shaft_bottom) / 2.0
    body.visual(
        Cylinder(radius=pin_head_radius, length=pin_base_length + support_embed),
        origin=Origin(
            xyz=(
                hinge_origin.xyz[0],
                hinge_origin.xyz[1],
                (body_height / 2.0) + (pin_base_length / 2.0) - (support_embed / 2.0),
            )
        ),
        material=pin_metal,
        name="pivot_pin_base",
    )
    body.visual(
        Cylinder(radius=pin_shaft_radius, length=pin_shaft_length),
        origin=Origin(
            xyz=(hinge_origin.xyz[0], hinge_origin.xyz[1], pin_shaft_center_z)
        ),
        material=pin_metal,
        name="pivot_pin_shaft",
    )
    body.visual(
        Cylinder(radius=pin_head_radius, length=pin_head_length),
        origin=Origin(
            xyz=(
                hinge_origin.xyz[0],
                hinge_origin.xyz[1],
                pin_shaft_top + (pin_head_length / 2.0),
            )
        ),
        material=pin_metal,
        name="pivot_pin_head",
    )

    cover = model.part("cover")
    cover.visual(cover_plate_mesh, material=brushed_steel, name="cover_plate")
    cover.visual(
        Box((flange_length, flange_thickness, flange_drop)),
        origin=Origin(
            xyz=(
                cover_center_x,
                left_flange_center_y,
                flange_center_z,
            )
        ),
        material=brushed_steel,
        name="left_return_flange",
    )
    cover.visual(
        Box((flange_length, flange_thickness, flange_drop)),
        origin=Origin(
            xyz=(
                cover_center_x,
                right_flange_center_y,
                flange_center_z,
            )
        ),
        material=brushed_steel,
        name="right_return_flange",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=hinge_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=8.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    hinge = object_model.get_articulation("body_to_cover")

    shell = body.get_visual("shell")
    connector = body.get_visual("connector")
    top_stop_rib = body.get_visual("top_stop_rib")
    pivot_pin_head = body.get_visual("pivot_pin_head")
    cover_plate = cover.get_visual("cover_plate")
    left_return_flange = cover.get_visual("left_return_flange")
    right_return_flange = cover.get_visual("right_return_flange")

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

    limits = hinge.motion_limits
    ctx.check(
        "cover_hinge_is_side_pivoted",
        hinge.axis == (0.0, 0.0, 1.0) and hinge.origin.xyz[1] < -0.005,
        details=f"axis={hinge.axis}, origin={hinge.origin.xyz}",
    )
    ctx.check(
        "cover_hinge_has_half_turn_range",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and abs(limits.upper - math.pi) < 1e-6,
        details=str(limits),
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            body,
            cover,
            elem_a=top_stop_rib,
            elem_b=cover_plate,
            contact_tol=2e-4,
            name="closed_cover_seats_on_body_stop",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem=cover_plate,
            negative_elem=connector,
            min_gap=0.0012,
            max_gap=0.0038,
            name="closed_cover_clears_connector_height",
        )
        ctx.expect_overlap(
            body,
            cover,
            axes="xy",
            min_overlap=0.010,
            elem_a=shell,
            elem_b=cover_plate,
            name="closed_cover_stays_over_body_footprint",
        )
        ctx.expect_gap(
            body,
            cover,
            axis="z",
            positive_elem=pivot_pin_head,
            negative_elem=cover_plate,
            min_gap=0.0,
            max_gap=0.0010,
            name="pivot_pin_head_sits_just_above_cover",
        )

    with ctx.pose({hinge: math.pi}):
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem=connector,
            negative_elem=cover_plate,
            min_gap=0.020,
            name="open_cover_swings_clear_of_connector",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="z",
            min_overlap=0.0010,
            elem_a=left_return_flange,
            elem_b=shell,
            name="open_cover_remains_low_profile",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="z",
            min_overlap=0.0010,
            elem_a=right_return_flange,
            elem_b=shell,
            name="open_cover_stays_flat_through_side_rails",
        )

    ctx.warn_if_articulation_overlaps(max_pose_samples=20)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
