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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_side_service_hatch(
    part,
    *,
    prefix: str,
    side: str,
    x_face: float,
    y_center: float,
    z_center: float,
    plate_size: tuple[float, float],
    adapter_thickness: float,
    cover_thickness: float,
    bolt_radius: float,
    bolt_length: float,
    adapter_material,
    cover_material,
    fastener_material,
) -> None:
    plate_y, plate_z = plate_size
    outward = 1.0 if side == "right" else -1.0

    adapter_center_x = x_face + outward * (adapter_thickness / 2.0)
    cover_center_x = x_face + outward * (adapter_thickness + cover_thickness / 2.0)
    bolt_center_x = x_face + outward * (
        adapter_thickness + cover_thickness + bolt_length / 2.0
    )

    part.visual(
        Box((adapter_thickness, plate_y, plate_z)),
        origin=Origin(xyz=(adapter_center_x, y_center, z_center)),
        material=adapter_material,
        name=f"{prefix}_adapter",
    )
    part.visual(
        Box((cover_thickness, plate_y * 0.86, plate_z * 0.82)),
        origin=Origin(xyz=(cover_center_x, y_center, z_center)),
        material=cover_material,
        name=f"{prefix}_cover",
    )

    bolt_offsets = (
        (-plate_y * 0.30, -plate_z * 0.28),
        (-plate_y * 0.30, plate_z * 0.28),
        (plate_y * 0.30, -plate_z * 0.28),
        (plate_y * 0.30, plate_z * 0.28),
    )
    for idx, (dy, dz) in enumerate(bolt_offsets, start=1):
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_length),
            origin=Origin(
                xyz=(bolt_center_x, y_center + dy, z_center + dz),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=fastener_material,
            name=f"{prefix}_bolt_{idx}",
        )


def _add_top_access_plate(
    part,
    *,
    prefix: str,
    x_center: float,
    y_center: float,
    z_top_surface: float,
    adapter_size: tuple[float, float],
    adapter_thickness: float,
    cover_thickness: float,
    bolt_radius: float,
    bolt_length: float,
    adapter_material,
    cover_material,
    fastener_material,
) -> None:
    adapter_x, adapter_y = adapter_size
    adapter_center_z = z_top_surface + adapter_thickness / 2.0
    cover_center_z = z_top_surface + adapter_thickness + cover_thickness / 2.0
    bolt_center_z = z_top_surface + adapter_thickness + cover_thickness + bolt_length / 2.0

    part.visual(
        Box((adapter_x, adapter_y, adapter_thickness)),
        origin=Origin(xyz=(x_center, y_center, adapter_center_z)),
        material=adapter_material,
        name=f"{prefix}_adapter",
    )
    part.visual(
        Box((adapter_x * 0.84, adapter_y * 0.78, cover_thickness)),
        origin=Origin(xyz=(x_center, y_center, cover_center_z)),
        material=cover_material,
        name=f"{prefix}_cover",
    )

    bolt_offsets = (
        (-adapter_x * 0.30, -adapter_y * 0.28),
        (-adapter_x * 0.30, adapter_y * 0.28),
        (adapter_x * 0.30, -adapter_y * 0.28),
        (adapter_x * 0.30, adapter_y * 0.28),
    )
    for idx, (dx, dy) in enumerate(bolt_offsets, start=1):
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_length),
            origin=Origin(xyz=(x_center + dx, y_center + dy, bolt_center_z)),
            material=fastener_material,
            name=f"{prefix}_bolt_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_sewing_box")

    body_color = model.material("body_paint", rgba=(0.25, 0.31, 0.28, 1.0))
    lid_color = model.material("lid_paint", rgba=(0.30, 0.34, 0.30, 1.0))
    reinforcement_color = model.material("reinforcement", rgba=(0.55, 0.53, 0.48, 1.0))
    hardware_color = model.material("hardware", rgba=(0.33, 0.35, 0.38, 1.0))
    hatch_color = model.material("hatch_cover", rgba=(0.46, 0.43, 0.37, 1.0))

    width = 0.46
    depth = 0.30
    body_height = 0.17
    floor_thickness = 0.016
    wall_thickness = 0.014

    lid_top_thickness = 0.012
    lid_skirt_thickness = 0.010
    lid_skirt_depth = 0.046
    lid_width = width + 0.024
    lid_depth = depth + 0.004
    lid_center_y = -(lid_depth / 2.0) - 0.008

    barrel_radius = 0.006
    hinge_axis_y = depth / 2.0 + barrel_radius
    hinge_axis_z = body_height + barrel_radius

    body = model.part("body")

    wall_height = body_height - floor_thickness
    wall_center_z = floor_thickness + wall_height / 2.0

    body.visual(
        Box((width, depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=body_color,
        name="floor_panel",
    )
    body.visual(
        Box((width, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(0.0, -(depth / 2.0) + wall_thickness / 2.0, wall_center_z)
        ),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((width, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(0.0, (depth / 2.0) - wall_thickness / 2.0, wall_center_z)
        ),
        material=body_color,
        name="back_wall",
    )
    body.visual(
        Box((wall_thickness, depth - (2.0 * wall_thickness), wall_height)),
        origin=Origin(
            xyz=(-(width / 2.0) + wall_thickness / 2.0, 0.0, wall_center_z)
        ),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall_thickness, depth - (2.0 * wall_thickness), wall_height)),
        origin=Origin(
            xyz=((width / 2.0) - wall_thickness / 2.0, 0.0, wall_center_z)
        ),
        material=body_color,
        name="right_wall",
    )

    body.visual(
        Box((width * 0.90, 0.010, 0.045)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.005, body_height - 0.035)),
        material=reinforcement_color,
        name="backer_rail",
    )
    for idx, x_pos in enumerate((-0.145, 0.145), start=1):
        body.visual(
            Box((0.094, 0.006, 0.055)),
            origin=Origin(xyz=(x_pos, depth / 2.0 + 0.003, body_height - 0.0275)),
            material=reinforcement_color,
            name=f"hinge_adapter_{idx}",
        )

    for idx, x_pos in enumerate((-0.194, 0.194), start=1):
        body.visual(
            Box((0.036, 0.006, 0.098)),
            origin=Origin(xyz=(x_pos, -(depth / 2.0) - 0.003, 0.065)),
            material=reinforcement_color,
            name=f"front_corner_strap_{idx}",
        )

    for idx, x_pos in enumerate((-0.140, 0.140), start=1):
        body.visual(
            Cylinder(radius=barrel_radius, length=0.120),
            origin=Origin(
                xyz=(x_pos, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hardware_color,
            name=f"body_hinge_barrel_{idx}",
        )

    _add_side_service_hatch(
        body,
        prefix="left_service",
        side="left",
        x_face=-(width / 2.0),
        y_center=0.040,
        z_center=0.096,
        plate_size=(0.146, 0.092),
        adapter_thickness=0.004,
        cover_thickness=0.003,
        bolt_radius=0.0045,
        bolt_length=0.003,
        adapter_material=reinforcement_color,
        cover_material=hatch_color,
        fastener_material=hardware_color,
    )
    _add_side_service_hatch(
        body,
        prefix="right_service",
        side="right",
        x_face=width / 2.0,
        y_center=-0.038,
        z_center=0.096,
        plate_size=(0.146, 0.092),
        adapter_thickness=0.004,
        cover_thickness=0.003,
        bolt_radius=0.0045,
        bolt_length=0.003,
        adapter_material=reinforcement_color,
        cover_material=hatch_color,
        fastener_material=hardware_color,
    )

    lid = model.part("lid")

    lid.visual(
        Box((lid_width, lid_depth, lid_top_thickness)),
        origin=Origin(xyz=(0.0, lid_center_y, 0.0)),
        material=lid_color,
        name="top_panel",
    )
    lid.visual(
        Box((lid_width, lid_skirt_thickness, lid_skirt_depth)),
        origin=Origin(
            xyz=(
                0.0,
                lid_center_y - (lid_depth / 2.0) + (lid_skirt_thickness / 2.0),
                -0.006 - (lid_skirt_depth / 2.0),
            )
        ),
        material=lid_color,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_skirt_thickness, lid_depth, lid_skirt_depth)),
        origin=Origin(
            xyz=(
                -(lid_width / 2.0) + (lid_skirt_thickness / 2.0),
                lid_center_y,
                -0.006 - (lid_skirt_depth / 2.0),
            )
        ),
        material=lid_color,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_skirt_thickness, lid_depth, lid_skirt_depth)),
        origin=Origin(
            xyz=(
                (lid_width / 2.0) - (lid_skirt_thickness / 2.0),
                lid_center_y,
                -0.006 - (lid_skirt_depth / 2.0),
            )
        ),
        material=lid_color,
        name="right_skirt",
    )

    lid.visual(
        Box((width * 0.74, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, lid_center_y + 0.098, -0.015)),
        material=reinforcement_color,
        name="lid_rear_stiffener",
    )
    lid.visual(
        Box((width * 0.66, 0.012, 0.020)),
        origin=Origin(
            xyz=(0.0, lid_center_y - 0.118, -0.016)
        ),
        material=reinforcement_color,
        name="front_closing_lip",
    )

    for idx, x_pos in enumerate((-0.045, 0.045), start=1):
        lid.visual(
            Box((0.030, 0.014, 0.018)),
            origin=Origin(xyz=(x_pos, -0.003, 0.007)),
            material=reinforcement_color,
            name=f"lid_hinge_strap_{idx}",
        )
    lid.visual(
        Cylinder(radius=barrel_radius, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_color,
        name="lid_hinge_barrel",
    )

    _add_top_access_plate(
        lid,
        prefix="lid_service",
        x_center=0.0,
        y_center=lid_center_y - 0.022,
        z_top_surface=lid_top_thickness / 2.0,
        adapter_size=(0.170, 0.110),
        adapter_thickness=0.003,
        cover_thickness=0.003,
        bolt_radius=0.0045,
        bolt_length=0.003,
        adapter_material=reinforcement_color,
        cover_material=hatch_color,
        fastener_material=hardware_color,
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.72,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    top_panel = lid.get_visual("top_panel")
    body.get_visual("left_service_cover")
    body.get_visual("right_service_cover")
    lid.get_visual("lid_service_cover")
    body.get_visual("body_hinge_barrel_1")
    body.get_visual("body_hinge_barrel_2")
    lid.get_visual("lid_hinge_barrel")

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
        "hinge_axis_widthwise",
        hinge.axis == (-1.0, 0.0, 0.0),
        f"expected widthwise hinge axis (-1, 0, 0), got {hinge.axis}",
    )
    ctx.check(
        "hinge_limits_present",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 1.4 <= limits.upper <= 1.9,
        f"expected realistic lid limits, got {limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=top_panel,
            min_overlap=0.22,
            name="lid_covers_body_plan",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=top_panel,
            negative_elem="front_wall",
            max_gap=0.001,
            max_penetration=1e-5,
            name="lid_seats_on_box_rim",
        )

    with ctx.pose({hinge: limits.upper if limits and limits.upper is not None else 1.72}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            min_gap=0.20,
            name="lid_opens_above_body",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
