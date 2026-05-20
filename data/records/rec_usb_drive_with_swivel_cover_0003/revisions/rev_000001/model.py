from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_swivel_usb_drive", assets=ASSETS)

    matte_body = model.material("matte_body", rgba=(0.16, 0.17, 0.19, 1.0))
    satin_body = model.material("satin_body", rgba=(0.30, 0.32, 0.35, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.84, 0.85, 0.87, 1.0))
    dark_insert = model.material("dark_insert", rgba=(0.08, 0.09, 0.10, 1.0))

    body_length = 0.0375
    body_width = 0.0178
    body_thickness = 0.0082
    body_front_x = 0.0160
    pivot_x = -0.0035
    open_angle = math.pi

    def yz_section(width: float, height: float, radius: float, x: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for y, z in rounded_rect_profile(width, height, radius)]

    def rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
        x, y, z = point
        c = math.cos(angle)
        s = math.sin(angle)
        return (c * x + s * z, y, -s * x + c * z)

    def cover_box_origin(closed_xyz: tuple[float, float, float]) -> Origin:
        return Origin(xyz=rotate_y(closed_xyz, open_angle), rpy=(0.0, open_angle, 0.0))

    body = model.part("body")
    body_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                yz_section(0.0164, 0.0070, 0.0021, -0.0215),
                yz_section(0.0178, 0.0082, 0.0027, -0.0060),
                yz_section(0.0175, 0.0080, 0.0026, 0.0075),
                yz_section(0.0168, 0.0074, 0.0023, body_front_x),
            ]
        ),
        ASSETS.mesh_path("premium_swivel_usb_body.obj"),
    )
    body.visual(body_shell_mesh, material=matte_body, name="body_shell")
    body.visual(
        Box((0.0270, 0.0106, 0.0011)),
        origin=Origin(xyz=(-0.0010, 0.0, 0.00355)),
        material=satin_body,
        name="top_inlay",
    )
    body.visual(
        Box((0.0068, 0.0136, 0.0056)),
        origin=Origin(xyz=(-0.0184, 0.0, 0.0)),
        material=satin_body,
        name="rear_cap",
    )
    body.visual(
        Box((0.0040, 0.0139, 0.0060)),
        origin=Origin(xyz=(0.0139, 0.0, 0.0)),
        material=satin_body,
        name="front_collar",
    )
    body.visual(
        Cylinder(radius=0.0022, length=0.0008),
        origin=Origin(xyz=(pivot_x, 0.0093, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="pivot_boss_left",
    )
    body.visual(
        Cylinder(radius=0.0022, length=0.0008),
        origin=Origin(xyz=(pivot_x, -0.0093, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="pivot_boss_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_thickness)),
        mass=0.016,
        origin=Origin(xyz=(-0.0025, 0.0, 0.0)),
    )

    connector = model.part("connector")
    usb_shell_length = 0.0122
    usb_shell_width = 0.0121
    usb_shell_height = 0.0046
    shell_wall = 0.00038
    side_wall_height = usb_shell_height - 2.0 * shell_wall
    connector.visual(
        Box((usb_shell_length, usb_shell_width, shell_wall)),
        origin=Origin(xyz=(usb_shell_length / 2.0, 0.0, usb_shell_height / 2.0 - shell_wall / 2.0)),
        material=brushed_steel,
        name="usb_shell_top",
    )
    connector.visual(
        Box((usb_shell_length, usb_shell_width, shell_wall)),
        origin=Origin(xyz=(usb_shell_length / 2.0, 0.0, -usb_shell_height / 2.0 + shell_wall / 2.0)),
        material=brushed_steel,
        name="usb_shell_bottom",
    )
    connector.visual(
        Box((usb_shell_length, shell_wall, side_wall_height)),
        origin=Origin(xyz=(usb_shell_length / 2.0, usb_shell_width / 2.0 - shell_wall / 2.0, 0.0)),
        material=brushed_steel,
        name="usb_shell_left",
    )
    connector.visual(
        Box((usb_shell_length, shell_wall, side_wall_height)),
        origin=Origin(xyz=(usb_shell_length / 2.0, -usb_shell_width / 2.0 + shell_wall / 2.0, 0.0)),
        material=brushed_steel,
        name="usb_shell_right",
    )
    connector.visual(
        Box((0.0012, usb_shell_width - 2.0 * shell_wall, side_wall_height)),
        origin=Origin(xyz=(0.0006, 0.0, 0.0)),
        material=brushed_steel,
        name="usb_shell_rear_bridge",
    )
    connector.visual(
        Box((0.0030, 0.0094, 0.0027)),
        origin=Origin(xyz=(0.0015, 0.0, -0.0001)),
        material=dark_insert,
        name="usb_carrier",
    )
    connector.visual(
        Box((0.0080, 0.0094, 0.0011)),
        origin=Origin(xyz=(0.0070, 0.0, 0.0)),
        material=dark_insert,
        name="usb_tongue",
    )
    connector.inertial = Inertial.from_geometry(
        Box((usb_shell_length, 0.0121, 0.0046)),
        mass=0.0025,
        origin=Origin(xyz=(usb_shell_length / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_connector",
        ArticulationType.FIXED,
        parent=body,
        child=connector,
        origin=Origin(xyz=(body_front_x, 0.0, 0.0)),
    )

    swivel_cover = model.part("swivel_cover")
    plate_thickness = 0.00085
    rail_thickness = 0.00085
    cover_clearance = 0.00045
    cover_half_height = body_thickness / 2.0 + cover_clearance + plate_thickness / 2.0
    cover_half_width = body_width / 2.0 + cover_clearance + rail_thickness / 2.0
    cover_height = body_thickness + 2.0 * cover_clearance + 2.0 * plate_thickness
    cover_length = 0.0310
    rail_length = 0.0280

    swivel_cover.visual(
        Box((cover_length, body_width + 0.0018, plate_thickness)),
        origin=cover_box_origin((0.0145, 0.0, cover_half_height)),
        material=satin_metal,
        name="cover_top",
    )
    swivel_cover.visual(
        Box((cover_length, body_width + 0.0018, plate_thickness)),
        origin=cover_box_origin((0.0145, 0.0, -cover_half_height)),
        material=satin_metal,
        name="cover_bottom",
    )
    swivel_cover.visual(
        Box((rail_length, rail_thickness, cover_height)),
        origin=cover_box_origin((0.0130, cover_half_width, 0.0)),
        material=satin_metal,
        name="cover_left_rail",
    )
    swivel_cover.visual(
        Box((rail_length, rail_thickness, cover_height)),
        origin=cover_box_origin((0.0130, -cover_half_width, 0.0)),
        material=satin_metal,
        name="cover_right_rail",
    )
    swivel_cover.visual(
        Cylinder(radius=0.0031, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="cap_left",
    )
    swivel_cover.visual(
        Cylinder(radius=0.0031, length=0.0016),
        origin=Origin(xyz=(0.0, -0.0105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="cap_right",
    )
    swivel_cover.inertial = Inertial.from_geometry(
        Box((cover_length, body_width + 0.0040, cover_height)),
        mass=0.006,
        origin=Origin(xyz=(0.0100, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_swivel_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel_cover,
        origin=Origin(xyz=(pivot_x, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=4.0,
            lower=-math.pi,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    body = object_model.get_part("body")
    connector = object_model.get_part("connector")
    swivel_cover = object_model.get_part("swivel_cover")
    body_to_swivel_cover = object_model.get_articulation("body_to_swivel_cover")

    body_shell = body.get_visual("body_shell")
    pivot_boss_left = body.get_visual("pivot_boss_left")
    pivot_boss_right = body.get_visual("pivot_boss_right")
    cover_top = swivel_cover.get_visual("cover_top")
    cover_bottom = swivel_cover.get_visual("cover_bottom")
    cover_left_rail = swivel_cover.get_visual("cover_left_rail")
    cover_right_rail = swivel_cover.get_visual("cover_right_rail")
    cap_left = swivel_cover.get_visual("cap_left")
    cap_right = swivel_cover.get_visual("cap_right")
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(body, connector)
    ctx.expect_contact(swivel_cover, body, elem_a=cap_left, elem_b=pivot_boss_left)
    ctx.expect_contact(swivel_cover, body, elem_a=cap_right, elem_b=pivot_boss_right)
    ctx.expect_gap(connector, body, axis="x", max_gap=0.0002, max_penetration=0.0)

    body_aabb = ctx.part_world_aabb(body)
    connector_aabb = ctx.part_world_aabb(connector)
    cover_aabb = ctx.part_world_aabb(swivel_cover)
    assert body_aabb is not None
    assert connector_aabb is not None
    assert cover_aabb is not None

    body_dx = body_aabb[1][0] - body_aabb[0][0]
    body_dy = body_aabb[1][1] - body_aabb[0][1]
    body_dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "body_proportions_realistic",
        0.035 <= body_dx <= 0.042 and 0.016 <= body_dy <= 0.020 and 0.007 <= body_dz <= 0.010,
        details=f"body dims were {(body_dx, body_dy, body_dz)}",
    )
    ctx.check(
        "connector_exposed_in_rest_pose",
        connector_aabb[1][0] > cover_aabb[1][0] + 0.009,
        details=f"connector max x {connector_aabb[1][0]:.4f} cover max x {cover_aabb[1][0]:.4f}",
    )
    ctx.check(
        "cover_reads_as_swung_open",
        cover_aabb[0][0] < body_aabb[0][0] - 0.006,
        details=f"cover min x {cover_aabb[0][0]:.4f} body min x {body_aabb[0][0]:.4f}",
    )

    limits = body_to_swivel_cover.motion_limits
    assert limits is not None
    assert limits.lower is not None
    assert limits.upper is not None

    with ctx.pose({body_to_swivel_cover: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="swivel_cover_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="swivel_cover_closed_no_floating")
        ctx.expect_contact(swivel_cover, body, elem_a=cap_left, elem_b=pivot_boss_left)
        ctx.expect_contact(swivel_cover, body, elem_a=cap_right, elem_b=pivot_boss_right)
        ctx.expect_gap(
            swivel_cover,
            body,
            axis="z",
            positive_elem=cover_top,
            negative_elem=body_shell,
            min_gap=0.0001,
            max_gap=0.0012,
            name="cover_top_seam_gap",
        )
        ctx.expect_gap(
            body,
            swivel_cover,
            axis="z",
            positive_elem=body_shell,
            negative_elem=cover_bottom,
            min_gap=0.0001,
            max_gap=0.0012,
            name="cover_bottom_seam_gap",
        )
        ctx.expect_gap(
            swivel_cover,
            body,
            axis="y",
            positive_elem=cover_left_rail,
            negative_elem=body_shell,
            min_gap=0.0001,
            max_gap=0.0012,
            name="cover_left_side_gap",
        )
        ctx.expect_gap(
            body,
            swivel_cover,
            axis="y",
            positive_elem=body_shell,
            negative_elem=cover_right_rail,
            min_gap=0.0001,
            max_gap=0.0012,
            name="cover_right_side_gap",
        )
        ctx.expect_overlap(
            swivel_cover,
            body,
            axes="x",
            min_overlap=0.020,
            name="cover_closed_length_overlap",
        )
        ctx.expect_overlap(
            swivel_cover,
            body,
            axes="z",
            min_overlap=0.0075,
            name="cover_closed_thickness_overlap",
        )
        ctx.expect_within(body, swivel_cover, axes="yz", margin=0.002)

    with ctx.pose({body_to_swivel_cover: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="swivel_cover_open_no_overlap")
        ctx.fail_if_isolated_parts(name="swivel_cover_open_no_floating")
        ctx.expect_contact(swivel_cover, body, elem_a=cap_left, elem_b=pivot_boss_left)
        ctx.expect_contact(swivel_cover, body, elem_a=cap_right, elem_b=pivot_boss_right)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
