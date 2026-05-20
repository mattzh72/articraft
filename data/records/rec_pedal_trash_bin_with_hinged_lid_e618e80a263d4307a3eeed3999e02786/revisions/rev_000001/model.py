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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _loop(width: float, depth: float, radius: float, z: float, y_center: float) -> list[tuple[float, float, float]]:
    return [(x, y + y_center, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _aabb_dims(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (max_x - min_x, max_y - min_y, max_z - min_z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_waste_bin")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    body_width = 0.230
    body_depth = 0.150
    body_height = 0.365
    wall_thickness = 0.003
    base_thickness = 0.004
    corner_radius = 0.018

    lid_width = 0.244
    lid_depth = 0.164
    lid_corner_radius = 0.020
    lid_dome_height = 0.034
    lid_skirt_height = 0.040
    lid_skin_thickness = 0.003
    lid_hinge_y = -(body_depth * 0.5) - 0.007
    lid_hinge_z = body_height + 0.007

    body = model.part("bin_body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=5.4,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    shell_ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(body_width, body_depth, corner_radius, corner_segments=8),
        [
            rounded_rect_profile(
                body_width - 2.0 * wall_thickness,
                body_depth - 2.0 * wall_thickness,
                max(corner_radius - wall_thickness, 0.004),
                corner_segments=8,
            )
        ],
        height=body_height,
        center=True,
    )
    body.visual(
        _mesh("waste_bin_shell", shell_ring),
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
        material=body_white,
        name="body_shell",
    )
    body.visual(
        Box((body_width - 0.004, body_depth - 0.004, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness * 0.5)),
        material=body_white,
        name="base_plate",
    )
    body.visual(
        Box((body_width - 0.050, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, lid_hinge_y + 0.004, body_height - 0.015)),
        material=hinge_gray,
        name="rear_hinge_mount",
    )
    for side_sign, name in ((-1.0, "left_pedal_bracket"), (1.0, "right_pedal_bracket")):
        body.visual(
            Box((0.012, 0.010, 0.034)),
            origin=Origin(
                xyz=(
                    side_sign * 0.055,
                    (body_depth * 0.5) + 0.002,
                    0.040,
                )
            ),
            material=hinge_gray,
            name=name,
        )
    for side_sign, stem_name in (
        (-1.0, "handle_mount_front"),
        (1.0, "handle_mount_rear"),
    ):
        y_pos = side_sign * 0.042
        body.visual(
            Box((0.010, 0.016, 0.016)),
            origin=Origin(xyz=((body_width * 0.5) + 0.002, y_pos, 0.240)),
            material=hinge_gray,
            name=stem_name,
        )
        body.visual(
            Cylinder(radius=0.0025, length=0.008),
            origin=Origin(
                xyz=((body_width * 0.5) + 0.0055, y_pos, 0.2505),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=hinge_gray,
            name=f"{stem_name}_pivot",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_dome_height + lid_skirt_height)),
        mass=1.1,
        origin=Origin(xyz=(0.0, lid_depth * 0.5, -0.005)),
    )

    lid_top = section_loft(
        [
            _loop(lid_width, lid_depth, lid_corner_radius, 0.004, lid_depth * 0.5),
            _loop(lid_width - 0.018, lid_depth - 0.018, lid_corner_radius * 0.92, 0.018, lid_depth * 0.5),
            _loop(lid_width - 0.072, lid_depth - 0.050, lid_corner_radius * 0.78, lid_dome_height, lid_depth * 0.5 + 0.003),
        ]
    )
    lid.visual(_mesh("waste_bin_lid_top", lid_top), material=body_white, name="lid_top_cap")
    lid.visual(
        Box((lid_width - 0.018, lid_skin_thickness, lid_skirt_height)),
        origin=Origin(xyz=(0.0, lid_depth - (lid_skin_thickness * 0.5), -0.016)),
        material=body_white,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_skin_thickness, lid_depth - 0.020, lid_skirt_height)),
        origin=Origin(xyz=((lid_width * 0.5) - (lid_skin_thickness * 0.5), lid_depth * 0.5, -0.016)),
        material=body_white,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_skin_thickness, lid_depth - 0.020, lid_skirt_height)),
        origin=Origin(xyz=(-(lid_width * 0.5) + (lid_skin_thickness * 0.5), lid_depth * 0.5, -0.016)),
        material=body_white,
        name="left_skirt",
    )
    lid.visual(
        Box((0.040, 0.010, 0.018)),
        origin=Origin(xyz=(-0.070, 0.004, -0.002)),
        material=hinge_gray,
        name="left_hinge_tab",
    )
    lid.visual(
        Box((0.040, 0.010, 0.018)),
        origin=Origin(xyz=(0.070, 0.004, -0.002)),
        material=hinge_gray,
        name="right_hinge_tab",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(-0.070, 0.000, 0.001), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_gray,
        name="left_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(0.070, 0.000, 0.001), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_gray,
        name="right_hinge_barrel",
    )

    pedal = model.part("pedal")
    pedal.inertial = Inertial.from_geometry(
        Box((0.130, 0.090, 0.045)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.045, -0.020)),
    )
    pedal.visual(
        Cylinder(radius=0.006, length=0.098),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_gray,
        name="pedal_pivot_rod",
    )
    pedal.visual(
        Box((0.014, 0.050, 0.024)),
        origin=Origin(xyz=(-0.046, 0.025, -0.016)),
        material=pedal_black,
        name="left_pedal_arm",
    )
    pedal.visual(
        Box((0.014, 0.050, 0.024)),
        origin=Origin(xyz=(0.046, 0.025, -0.016)),
        material=pedal_black,
        name="right_pedal_arm",
    )
    pedal.visual(
        Box((0.118, 0.048, 0.012)),
        origin=Origin(xyz=(0.0, 0.070, -0.024)),
        material=pedal_black,
        name="pedal_tread",
    )
    pedal.visual(
        Box((0.094, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, 0.071, -0.0175)),
        material=rubber,
        name="pedal_pad",
    )

    handle = model.part("carry_handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.060, 0.094, 0.132)),
        mass=0.18,
        origin=Origin(xyz=(0.030, 0.0, -0.060)),
    )
    handle_geom = wire_from_points(
        [
            (0.0, -0.042, 0.0),
            (0.016, -0.042, -0.034),
            (0.026, -0.030, -0.080),
            (0.030, 0.000, -0.128),
            (0.026, 0.030, -0.080),
            (0.016, 0.042, -0.034),
            (0.0, 0.042, 0.0),
        ],
        radius=0.004,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.016,
        corner_segments=8,
    )
    handle.visual(_mesh("waste_bin_handle_loop", handle_geom), material=charcoal, name="handle_loop")
    handle.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=Origin(xyz=(0.0, -0.042, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=hinge_gray,
        name="front_handle_pivot",
    )
    handle.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=Origin(xyz=(0.0, 0.042, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=hinge_gray,
        name="rear_handle_pivot",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, lid_hinge_y, lid_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, (body_depth * 0.5) + 0.010, 0.056)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(34.0),
        ),
    )
    model.articulation(
        "body_to_carry_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=((body_width * 0.5) + 0.011, 0.0, 0.254)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(112.0),
        ),
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

    body = object_model.get_part("bin_body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    handle = object_model.get_part("carry_handle")

    lid_joint = object_model.get_articulation("body_to_lid")
    pedal_joint = object_model.get_articulation("body_to_pedal")
    handle_joint = object_model.get_articulation("body_to_carry_handle")

    ctx.check(
        "all prompt parts exist",
        all(part is not None for part in (body, lid, pedal, handle)),
        details=f"parts={[p.name for p in (body, lid, pedal, handle)]}",
    )

    body_dims = _aabb_dims(ctx.part_world_aabb(body))
    ctx.check(
        "bin body is tall and narrow",
        body_dims is not None and body_dims[2] > 0.34 and body_dims[2] > body_dims[0] * 1.4 and body_dims[1] < body_dims[0],
        details=f"body_dims={body_dims}",
    )

    with ctx.pose({lid_joint: 0.0, pedal_joint: 0.0, handle_joint: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_top_cap",
            elem_b="body_shell",
            min_overlap=0.12,
            name="lid covers the top opening",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_top_cap",
            negative_elem="body_shell",
            min_gap=0.006,
            max_gap=0.018,
            name="lid shell sits just above the rim",
        )
        ctx.expect_gap(
            handle,
            body,
            axis="x",
            positive_elem="handle_loop",
            negative_elem="body_shell",
            min_gap=0.005,
            max_gap=0.028,
            name="handle rests just off the side wall",
        )
        ctx.expect_gap(
            pedal,
            body,
            axis="y",
            positive_elem="pedal_tread",
            negative_elem="body_shell",
            min_gap=0.010,
            max_gap=0.090,
            name="pedal sits ahead of the front wall",
        )

    lid_rest = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        lid_open = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on rear hinge",
        lid_rest is not None and lid_open is not None and lid_open[1][2] > lid_rest[1][2] + 0.08,
        details=f"rest={lid_rest}, open={lid_open}",
    )

    pedal_rest = ctx.part_world_aabb(pedal)
    with ctx.pose({pedal_joint: pedal_joint.motion_limits.upper}):
        pedal_pressed = ctx.part_world_aabb(pedal)
    ctx.check(
        "pedal rotates downward when pressed",
        pedal_rest is not None
        and pedal_pressed is not None
        and pedal_pressed[0][2] < pedal_rest[0][2] - 0.020
        and pedal_pressed[1][2] > pedal_rest[1][2] - 0.004,
        details=f"rest={pedal_rest}, pressed={pedal_pressed}",
    )

    handle_rest = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        handle_lifted = ctx.part_world_aabb(handle)
    ctx.check(
        "carry handle lifts upward",
        handle_rest is not None
        and handle_lifted is not None
        and handle_lifted[1][2] > handle_rest[1][2] + 0.035
        and handle_lifted[1][0] > handle_rest[1][0] - 0.005,
        details=f"rest={handle_rest}, lifted={handle_lifted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
