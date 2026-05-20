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
)


def _rounded_panel_mesh(
    width: float,
    height: float,
    thickness: float,
    radius: float,
    name: str,
):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, radius),
        thickness,
        cap=True,
        closed=True,
    )
    geom.rotate_x(pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_router")

    body_width = 0.240
    body_depth = 0.034
    body_height = 0.155
    front_y = body_depth / 2.0
    top_z = body_height / 2.0
    bottom_z = -body_height / 2.0

    cover_width = 0.214
    cover_height = 0.058
    cover_thickness = 0.004
    cover_hinge_y = front_y
    cover_hinge_z = bottom_z + 0.003

    shell_mat = model.material("shell_white", rgba=(0.95, 0.95, 0.97, 1.0))
    trim_mat = model.material("trim_light_gray", rgba=(0.83, 0.84, 0.86, 1.0))
    dark_mat = model.material("dark_plastic", rgba=(0.14, 0.15, 0.17, 1.0))
    led_mat = model.material("status_tint", rgba=(0.45, 0.72, 0.78, 0.85))
    metal_mat = model.material("port_metal", rgba=(0.65, 0.68, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_panel_mesh(
            body_width,
            body_height,
            body_depth,
            radius=0.020,
            name="body_shell_mesh",
        ),
        material=shell_mat,
        name="body_shell",
    )
    body.visual(
        Box((0.170, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.001, top_z + 0.0015)),
        material=trim_mat,
        name="top_mount_rail",
    )
    body.visual(
        Box((0.024, 0.014, 0.010)),
        origin=Origin(xyz=(-0.068, -0.001, top_z + 0.0015)),
        material=trim_mat,
        name="left_antenna_pad",
    )
    body.visual(
        Box((0.024, 0.014, 0.010)),
        origin=Origin(xyz=(0.068, -0.001, top_z + 0.0015)),
        material=trim_mat,
        name="right_antenna_pad",
    )
    body.visual(
        Box((0.108, 0.0015, 0.010)),
        origin=Origin(xyz=(0.0, front_y + 0.00025, 0.030)),
        material=led_mat,
        name="status_window",
    )
    body.visual(
        Box((0.176, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, front_y - 0.003, -0.045)),
        material=trim_mat,
        name="port_bay_support",
    )
    body.visual(
        Box((0.172, 0.004, 0.036)),
        origin=Origin(xyz=(0.0, front_y - 0.0085, -0.045)),
        material=dark_mat,
        name="port_recess",
    )
    for index, x_pos in enumerate((-0.060, -0.020, 0.020, 0.060), start=1):
        body.visual(
            Box((0.030, 0.007, 0.014)),
            origin=Origin(xyz=(x_pos, front_y - 0.0105, -0.046)),
            material=metal_mat,
            name=f"ethernet_port_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=0.78,
    )

    service_cover = model.part("service_cover")
    service_cover.visual(
        _rounded_panel_mesh(
            cover_width,
            cover_height,
            cover_thickness,
            radius=0.010,
            name="service_cover_mesh",
        ),
        origin=Origin(xyz=(0.0, cover_thickness / 2.0, cover_height / 2.0)),
        material=shell_mat,
        name="cover_shell",
    )
    service_cover.visual(
        Box((0.072, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.0045, 0.046)),
        material=trim_mat,
        name="cover_pull_lip",
    )
    service_cover.inertial = Inertial.from_geometry(
        Box((cover_width, cover_thickness, cover_height)),
        mass=0.10,
        origin=Origin(xyz=(0.0, cover_thickness / 2.0, cover_height / 2.0)),
    )

    antenna_width = 0.016
    antenna_depth = 0.004
    antenna_height = 0.072

    for antenna_name in ("left_antenna", "right_antenna"):
        antenna = model.part(antenna_name)
        antenna.visual(
            Cylinder(radius=0.004, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.004), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_mat,
            name="hinge_barrel",
        )
        antenna.visual(
            Box((0.010, 0.006, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=dark_mat,
            name="hinge_stalk",
        )
        antenna.visual(
            _rounded_panel_mesh(
                antenna_width,
                antenna_height,
                antenna_depth,
                radius=0.006,
                name=f"{antenna_name}_blade_mesh",
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.046)),
            material=dark_mat,
            name="antenna_blade",
        )
        antenna.inertial = Inertial.from_geometry(
            Box((antenna_width, antenna_depth, antenna_height + 0.016)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, 0.042)),
        )

    model.articulation(
        "body_to_service_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_cover,
        origin=Origin(xyz=(0.0, cover_hinge_y, cover_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.40,
        ),
    )
    model.articulation(
        "body_to_left_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=model.get_part("left_antenna"),
        origin=Origin(xyz=(-0.068, 0.0, top_z + 0.0065)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-0.35,
            upper=1.05,
        ),
    )
    model.articulation(
        "body_to_right_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=model.get_part("right_antenna"),
        origin=Origin(xyz=(0.068, 0.0, top_z + 0.0065)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-0.35,
            upper=1.05,
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

    body = object_model.get_part("body")
    service_cover = object_model.get_part("service_cover")
    left_antenna = object_model.get_part("left_antenna")
    right_antenna = object_model.get_part("right_antenna")
    cover_joint = object_model.get_articulation("body_to_service_cover")
    left_joint = object_model.get_articulation("body_to_left_antenna")
    right_joint = object_model.get_articulation("body_to_right_antenna")

    ctx.check(
        "router parts exist",
        all(part is not None for part in (body, service_cover, left_antenna, right_antenna)),
    )
    ctx.check(
        "service cover uses lower horizontal hinge axis",
        cover_joint.axis == (-1.0, 0.0, 0.0),
        details=f"axis={cover_joint.axis}",
    )
    ctx.check(
        "antenna hinges pitch on matching top-edge axes",
        left_joint.axis == (-1.0, 0.0, 0.0) and right_joint.axis == (-1.0, 0.0, 0.0),
        details=f"left={left_joint.axis}, right={right_joint.axis}",
    )

    with ctx.pose({cover_joint: 0.0}):
        ctx.expect_gap(
            service_cover,
            body,
            axis="y",
            positive_elem="cover_shell",
            negative_elem="body_shell",
            max_gap=0.0015,
            max_penetration=0.0,
            name="service cover closes nearly flush to the router face",
        )
        ctx.expect_gap(
            service_cover,
            body,
            axis="y",
            positive_elem="cover_shell",
            negative_elem="port_recess",
            min_gap=0.004,
            max_gap=0.008,
            name="service cover leaves a recessed cavity over the ports",
        )
        ctx.expect_overlap(
            service_cover,
            body,
            axes="xz",
            elem_a="cover_shell",
            elem_b="body_shell",
            min_overlap=0.050,
            name="service cover spans the lower front portion of the body",
        )

    closed_cover_aabb = None
    open_cover_aabb = None
    with ctx.pose({cover_joint: 0.0}):
        closed_cover_aabb = ctx.part_world_aabb(service_cover)
    with ctx.pose({cover_joint: 1.15}):
        open_cover_aabb = ctx.part_world_aabb(service_cover)

    ctx.check(
        "service cover swings downward when opened",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.025
        and open_cover_aabb[1][2] < closed_cover_aabb[1][2] - 0.025,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    rest_left_aabb = None
    tipped_left_aabb = None
    rest_right_aabb = None
    tipped_right_aabb = None
    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        rest_left_aabb = ctx.part_world_aabb(left_antenna)
        rest_right_aabb = ctx.part_world_aabb(right_antenna)
    with ctx.pose({left_joint: 0.75, right_joint: 0.75}):
        tipped_left_aabb = ctx.part_world_aabb(left_antenna)
        tipped_right_aabb = ctx.part_world_aabb(right_antenna)

    ctx.check(
        "left antenna pitches forward from the top edge hinge",
        rest_left_aabb is not None
        and tipped_left_aabb is not None
        and tipped_left_aabb[1][1] > rest_left_aabb[1][1] + 0.020
        and tipped_left_aabb[1][2] < rest_left_aabb[1][2] - 0.012,
        details=f"rest={rest_left_aabb}, tipped={tipped_left_aabb}",
    )
    ctx.check(
        "right antenna pitches forward from the top edge hinge",
        rest_right_aabb is not None
        and tipped_right_aabb is not None
        and tipped_right_aabb[1][1] > rest_right_aabb[1][1] + 0.020
        and tipped_right_aabb[1][2] < rest_right_aabb[1][2] - 0.012,
        details=f"rest={rest_right_aabb}, tipped={tipped_right_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
