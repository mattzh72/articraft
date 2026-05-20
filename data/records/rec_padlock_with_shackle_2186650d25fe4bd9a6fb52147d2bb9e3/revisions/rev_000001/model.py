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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _body_shell_mesh(*, width: float, depth: float, height: float, corner_radius: float):
    shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, depth, corner_radius, corner_segments=8),
        height,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(shell, "marine_padlock_body_shell")


def _shackle_mesh() :
    path_points = [
        (0.0000, 0.0000, -0.0020),
        (0.0000, 0.0000, 0.0120),
        (-0.0015, 0.0000, 0.0210),
        (-0.0080, 0.0000, 0.0310),
        (-0.0155, 0.0000, 0.0340),
        (-0.0230, 0.0000, 0.0310),
        (-0.0295, 0.0000, 0.0210),
        (-0.0310, 0.0000, 0.0120),
        (-0.0310, 0.0000, -0.0020),
    ]
    shackle_geom = tube_from_spline_points(
        path_points,
        radius=0.0048,
        samples_per_segment=20,
        radial_segments=22,
        cap_ends=True,
    )
    return mesh_from_geometry(shackle_geom, "marine_padlock_shackle")


def _key_escutcheon_mesh():
    escutcheon = ExtrudeWithHolesGeometry(
        _circle_profile(0.0086, segments=36),
        [rounded_rect_profile(0.0034, 0.0056, 0.0008, corner_segments=4)],
        0.0022,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(escutcheon, "marine_padlock_key_escutcheon")


def _add_socket_collar(
    body,
    *,
    center_x: float,
    side_name: str,
    outer_width: float,
    outer_depth: float,
    opening_width: float,
    opening_depth: float,
    base_z: float,
    height: float,
    material,
) -> None:
    side_wall_width = (outer_width - opening_width) * 0.5
    front_wall_depth = (outer_depth - opening_depth) * 0.5
    wall_z = base_z + height * 0.5
    outer_x = center_x + (opening_width * 0.5 + side_wall_width * 0.5)
    inner_x = center_x - (opening_width * 0.5 + side_wall_width * 0.5)
    front_y = opening_depth * 0.5 + front_wall_depth * 0.5
    rear_y = -front_y

    if side_name == "left":
        outer_x, inner_x = inner_x, outer_x

    body.visual(
        Box((side_wall_width, outer_depth, height)),
        origin=Origin(xyz=(outer_x, 0.0, wall_z)),
        material=material,
        name=f"{side_name}_socket_outer_wall",
    )
    body.visual(
        Box((side_wall_width, outer_depth, height)),
        origin=Origin(xyz=(inner_x, 0.0, wall_z)),
        material=material,
        name=f"{side_name}_socket_inner_wall",
    )
    body.visual(
        Box((opening_width, front_wall_depth, height)),
        origin=Origin(xyz=(center_x, front_y, wall_z)),
        material=material,
        name=f"{side_name}_socket_front_wall",
    )
    body.visual(
        Box((opening_width, front_wall_depth, height)),
        origin=Origin(xyz=(center_x, rear_y, wall_z)),
        material=material,
        name=f"{side_name}_socket_rear_wall",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_padlock")

    body_blue = model.material("body_blue", rgba=(0.16, 0.27, 0.38, 1.0))
    bumper_black = model.material("bumper_black", rgba=(0.11, 0.12, 0.13, 1.0))
    shackle_coat = model.material("shackle_coat", rgba=(0.88, 0.33, 0.13, 1.0))
    stainless = model.material("stainless", rgba=(0.77, 0.80, 0.83, 1.0))
    cavity_black = model.material("cavity_black", rgba=(0.05, 0.05, 0.06, 1.0))
    flap_rubber = model.material("flap_rubber", rgba=(0.13, 0.14, 0.15, 1.0))

    body_width = 0.055
    body_depth = 0.028
    body_height = 0.042
    socket_base_z = body_height
    socket_height = 0.007
    socket_center_offset = 0.0155

    body = model.part("body")
    body.visual(
        _body_shell_mesh(
            width=body_width,
            depth=body_depth,
            height=body_height,
            corner_radius=0.0042,
        ),
        material=body_blue,
        name="body_shell",
    )
    body.visual(
        Box((0.048, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=bumper_black,
        name="lower_bumper",
    )
    body.visual(
        Box((0.030, 0.0008, 0.024)),
        origin=Origin(xyz=(0.0, body_depth * 0.5 - 0.0004, 0.018)),
        material=bumper_black,
        name="front_seal_panel",
    )
    body.visual(
        Box((0.0026, 0.0010, 0.0042)),
        origin=Origin(xyz=(0.0, body_depth * 0.5 - 0.0010, 0.0195)),
        material=cavity_black,
        name="key_recess",
    )
    body.visual(
        Box((0.016, body_depth, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, socket_base_z + 0.003)),
        material=body_blue,
        name="top_bridge",
    )
    _add_socket_collar(
        body,
        center_x=-socket_center_offset,
        side_name="left",
        outer_width=0.017,
        outer_depth=0.020,
        opening_width=0.012,
        opening_depth=0.012,
        base_z=socket_base_z,
        height=socket_height,
        material=body_blue,
    )
    _add_socket_collar(
        body,
        center_x=socket_center_offset,
        side_name="right",
        outer_width=0.017,
        outer_depth=0.020,
        opening_width=0.012,
        opening_depth=0.012,
        base_z=socket_base_z,
        height=socket_height,
        material=body_blue,
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height + socket_height)),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, (body_height + socket_height) * 0.5)),
    )

    key_cylinder = model.part("key_cylinder")
    key_cylinder.visual(
        _key_escutcheon_mesh(),
        origin=Origin(xyz=(0.0, 0.0011, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=stainless,
        name="escutcheon",
    )
    key_cylinder.inertial = Inertial.from_geometry(
        Box((0.018, 0.003, 0.018)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
    )
    model.articulation(
        "body_to_key_cylinder",
        ArticulationType.FIXED,
        parent=body,
        child=key_cylinder,
        origin=Origin(xyz=(0.0, body_depth * 0.5, 0.0195)),
    )

    shackle = model.part("shackle")
    shackle.visual(
        _shackle_mesh(),
        material=shackle_coat,
        name="shackle_coating",
    )
    shackle.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002), rpy=(0.0, 0.0, 0.0)),
        material=stainless,
        name="retained_leg_tip",
    )
    shackle.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(-0.031, 0.0, -0.002), rpy=(0.0, 0.0, 0.0)),
        material=stainless,
        name="free_leg_tip",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((0.041, 0.011, 0.038)),
        mass=0.16,
        origin=Origin(xyz=(-0.0155, 0.0, 0.015)),
    )
    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(socket_center_offset, 0.0, 0.044)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    drain_flap = model.part("drain_flap")
    drain_flap.visual(
        Cylinder(radius=0.0013, length=0.008),
        origin=Origin(xyz=(0.0, 0.0013, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=flap_rubber,
        name="flap_hinge_barrel",
    )
    drain_flap.visual(
        Box((0.010, 0.0018, 0.009)),
        origin=Origin(xyz=(0.0, 0.0009, -0.0045)),
        material=flap_rubber,
        name="flap_panel",
    )
    drain_flap.visual(
        Box((0.006, 0.0026, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0013, -0.0090)),
        material=flap_rubber,
        name="flap_lip",
    )
    drain_flap.inertial = Inertial.from_geometry(
        Box((0.010, 0.003, 0.010)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0015, -0.004)),
    )
    model.articulation(
        "body_to_drain_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drain_flap,
        origin=Origin(xyz=(0.0, body_depth * 0.5, 0.0105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=4.0,
            lower=0.0,
            upper=1.3,
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
    shackle = object_model.get_part("shackle")
    key_cylinder = object_model.get_part("key_cylinder")
    drain_flap = object_model.get_part("drain_flap")

    shackle_joint = object_model.get_articulation("body_to_shackle")
    flap_joint = object_model.get_articulation("body_to_drain_flap")

    ctx.expect_gap(
        key_cylinder,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.0015,
        name="key cylinder mounts flush on the front face",
    )
    ctx.expect_gap(
        drain_flap,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.0015,
        name="drain flap rests just proud of the body face",
    )
    ctx.expect_overlap(
        shackle,
        body,
        axes="x",
        min_overlap=0.020,
        name="closed shackle spans the body width",
    )

    body_aabb = ctx.part_world_aabb(body)
    free_tip_closed = ctx.part_element_world_aabb(shackle, elem="free_leg_tip")
    retained_tip_closed = ctx.part_element_world_aabb(shackle, elem="retained_leg_tip")
    body_top = body_aabb[1][2] if body_aabb is not None else None
    ctx.check(
        "closed shackle tips sit inside the top entry collars",
        body_top is not None
        and free_tip_closed is not None
        and retained_tip_closed is not None
        and free_tip_closed[1][2] <= body_top + 0.0005
        and retained_tip_closed[1][2] <= body_top + 0.0005
        and free_tip_closed[0][2] >= body_top - 0.010
        and retained_tip_closed[0][2] >= body_top - 0.010,
        details=(
            f"body_top={body_top}, free_tip_closed={free_tip_closed}, "
            f"retained_tip_closed={retained_tip_closed}"
        ),
    )

    shackle_upper = (
        shackle_joint.motion_limits.upper
        if shackle_joint.motion_limits is not None and shackle_joint.motion_limits.upper is not None
        else 1.25
    )
    with ctx.pose({shackle_joint: shackle_upper}):
        free_tip_open = ctx.part_element_world_aabb(shackle, elem="free_leg_tip")
        retained_tip_open = ctx.part_element_world_aabb(shackle, elem="retained_leg_tip")

    ctx.check(
        "shackle opens upward from the free side",
        free_tip_closed is not None
        and free_tip_open is not None
        and retained_tip_open is not None
        and free_tip_open[0][2] > free_tip_closed[0][2] + 0.020
        and free_tip_open[0][2] > retained_tip_open[0][2] + 0.012,
        details=(
            f"free_tip_closed={free_tip_closed}, free_tip_open={free_tip_open}, "
            f"retained_tip_open={retained_tip_open}"
        ),
    )

    flap_upper = (
        flap_joint.motion_limits.upper
        if flap_joint.motion_limits is not None and flap_joint.motion_limits.upper is not None
        else 1.3
    )
    flap_closed = ctx.part_element_world_aabb(drain_flap, elem="flap_panel")
    with ctx.pose({flap_joint: flap_upper}):
        flap_open = ctx.part_element_world_aabb(drain_flap, elem="flap_panel")

    ctx.check(
        "drain flap swings outward on its small hinge",
        flap_closed is not None
        and flap_open is not None
        and flap_open[1][1] > flap_closed[1][1] + 0.003
        and flap_open[0][2] > flap_closed[0][2] + 0.002,
        details=f"flap_closed={flap_closed}, flap_open={flap_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
