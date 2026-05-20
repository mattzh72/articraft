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
    Sphere,
    TestContext,
    TestReport,
)


POST_RADIUS = 0.028
POST_SHAFT_LENGTH = 1.000
POST_SHAFT_CENTER_Z = 0.420
POST_CAP_CENTER_Z = 0.920
SOCKET_FLANGE_RADIUS = 0.073
SOCKET_DEPTH = 1.060
RETRACTED_TRAVEL = -0.945
SOCKET_CLEAR_RADIUS = 0.041
SOCKET_SEGMENTS = 48
SOCKET_FLANGE_THICKNESS = 0.006
SOCKET_FLANGE_CENTER_RADIUS = 0.057
SOCKET_FLANGE_RADIAL_THICKNESS = 0.032
SOCKET_FLANGE_TANGENTIAL_WIDTH = 0.010
SOCKET_GUIDE_CENTER_RADIUS = 0.036
SOCKET_GUIDE_RADIAL_THICKNESS = 0.016
SOCKET_GUIDE_TANGENTIAL_WIDTH = 0.006
SOCKET_GUIDE_LENGTH = 0.086
SOCKET_STAVE_CENTER_RADIUS = 0.048
SOCKET_STAVE_RADIAL_THICKNESS = 0.010
SOCKET_STAVE_TANGENTIAL_WIDTH = 0.008
SOCKET_STAVE_LENGTH = 1.026


def _polar_box_origin(radius: float, angle: float, z_center: float) -> Origin:
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z_center),
        rpy=(0.0, 0.0, angle),
    )


def _add_segmented_ring(
    part,
    *,
    segment_count: int,
    center_radius: float,
    radial_thickness: float,
    tangential_width: float,
    thickness: float,
    z_center: float,
    material,
    name_prefix: str,
) -> None:
    for index in range(segment_count):
        angle = 2.0 * math.pi * index / segment_count
        part.visual(
            Box((radial_thickness, tangential_width, thickness)),
            origin=_polar_box_origin(center_radius, angle, z_center),
            material=material,
            name=f"{name_prefix}_{index:02d}",
        )


def _add_segmented_sleeve(
    part,
    *,
    segment_count: int,
    center_radius: float,
    radial_thickness: float,
    tangential_width: float,
    length: float,
    z_center: float,
    material,
    name_prefix: str,
) -> None:
    for index in range(segment_count):
        angle = 2.0 * math.pi * index / segment_count
        part.visual(
            Box((radial_thickness, tangential_width, length)),
            origin=_polar_box_origin(center_radius, angle, z_center),
            material=material,
            name=f"{name_prefix}_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_post_barrier")

    chrome = model.material("chrome", rgba=(0.83, 0.85, 0.88, 1.0))
    stainless = model.material("stainless", rgba=(0.63, 0.66, 0.70, 1.0))

    ground_socket = model.part("ground_socket")
    _add_segmented_ring(
        ground_socket,
        segment_count=SOCKET_SEGMENTS,
        center_radius=SOCKET_FLANGE_CENTER_RADIUS,
        radial_thickness=SOCKET_FLANGE_RADIAL_THICKNESS,
        tangential_width=SOCKET_FLANGE_TANGENTIAL_WIDTH,
        thickness=SOCKET_FLANGE_THICKNESS,
        z_center=-SOCKET_FLANGE_THICKNESS * 0.5,
        material=stainless,
        name_prefix="socket_flange",
    )
    _add_segmented_sleeve(
        ground_socket,
        segment_count=SOCKET_SEGMENTS,
        center_radius=SOCKET_GUIDE_CENTER_RADIUS,
        radial_thickness=SOCKET_GUIDE_RADIAL_THICKNESS,
        tangential_width=SOCKET_GUIDE_TANGENTIAL_WIDTH,
        length=SOCKET_GUIDE_LENGTH,
        z_center=-SOCKET_GUIDE_LENGTH * 0.5,
        material=stainless,
        name_prefix="socket_guide",
    )
    _add_segmented_sleeve(
        ground_socket,
        segment_count=SOCKET_SEGMENTS,
        center_radius=SOCKET_STAVE_CENTER_RADIUS,
        radial_thickness=SOCKET_STAVE_RADIAL_THICKNESS,
        tangential_width=SOCKET_STAVE_TANGENTIAL_WIDTH,
        length=SOCKET_STAVE_LENGTH,
        z_center=-(SOCKET_FLANGE_THICKNESS + SOCKET_STAVE_LENGTH) * 0.5,
        material=stainless,
        name_prefix="socket_sleeve",
    )
    ground_socket.inertial = Inertial.from_geometry(
        Cylinder(radius=SOCKET_FLANGE_RADIUS, length=SOCKET_DEPTH),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, -SOCKET_DEPTH * 0.5)),
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, POST_SHAFT_CENTER_Z)),
        material=chrome,
        name="post_shaft",
    )
    post.visual(
        Sphere(radius=POST_RADIUS),
        origin=Origin(xyz=(0.0, 0.0, POST_CAP_CENTER_Z)),
        material=chrome,
        name="post_cap",
    )
    post.inertial = Inertial.from_geometry(
        Box((POST_RADIUS * 2.0, POST_RADIUS * 2.0, 1.02)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
    )

    model.articulation(
        "socket_to_post",
        ArticulationType.PRISMATIC,
        parent=ground_socket,
        child=post,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.25,
            lower=RETRACTED_TRAVEL,
            upper=0.0,
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

    ground_socket = object_model.get_part("ground_socket")
    post = object_model.get_part("post")
    slide = object_model.get_articulation("socket_to_post")

    ctx.expect_origin_distance(
        ground_socket,
        post,
        axes="xy",
        max_dist=0.001,
        name="post stays centered over the socket",
    )
    ctx.expect_overlap(
        post,
        ground_socket,
        axes="z",
        elem_a="post_shaft",
        min_overlap=0.075,
        name="extended post retains insertion in the socket",
    )
    ctx.expect_contact(
        post,
        ground_socket,
        contact_tol=1e-6,
        name="post is supported by the socket guide collar",
    )
    ctx.expect_gap(
        post,
        ground_socket,
        axis="z",
        positive_elem="post_cap",
        min_gap=0.89,
        name="extended post stands well above the flush socket",
    )

    rest_cap_aabb = ctx.part_element_world_aabb(post, elem="post_cap")
    rest_cap_top = rest_cap_aabb[1][2] if rest_cap_aabb is not None else None

    with ctx.pose({slide: RETRACTED_TRAVEL}):
        ctx.expect_origin_distance(
            ground_socket,
            post,
            axes="xy",
            max_dist=0.001,
            name="retracted post remains coaxial with the socket",
        )
        ctx.expect_overlap(
            post,
            ground_socket,
            axes="z",
            elem_a="post_shaft",
            min_overlap=0.98,
            name="retracted post stores inside the full socket depth",
        )

        cap_aabb = ctx.part_element_world_aabb(post, elem="post_cap")
        socket_aabb = ctx.part_world_aabb(ground_socket)
        cap_top = cap_aabb[1][2] if cap_aabb is not None else None
        socket_top = socket_aabb[1][2] if socket_aabb is not None else None

        ctx.check(
            "retracted cap sits nearly flush with the bezel",
            cap_top is not None
            and socket_top is not None
            and socket_top - 0.002 <= cap_top <= socket_top + 0.006,
            details=f"cap_top={cap_top}, socket_top={socket_top}",
        )
        ctx.check(
            "prismatic travel retracts the post downward into the socket",
            rest_cap_top is not None
            and cap_top is not None
            and rest_cap_top - cap_top >= 0.93,
            details=f"rest_cap_top={rest_cap_top}, retracted_cap_top={cap_top}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
