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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


OUTER_TUBE_OUTER_RADIUS = 0.0158
OUTER_TUBE_INNER_RADIUS = 0.0139
OUTER_TUBE_LENGTH = 0.238
COLLAR_OUTER_RADIUS = 0.0205
COLLAR_INNER_RADIUS = 0.0138
COLLAR_Z0 = 0.218
COLLAR_Z1 = 0.255
PRISMATIC_ORIGIN_Z = 0.252
TRAVEL = 0.150
STANCHION_OUTER_RADIUS = 0.0127
STANCHION_INNER_RADIUS = 0.0110
STANCHION_Z0 = -0.220
STANCHION_Z1 = 0.135
LEVER_HINGE_Y = 0.0245
LEVER_HINGE_Z = 0.240


def _tube_shell_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    name: str,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def _add_side_lever_geometry(part, *, side: float, material):
    prefix = "left" if side < 0.0 else "right"
    part.visual(
        Cylinder(radius=0.0028, length=0.0074),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_barrel",
    )
    part.visual(
        Box((0.009, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, side * 0.0045, -0.014)),
        material=material,
        name=f"{prefix}_arm",
    )
    part.visual(
        Box((0.014, 0.008, 0.022)),
        origin=Origin(
            xyz=(0.0, side * 0.0095, -0.038),
            rpy=(side * 0.10, 0.0, 0.0),
        ),
        material=material,
        name=f"{prefix}_paddle",
    )
    part.visual(
        Cylinder(radius=0.0032, length=0.016),
        origin=Origin(
            xyz=(0.0, side * 0.0115, -0.050),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name=f"{prefix}_tip_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_actuation_dropper_seatpost")

    anodized_black = model.material("anodized_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.22, 0.23, 0.24, 1.0))
    matte_charcoal = model.material("matte_charcoal", rgba=(0.10, 0.10, 0.11, 1.0))

    outer_body = model.part("outer_body")
    outer_body.visual(
        _tube_shell_mesh(
            outer_radius=OUTER_TUBE_OUTER_RADIUS,
            inner_radius=OUTER_TUBE_INNER_RADIUS,
            z0=0.0,
            z1=OUTER_TUBE_LENGTH,
            name="outer_body_lower_tube",
        ),
        material=anodized_black,
        name="lower_tube_shell",
    )
    outer_body.visual(
        _tube_shell_mesh(
            outer_radius=COLLAR_OUTER_RADIUS,
            inner_radius=COLLAR_INNER_RADIUS,
            z0=COLLAR_Z0,
            z1=COLLAR_Z1,
            name="outer_body_collar_ring",
        ),
        material=satin_black,
        name="collar_ring_shell",
    )
    outer_body.visual(
        _tube_shell_mesh(
            outer_radius=0.0183,
            inner_radius=0.0134,
            z0=0.247,
            z1=0.262,
            name="outer_body_wiper_cap",
        ),
        material=dark_hardware,
        name="wiper_cap",
    )
    for side_name, side in (("left", -1.0), ("right", 1.0)):
        outer_body.visual(
            Box((0.018, 0.004, 0.012)),
            origin=Origin(xyz=(0.0, side * 0.0218, 0.238)),
            material=dark_hardware,
            name=f"{side_name}_hinge_rib",
        )
        outer_body.visual(
            Box((0.0044, 0.006, 0.011)),
            origin=Origin(xyz=(-0.0060, side * 0.0235, LEVER_HINGE_Z)),
            material=dark_hardware,
            name=f"{side_name}_fork_front",
        )
        outer_body.visual(
            Box((0.0044, 0.006, 0.011)),
            origin=Origin(xyz=(0.0060, side * 0.0235, LEVER_HINGE_Z)),
            material=dark_hardware,
            name=f"{side_name}_fork_rear",
        )

    outer_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.270),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, 0.131)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        _tube_shell_mesh(
            outer_radius=STANCHION_OUTER_RADIUS,
            inner_radius=STANCHION_INNER_RADIUS,
            z0=STANCHION_Z0,
            z1=STANCHION_Z1,
            name="inner_post_stanchion",
        ),
        material=satin_black,
        name="stanchion",
    )
    inner_post.visual(
        _tube_shell_mesh(
            outer_radius=0.0164,
            inner_radius=0.0118,
            z0=0.010,
            z1=0.024,
            name="inner_post_seal_collar",
        ),
        material=dark_hardware,
        name="seal_collar",
    )
    inner_post.visual(
        Box((0.030, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=dark_hardware,
        name="saddle_head_block",
    )
    inner_post.visual(
        Box((0.022, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, -0.014, 0.153)),
        material=dark_hardware,
        name="rear_clamp_yoke",
    )
    inner_post.visual(
        Box((0.022, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.014, 0.153)),
        material=dark_hardware,
        name="front_clamp_yoke",
    )
    inner_post.visual(
        Box((0.032, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
        material=matte_charcoal,
        name="top_clamp_plate",
    )
    inner_post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.390),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, PRISMATIC_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.22,
            lower=0.0,
            upper=TRAVEL,
        ),
    )

    left_lever = model.part("left_lever")
    _add_side_lever_geometry(left_lever, side=-1.0, material=matte_charcoal)
    left_lever.inertial = Inertial.from_geometry(
        Box((0.016, 0.012, 0.056)),
        mass=0.06,
        origin=Origin(xyz=(0.0, -0.007, -0.025)),
    )

    right_lever = model.part("right_lever")
    _add_side_lever_geometry(right_lever, side=1.0, material=matte_charcoal)
    right_lever.inertial = Inertial.from_geometry(
        Box((0.016, 0.012, 0.056)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.007, -0.025)),
    )

    model.articulation(
        "outer_to_left_lever",
        ArticulationType.REVOLUTE,
        parent=outer_body,
        child=left_lever,
        origin=Origin(xyz=(0.0, -LEVER_HINGE_Y, LEVER_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=6.0,
            lower=-0.30,
            upper=0.70,
        ),
    )
    model.articulation(
        "outer_to_right_lever",
        ArticulationType.REVOLUTE,
        parent=outer_body,
        child=right_lever,
        origin=Origin(xyz=(0.0, LEVER_HINGE_Y, LEVER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=6.0,
            lower=-0.30,
            upper=0.70,
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

    outer_body = object_model.get_part("outer_body")
    inner_post = object_model.get_part("inner_post")
    left_lever = object_model.get_part("left_lever")
    right_lever = object_model.get_part("right_lever")

    slide_joint = object_model.get_articulation("outer_to_inner")
    left_hinge = object_model.get_articulation("outer_to_left_lever")
    right_hinge = object_model.get_articulation("outer_to_right_lever")

    slide_upper = slide_joint.motion_limits.upper or 0.0
    left_upper = right_upper = 0.0
    if left_hinge.motion_limits is not None and left_hinge.motion_limits.upper is not None:
        left_upper = left_hinge.motion_limits.upper
    if right_hinge.motion_limits is not None and right_hinge.motion_limits.upper is not None:
        right_upper = right_hinge.motion_limits.upper

    ctx.expect_within(
        inner_post,
        outer_body,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="lower_tube_shell",
        margin=0.0015,
        name="stanchion stays centered in the lower tube at rest",
    )
    ctx.expect_overlap(
        inner_post,
        outer_body,
        axes="z",
        elem_a="stanchion",
        elem_b="lower_tube_shell",
        min_overlap=0.18,
        name="collapsed seatpost keeps deep insertion in the outer tube",
    )
    ctx.expect_origin_distance(
        left_lever,
        right_lever,
        axes="y",
        min_dist=0.045,
        name="dual actuators remain mounted on opposite sides of the collar",
    )

    inner_rest = ctx.part_world_position(inner_post)
    left_tip_rest = _aabb_center(ctx.part_element_world_aabb(left_lever, elem="left_tip_pad"))
    right_tip_rest = _aabb_center(ctx.part_element_world_aabb(right_lever, elem="right_tip_pad"))

    with ctx.pose({slide_joint: slide_upper}):
        ctx.expect_within(
            inner_post,
            outer_body,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="lower_tube_shell",
            margin=0.0015,
            name="stanchion stays centered in the lower tube at full extension",
        )
        ctx.expect_overlap(
            inner_post,
            outer_body,
            axes="z",
            elem_a="stanchion",
            elem_b="lower_tube_shell",
            min_overlap=0.055,
            name="extended seatpost still retains insertion in the outer tube",
        )
        inner_extended = ctx.part_world_position(inner_post)

    ctx.check(
        "inner tube extends upward",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[2] > inner_rest[2] + 0.12,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    with ctx.pose({left_hinge: left_upper, right_hinge: right_upper}):
        left_tip_open = _aabb_center(ctx.part_element_world_aabb(left_lever, elem="left_tip_pad"))
        right_tip_open = _aabb_center(ctx.part_element_world_aabb(right_lever, elem="right_tip_pad"))

    ctx.check(
        "left lever pivots outward",
        left_tip_rest is not None
        and left_tip_open is not None
        and left_tip_open[1] < left_tip_rest[1] - 0.004,
        details=f"rest={left_tip_rest}, open={left_tip_open}",
    )
    ctx.check(
        "right lever pivots outward",
        right_tip_rest is not None
        and right_tip_open is not None
        and right_tip_open[1] > right_tip_rest[1] + 0.004,
        details=f"rest={right_tip_rest}, open={right_tip_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
