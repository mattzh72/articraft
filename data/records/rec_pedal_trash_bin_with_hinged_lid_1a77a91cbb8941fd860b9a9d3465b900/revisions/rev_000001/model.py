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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_HEIGHT = 0.46
BODY_OUTER_PROFILE = [
    (-0.140, -0.135),
    (0.030, -0.125),
    (0.095, -0.095),
    (0.135, -0.045),
    (0.148, 0.000),
    (0.135, 0.045),
    (0.095, 0.095),
    (0.030, 0.125),
    (-0.140, 0.135),
]
BODY_INNER_PROFILE = [
    (-0.132, -0.127),
    (0.024, -0.118),
    (0.083, -0.088),
    (0.120, -0.040),
    (0.132, 0.000),
    (0.120, 0.040),
    (0.083, 0.088),
    (0.024, 0.118),
    (-0.132, 0.127),
]
FRONT_WALL_PROFILE = [
    (-0.028, -0.126),
    (0.030, -0.125),
    (0.095, -0.095),
    (0.135, -0.045),
    (0.148, 0.000),
    (0.135, 0.045),
    (0.095, 0.095),
    (0.030, 0.125),
    (-0.028, 0.126),
    (-0.018, 0.112),
    (0.015, 0.106),
    (0.076, 0.080),
    (0.115, 0.037),
    (0.126, 0.000),
    (0.115, -0.037),
    (0.076, -0.080),
    (0.015, -0.106),
    (-0.018, -0.112),
]
LEFT_LID_PROFILE = [
    (0.000, -0.054),
    (0.000, 0.054),
    (0.180, 0.054),
    (0.232, 0.048),
    (0.268, 0.030),
    (0.283, 0.006),
    (0.286, -0.022),
    (0.288, -0.054),
]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_profile_y(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(x, -y) for x, y in reversed(profile)]


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_lid_step_bin")

    brushed_steel = model.material("brushed_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    pedal_rubber = model.material("pedal_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.22, 0.23, 0.25, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.30, 0.28, BODY_HEIGHT)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    body.visual(
        Box((0.012, 0.266, BODY_HEIGHT - 0.012)),
        origin=Origin(xyz=(-0.134, 0.0, 0.236)),
        material=brushed_steel,
        name="rear_wall",
    )
    body.visual(
        Box((0.116, 0.012, BODY_HEIGHT - 0.012)),
        origin=Origin(xyz=(-0.078, 0.127, 0.236)),
        material=brushed_steel,
        name="left_wall",
    )
    body.visual(
        Box((0.116, 0.012, BODY_HEIGHT - 0.012)),
        origin=Origin(xyz=(-0.078, -0.127, 0.236)),
        material=brushed_steel,
        name="right_wall",
    )
    body.visual(
        _mesh("step_bin_front_wall", ExtrudeGeometry.from_z0(FRONT_WALL_PROFILE, BODY_HEIGHT - 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=brushed_steel,
        name="body_shell",
    )

    floor_geom = ExtrudeGeometry.from_z0(BODY_INNER_PROFILE, 0.012)
    body.visual(
        _mesh("step_bin_body_floor", floor_geom),
        material=brushed_steel,
        name="body_floor",
    )
    body.visual(
        Box((0.076, 0.024, 0.024)),
        origin=Origin(xyz=(0.120, 0.097, 0.030)),
        material=hinge_dark,
        name="left_pedal_web",
    )
    body.visual(
        Box((0.076, 0.024, 0.024)),
        origin=Origin(xyz=(0.120, -0.097, 0.030)),
        material=hinge_dark,
        name="right_pedal_web",
    )
    body.visual(
        Box((0.020, 0.024, 0.054)),
        origin=Origin(xyz=(0.158, 0.097, 0.039)),
        material=hinge_dark,
        name="left_pedal_bracket",
    )
    body.visual(
        Box((0.020, 0.024, 0.054)),
        origin=Origin(xyz=(0.158, -0.097, 0.039)),
        material=hinge_dark,
        name="right_pedal_bracket",
    )
    body.visual(
        Box((0.006, 0.220, 0.020)),
        origin=Origin(xyz=(-0.134, 0.0, BODY_HEIGHT - 0.019)),
        material=hinge_dark,
        name="rear_hinge_rail",
    )

    left_lid = model.part("left_lid")
    left_lid.inertial = Inertial.from_geometry(
        Box((0.26, 0.11, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.130, 0.0, -0.008)),
    )
    left_lid.visual(
        _mesh("left_lid_panel_mesh", ExtrudeGeometry.from_z0(LEFT_LID_PROFILE, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_plastic,
        name="left_panel",
    )
    left_lid.visual(
        Cylinder(radius=0.005, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(pi * 0.5, 0.0, 0.0)),
        material=hinge_dark,
        name="left_hinge_barrel",
    )
    left_lid.visual(
        Box((0.010, 0.036, 0.014)),
        origin=Origin(xyz=(0.281, -0.016, -0.003)),
        material=dark_plastic,
        name="left_front_lip",
    )

    right_lid = model.part("right_lid")
    right_lid.inertial = Inertial.from_geometry(
        Box((0.26, 0.11, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.130, 0.0, -0.008)),
    )
    right_lid.visual(
        _mesh("right_lid_panel_mesh", ExtrudeGeometry.from_z0(_mirror_profile_y(LEFT_LID_PROFILE), 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_plastic,
        name="right_panel",
    )
    right_lid.visual(
        Cylinder(radius=0.005, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(pi * 0.5, 0.0, 0.0)),
        material=hinge_dark,
        name="right_hinge_barrel",
    )
    right_lid.visual(
        Box((0.010, 0.036, 0.014)),
        origin=Origin(xyz=(0.281, 0.016, -0.003)),
        material=dark_plastic,
        name="right_front_lip",
    )

    pedal = model.part("pedal")
    pedal.inertial = Inertial.from_geometry(
        Box((0.14, 0.18, 0.04)),
        mass=0.25,
        origin=Origin(xyz=(0.072, 0.0, -0.014)),
    )
    pedal.visual(
        Cylinder(radius=0.006, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=hinge_dark,
        name="pedal_axle",
    )
    pedal.visual(
        Box((0.060, 0.014, 0.010)),
        origin=Origin(xyz=(0.034, 0.056, -0.009)),
        material=hinge_dark,
        name="left_pedal_arm",
    )
    pedal.visual(
        Box((0.060, 0.014, 0.010)),
        origin=Origin(xyz=(0.034, -0.056, -0.009)),
        material=hinge_dark,
        name="right_pedal_arm",
    )
    pedal.visual(
        Box((0.094, 0.176, 0.012)),
        origin=Origin(xyz=(0.082, 0.0, -0.020)),
        material=pedal_rubber,
        name="pedal_pad",
    )
    pedal.visual(
        Box((0.082, 0.130, 0.004)),
        origin=Origin(xyz=(0.082, 0.0, -0.012)),
        material=dark_plastic,
        name="pedal_top_trim",
    )

    model.articulation(
        "body_to_left_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_lid,
        origin=Origin(xyz=(-0.126, 0.059, BODY_HEIGHT - 0.004)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "body_to_right_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_lid,
        origin=Origin(xyz=(-0.126, -0.059, BODY_HEIGHT - 0.004)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.158, 0.0, 0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=0.48,
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
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    pedal = object_model.get_part("pedal")
    left_hinge = object_model.get_articulation("body_to_left_lid")
    right_hinge = object_model.get_articulation("body_to_right_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")

    ctx.check(
        "lid hinges sit on the rear edge",
        left_hinge.axis == (0.0, -1.0, 0.0)
        and right_hinge.axis == (0.0, -1.0, 0.0)
        and left_hinge.origin.xyz[0] < -0.11
        and right_hinge.origin.xyz[0] < -0.11
        and left_hinge.origin.xyz[2] > 0.44
        and right_hinge.origin.xyz[2] > 0.44,
        details=f"left={left_hinge.origin.xyz}, right={right_hinge.origin.xyz}",
    )
    ctx.check(
        "pedal pivot is low and front mounted",
        pedal_hinge.axis == (0.0, 1.0, 0.0)
        and pedal_hinge.origin.xyz[0] > 0.12
        and 0.04 < pedal_hinge.origin.xyz[2] < 0.10,
        details=f"pedal pivot={pedal_hinge.origin.xyz}",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, pedal_hinge: 0.0}):
        ctx.expect_gap(
            left_lid,
            right_lid,
            axis="y",
            positive_elem="left_panel",
            negative_elem="right_panel",
            min_gap=0.003,
            max_gap=0.012,
            name="split lids keep a center seam gap",
        )
        ctx.expect_overlap(
            left_lid,
            body,
            axes="xy",
            elem_a="left_panel",
            min_overlap=0.040,
            name="left flap covers the body opening footprint",
        )
        ctx.expect_overlap(
            right_lid,
            body,
            axes="xy",
            elem_a="right_panel",
            min_overlap=0.040,
            name="right flap covers the body opening footprint",
        )
        ctx.expect_gap(
            pedal,
            body,
            axis="x",
            positive_elem="pedal_pad",
            min_gap=0.0,
            max_gap=0.050,
            name="pedal sits ahead of the front shell",
        )

        closed_left_front = _aabb_center(ctx.part_element_world_aabb(left_lid, elem="left_front_lip"))
        closed_right_front = _aabb_center(ctx.part_element_world_aabb(right_lid, elem="right_front_lip"))
        closed_pedal = _aabb_center(ctx.part_element_world_aabb(pedal, elem="pedal_pad"))

    with ctx.pose({left_hinge: 1.10, right_hinge: 0.0, pedal_hinge: 0.0}):
        left_only_open = _aabb_center(ctx.part_element_world_aabb(left_lid, elem="left_front_lip"))
        right_stays_closed = _aabb_center(ctx.part_element_world_aabb(right_lid, elem="right_front_lip"))

    ctx.check(
        "left flap opens independently of the right flap",
        closed_left_front is not None
        and left_only_open is not None
        and closed_right_front is not None
        and right_stays_closed is not None
        and left_only_open[2] > closed_left_front[2] + 0.10
        and abs(right_stays_closed[2] - closed_right_front[2]) < 0.01,
        details=(
            f"closed_left={closed_left_front}, open_left={left_only_open}, "
            f"closed_right={closed_right_front}, right_when_left_opens={right_stays_closed}"
        ),
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 1.10, pedal_hinge: 0.0}):
        right_only_open = _aabb_center(ctx.part_element_world_aabb(right_lid, elem="right_front_lip"))

    ctx.check(
        "right flap opens upward",
        closed_right_front is not None
        and right_only_open is not None
        and right_only_open[2] > closed_right_front[2] + 0.10,
        details=f"closed={closed_right_front}, open={right_only_open}",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, pedal_hinge: 0.42}):
        pressed_pedal = _aabb_center(ctx.part_element_world_aabb(pedal, elem="pedal_pad"))

    ctx.check(
        "front pedal rotates downward when pressed",
        closed_pedal is not None
        and pressed_pedal is not None
        and pressed_pedal[2] < closed_pedal[2] - 0.020,
        details=f"rest={closed_pedal}, pressed={pressed_pedal}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
