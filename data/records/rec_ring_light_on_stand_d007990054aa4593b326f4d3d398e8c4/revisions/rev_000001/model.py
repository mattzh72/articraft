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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _ring_mesh(outer_radius: float, inner_radius: float, thickness: float):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        thickness,
        center=True,
        cap=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return geom


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_ring_light")

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.22, 0.24, 1.0))
    diffuser = model.material("diffuser", rgba=(0.94, 0.95, 0.96, 0.92))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=charcoal,
        name="hub_shell",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=charcoal,
        name="hub_collar",
    )
    body.visual(
        Box((0.024, 0.022, 0.018)),
        origin=Origin(xyz=(-0.009, 0.0, 0.056)),
        material=charcoal,
        name="rear_neck",
    )
    body.visual(
        Cylinder(radius=0.0085, length=0.096),
        origin=Origin(xyz=(-0.018, 0.0, 0.103)),
        material=matte_black,
        name="stem_tube",
    )
    body.visual(
        Box((0.024, 0.040, 0.034)),
        origin=Origin(xyz=(-0.018, 0.0, 0.149)),
        material=charcoal,
        name="tilt_block",
    )
    body.visual(
        Box((0.008, 0.126, 0.012)),
        origin=Origin(xyz=(-0.018, 0.0, 0.168)),
        material=charcoal,
        name="yoke_crossbar",
    )
    body.visual(
        Box((0.004, 0.012, 0.022)),
        origin=Origin(xyz=(-0.0145, -0.058, 0.168)),
        material=charcoal,
        name="left_yoke_ear",
    )
    body.visual(
        Box((0.004, 0.012, 0.022)),
        origin=Origin(xyz=(-0.0145, 0.058, 0.168)),
        material=charcoal,
        name="right_yoke_ear",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    leg_hinge_radius = 0.032
    for index, angle in enumerate(leg_angles, start=1):
        body.visual(
            Box((0.014, 0.018, 0.012)),
            origin=Origin(
                xyz=(0.020 * math.cos(angle), 0.020 * math.sin(angle), 0.032),
                rpy=(0.0, 0.0, angle),
            ),
            material=charcoal,
            name=f"leg_mount_{index}",
        )

    head = model.part("light_head")
    head.visual(
        mesh_from_geometry(_ring_mesh(0.095, 0.060, 0.018), "ring_head_housing"),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=matte_black,
        name="head_housing",
    )
    head.visual(
        mesh_from_geometry(_ring_mesh(0.087, 0.064, 0.004), "ring_head_diffuser"),
        origin=Origin(xyz=(0.0075, 0.0, 0.058)),
        material=diffuser,
        name="head_diffuser",
    )
    head.visual(
        Box((0.010, 0.132, 0.024)),
        origin=Origin(xyz=(-0.007, 0.0, 0.012)),
        material=charcoal,
        name="tilt_bridge",
    )
    head.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(
            xyz=(-0.007, -0.057, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=charcoal,
        name="left_pivot_stub",
    )
    head.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(
            xyz=(-0.007, 0.057, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=charcoal,
        name="right_pivot_stub",
    )
    head.visual(
        Box((0.022, 0.020, 0.018)),
        origin=Origin(xyz=(-0.019, 0.0, 0.013)),
        material=matte_black,
        name="rear_housing",
    )

    model.articulation(
        "body_to_light_head",
        ArticulationType.REVOLUTE,
        parent=body,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-0.65,
            upper=0.95,
        ),
    )

    leg_angle = 0.32
    beam_length = 0.085
    beam_offset = 0.009 + 0.5 * beam_length
    beam_center = (
        beam_offset * math.cos(leg_angle),
        0.0,
        -beam_offset * math.sin(leg_angle),
    )
    foot_center = (0.082, 0.0, -0.030)

    for index, angle in enumerate(leg_angles, start=1):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.005, length=0.014),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.020, 0.016, 0.010)),
            origin=Origin(xyz=(0.011, 0.0, -0.003)),
            material=charcoal,
            name="hinge_shoulder",
        )
        leg.visual(
            Box((beam_length, 0.015, 0.006)),
            origin=Origin(xyz=beam_center, rpy=(0.0, leg_angle, 0.0)),
            material=matte_black,
            name="leg_beam",
        )
        leg.visual(
            Box((0.018, 0.020, 0.004)),
            origin=Origin(xyz=foot_center),
            material=rubber,
            name="foot_pad",
        )

        model.articulation(
            f"body_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=leg,
            origin=Origin(
                xyz=(
                    leg_hinge_radius * math.cos(angle),
                    leg_hinge_radius * math.sin(angle),
                    0.032,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=3.0,
                lower=-1.85,
                upper=0.10,
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
    head = object_model.get_part("light_head")
    head_tilt = object_model.get_articulation("body_to_light_head")

    leg_parts = [object_model.get_part(f"leg_{index}") for index in range(1, 4)]
    leg_joints = [
        object_model.get_articulation(f"body_to_leg_{index}") for index in range(1, 4)
    ]

    ctx.check(
        "head tilt uses a horizontal axis",
        head_tilt.axis == (0.0, -1.0, 0.0),
        details=f"axis={head_tilt.axis}",
    )

    for index, leg_joint in enumerate(leg_joints, start=1):
        ctx.check(
            f"leg {index} hinge uses a horizontal fold axis",
            leg_joint.axis == (0.0, 1.0, 0.0),
            details=f"axis={leg_joint.axis}",
        )

    ctx.expect_gap(
        head,
        body,
        axis="x",
        positive_elem="left_pivot_stub",
        negative_elem="left_yoke_ear",
        min_gap=0.0,
        max_gap=0.002,
        name="left head pivot sits just ahead of the yoke ear",
    )
    ctx.expect_gap(
        head,
        body,
        axis="x",
        positive_elem="right_pivot_stub",
        negative_elem="right_yoke_ear",
        min_gap=0.0,
        max_gap=0.002,
        name="right head pivot sits just ahead of the yoke ear",
    )
    ctx.expect_gap(
        head,
        body,
        axis="x",
        positive_elem="head_housing",
        negative_elem="stem_tube",
        min_gap=0.0003,
        max_gap=0.010,
        name="ring housing stays just forward of the rear stem",
    )

    with ctx.pose({head_tilt: head_tilt.motion_limits.lower}):
        low_aabb = ctx.part_element_world_aabb(head, elem="head_diffuser")
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        high_aabb = ctx.part_element_world_aabb(head, elem="head_diffuser")

    low_center = _aabb_center(low_aabb)
    high_center = _aabb_center(high_aabb)
    ctx.check(
        "head tilts through a visible forward-backward arc",
        low_center is not None
        and high_center is not None
        and high_center[0] < low_center[0] - 0.060
        and abs(high_center[2] - low_center[2]) > 0.001,
        details=f"lower_center={low_center}, upper_center={high_center}",
    )

    hub_aabb = ctx.part_element_world_aabb(body, elem="hub_shell")
    hub_center = _aabb_center(hub_aabb)
    if hub_center is None:
        ctx.fail("hub shell probe available", "hub shell AABB was not available")

    rest_foot_centers = []
    for index, leg in enumerate(leg_parts, start=1):
        foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_pad")
        foot_center = _aabb_center(foot_aabb)
        rest_foot_centers.append(foot_center)
        ctx.check(
            f"leg {index} foot reaches near tabletop height",
            foot_aabb is not None
            and foot_aabb[0][2] < 0.006
            and foot_aabb[0][2] > -0.010,
            details=f"foot_aabb={foot_aabb}",
        )
        radial = None
        if hub_center is not None and foot_center is not None:
            radial = math.hypot(foot_center[0] - hub_center[0], foot_center[1] - hub_center[1])
        ctx.check(
            f"leg {index} opens to a stable radius",
            radial is not None and radial > 0.075,
            details=f"hub_center={hub_center}, foot_center={foot_center}, radial={radial}",
        )

    fold_pose = {leg_joints[0]: -1.45}
    with ctx.pose(fold_pose):
        folded_foot_aabb = ctx.part_element_world_aabb(leg_parts[0], elem="foot_pad")

    folded_foot_center = _aabb_center(folded_foot_aabb)
    ctx.check(
        "folded leg tucks upward toward the hub",
        hub_center is not None
        and rest_foot_centers[0] is not None
        and folded_foot_center is not None
        and folded_foot_center[2] > rest_foot_centers[0][2] + 0.040
        and math.hypot(
            folded_foot_center[0] - hub_center[0],
            folded_foot_center[1] - hub_center[1],
        )
        < math.hypot(
            rest_foot_centers[0][0] - hub_center[0],
            rest_foot_centers[0][1] - hub_center[1],
        )
        - 0.020,
        details=(
            f"hub_center={hub_center}, rest_foot={rest_foot_centers[0]}, "
            f"folded_foot={folded_foot_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
