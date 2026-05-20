from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _polar_offset(angle: float, radial: float, tangential: float = 0.0, z: float = 0.0) -> tuple[float, float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (radial * c - tangential * s, radial * s + tangential * c, z)


def _build_leg_mesh() -> object:
    leg_path = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.090, 0.0, -0.070),
            (0.205, 0.0, -0.190),
            (0.325, 0.0, -0.355),
        ],
        radius=0.0105,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return leg_path


def _build_handle_mesh() -> object:
    rod = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (-0.045, 0.0, -0.008),
            (-0.155, 0.0, -0.045),
            (-0.285, 0.0, -0.090),
        ],
        radius=0.006,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return rod


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="photo_tripod_pan_head")

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    aluminum = model.material("aluminum", rgba=(0.62, 0.64, 0.67, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.046, length=0.040),
        material=graphite,
        name="hub",
    )
    crown.visual(
        Cylinder(radius=0.030, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=matte_black,
        name="shoulder",
    )
    crown.visual(
        Cylinder(radius=0.016, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=matte_black,
        name="column",
    )
    crown.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.249)),
        material=graphite,
        name="top_collar",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        for suffix, tangential in (("a", -0.0075), ("b", 0.0075)):
            crown.visual(
                Box((0.018, 0.005, 0.018)),
                origin=Origin(
                    xyz=_polar_offset(angle, radial=0.041, tangential=tangential, z=-0.008),
                    rpy=(0.0, 0.0, angle),
                ),
                material=graphite,
                name=f"hinge_lug_{index}_{suffix}",
            )

        leg = model.part(f"leg_{index}")
        leg.visual(
            mesh_from_geometry(_build_leg_mesh(), f"tripod_leg_{index}"),
            material=matte_black,
            name="tube",
        )
        leg.visual(
            Sphere(radius=0.014),
            origin=Origin(xyz=(0.325, 0.0, -0.355)),
            material=rubber,
            name="foot",
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=_polar_offset(angle, radial=0.0555, tangential=0.0, z=-0.008),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=1.2,
                lower=math.radians(-78.0),
                upper=math.radians(12.0),
            ),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=graphite,
        name="pan_base",
    )
    pan_head.visual(
        Box((0.078, 0.050, 0.034)),
        origin=Origin(xyz=(0.006, 0.0, 0.032)),
        material=matte_black,
        name="body",
    )
    pan_head.visual(
        Box((0.034, 0.006, 0.048)),
        origin=Origin(xyz=(0.012, -0.026, 0.060)),
        material=graphite,
        name="right_yoke",
    )
    pan_head.visual(
        Box((0.034, 0.006, 0.048)),
        origin=Origin(xyz=(0.012, 0.026, 0.060)),
        material=graphite,
        name="left_yoke",
    )
    pan_head.visual(
        Box((0.018, 0.016, 0.020)),
        origin=Origin(xyz=(-0.008, 0.024, 0.038)),
        material=graphite,
        name="handle_bracket",
    )
    pan_head.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(-0.008, 0.030, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="handle_boss",
    )

    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.0),
    )

    camera_support = model.part("camera_support")
    camera_support.visual(
        Cylinder(radius=0.011, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="tilt_barrel",
    )
    camera_support.visual(
        Box((0.060, 0.022, 0.024)),
        origin=Origin(xyz=(0.030, 0.0, -0.002)),
        material=matte_black,
        name="arm",
    )
    camera_support.visual(
        Box((0.040, 0.038, 0.018)),
        origin=Origin(xyz=(0.070, 0.0, 0.011)),
        material=graphite,
        name="riser",
    )
    camera_support.visual(
        Box((0.120, 0.055, 0.008)),
        origin=Origin(xyz=(0.090, 0.0, 0.022)),
        material=graphite,
        name="platform",
    )
    camera_support.visual(
        Box((0.080, 0.042, 0.002)),
        origin=Origin(xyz=(0.090, 0.0, 0.027)),
        material=rubber,
        name="pad",
    )
    camera_support.visual(
        Cylinder(radius=0.0045, length=0.006),
        origin=Origin(xyz=(0.090, 0.0, 0.031)),
        material=aluminum,
        name="camera_screw",
    )

    model.articulation(
        "pan_head_to_camera_support",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_support,
        origin=Origin(xyz=(0.012, 0.0, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=math.radians(-70.0),
            upper=math.radians(85.0),
        ),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_geometry(_build_handle_mesh(), "pan_handle"),
        material=matte_black,
        name="arm",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.100),
        origin=Origin(xyz=(-0.260, 0.0, -0.082), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip",
    )

    model.articulation(
        "pan_head_to_handle",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=handle,
        origin=Origin(xyz=(-0.008, 0.040, 0.038)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=math.radians(-55.0),
            upper=math.radians(55.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    camera_support = object_model.get_part("camera_support")
    handle = object_model.get_part("handle")
    leg_0 = object_model.get_part("leg_0")

    pan_joint = object_model.get_articulation("crown_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_camera_support")
    handle_joint = object_model.get_articulation("pan_head_to_handle")
    leg_joint = object_model.get_articulation("crown_to_leg_0")

    for index in range(3):
        for suffix in ("a", "b"):
            ctx.allow_overlap(
                crown,
                object_model.get_part(f"leg_{index}"),
                elem_a=f"hinge_lug_{index}_{suffix}",
                elem_b="tube",
                reason=(
                    "Each deployed leg root is simplified as a single tube seated in the crown clevis; "
                    "the hinge engagement is represented by a slight local lug-to-tube embed instead of "
                    "a separately modeled forked eye and pin stack."
                ),
            )

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    ctx.expect_contact(
        pan_head,
        crown,
        elem_a="pan_base",
        elem_b="top_collar",
        contact_tol=0.001,
        name="pan head seats on the crown collar",
    )

    rest_support = ctx.part_world_position(camera_support)
    with ctx.pose({pan_joint: math.pi / 2.0}):
        turned_support = ctx.part_world_position(camera_support)
    ctx.check(
        "pan head rotates around the vertical axis",
        rest_support is not None
        and turned_support is not None
        and abs(turned_support[0]) < 0.004
        and turned_support[1] > 0.008
        and abs(turned_support[2] - rest_support[2]) < 0.003,
        details=f"rest={rest_support}, turned={turned_support}",
    )

    rest_platform = elem_center("camera_support", "platform")
    with ctx.pose({tilt_joint: math.radians(50.0)}):
        tilted_platform = elem_center("camera_support", "platform")
    ctx.check(
        "camera support tilts upward",
        rest_platform is not None
        and tilted_platform is not None
        and tilted_platform[2] > rest_platform[2] + 0.035
        and tilted_platform[0] < rest_platform[0] - 0.010,
        details=f"rest={rest_platform}, tilted={tilted_platform}",
    )

    rest_grip = elem_center("handle", "grip")
    with ctx.pose({handle_joint: math.radians(45.0)}):
        raised_grip = elem_center("handle", "grip")
    ctx.check(
        "side handle pivots at the head body",
        rest_grip is not None
        and raised_grip is not None
        and raised_grip[2] > rest_grip[2] + 0.050,
        details=f"rest={rest_grip}, raised={raised_grip}",
    )

    rest_foot = elem_center("leg_0", "foot")
    with ctx.pose({leg_joint: leg_joint.motion_limits.lower}):
        folded_foot = elem_center("leg_0", "foot")
    ctx.check(
        "front leg folds upward at the crown hinge",
        rest_foot is not None and folded_foot is not None and folded_foot[2] > rest_foot[2] + 0.200,
        details=f"rest={rest_foot}, folded={folded_foot}",
    )

    ctx.expect_gap(
        pan_head,
        crown,
        axis="z",
        positive_elem="pan_base",
        negative_elem="hub",
        min_gap=0.210,
        max_gap=0.260,
        name="pan head stays above the crown hub",
    )

    return ctx.report()


object_model = build_object_model()
