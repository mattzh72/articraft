from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _leg_tube(angle: float):
    ca = math.cos(angle)
    sa = math.sin(angle)
    return tube_from_spline_points(
        [
            (0.000, 0.000, 0.270),
            (0.34 * ca, 0.34 * sa, 0.145),
            (0.62 * ca, 0.62 * sa, 0.030),
        ],
        radius=0.018,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )


def _spreader_tube(angle_a: float, angle_b: float):
    return tube_from_spline_points(
        [
            (0.33 * math.cos(angle_a), 0.33 * math.sin(angle_a), 0.185),
            (0.00, 0.00, 0.230),
            (0.33 * math.cos(angle_b), 0.33 * math.sin(angle_b), 0.185),
        ],
        radius=0.010,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )


def _dish_shell_mesh():
    # A shallow closed parabolic reflector shell. The profile is revolved about
    # its local +Z axis, then the visual rotates that axis to the dish boresight.
    return LatheGeometry(
        [
            (0.000, -0.018),
            (0.070, -0.014),
            (0.170, 0.012),
            (0.285, 0.070),
            (0.382, 0.135),
            (0.394, 0.154),
            (0.365, 0.154),
            (0.270, 0.093),
            (0.160, 0.040),
            (0.055, 0.014),
            (0.000, 0.010),
        ],
        segments=96,
    )


def _dish_rib_mesh():
    return tube_from_spline_points(
        [
            (0.120, 0.000, 0.000),
            (0.175, 0.000, 0.110),
            (0.235, 0.000, 0.225),
            (0.292, 0.000, 0.356),
        ],
        radius=0.010,
        samples_per_segment=12,
        radial_segments=12,
        cap_ends=True,
    )


def _feed_arm_mesh():
    return tube_from_spline_points(
        [
            (0.286, 0.000, -0.390),
            (0.365, 0.000, -0.295),
            (0.480, 0.000, -0.110),
            (0.555, 0.000, 0.000),
        ],
        radius=0.014,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_satellite_dish_tripod")

    matte_white = model.material("matte_white", rgba=(0.86, 0.88, 0.88, 1.0))
    light_gray = model.material("light_gray", rgba=(0.62, 0.65, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.025, 0.026, 0.028, 1.0))
    rubber = model.material("rubber", rgba=(0.035, 0.035, 0.033, 1.0))

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.045, length=0.89),
        origin=Origin(xyz=(0.0, 0.0, 0.605)),
        material=dark_steel,
        name="center_mast",
    )
    tripod.visual(
        Cylinder(radius=0.083, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.242)),
        material=light_gray,
        name="leg_hub",
    )
    tripod.visual(
        Cylinder(radius=0.118, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.025)),
        material=dark_steel,
        name="fixed_bearing",
    )
    for index in range(3):
        angle = index * math.tau / 3.0 + math.pi / 2.0
        tripod.visual(
            _mesh(f"tripod_leg_{index}", _leg_tube(angle)),
            material=dark_steel,
            name=f"leg_{index}",
        )
        tripod.visual(
            Cylinder(radius=0.092, length=0.036),
            origin=Origin(
                xyz=(0.62 * math.cos(angle), 0.62 * math.sin(angle), 0.018)
            ),
            material=rubber,
            name=f"foot_{index}",
        )
    for index in range(3):
        angle_a = index * math.tau / 3.0 + math.pi / 2.0
        angle_b = (index + 1) * math.tau / 3.0 + math.pi / 2.0
        tripod.visual(
            _mesh(f"spreader_{index}", _spreader_tube(angle_a, angle_b)),
            material=light_gray,
            name=f"spreader_{index}",
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.142, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="turntable_disk",
    )
    head.visual(
        Cylinder(radius=0.070, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=light_gray,
        name="rotating_neck",
    )
    head.visual(
        Box((0.190, 0.690, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=light_gray,
        name="yoke_bridge",
    )
    for side, y in enumerate((-0.315, 0.315)):
        head.visual(
            Box((0.115, 0.050, 0.185)),
            origin=Origin(xyz=(0.0, y, 0.188)),
            material=light_gray,
            name=f"yoke_arm_{side}",
        )
        head.visual(
            Cylinder(radius=0.077, length=0.085),
            origin=Origin(xyz=(0.0, y, 0.320), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bearing_collar_{side}",
        )

    dish = model.part("dish")
    dish.visual(
        Cylinder(radius=0.043, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    dish.visual(
        Cylinder(radius=0.110, length=0.180),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_gray,
        name="rear_hub",
    )
    dish.visual(
        _mesh("reflector_shell", _dish_shell_mesh()),
        origin=Origin(xyz=(0.135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_white,
        name="reflector_shell",
    )
    dish.visual(
        _mesh(
            "rim_ring",
            TorusGeometry(radius=0.389, tube=0.018, radial_segments=18, tubular_segments=96),
        ),
        origin=Origin(xyz=(0.289, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_gray,
        name="rim_ring",
    )
    dish.visual(
        _mesh(
            "rear_stiffener_ring",
            TorusGeometry(radius=0.245, tube=0.014, radial_segments=14, tubular_segments=72),
        ),
        origin=Origin(xyz=(0.188, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_stiffener_ring",
    )
    rib_mesh = _mesh("back_rib", _dish_rib_mesh())
    for index in range(6):
        dish.visual(
            rib_mesh,
            origin=Origin(rpy=(index * math.tau / 6.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"back_rib_{index}",
        )
    dish.visual(
        _mesh("feed_arm", _feed_arm_mesh()),
        material=dark_steel,
        name="feed_arm",
    )
    dish.visual(
        Box((0.065, 0.070, 0.060)),
        origin=Origin(xyz=(0.562, 0.0, 0.0)),
        material=black_plastic,
        name="feed_block",
    )
    dish.visual(
        Cylinder(radius=0.032, length=0.105),
        origin=Origin(xyz=(0.610, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_gray,
        name="feed_horn",
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 1.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=head,
        child=dish,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.9, lower=-0.15, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    head = object_model.get_part("head")
    dish = object_model.get_part("dish")
    azimuth = object_model.get_articulation("azimuth_axis")
    elevation = object_model.get_articulation("elevation_axis")

    for collar in ("bearing_collar_0", "bearing_collar_1"):
        ctx.allow_overlap(
            head,
            dish,
            elem_a=collar,
            elem_b="trunnion_shaft",
            reason="The elevation trunnion shaft is intentionally captured inside the yoke bearing collar proxy.",
        )
        ctx.expect_within(
            dish,
            head,
            axes="xz",
            inner_elem="trunnion_shaft",
            outer_elem=collar,
            margin=0.0,
            name=f"trunnion fits inside {collar}",
        )
        ctx.expect_overlap(
            dish,
            head,
            axes="y",
            elem_a="trunnion_shaft",
            elem_b=collar,
            min_overlap=0.050,
            name=f"trunnion passes through {collar}",
        )

    ctx.expect_gap(
        head,
        tripod,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="fixed_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable disk sits on fixed bearing",
    )
    ctx.expect_overlap(
        head,
        tripod,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="fixed_bearing",
        min_overlap=0.10,
        name="azimuth bearing has broad footprint",
    )

    ctx.check(
        "azimuth axis is vertical revolute",
        azimuth.articulation_type == ArticulationType.REVOLUTE
        and tuple(azimuth.axis) == (0.0, 0.0, 1.0),
    )
    ctx.check(
        "elevation axis is horizontal at yoke",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) == (0.0, -1.0, 0.0),
    )

    def elem_center(part, elem):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_feed = elem_center(dish, "feed_horn")
    with ctx.pose({elevation: 0.85}):
        raised_feed = elem_center(dish, "feed_horn")
    ctx.check(
        "positive elevation raises feed",
        rest_feed is not None
        and raised_feed is not None
        and raised_feed[2] > rest_feed[2] + 0.25,
        details=f"rest={rest_feed}, raised={raised_feed}",
    )

    with ctx.pose({azimuth: math.pi / 2.0}):
        rotated_feed = elem_center(dish, "feed_horn")
    ctx.check(
        "azimuth rotates dish around mast",
        rest_feed is not None
        and rotated_feed is not None
        and rotated_feed[1] > 0.45,
        details=f"rest={rest_feed}, rotated={rotated_feed}",
    )

    return ctx.report()


object_model = build_object_model()
