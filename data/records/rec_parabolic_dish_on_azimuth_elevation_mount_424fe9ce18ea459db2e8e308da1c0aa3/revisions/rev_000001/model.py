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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mobile_communications_dish")

    frame_gray = model.material("frame_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.15, 0.16, 0.18, 1.0))
    off_white = model.material("off_white", rgba=(0.87, 0.89, 0.90, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    reflector_outer = [
        (0.02, -0.24),
        (0.16, -0.235),
        (0.34, -0.220),
        (0.54, -0.180),
        (0.70, -0.105),
        (0.80, 0.00),
    ]
    reflector_inner = [
        (0.01, -0.225),
        (0.15, -0.220),
        (0.32, -0.205),
        (0.52, -0.168),
        (0.685, -0.098),
        (0.785, -0.006),
    ]
    reflector_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            reflector_outer,
            reflector_inner,
            segments=72,
            start_cap="round",
            end_cap="flat",
            lip_samples=10,
        ),
        "mobile_dish_reflector",
    )

    rear_rib_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.15, 0.0, 0.0),
                (0.34, 0.0, 0.16),
                (0.50, 0.0, 0.35),
            ],
            radius=0.026,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
        "mobile_dish_rear_rib",
    )
    feed_strut_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.67, 0.0, 0.56),
                (0.88, 0.0, 0.27),
                (1.07, 0.0, 0.02),
            ],
            radius=0.020,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
        "mobile_dish_feed_strut",
    )
    transport_leg_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.0),
                (0.12, 0.0, -0.04),
                (0.30, 0.0, -0.14),
                (0.50, 0.0, -0.28),
            ],
            radius=0.035,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "mobile_dish_transport_leg",
    )

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((2.10, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, 0.46, 0.05)),
        material=frame_gray,
        name="rail_left",
    )
    base_frame.visual(
        Box((2.10, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, -0.46, 0.05)),
        material=frame_gray,
        name="rail_right",
    )
    base_frame.visual(
        Box((0.18, 0.92, 0.10)),
        origin=Origin(xyz=(0.96, 0.0, 0.05)),
        material=frame_gray,
        name="front_crossmember",
    )
    base_frame.visual(
        Box((0.18, 0.92, 0.10)),
        origin=Origin(xyz=(-0.96, 0.0, 0.05)),
        material=frame_gray,
        name="rear_crossmember",
    )
    base_frame.visual(
        Box((0.62, 0.92, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_metal,
        name="center_deck",
    )
    base_frame.visual(
        Cylinder(radius=0.27, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_metal,
        name="azimuth_bearing_base",
    )
    base_frame.visual(
        Box((0.09, 0.05, 0.12)),
        origin=Origin(xyz=(1.005, 0.09, 0.09)),
        material=frame_gray,
        name="hinge_cheek_pos",
    )
    base_frame.visual(
        Box((0.09, 0.05, 0.12)),
        origin=Origin(xyz=(1.005, -0.09, 0.09)),
        material=frame_gray,
        name="hinge_cheek_neg",
    )
    base_frame.visual(
        Cylinder(radius=0.032, length=0.04),
        origin=Origin(xyz=(1.08, 0.09, 0.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel_pos",
    )
    base_frame.visual(
        Cylinder(radius=0.032, length=0.04),
        origin=Origin(xyz=(1.08, -0.09, 0.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel_neg",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((2.10, 1.02, 0.32)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.23, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_metal,
        name="azimuth_turntable",
    )
    pedestal.visual(
        Box((0.34, 0.34, 0.52)),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=frame_gray,
        name="lower_mast",
    )
    pedestal.visual(
        Box((0.26, 0.62, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=frame_gray,
        name="upper_mast",
    )
    pedestal.visual(
        Box((0.22, 0.22, 0.92)),
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        material=dark_metal,
        name="pedestal_core",
    )
    pedestal.visual(
        Box((0.42, 0.10, 0.52)),
        origin=Origin(xyz=(0.16, 0.34, 1.08)),
        material=frame_gray,
        name="yoke_arm_pos",
    )
    pedestal.visual(
        Box((0.42, 0.10, 0.52)),
        origin=Origin(xyz=(0.16, -0.34, 1.08)),
        material=frame_gray,
        name="yoke_arm_neg",
    )
    pedestal.visual(
        Box((0.16, 0.78, 0.10)),
        origin=Origin(xyz=(0.02, 0.0, 1.27)),
        material=dark_metal,
        name="yoke_bridge",
    )
    pedestal.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.22, 0.33, 1.08), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_housing_pos",
    )
    pedestal.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.22, -0.33, 1.08), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_housing_neg",
    )
    pedestal.visual(
        Box((0.24, 0.22, 0.18)),
        origin=Origin(xyz=(-0.10, 0.0, 0.74)),
        material=dark_metal,
        name="azimuth_drive_box",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.70, 0.90, 1.40)),
        mass=260.0,
        origin=Origin(xyz=(0.04, 0.0, 0.70)),
    )

    elevation_frame = model.part("elevation_frame")
    elevation_frame.visual(
        Cylinder(radius=0.055, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_shaft",
    )
    elevation_frame.visual(
        Box((0.18, 0.22, 0.24)),
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
        material=frame_gray,
        name="hub_block",
    )
    elevation_frame.visual(
        Cylinder(radius=0.11, length=0.40),
        origin=Origin(xyz=(0.29, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub_barrel",
    )
    elevation_frame.visual(
        Cylinder(radius=0.03, length=0.66),
        origin=Origin(xyz=(0.75, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="feed_boom",
    )
    elevation_frame.visual(
        reflector_mesh,
        origin=Origin(xyz=(0.71, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="main_reflector",
    )
    for index, angle in enumerate(
        (
            math.pi / 4.0,
            3.0 * math.pi / 4.0,
            5.0 * math.pi / 4.0,
            7.0 * math.pi / 4.0,
        )
    ):
        elevation_frame.visual(
            rear_rib_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=dark_metal,
            name=f"rear_rib_{index}",
        )
    for index, angle in enumerate(
        (
            math.pi / 6.0,
            5.0 * math.pi / 6.0,
            3.0 * math.pi / 2.0,
        )
    ):
        elevation_frame.visual(
            feed_strut_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=aluminum,
            name=f"feed_strut_{index}",
        )
    elevation_frame.visual(
        Cylinder(radius=0.055, length=0.20),
        origin=Origin(xyz=(1.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="feed_mount",
    )
    elevation_frame.visual(
        Cylinder(radius=0.065, length=0.16),
        origin=Origin(xyz=(1.21, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_horn",
    )
    elevation_frame.inertial = Inertial.from_geometry(
        Box((1.40, 1.70, 1.70)),
        mass=120.0,
        origin=Origin(xyz=(0.72, 0.0, 0.0)),
    )

    transport_leg = model.part("transport_leg")
    transport_leg.visual(
        Cylinder(radius=0.028, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="leg_hinge_barrel",
    )
    transport_leg.visual(
        Box((0.16, 0.12, 0.10)),
        origin=Origin(xyz=(0.05, 0.0, -0.04)),
        material=dark_metal,
        name="hinge_gusset",
    )
    transport_leg.visual(
        transport_leg_mesh,
        origin=Origin(),
        material=frame_gray,
        name="support_foot",
    )
    transport_leg.visual(
        Box((0.14, 0.18, 0.06)),
        origin=Origin(xyz=(0.48, 0.0, -0.28)),
        material=rubber,
        name="foot_pad",
    )
    transport_leg.inertial = Inertial.from_geometry(
        Box((0.68, 0.22, 0.46)),
        mass=26.0,
        origin=Origin(xyz=(0.28, 0.0, -0.18)),
    )

    model.articulation(
        "azimuth_base",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=0.7),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=elevation_frame,
        origin=Origin(xyz=(0.22, 0.0, 1.08)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.9,
            lower=math.radians(-10.0),
            upper=math.radians(85.0),
        ),
    )
    model.articulation(
        "transport_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=transport_leg,
        origin=Origin(xyz=(1.08, 0.0, 0.12)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=1.2,
            lower=math.radians(-40.0),
            upper=math.radians(65.0),
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

    base_frame = object_model.get_part("base_frame")
    pedestal = object_model.get_part("pedestal")
    elevation_frame = object_model.get_part("elevation_frame")
    transport_leg = object_model.get_part("transport_leg")

    azimuth = object_model.get_articulation("azimuth_base")
    elevation = object_model.get_articulation("elevation_axis")
    leg_hinge = object_model.get_articulation("transport_leg_hinge")

    ctx.expect_contact(
        elevation_frame,
        pedestal,
        elem_a="trunnion_shaft",
        elem_b="trunnion_housing_pos",
        name="elevation frame is mounted in the yoke",
    )
    ctx.expect_contact(
        transport_leg,
        base_frame,
        elem_a="leg_hinge_barrel",
        elem_b="hinge_barrel_pos",
        name="transport leg is mounted on the base hinge",
    )
    ctx.expect_gap(
        elevation_frame,
        base_frame,
        axis="z",
        min_gap=0.12,
        name="dish assembly clears the base frame vertically",
    )

    def _center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) * 0.5,
            (lo[1] + hi[1]) * 0.5,
            (lo[2] + hi[2]) * 0.5,
        )

    reflector_rest = _center(ctx.part_element_world_aabb(elevation_frame, elem="main_reflector"))
    with ctx.pose({azimuth: math.pi / 2.0}):
        reflector_turned = _center(
            ctx.part_element_world_aabb(elevation_frame, elem="main_reflector")
        )
    ctx.check(
        "azimuth rotation swings the reflector sideways",
        reflector_rest is not None
        and reflector_turned is not None
        and abs(reflector_rest[0]) > 0.45
        and abs(reflector_turned[0]) < 0.18
        and reflector_turned[1] > 0.45,
        details=f"rest={reflector_rest}, turned={reflector_turned}",
    )

    feed_rest = _center(ctx.part_element_world_aabb(elevation_frame, elem="feed_horn"))
    with ctx.pose({elevation: math.radians(60.0)}):
        feed_raised = _center(ctx.part_element_world_aabb(elevation_frame, elem="feed_horn"))
    ctx.check(
        "positive elevation raises the feed and reflector",
        feed_rest is not None and feed_raised is not None and feed_raised[2] > feed_rest[2] + 0.35,
        details=f"rest={feed_rest}, raised={feed_raised}",
    )

    foot_rest = _center(ctx.part_element_world_aabb(transport_leg, elem="support_foot"))
    with ctx.pose({leg_hinge: math.radians(60.0)}):
        foot_folded = _center(ctx.part_element_world_aabb(transport_leg, elem="support_foot"))
    ctx.check(
        "transport leg folds upward toward the frame",
        foot_rest is not None and foot_folded is not None and foot_folded[2] > foot_rest[2] + 0.18,
        details=f"rest={foot_rest}, folded={foot_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
