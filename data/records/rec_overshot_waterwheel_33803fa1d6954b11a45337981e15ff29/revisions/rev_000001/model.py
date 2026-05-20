from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_overshot_waterwheel")

    timber = model.material("timber", rgba=(0.53, 0.37, 0.22, 1.0))
    weathered_timber = model.material("weathered_timber", rgba=(0.44, 0.31, 0.20, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.22, 0.23, 0.24, 1.0))
    iron = model.material("iron", rgba=(0.46, 0.47, 0.50, 1.0))

    axle_z = 2.0
    def rect_profile(width: float, height: float) -> list[tuple[float, float]]:
        half_w = width * 0.5
        half_h = height * 0.5
        return [
            (-half_w, -half_h),
            (half_w, -half_h),
            (half_w, half_h),
            (-half_w, half_h),
        ]

    def bearing_plate_geometry(
        *, width: float, height: float, thickness: float, hole_radius: float
    ):
        return (
            ExtrudeWithHolesGeometry(
                rect_profile(width, height),
                [superellipse_profile(hole_radius * 2.0, hole_radius * 2.0, exponent=2.0, segments=32)],
                thickness,
                center=True,
            )
            .rotate_x(math.pi / 2.0)
        )

    def add_box_component(
        geom,
        *,
        size: tuple[float, float, float],
        center: tuple[float, float, float],
        yaw: float = 0.0,
    ):
        piece = BoxGeometry(size)
        if abs(yaw) > 1e-9:
            piece.rotate_y(yaw)
        piece.translate(*center)
        geom.merge(piece)

    def build_wheel_mesh():
        wheel_geom = CylinderGeometry(radius=0.24, height=0.72, radial_segments=28).rotate_x(
            math.pi / 2.0
        )
        wheel_geom.merge(
            CylinderGeometry(radius=0.31, height=0.10, radial_segments=28)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, -0.36, 0.0)
        )
        wheel_geom.merge(
            CylinderGeometry(radius=0.31, height=0.10, radial_segments=28)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.36, 0.0)
        )

        spoke_count = 8
        for spoke_index in range(spoke_count):
            theta = 2.0 * math.pi * spoke_index / spoke_count
            add_box_component(
                wheel_geom,
                size=(1.40, 0.72, 0.10),
                center=(math.cos(theta) * 0.90, 0.0, math.sin(theta) * 0.90),
                yaw=-theta,
            )

        bucket_count = 16
        bucket_step = 2.0 * math.pi / bucket_count
        rim_center_radius = 1.66
        floor_radius = 1.56
        partition_radius = 1.51
        tangent_length = 0.68

        for bucket_index in range(bucket_count):
            theta = bucket_index * bucket_step
            theta_mid = theta + bucket_step * 0.5
            tangent_yaw = -(theta_mid + math.pi / 2.0)

            add_box_component(
                wheel_geom,
                size=(0.46, 0.98, 0.06),
                center=(math.cos(theta) * partition_radius, 0.0, math.sin(theta) * partition_radius),
                yaw=-theta,
            )
            add_box_component(
                wheel_geom,
                size=(tangent_length, 0.98, 0.06),
                center=(math.cos(theta_mid) * floor_radius, 0.0, math.sin(theta_mid) * floor_radius),
                yaw=tangent_yaw,
            )
            for side_y in (-0.51, 0.51):
                add_box_component(
                    wheel_geom,
                    size=(tangent_length * 1.04, 0.09, 0.16),
                    center=(math.cos(theta_mid) * rim_center_radius, side_y, math.sin(theta_mid) * rim_center_radius),
                    yaw=tangent_yaw,
                )

        return wheel_geom

    support_frame = model.part("support_frame")

    frame_side_y = 0.84
    front_post_x = -1.95
    rear_post_x = 0.25
    sill_center_x = -0.85

    for side_name, side_y, sill_name, bearing_name in (
        ("left", -frame_side_y, "left_base_sill", "left_bearing_plate"),
        ("right", frame_side_y, "right_base_sill", "right_bearing_plate"),
    ):
        support_frame.visual(
            Box((2.45, 0.16, 0.18)),
            origin=Origin(xyz=(sill_center_x, side_y, 0.09)),
            material=weathered_timber,
            name=sill_name,
        )
        support_frame.visual(
            Box((0.18, 0.14, 4.05)),
            origin=Origin(xyz=(front_post_x, side_y, 2.025)),
            material=weathered_timber,
        )
        support_frame.visual(
            Box((0.18, 0.14, 4.05)),
            origin=Origin(xyz=(rear_post_x, side_y, 2.025)),
            material=weathered_timber,
        )
        support_frame.visual(
            Box((2.28, 0.14, 0.18)),
            origin=Origin(xyz=(sill_center_x, side_y, 3.77)),
            material=weathered_timber,
        )
        support_frame.visual(
            Box((2.10, 0.12, 0.14)),
            origin=Origin(xyz=(sill_center_x, side_y, 1.15)),
            material=weathered_timber,
        )
        support_frame.visual(
            Box((0.42, 0.14, 0.36)),
            origin=Origin(xyz=(0.05, side_y, 1.70)),
            material=weathered_timber,
        )
        support_frame.visual(
            Box((0.42, 0.14, 0.30)),
            origin=Origin(xyz=(0.05, side_y, 2.27)),
            material=weathered_timber,
        )
        support_frame.visual(
            mesh_from_geometry(
                bearing_plate_geometry(width=0.36, height=0.40, thickness=0.08, hole_radius=0.14),
                f"{side_name}_bearing_plate_mesh",
            ),
            origin=Origin(xyz=(0.05, side_y, axle_z)),
            material=timber,
            name=bearing_name,
        )

    support_frame.visual(
        Box((0.22, 1.68, 0.14)),
        origin=Origin(xyz=(sill_center_x, 0.0, 0.16)),
        material=weathered_timber,
    )
    support_frame.visual(
        Box((0.18, 1.68, 0.14)),
        origin=Origin(xyz=(front_post_x, 0.0, 3.38)),
        material=weathered_timber,
    )
    support_frame.visual(
        Box((0.18, 1.68, 0.14)),
        origin=Origin(xyz=(rear_post_x, 0.0, 2.90)),
        material=weathered_timber,
    )

    support_frame.visual(
        Box((1.30, 0.88, 0.07)),
        origin=Origin(xyz=(-1.75, 0.0, 3.46)),
        material=timber,
        name="trough_bottom",
    )
    support_frame.visual(
        Box((1.30, 0.06, 0.30)),
        origin=Origin(xyz=(-1.75, -0.47, 3.61)),
        material=timber,
    )
    support_frame.visual(
        Box((1.30, 0.06, 0.30)),
        origin=Origin(xyz=(-1.75, 0.47, 3.61)),
        material=timber,
    )
    support_frame.visual(
        Box((0.06, 1.00, 0.30)),
        origin=Origin(xyz=(-2.37, 0.0, 3.61)),
        material=timber,
    )
    support_frame.visual(
        Box((0.05, 1.00, 0.06)),
        origin=Origin(xyz=(-1.10, 0.0, 3.75)),
        material=weathered_timber,
        name="trough_lip",
    )
    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(build_wheel_mesh(), "overshot_wheel_body"),
        material=timber,
        name="wheel_body",
    )
    wheel.visual(
        Cylinder(radius=0.10, length=1.60),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="axle_shaft",
    )
    inspection_flap = model.part("inspection_flap")
    inspection_flap.visual(
        Cylinder(radius=0.025, length=0.92),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hinge_barrel",
    )
    inspection_flap.visual(
        Box((0.04, 0.92, 0.30)),
        origin=Origin(xyz=(0.02, 0.0, -0.15)),
        material=timber,
        name="flap_panel",
    )
    inspection_flap.visual(
        Box((0.03, 0.12, 0.18)),
        origin=Origin(xyz=(0.045, -0.26, -0.16)),
        material=weathered_timber,
    )
    inspection_flap.visual(
        Box((0.03, 0.12, 0.18)),
        origin=Origin(xyz=(0.045, 0.26, -0.16)),
        material=weathered_timber,
    )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.5),
    )
    model.articulation(
        "trough_to_flap",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=inspection_flap,
        origin=Origin(xyz=(-1.05, 0.0, 3.75)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=1.10,
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
    support_frame = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    inspection_flap = object_model.get_part("inspection_flap")
    wheel_joint = object_model.get_articulation("support_to_wheel")
    flap_joint = object_model.get_articulation("trough_to_flap")

    wheel_limits = wheel_joint.motion_limits
    flap_limits = flap_joint.motion_limits

    ctx.check(
        "wheel uses a continuous horizontal axle articulation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and wheel_limits is not None
        and wheel_limits.lower is None
        and wheel_limits.upper is None
        and abs(wheel_joint.axis[1]) > 0.99,
        details=(
            f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}, "
            f"limits={wheel_limits}"
        ),
    )
    ctx.check(
        "inspection flap uses a small hinged gate articulation",
        flap_joint.articulation_type == ArticulationType.REVOLUTE
        and flap_limits is not None
        and flap_limits.lower is not None
        and flap_limits.upper is not None
        and flap_limits.lower >= -1e-6
        and 0.9 <= flap_limits.upper <= 1.2
        and flap_joint.axis[1] < -0.99,
        details=(
            f"type={flap_joint.articulation_type}, axis={flap_joint.axis}, "
            f"limits={flap_limits}"
        ),
    )

    ctx.expect_overlap(
        wheel,
        support_frame,
        axes="xz",
        elem_a="axle_shaft",
        elem_b="left_bearing_plate",
        min_overlap=0.18,
        name="axle aligns with the left bearing frame",
    )
    ctx.expect_overlap(
        wheel,
        support_frame,
        axes="xz",
        elem_a="axle_shaft",
        elem_b="right_bearing_plate",
        min_overlap=0.18,
        name="axle aligns with the right bearing frame",
    )
    ctx.expect_gap(
        wheel,
        support_frame,
        axis="z",
        positive_elem="wheel_body",
        negative_elem="left_base_sill",
        min_gap=0.04,
        max_gap=0.15,
        name="wheel clears the timber sill above the stream bed",
    )
    ctx.expect_overlap(
        inspection_flap,
        support_frame,
        axes="y",
        elem_a="flap_panel",
        elem_b="trough_lip",
        min_overlap=0.86,
        name="inspection flap spans the trough outlet width",
    )
    ctx.expect_gap(
        inspection_flap,
        support_frame,
        axis="x",
        positive_elem="flap_panel",
        negative_elem="trough_lip",
        min_gap=0.01,
        max_gap=0.05,
        name="inspection flap hangs just beyond the trough lip",
    )

    wheel_rest = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: math.pi / 3.0}):
        wheel_spun = ctx.part_world_position(wheel)
        ctx.expect_overlap(
            wheel,
            support_frame,
            axes="xz",
            elem_a="axle_shaft",
            elem_b="left_bearing_plate",
            min_overlap=0.18,
            name="left bearing stays centered on the axle while the wheel spins",
        )
        ctx.expect_overlap(
            wheel,
            support_frame,
            axes="xz",
            elem_a="axle_shaft",
            elem_b="right_bearing_plate",
            min_overlap=0.18,
            name="right bearing stays centered on the axle while the wheel spins",
        )
    ctx.check(
        "wheel spins around a fixed axle center",
        wheel_rest is not None
        and wheel_spun is not None
        and max(abs(a - b) for a, b in zip(wheel_rest, wheel_spun)) <= 1e-5,
        details=f"rest={wheel_rest}, spun={wheel_spun}",
    )

    flap_closed = ctx.part_element_world_aabb(inspection_flap, elem="flap_panel")
    with ctx.pose({flap_joint: 0.95}):
        flap_open = ctx.part_element_world_aabb(inspection_flap, elem="flap_panel")
    ctx.check(
        "inspection flap opens upward",
        flap_closed is not None
        and flap_open is not None
        and flap_open[0][2] > flap_closed[0][2] + 0.10,
        details=f"closed={flap_closed}, open={flap_open}",
    )
    ctx.check(
        "inspection flap opens outward over the wheel",
        flap_closed is not None
        and flap_open is not None
        and flap_open[1][0] > flap_closed[1][0] + 0.10,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
