from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_double_crankset")

    alloy = model.material("alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    spindle_finish = model.material("spindle_finish", rgba=(0.42, 0.43, 0.45, 1.0))
    chainring_finish = model.material("chainring_finish", rgba=(0.54, 0.55, 0.57, 1.0))
    pedal_finish = model.material("pedal_finish", rgba=(0.14, 0.14, 0.15, 1.0))

    def circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
        return [
            (
                radius * math.cos(2.0 * math.pi * i / segments),
                radius * math.sin(2.0 * math.pi * i / segments),
            )
            for i in range(segments)
        ]

    def toothed_ring_profile(
        teeth: int,
        root_radius: float,
        tip_radius: float,
    ) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for i in range(teeth):
            step = 2.0 * math.pi / teeth
            base = i * step
            samples = (
                (0.00, root_radius),
                (0.16, root_radius),
                (0.28, tip_radius),
                (0.50, tip_radius),
                (0.72, tip_radius),
                (0.84, root_radius),
            )
            for frac, radius in samples:
                angle = base + frac * step
                points.append((radius * math.cos(angle), radius * math.sin(angle)))
        return points

    def extrude_profile_x(
        outer_profile: list[tuple[float, float]],
        hole_profiles: list[list[tuple[float, float]]],
        thickness: float,
    ):
        geom = ExtrudeWithHolesGeometry(
            outer_profile,
            hole_profiles,
            thickness,
            cap=True,
            center=True,
            closed=True,
        )
        geom.rotate_y(math.pi / 2.0)
        return geom

    def yz_square_section(x_pos: float, half_side: float) -> tuple[tuple[float, float, float], ...]:
        return (
            (x_pos, -half_side, -half_side),
            (x_pos, half_side, -half_side),
            (x_pos, half_side, half_side),
            (x_pos, -half_side, half_side),
        )

    def arm_section(
        width_x: float,
        width_y: float,
        z_pos: float,
        x_offset: float,
    ) -> tuple[tuple[float, float, float], ...]:
        radius = min(width_x, width_y) * 0.22
        return tuple(
            (x_val + x_offset, y_val, z_pos)
            for x_val, y_val in rounded_rect_profile(
                width_x,
                width_y,
                radius,
                corner_segments=6,
            )
        )

    def pedal_body_section(
        side: float,
        y_pos: float,
        width_x: float,
        height_z: float,
        x_center: float,
    ) -> tuple[tuple[float, float, float], ...]:
        radius = min(width_x, height_z) * 0.28
        return tuple(
            (side * (x_center + x_val), y_pos, z_val)
            for x_val, z_val in rounded_rect_profile(
                width_x,
                height_z,
                radius,
                corner_segments=6,
            )
        )

    def make_spindle_geometry():
        spindle_geom = CylinderGeometry(radius=0.0085, height=0.100, radial_segments=32).rotate_y(
            math.pi / 2.0
        )
        spindle_geom.merge(
            CylinderGeometry(radius=0.012, height=0.028, radial_segments=32).rotate_y(math.pi / 2.0)
        )
        spindle_geom.merge(
            section_loft(
                [
                    yz_square_section(0.050, 0.0067),
                    yz_square_section(0.068, 0.0084),
                ]
            )
        )
        spindle_geom.merge(
            section_loft(
                [
                    yz_square_section(-0.050, 0.0067),
                    yz_square_section(-0.068, 0.0084),
                ]
            )
        )
        return spindle_geom

    def make_arm_geometry(side: float):
        arm_geom = CylinderGeometry(radius=0.027, height=0.022, radial_segments=28).rotate_y(
            math.pi / 2.0
        )
        arm_geom.translate(side * 0.011, 0.0, 0.0)
        arm_geom.merge(
            section_loft(
                [
                    arm_section(0.022, 0.046, 0.006, side * 0.008),
                    arm_section(0.020, 0.036, -0.030, side * 0.009),
                    arm_section(0.018, 0.028, -0.085, side * 0.011),
                    arm_section(0.016, 0.021, -0.140, side * 0.013),
                    arm_section(0.015, 0.018, -0.168, side * 0.014),
                ]
            )
        )
        arm_geom.merge(
            CylinderGeometry(radius=0.016, height=0.012, radial_segments=28)
            .rotate_y(math.pi / 2.0)
            .translate(side * 0.014, 0.0, -0.170)
        )
        return arm_geom

    def make_spider_geometry():
        spider_geom = extrude_profile_x(
            circle_profile(0.040, 80),
            [circle_profile(0.013, 64)],
            0.022,
        )
        spider_geom.translate(-0.006, 0.0, 0.0)

        arm_angles = [math.pi + i * (2.0 * math.pi / 5.0) for i in range(5)]
        for angle in arm_angles:
            y_mid = -0.061 * math.sin(angle)
            z_mid = 0.061 * math.cos(angle)
            spider_geom.merge(
                BoxGeometry((0.012, 0.012, 0.064))
                .rotate_x(angle)
                .translate(-0.011, y_mid, z_mid)
            )
            spider_geom.merge(
                CylinderGeometry(radius=0.007, height=0.010, radial_segments=24)
                .rotate_y(math.pi / 2.0)
                .translate(-0.011, -0.056 * math.sin(angle), 0.056 * math.cos(angle))
            )
        return spider_geom

    def make_pedal_geometry(side: float):
        pedal_geom = CylinderGeometry(radius=0.005, height=0.024, radial_segments=22).rotate_y(
            math.pi / 2.0
        )
        pedal_geom.translate(side * 0.012, 0.0, 0.0)
        pedal_geom.merge(
            CylinderGeometry(radius=0.0075, height=0.010, radial_segments=22)
            .rotate_y(math.pi / 2.0)
            .translate(side * 0.018, 0.0, 0.0)
        )
        pedal_geom.merge(
            section_loft(
                [
                    pedal_body_section(side, -0.032, 0.020, 0.010, 0.034),
                    pedal_body_section(side, -0.018, 0.024, 0.012, 0.035),
                    pedal_body_section(side, 0.000, 0.028, 0.014, 0.036),
                    pedal_body_section(side, 0.018, 0.024, 0.012, 0.035),
                    pedal_body_section(side, 0.032, 0.020, 0.010, 0.034),
                ]
            )
        )
        return pedal_geom

    support = model.part("support")

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_geometry(make_spindle_geometry(), "spindle_body"),
        material=spindle_finish,
        name="spindle_body",
    )

    right_crank = model.part("right_crank")
    right_crank.visual(
        mesh_from_geometry(make_arm_geometry(1.0), "right_crank_arm"),
        material=alloy,
        name="right_crank_arm",
    )
    right_crank.visual(
        mesh_from_geometry(make_spider_geometry(), "right_spider"),
        material=alloy,
        name="right_spider",
    )
    right_crank.visual(
        mesh_from_geometry(
            extrude_profile_x(
                toothed_ring_profile(52, 0.109, 0.114),
                [circle_profile(0.090, 88)],
                0.0036,
            ).translate(-0.0105, 0.0, 0.0),
            "outer_chainring",
        ),
        material=chainring_finish,
        name="outer_chainring",
    )
    right_crank.visual(
        mesh_from_geometry(
            extrude_profile_x(
                toothed_ring_profile(36, 0.079, 0.084),
                [circle_profile(0.061, 72)],
                0.0032,
            ).translate(-0.0135, 0.0, 0.0),
            "inner_chainring",
        ),
        material=chainring_finish,
        name="inner_chainring",
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        mesh_from_geometry(make_arm_geometry(-1.0), "left_crank_arm"),
        material=alloy,
        name="left_crank_arm",
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        mesh_from_geometry(make_pedal_geometry(1.0), "right_pedal_body"),
        material=pedal_finish,
        name="right_pedal_body",
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        mesh_from_geometry(make_pedal_geometry(-1.0), "left_pedal_body"),
        material=pedal_finish,
        name="left_pedal_body",
    )

    model.articulation(
        "support_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=30.0),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(xyz=(0.068, 0.0, 0.0)),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(xyz=(-0.068, 0.0, 0.0), rpy=(math.pi, 0.0, 0.0)),
    )
    model.articulation(
        "right_crank_to_right_pedal",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child=right_pedal,
        origin=Origin(xyz=(0.020, 0.0, -0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=40.0),
    )
    model.articulation(
        "left_crank_to_left_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(-0.020, 0.0, -0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=40.0),
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

    spindle = object_model.get_part("spindle")
    right_crank = object_model.get_part("right_crank")
    left_crank = object_model.get_part("left_crank")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    crank_spin = object_model.get_articulation("support_to_spindle")
    right_pedal_spin = object_model.get_articulation("right_crank_to_right_pedal")
    left_pedal_spin = object_model.get_articulation("left_crank_to_left_pedal")

    with ctx.pose({crank_spin: 0.0, right_pedal_spin: 0.0, left_pedal_spin: 0.0}):
        ctx.expect_contact(
            right_pedal,
            right_crank,
            name="right pedal spindle seats against right crank arm",
        )
        ctx.expect_contact(
            left_pedal,
            left_crank,
            name="left pedal spindle seats against left crank arm",
        )
        ctx.expect_origin_gap(
            spindle,
            right_pedal,
            axis="z",
            min_gap=0.15,
            name="right pedal hangs below spindle at rest",
        )
        ctx.expect_origin_gap(
            left_pedal,
            spindle,
            axis="z",
            min_gap=0.15,
            name="left pedal sits above spindle at rest",
        )
        ctx.expect_origin_distance(
            right_pedal,
            left_pedal,
            axes="y",
            max_dist=0.005,
            name="pedals align laterally in crank plane at rest",
        )

    with ctx.pose({crank_spin: math.pi / 2.0}):
        ctx.expect_origin_gap(
            right_pedal,
            spindle,
            axis="y",
            min_gap=0.15,
            name="positive spindle rotation swings right pedal forward",
        )
        ctx.expect_origin_gap(
            spindle,
            left_pedal,
            axis="y",
            min_gap=0.15,
            name="positive spindle rotation swings left pedal rearward",
        )

    outer_aabb = ctx.part_element_world_aabb(right_crank, elem="outer_chainring")
    inner_aabb = ctx.part_element_world_aabb(right_crank, elem="inner_chainring")
    outer_diameter = None
    inner_diameter = None
    if outer_aabb is not None:
        outer_diameter = max(
            outer_aabb[1][1] - outer_aabb[0][1],
            outer_aabb[1][2] - outer_aabb[0][2],
        )
    if inner_aabb is not None:
        inner_diameter = max(
            inner_aabb[1][1] - inner_aabb[0][1],
            inner_aabb[1][2] - inner_aabb[0][2],
        )
    ctx.check(
        "outer chainring is larger than inner chainring",
        outer_diameter is not None
        and inner_diameter is not None
        and outer_diameter > inner_diameter + 0.04,
        details=f"outer_diameter={outer_diameter}, inner_diameter={inner_diameter}",
    )

    for joint in (crank_spin, right_pedal_spin, left_pedal_spin):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is continuous",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"limits={limits}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
