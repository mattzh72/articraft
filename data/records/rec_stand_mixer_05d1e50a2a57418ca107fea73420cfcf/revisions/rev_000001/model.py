from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)


def _plan_section(
    *,
    center_x: float,
    width_x: float,
    depth_y: float,
    z: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + x, y, z)
        for x, y in rounded_rect_profile(width_x, depth_y, corner_radius)
    ]


def _station_section(
    *,
    x: float,
    width_y: float,
    height_z: float,
    z_center: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(width_y, height_z, corner_radius)
    ]


def _build_base_geometry():
    base_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.320, 0.208, 0.052),
        0.024,
    )

    pedestal_geom = section_loft(
        [
            _plan_section(
                center_x=-0.068,
                width_x=0.122,
                depth_y=0.112,
                z=0.024,
                corner_radius=0.028,
            ),
            _plan_section(
                center_x=-0.086,
                width_x=0.094,
                depth_y=0.104,
                z=0.148,
                corner_radius=0.024,
            ),
            _plan_section(
                center_x=-0.103,
                width_x=0.058,
                depth_y=0.092,
                z=0.250,
                corner_radius=0.018,
            ),
        ]
    )
    base_geom.merge(pedestal_geom)

    bowl_pad = CylinderGeometry(radius=0.073, height=0.010).translate(0.086, 0.0, 0.029)
    base_geom.merge(bowl_pad)
    return base_geom


def _build_head_geometry():
    head_geom = section_loft(
        [
            _station_section(
                x=0.000,
                width_y=0.086,
                height_z=0.102,
                z_center=0.008,
                corner_radius=0.020,
            ),
            _station_section(
                x=0.086,
                width_y=0.142,
                height_z=0.164,
                z_center=0.006,
                corner_radius=0.034,
            ),
            _station_section(
                x=0.182,
                width_y=0.122,
                height_z=0.142,
                z_center=-0.001,
                corner_radius=0.028,
            ),
            _station_section(
                x=0.272,
                width_y=0.072,
                height_z=0.092,
                z_center=-0.012,
                corner_radius=0.016,
            ),
        ]
    )

    hinge_barrel = (
        CylinderGeometry(radius=0.018, height=0.126)
        .rotate_x(math.pi / 2.0)
        .translate(0.004, 0.0, 0.000)
    )
    drive_boss = CylinderGeometry(radius=0.026, height=0.036).translate(0.194, 0.0, -0.070)

    head_geom.merge(hinge_barrel)
    head_geom.merge(drive_boss)
    return head_geom


def _build_bowl_geometry():
    return LatheGeometry.from_shell_profiles(
        [
            (0.032, 0.000),
            (0.053, 0.012),
            (0.091, 0.051),
            (0.108, 0.116),
            (0.111, 0.146),
        ],
        [
            (0.000, 0.004),
            (0.046, 0.016),
            (0.093, 0.055),
            (0.101, 0.141),
        ],
        segments=64,
        end_cap="round",
        lip_samples=10,
    )


def _build_beater_geometry():
    beater_geom = CylinderGeometry(radius=0.005, height=0.060).translate(0.0, 0.0, -0.030)
    beater_geom.merge(CylinderGeometry(radius=0.011, height=0.014).translate(0.0, 0.0, -0.007))

    paddle_loop = wire_from_points(
        [
            (0.000, 0.000, -0.040),
            (-0.022, 0.000, -0.060),
            (-0.022, 0.000, -0.112),
            (0.000, 0.000, -0.128),
            (0.022, 0.000, -0.112),
            (0.022, 0.000, -0.060),
        ],
        radius=0.003,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.008,
    )
    center_spine = CylinderGeometry(radius=0.0026, height=0.050).translate(0.0, 0.0, -0.065)
    cross_bar = (
        CylinderGeometry(radius=0.0026, height=0.040)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, -0.093)
    )

    beater_geom.merge(paddle_loop)
    beater_geom.merge(center_spine)
    beater_geom.merge(cross_bar)
    return beater_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_stand_mixer")

    enamel = model.material("enamel", rgba=(0.92, 0.93, 0.89, 1.0))
    steel = model.material("steel", rgba=(0.84, 0.85, 0.87, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.16, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_build_base_geometry(), "mixer_base"),
        material=enamel,
        name="base_shell",
    )
    base.visual(
        Box((0.024, 0.016, 0.086)),
        origin=Origin(xyz=(-0.111, -0.052, 0.293)),
        material=enamel,
        name="hinge_cheek_0",
    )
    base.visual(
        Box((0.024, 0.016, 0.086)),
        origin=Origin(xyz=(-0.111, 0.052, 0.293)),
        material=enamel,
        name="hinge_cheek_1",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(-0.099, -0.052, 0.342), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_stub_0",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(-0.099, 0.052, 0.342), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_stub_1",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(_build_bowl_geometry(), "mixer_bowl"),
        material=steel,
        name="bowl_shell",
    )
    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.086, 0.0, 0.034)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_build_head_geometry(), "mixer_head"),
        material=enamel,
        name="head_shell",
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.103, 0.0, 0.342)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    lock_button = model.part("lock_button")
    lock_button.visual(
        Box((0.016, 0.030, 0.010)),
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
        material=dark_trim,
        name="lock_button_cap",
    )
    model.articulation(
        "base_to_lock_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_button,
        origin=Origin(xyz=(-0.133, 0.0, 0.166)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.006,
        ),
    )

    beater = model.part("beater")
    beater.visual(
        mesh_from_geometry(_build_beater_geometry(), "mixer_beater"),
        material=steel,
        name="beater_shell",
    )
    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.194, 0.0, -0.088)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=18.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    lock_button = object_model.get_part("lock_button")

    head_hinge = object_model.get_articulation("base_to_head")
    button_slide = object_model.get_articulation("base_to_lock_button")

    ctx.allow_overlap(
        base,
        head,
        elem_a="hinge_cheek_0",
        elem_b="head_shell",
        reason="The rear hinge cheek closely captures the exposed head barrel in this simplified clevis mount.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="hinge_cheek_1",
        elem_b="head_shell",
        reason="The rear hinge cheek closely captures the exposed head barrel in this simplified clevis mount.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="hinge_stub_0",
        elem_b="head_shell",
        reason="The rear hinge trunnion is intentionally nested inside the exposed head barrel.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="hinge_stub_1",
        elem_b="head_shell",
        reason="The rear hinge trunnion is intentionally nested inside the exposed head barrel.",
    )

    ctx.expect_overlap(
        bowl,
        base,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="base_shell",
        min_overlap=0.12,
        name="bowl footprint stays over the low base",
    )
    ctx.expect_origin_gap(
        bowl,
        base,
        axis="z",
        min_gap=0.030,
        max_gap=0.040,
        name="bowl origin sits on the low base deck",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        positive_elem="head_shell",
        negative_elem="bowl_shell",
        min_gap=0.020,
        name="closed head clears the bowl rim",
    )

    rest_beater_pos = ctx.part_world_position(beater)
    with ctx.pose({head_hinge: head_hinge.motion_limits.upper}):
        lifted_beater_pos = ctx.part_world_position(beater)
        ctx.expect_gap(
            beater,
            bowl,
            axis="z",
            positive_elem="beater_shell",
            negative_elem="bowl_shell",
            min_gap=0.030,
            name="tilted head lifts the beater above the bowl",
        )

    ctx.check(
        "tilt head raises the beater",
        rest_beater_pos is not None
        and lifted_beater_pos is not None
        and lifted_beater_pos[2] > rest_beater_pos[2] + 0.09,
        details=f"rest={rest_beater_pos}, lifted={lifted_beater_pos}",
    )

    rest_button_pos = ctx.part_world_position(lock_button)
    with ctx.pose({button_slide: button_slide.motion_limits.upper}):
        pressed_button_pos = ctx.part_world_position(lock_button)

    ctx.check(
        "lock button presses into the pedestal",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[0] > rest_button_pos[0] + 0.004,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
