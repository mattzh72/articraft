from __future__ import annotations

from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _oval_loop(
    y: float,
    width_x: float,
    height_z: float,
    *,
    exponent: float = 2.7,
    segments: int = 56,
) -> list[tuple[float, float, float]]:
    """A superellipse loop in the local XZ plane at one Y station."""
    return [(x, y, z) for x, z in superellipse_profile(width_x, height_z, exponent=exponent, segments=segments)]


def _oval_solid(sections: list[tuple[float, float, float]]) -> object:
    loops = [_oval_loop(y, width_x, height_z) for y, width_x, height_z in sections]
    return repair_loft(section_loft(loops), repair="mesh")


def _oval_ring(width_x: float, height_z: float, y: float, tube_radius: float) -> object:
    points = [
        (0.5 * width_x * cos(tau * i / 28.0), y, 0.5 * height_z * sin(tau * i / 28.0))
        for i in range(28)
    ]
    return tube_from_spline_points(
        points,
        radius=tube_radius,
        samples_per_segment=3,
        closed_spline=True,
        radial_segments=16,
        cap_ends=False,
        up_hint=(0.0, 1.0, 0.0),
    )


def _oval_slab(width_x: float, height_z: float, y: float, thickness: float) -> object:
    return repair_loft(
        section_loft(
            [
                _oval_loop(y - thickness * 0.5, width_x, height_z, exponent=2.4, segments=48),
                _oval_loop(y + thickness * 0.5, width_x, height_z, exponent=2.4, segments=48),
            ]
        ),
        repair="mesh",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_folding_headphones")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.055, 0.058, 0.064, 1.0))
    cushion_foam = model.material("cushion_foam", rgba=(0.020, 0.020, 0.023, 1.0))
    speaker_cloth = model.material("speaker_cloth", rgba=(0.010, 0.011, 0.013, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.56, 0.58, 0.60, 1.0))
    hinge_black = model.material("hinge_black", rgba=(0.030, 0.031, 0.034, 1.0))

    headband = model.part("headband")
    outer_band = sweep_profile_along_spline(
        [
            (0.0, -0.130, 0.205),
            (0.0, -0.116, 0.272),
            (0.0, -0.060, 0.323),
            (0.0, 0.000, 0.340),
            (0.0, 0.060, 0.323),
            (0.0, 0.116, 0.272),
            (0.0, 0.130, 0.205),
        ],
        profile=rounded_rect_profile(0.034, 0.014, radius=0.004, corner_segments=6),
        samples_per_segment=9,
        cap_profile=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    headband.visual(mesh_from_geometry(outer_band, "headband_outer"), material=matte_black, name="headband_outer")

    underside_pad = tube_from_spline_points(
        [
            (0.0, -0.104, 0.205),
            (0.0, -0.070, 0.278),
            (0.0, 0.000, 0.306),
            (0.0, 0.070, 0.278),
            (0.0, 0.104, 0.205),
        ],
        radius=0.0105,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    headband.visual(mesh_from_geometry(underside_pad, "headband_cushion"), material=cushion_foam, name="headband_cushion")

    for side_name, side in (("left", -1.0), ("right", 1.0)):
        y = side * 0.130
        # Open-channel telescoping sleeve: two rails with a top bridge leave a real
        # slot for the metal slider instead of intersecting it as a solid block.
        headband.visual(
            Box((0.008, 0.032, 0.106)),
            origin=Origin(xyz=(-0.018, y, 0.178)),
            material=matte_black,
            name=f"{side_name}_sleeve_front",
        )
        headband.visual(
            Box((0.008, 0.032, 0.106)),
            origin=Origin(xyz=(0.018, y, 0.178)),
            material=matte_black,
            name=f"{side_name}_sleeve_rear",
        )
        headband.visual(
            Box((0.044, 0.033, 0.024)),
            origin=Origin(xyz=(0.0, y, 0.232)),
            material=matte_black,
            name=f"{side_name}_sleeve_bridge",
        )

    left_slider = model.part("left_slider")
    right_slider = model.part("right_slider")
    for slider in (left_slider, right_slider):
        slider.visual(
            Box((0.028, 0.012, 0.120)),
            origin=Origin(xyz=(0.0, 0.0, -0.035)),
            material=brushed_metal,
            name="slider_blade",
        )
        slider.visual(
            Box((0.017, 0.014, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, -0.084)),
            material=hinge_black,
            name="hinge_lug",
        )
        slider.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.095), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_black,
            name="fold_barrel",
        )
        slider.visual(
            Cylinder(radius=0.0055, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, -0.095), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_metal,
            name="fold_pin",
        )

    left_yoke = model.part("left_yoke")
    right_yoke = model.part("right_yoke")
    yoke_path = [
        (-0.052, 0.0, -0.130),
        (-0.052, 0.0, -0.084),
        (-0.034, 0.0, -0.032),
        (0.000, 0.0, -0.020),
        (0.034, 0.0, -0.032),
        (0.052, 0.0, -0.084),
        (0.052, 0.0, -0.130),
    ]
    for yoke in (left_yoke, right_yoke):
        fork = tube_from_spline_points(
            yoke_path,
            radius=0.0048,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        )
        yoke.visual(mesh_from_geometry(fork, f"{yoke.name}_fork"), material=brushed_metal, name="yoke_fork")
        yoke.visual(
            Box((0.018, 0.014, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
            material=hinge_black,
            name="hinge_neck",
        )
        yoke.visual(
            Box((0.006, 0.008, 0.028)),
            origin=Origin(xyz=(-0.014, 0.0, -0.034)),
            material=hinge_black,
            name="socket_web_0",
        )
        yoke.visual(
            Box((0.006, 0.008, 0.028)),
            origin=Origin(xyz=(0.014, 0.0, -0.034)),
            material=hinge_black,
            name="socket_web_1",
        )
        yoke.visual(
            Cylinder(radius=0.012, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
            material=hinge_black,
            name="swivel_socket",
        )
        yoke.visual(
            Box((0.052, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=hinge_black,
            name="hinge_bridge",
        )
        yoke.visual(
            Box((0.010, 0.012, 0.020)),
            origin=Origin(xyz=(-0.019, 0.0, -0.016)),
            material=hinge_black,
            name="outer_web",
        )
        yoke.visual(
            Box((0.010, 0.012, 0.020)),
            origin=Origin(xyz=(0.019, 0.0, -0.016)),
            material=hinge_black,
            name="inner_web",
        )
        yoke.visual(
            Cylinder(radius=0.0105, length=0.012),
            origin=Origin(xyz=(-0.019, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_black,
            name="outer_knuckle",
        )
        yoke.visual(
            Cylinder(radius=0.0105, length=0.012),
            origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_black,
            name="inner_knuckle",
        )
        yoke.visual(
            Cylinder(radius=0.007, length=0.012),
            origin=Origin(xyz=(-0.052, 0.0, -0.130), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_black,
            name="front_pivot",
        )
        yoke.visual(
            Cylinder(radius=0.007, length=0.012),
            origin=Origin(xyz=(0.052, 0.0, -0.130), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_black,
            name="rear_pivot",
        )

    def add_cup_visuals(part, mesh_prefix: str) -> None:
        shell = _oval_solid(
            [
                (-0.031, 0.068, 0.096),
                (-0.020, 0.082, 0.115),
                (0.000, 0.090, 0.126),
                (0.020, 0.082, 0.115),
                (0.031, 0.068, 0.096),
            ]
        )
        part.visual(mesh_from_geometry(shell, f"{mesh_prefix}_cup_shell"), material=satin_black, name="cup_shell")
        for y_face, label in ((0.034, "inner_pad"), (-0.034, "outer_pad")):
            pad = _oval_ring(0.077, 0.108, y_face, 0.0085)
            part.visual(mesh_from_geometry(pad, f"{mesh_prefix}_{label}"), material=cushion_foam, name=label)
            cloth = _oval_slab(0.061, 0.088, y_face, 0.0022)
            part.visual(mesh_from_geometry(cloth, f"{mesh_prefix}_{label}_cloth"), material=speaker_cloth, name=f"{label}_cloth")
        part.visual(
            Cylinder(radius=0.0075, length=0.018),
            origin=Origin(xyz=(-0.038, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_black,
            name="front_boss",
        )
        part.visual(
            Cylinder(radius=0.0075, length=0.018),
            origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_black,
            name="rear_boss",
        )
        part.visual(
            Cylinder(radius=0.007, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.075)),
            material=brushed_metal,
            name="swivel_post",
        )

    left_cup = model.part("left_cup")
    right_cup = model.part("right_cup")
    add_cup_visuals(left_cup, "left")
    add_cup_visuals(right_cup, "right")

    model.articulation(
        "left_size_slide",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=left_slider,
        origin=Origin(xyz=(0.0, -0.130, 0.175)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.10, lower=0.0, upper=0.040),
    )
    model.articulation(
        "right_size_slide",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=right_slider,
        origin=Origin(xyz=(0.0, 0.130, 0.175)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.10, lower=0.0, upper=0.040),
    )
    model.articulation(
        "left_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=left_slider,
        child=left_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.62),
    )
    model.articulation(
        "right_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=right_slider,
        child=right_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.62),
    )
    model.articulation(
        "left_cup_swivel",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "right_cup_swivel",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_slider = object_model.get_part("left_slider")
    right_slider = object_model.get_part("right_slider")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")

    left_slide = object_model.get_articulation("left_size_slide")
    right_slide = object_model.get_articulation("right_size_slide")
    left_fold = object_model.get_articulation("left_fold_hinge")
    right_fold = object_model.get_articulation("right_fold_hinge")
    left_swivel = object_model.get_articulation("left_cup_swivel")
    right_swivel = object_model.get_articulation("right_cup_swivel")

    for side_name, slider, yoke, cup in (
        ("left", left_slider, left_yoke, left_cup),
        ("right", right_slider, right_yoke, right_cup),
    ):
        for knuckle in ("outer_knuckle", "inner_knuckle"):
            ctx.allow_overlap(
                slider,
                yoke,
                elem_a="fold_pin",
                elem_b=knuckle,
                reason="The steel fold pin is intentionally captured inside the hinge knuckle bore.",
            )
            ctx.expect_overlap(
                slider,
                yoke,
                axes="x",
                elem_a="fold_pin",
                elem_b=knuckle,
                min_overlap=0.008,
                name=f"{side_name} fold pin retained in {knuckle}",
            )
            ctx.expect_overlap(
                slider,
                yoke,
                axes="yz",
                elem_a="fold_pin",
                elem_b=knuckle,
                min_overlap=0.004,
                name=f"{side_name} fold pin centered in {knuckle}",
            )
        ctx.allow_overlap(
            cup,
            yoke,
            elem_a="swivel_post",
            elem_b="swivel_socket",
            reason="The cup swivel post is intentionally seated inside the yoke socket.",
        )
        ctx.expect_within(
            cup,
            yoke,
            axes="xy",
            inner_elem="swivel_post",
            outer_elem="swivel_socket",
            margin=0.003,
            name=f"{side_name} swivel post stays in socket",
        )
        ctx.expect_overlap(
            cup,
            yoke,
            axes="z",
            elem_a="swivel_post",
            elem_b="swivel_socket",
            min_overlap=0.010,
            name=f"{side_name} swivel post has captured depth",
        )

    ctx.expect_origin_gap(left_yoke, left_cup, axis="z", min_gap=0.115, max_gap=0.145, name="left cup hangs below yoke")
    ctx.expect_origin_gap(right_yoke, right_cup, axis="z", min_gap=0.115, max_gap=0.145, name="right cup hangs below yoke")

    rest_left = ctx.part_world_position(left_slider)
    rest_right = ctx.part_world_position(right_slider)
    with ctx.pose({left_slide: 0.040, right_slide: 0.040}):
        extended_left = ctx.part_world_position(left_slider)
        extended_right = ctx.part_world_position(right_slider)
    ctx.check(
        "sliders telescope downward",
        rest_left is not None
        and rest_right is not None
        and extended_left is not None
        and extended_right is not None
        and extended_left[2] < rest_left[2] - 0.035
        and extended_right[2] < rest_right[2] - 0.035,
        details=f"rest_left={rest_left}, extended_left={extended_left}, rest_right={rest_right}, extended_right={extended_right}",
    )

    rest_left_cup = ctx.part_world_position(left_cup)
    rest_right_cup = ctx.part_world_position(right_cup)
    with ctx.pose({left_fold: 1.20, right_fold: 1.20}):
        folded_left_cup = ctx.part_world_position(left_cup)
        folded_right_cup = ctx.part_world_position(right_cup)
    ctx.check(
        "fold hinges swing cups inward",
        rest_left_cup is not None
        and rest_right_cup is not None
        and folded_left_cup is not None
        and folded_right_cup is not None
        and folded_left_cup[1] > rest_left_cup[1] + 0.045
        and folded_right_cup[1] < rest_right_cup[1] - 0.045,
        details=f"rest_left={rest_left_cup}, folded_left={folded_left_cup}, rest_right={rest_right_cup}, folded_right={folded_right_cup}",
    )

    rest_aabb = ctx.part_world_aabb(left_cup)
    with ctx.pose({left_swivel: 0.75, right_swivel: -0.75}):
        swivel_aabb = ctx.part_world_aabb(left_cup)
    ctx.check(
        "cup swivel changes footprint",
        rest_aabb is not None
        and swivel_aabb is not None
        and (swivel_aabb[1][1] - swivel_aabb[0][1]) > (rest_aabb[1][1] - rest_aabb[0][1]) + 0.010,
        details=f"rest_aabb={rest_aabb}, swivel_aabb={swivel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
