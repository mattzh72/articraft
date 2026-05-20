from __future__ import annotations

import math

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
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _oval_section(
    width_x: float,
    width_y: float,
    z: float,
    *,
    center_x: float = 0.0,
    exponent: float = 2.6,
    segments: int = 64,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + x, y, z)
        for x, y in superellipse_profile(width_x, width_y, exponent=exponent, segments=segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_size_rice_cooker")

    warm_white = model.material("warm_white", rgba=(0.93, 0.91, 0.86, 1.0))
    lid_white = model.material("lid_white", rgba=(0.97, 0.96, 0.92, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.58, 0.60, 0.61, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.11, 0.12, 0.13, 1.0))
    black_glass = model.material("black_glass", rgba=(0.02, 0.025, 0.03, 1.0))
    display_blue = model.material("display_blue", rgba=(0.10, 0.34, 0.54, 1.0))
    button_red = model.material("button_red", rgba=(0.82, 0.10, 0.08, 1.0))

    body = model.part("body")
    body_shell = section_loft(
        [
            _oval_section(0.36, 0.43, 0.010, exponent=2.9),
            _oval_section(0.42, 0.52, 0.055, exponent=2.7),
            _oval_section(0.435, 0.545, 0.145, exponent=2.6),
            _oval_section(0.390, 0.490, 0.225, exponent=2.8),
            _oval_section(0.320, 0.400, 0.245, exponent=3.0),
        ]
    )
    body.visual(mesh_from_geometry(body_shell, "body_shell"), material=warm_white, name="shell")
    body.visual(
        Box((0.37, 0.45, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_gray,
        name="base_shadow",
    )
    body.visual(
        Box((0.038, 0.330, 0.028)),
        origin=Origin(xyz=(-0.205, 0.0, 0.208)),
        material=soft_gray,
        name="rear_hinge_band",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.330),
        origin=Origin(xyz=(-0.226, 0.0, 0.232), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="rear_hinge_pin",
    )

    for y_sign, suffix in ((1.0, "0"), (-1.0, "1")):
        body.visual(
            Box((0.070, 0.030, 0.045)),
            origin=Origin(xyz=(0.0, y_sign * 0.248, 0.250)),
            material=warm_white,
            name=f"shoulder_pedestal_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(xyz=(0.0, y_sign * 0.284, 0.286), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=soft_gray,
            name=f"side_pivot_{suffix}",
        )
        body.visual(
            Box((0.040, 0.018, 0.030)),
            origin=Origin(xyz=(0.0, y_sign * 0.266, 0.286)),
            material=soft_gray,
            name=f"pivot_bridge_{suffix}",
        )

    body.visual(
        Box((0.008, 0.175, 0.115)),
        origin=Origin(xyz=(0.216, 0.0, 0.092)),
        material=black_glass,
        name="control_panel",
    )
    body.visual(
        Box((0.004, 0.112, 0.035)),
        origin=Origin(xyz=(0.222, 0.0, 0.126)),
        material=display_blue,
        name="display_window",
    )
    body.visual(
        Box((0.004, 0.125, 0.010)),
        origin=Origin(xyz=(0.221, 0.0, 0.048)),
        material=soft_gray,
        name="status_strip",
    )

    lid = model.part("lid")
    lid_dome = section_loft(
        [
            _oval_section(0.390, 0.445, -0.007, center_x=0.190, exponent=2.8),
            _oval_section(0.365, 0.420, 0.030, center_x=0.190, exponent=2.45),
            _oval_section(0.270, 0.315, 0.060, center_x=0.195, exponent=2.20),
            _oval_section(0.135, 0.160, 0.075, center_x=0.205, exponent=2.00),
        ]
    )
    lid.visual(mesh_from_geometry(lid_dome, "lid_dome"), material=lid_white, name="lid_dome")
    lid.visual(
        Cylinder(radius=0.012, length=0.355),
        origin=Origin(xyz=(-0.021, 0.0, -0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_gray,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.045, 0.315, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=lid_white,
        name="hinge_web",
    )
    lid.visual(
        Cylinder(radius=0.023, length=0.010),
        origin=Origin(xyz=(0.215, 0.0, 0.076)),
        material=soft_gray,
        name="steam_vent",
    )
    lid.visual(
        Box((0.050, 0.012, 0.008)),
        origin=Origin(xyz=(0.230, 0.0, 0.083)),
        material=dark_gray,
        name="vent_slot",
    )

    carry_handle = model.part("carry_handle")
    handle_tube = tube_from_spline_points(
        [
            (0.0, -0.330, 0.000),
            (0.0, -0.255, 0.055),
            (0.0, -0.125, 0.100),
            (0.0, 0.000, 0.118),
            (0.0, 0.125, 0.100),
            (0.0, 0.255, 0.055),
            (0.0, 0.330, 0.000),
        ],
        radius=0.0105,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    carry_handle.visual(mesh_from_geometry(handle_tube, "carry_handle_tube"), material=dark_gray, name="handle_tube")

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_gray,
        name="button_plunger",
    )
    power_button.visual(
        Cylinder(radius=0.008, length=0.002),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_red,
        name="power_mark",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.205, 0.0, 0.258)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.5, lower=0.0, upper=math.radians(72.0)),
    )
    model.articulation(
        "handle_pivots",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carry_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.286)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=math.radians(-65.0), upper=math.radians(65.0)),
    )
    model.articulation(
        "power_button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(0.220, 0.0, 0.079)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.04, lower=0.0, upper=0.008),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    carry_handle = object_model.get_part("carry_handle")
    power_button = object_model.get_part("power_button")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_pivots = object_model.get_articulation("handle_pivots")
    power_slide = object_model.get_articulation("power_button_slide")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="rear_hinge_pin",
        elem_b="lid_hinge_barrel",
        reason="The lid barrel is intentionally captured around the rear hinge pin.",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="y",
        elem_a="rear_hinge_pin",
        elem_b="lid_hinge_barrel",
        min_overlap=0.25,
        name="rear hinge barrel spans the hinge pin",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_hinge_barrel",
        negative_elem="rear_hinge_pin",
        max_penetration=0.020,
        name="hinge pin capture is local and shallow",
    )
    for pivot_name in ("side_pivot_0", "side_pivot_1"):
        ctx.allow_overlap(
            body,
            carry_handle,
            elem_a=pivot_name,
            elem_b="handle_tube",
            reason="The folding handle tube is locally captured by the shoulder pivot bushing.",
        )
        ctx.expect_overlap(
            body,
            carry_handle,
            axes="xz",
            elem_a=pivot_name,
            elem_b="handle_tube",
            min_overlap=0.010,
            name=f"{pivot_name} captures the folding handle end",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_dome",
        negative_elem="shell",
        min_gap=0.0,
        max_gap=0.018,
        name="lid rests just above the broad body shell",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_dome",
        elem_b="shell",
        min_overlap=0.28,
        name="lid footprint covers the cooker opening",
    )
    ctx.expect_gap(
        power_button,
        body,
        axis="x",
        positive_elem="button_plunger",
        negative_elem="control_panel",
        max_gap=0.0015,
        max_penetration=0.0,
        name="power button is seated on the lower front panel",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: math.radians(62.0)}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge lifts the front of the lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.055,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    centered_handle_aabb = ctx.part_world_aabb(carry_handle)
    with ctx.pose({handle_pivots: math.radians(50.0)}):
        folded_handle_aabb = ctx.part_world_aabb(carry_handle)
    ctx.check(
        "carry handle rotates forward from the side pivots",
        centered_handle_aabb is not None
        and folded_handle_aabb is not None
        and folded_handle_aabb[1][0] > centered_handle_aabb[1][0] + 0.060,
        details=f"centered={centered_handle_aabb}, folded={folded_handle_aabb}",
    )

    button_rest = ctx.part_world_position(power_button)
    with ctx.pose({power_slide: 0.008}):
        button_pressed = ctx.part_world_position(power_button)
    ctx.check(
        "power button translates inward when pressed",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[0] < button_rest[0] - 0.006,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
