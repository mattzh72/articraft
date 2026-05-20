from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, sin, tau
from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 48,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (cx + radius * cos(tau * index / segments), cy + radius * sin(tau * index / segments))
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cd_dj_player", assets=ASSETS)

    housing = model.material("housing", rgba=(0.13, 0.14, 0.15, 1.0))
    trim = model.material("trim", rgba=(0.21, 0.23, 0.25, 1.0))
    deck = model.material("deck", rgba=(0.08, 0.09, 0.10, 1.0))
    metal = model.material("metal", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.05, 1.0))
    display = model.material("display", rgba=(0.18, 0.32, 0.46, 0.95))
    button_blue = model.material("button_blue", rgba=(0.20, 0.46, 0.92, 1.0))
    button_green = model.material("button_green", rgba=(0.18, 0.72, 0.38, 1.0))
    accent = model.material("accent", rgba=(0.90, 0.42, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.34, 0.28, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=housing,
        name="bottom_shell",
    )
    body.visual(
        Box((0.34, 0.28, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=trim,
        name="top_shell",
    )
    body.visual(
        Box((0.01, 0.28, 0.042)),
        origin=Origin(xyz=(-0.165, 0.0, 0.021)),
        material=housing,
        name="left_wall",
    )
    body.visual(
        Box((0.01, 0.28, 0.042)),
        origin=Origin(xyz=(0.165, 0.0, 0.021)),
        material=housing,
        name="right_wall",
    )
    body.visual(
        Box((0.34, 0.01, 0.042)),
        origin=Origin(xyz=(0.0, 0.135, 0.021)),
        material=housing,
        name="rear_wall",
    )
    body.visual(
        Box((0.09, 0.01, 0.042)),
        origin=Origin(xyz=(-0.125, -0.135, 0.021)),
        material=housing,
        name="front_left_pillar",
    )
    body.visual(
        Box((0.09, 0.01, 0.042)),
        origin=Origin(xyz=(0.125, -0.135, 0.021)),
        material=housing,
        name="front_right_pillar",
    )
    body.visual(
        Box((0.16, 0.01, 0.018)),
        origin=Origin(xyz=(0.0, -0.135, 0.009)),
        material=housing,
        name="front_lower",
    )
    body.visual(
        Box((0.16, 0.01, 0.01)),
        origin=Origin(xyz=(0.0, -0.135, 0.037)),
        material=trim,
        name="front_slot_lintel",
    )
    body.visual(
        Box((0.20, 0.22, 0.002)),
        origin=Origin(xyz=(-0.04, 0.01, 0.051)),
        material=deck,
        name="control_deck",
    )
    wheel_bezel = ExtrudeWithHolesGeometry(
        _circle_profile(0.076, segments=56),
        [_circle_profile(0.063, segments=56)],
        0.002,
        center=False,
    )
    body.visual(
        mesh_from_geometry(wheel_bezel, "assets/meshes/wheel_bezel.obj"),
        origin=Origin(xyz=(0.07, 0.02, 0.05)),
        material=deck,
        name="wheel_well",
    )
    body.visual(
        Cylinder(radius=0.03, length=0.006),
        origin=Origin(xyz=(0.07, 0.02, 0.053)),
        material=trim,
        name="wheel_pedestal",
    )
    body.visual(
        Box((0.014, 0.13, 0.002)),
        origin=Origin(xyz=(0.150, 0.02, 0.051)),
        material=deck,
        name="pitch_rail",
    )
    body.visual(
        Box((0.018, 0.15, 0.001)),
        origin=Origin(xyz=(0.150, 0.02, 0.0505)),
        material=rubber,
        name="pitch_slot_shadow",
    )
    body.visual(
        Box((0.016, 0.16, 0.004)),
        origin=Origin(xyz=(-0.058, -0.055, 0.016)),
        material=trim,
        name="tray_rail_left",
    )
    body.visual(
        Box((0.016, 0.16, 0.004)),
        origin=Origin(xyz=(0.058, -0.055, 0.016)),
        material=trim,
        name="tray_rail_right",
    )
    body.visual(
        Box((0.07, 0.04, 0.003)),
        origin=Origin(xyz=(-0.085, 0.055, 0.0515)),
        material=display,
        name="screen",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(-0.102, -0.03, 0.052)),
        material=button_blue,
        name="cue_button",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(-0.06, -0.03, 0.052)),
        material=button_green,
        name="play_button",
    )
    body.visual(
        Box((0.05, 0.014, 0.003)),
        origin=Origin(xyz=(-0.085, 0.0, 0.0515)),
        material=trim,
        name="transport_bar",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.34, 0.28, 0.05)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.016, 0.14, 0.004)),
        origin=Origin(xyz=(-0.058, 0.0, 0.002)),
        material=trim,
        name="runner_left",
    )
    tray.visual(
        Box((0.016, 0.14, 0.004)),
        origin=Origin(xyz=(0.058, 0.0, 0.002)),
        material=trim,
        name="runner_right",
    )
    tray_top_lip = ExtrudeWithHolesGeometry(
        [
            (-0.0775, -0.08),
            (0.0775, -0.08),
            (0.0775, 0.08),
            (-0.0775, 0.08),
        ],
        [_circle_profile(0.056, center=(0.0, 0.006), segments=56)],
        0.004,
        center=False,
    )
    tray.visual(
        mesh_from_geometry(tray_top_lip, "assets/meshes/tray_top_lip.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=trim,
        name="tray_plate",
    )
    tray.visual(
        Box((0.158, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.081, 0.007)),
        material=housing,
        name="front_face",
    )
    tray.visual(
        Cylinder(radius=0.054, length=0.004),
        origin=Origin(xyz=(0.0, 0.006, 0.002)),
        material=rubber,
        name="disc_well",
    )
    tray.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.006, 0.004)),
        material=metal,
        name="disc_spindle",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.155, 0.16, 0.016)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.062, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=metal,
        name="platter_base",
    )
    platter.visual(
        Cylinder(radius=0.072, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=metal,
        name="platter_rim",
    )
    platter.visual(
        Cylinder(radius=0.055, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=rubber,
        name="touch_surface",
    )
    platter.visual(
        Cylinder(radius=0.021, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=deck,
        name="center_label",
    )
    platter.visual(
        Box((0.026, 0.006, 0.0015)),
        origin=Origin(xyz=(0.026, 0.0, 0.01475)),
        material=accent,
        name="position_marker",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.072, length=0.014),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    slider = model.part("pitch_slider")
    slider.visual(
        Box((0.014, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal,
        name="slider_shoe",
    )
    slider.visual(
        Box((0.01, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=accent,
        name="slider_knob",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.014, 0.024, 0.018)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -0.055, 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.45, lower=0.0, upper=0.10),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.REVOLUTE,
        parent=body,
        child=platter,
        origin=Origin(xyz=(0.07, 0.02, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0, lower=-6.283, upper=6.283),
    )
    model.articulation(
        "pitch_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(0.150, 0.02, 0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.35, lower=-0.045, upper=0.045),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    body = object_model.get_part("body")
    tray = object_model.get_part("tray")
    platter = object_model.get_part("platter")
    slider = object_model.get_part("pitch_slider")

    tray_slide = object_model.get_articulation("tray_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")
    pitch_slide = object_model.get_articulation("pitch_slide")

    tray_rail_left = body.get_visual("tray_rail_left")
    tray_runner_left = tray.get_visual("runner_left")
    tray_face = tray.get_visual("front_face")
    front_lower = body.get_visual("front_lower")
    front_slot_lintel = body.get_visual("front_slot_lintel")
    wheel_well = body.get_visual("wheel_well")
    wheel_pedestal = body.get_visual("wheel_pedestal")
    platter_base = platter.get_visual("platter_base")
    platter_rim = platter.get_visual("platter_rim")
    platter_marker = platter.get_visual("position_marker")
    pitch_rail = body.get_visual("pitch_rail")
    slider_shoe = slider.get_visual("slider_shoe")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    ctx.check(
        "tray is prismatic",
        tray_slide.articulation_type == ArticulationType.PRISMATIC,
        f"expected PRISMATIC tray joint, got {tray_slide.articulation_type!r}",
    )
    ctx.check(
        "tray axis points out the front",
        tray_slide.axis == (0.0, -1.0, 0.0),
        f"unexpected tray axis {tray_slide.axis!r}",
    )
    ctx.check(
        "jog wheel is revolute",
        wheel_spin.articulation_type == ArticulationType.REVOLUTE,
        f"expected REVOLUTE jog wheel, got {wheel_spin.articulation_type!r}",
    )
    ctx.check(
        "jog wheel spins about vertical axis",
        wheel_spin.axis == (0.0, 0.0, 1.0),
        f"unexpected jog wheel axis {wheel_spin.axis!r}",
    )
    ctx.check(
        "pitch slider is prismatic",
        pitch_slide.articulation_type == ArticulationType.PRISMATIC,
        f"expected PRISMATIC pitch joint, got {pitch_slide.articulation_type!r}",
    )
    ctx.check(
        "pitch slider travels along the rail",
        pitch_slide.axis == (0.0, 1.0, 0.0),
        f"unexpected pitch slider axis {pitch_slide.axis!r}",
    )

    with ctx.pose({tray_slide: 0.0}):
        ctx.expect_contact(
            tray,
            body,
            elem_a=tray_runner_left,
            elem_b=tray_rail_left,
            name="tray runner contacts the left rail at rest",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.156,
            elem_a=tray_face,
            elem_b=front_slot_lintel,
            name="tray bezel matches the slot width",
        )
        ctx.expect_gap(
            body,
            tray,
            axis="z",
            min_gap=0.0005,
            max_gap=0.0025,
            positive_elem=front_slot_lintel,
            negative_elem=tray_face,
            name="tray bezel clears the upper slot lintel",
        )
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            min_gap=0.0005,
            max_gap=0.0025,
            positive_elem=tray_face,
            negative_elem=front_lower,
            name="tray bezel clears the lower slot edge",
        )

    with ctx.pose({tray_slide: 0.10}):
        ctx.expect_contact(
            tray,
            body,
            elem_a=tray_runner_left,
            elem_b=tray_rail_left,
            name="tray stays engaged with the rail when extended",
        )
        ctx.expect_gap(
            body,
            tray,
            axis="y",
            min_gap=0.09,
            max_gap=0.095,
            positive_elem=front_lower,
            negative_elem=tray_face,
            name="disc tray extends clearly beyond the player front",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.156,
            elem_a=tray_face,
            elem_b=front_slot_lintel,
            name="extended tray remains centered in the slot opening",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="tray_extended_no_overlap")
        ctx.fail_if_isolated_parts(name="tray_extended_no_floating")

    with ctx.pose({wheel_spin: 0.0}):
        ctx.expect_contact(
            platter,
            body,
            elem_a=platter_base,
            elem_b=wheel_pedestal,
            name="jog wheel sits on the spindle pedestal",
        )
        ctx.expect_within(
            platter,
            body,
            axes="xy",
            inner_elem=platter_rim,
            outer_elem=wheel_well,
            name="jog wheel nests inside the top deck well",
        )
        ctx.expect_within(
            platter,
            body,
            axes="xy",
            inner_elem=platter_marker,
            outer_elem=wheel_well,
            name="rotation marker stays over the jog wheel well",
        )

    with ctx.pose({wheel_spin: 3.14}):
        ctx.expect_contact(
            platter,
            body,
            elem_a=platter_base,
            elem_b=wheel_pedestal,
            name="jog wheel remains seated while rotated",
        )
        ctx.expect_within(
            platter,
            body,
            axes="xy",
            inner_elem=platter_marker,
            outer_elem=wheel_well,
            name="rotation marker remains inside the jog wheel opening after a half turn",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_half_turn_no_overlap")

    with ctx.pose({pitch_slide: 0.0}):
        ctx.expect_contact(
            slider,
            body,
            elem_a=slider_shoe,
            elem_b=pitch_rail,
            name="pitch slider shoe rests on the rail",
        )
        ctx.expect_within(
            slider,
            body,
            axes="xy",
            inner_elem=slider_shoe,
            outer_elem=pitch_rail,
            name="pitch slider stays within the rail footprint at center",
        )

    with ctx.pose({pitch_slide: 0.045}):
        ctx.expect_contact(
            slider,
            body,
            elem_a=slider_shoe,
            elem_b=pitch_rail,
            name="pitch slider remains on the rail at maximum up throw",
        )
        ctx.expect_within(
            slider,
            body,
            axes="xy",
            inner_elem=slider_shoe,
            outer_elem=pitch_rail,
            name="pitch slider upper throw stays on the rail",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="pitch_slider_upper_no_overlap")

    with ctx.pose({pitch_slide: -0.045}):
        ctx.expect_contact(
            slider,
            body,
            elem_a=slider_shoe,
            elem_b=pitch_rail,
            name="pitch slider remains on the rail at maximum down throw",
        )
        ctx.expect_within(
            slider,
            body,
            axes="xy",
            inner_elem=slider_shoe,
            outer_elem=pitch_rail,
            name="pitch slider lower throw stays on the rail",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="pitch_slider_lower_no_overlap")
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
