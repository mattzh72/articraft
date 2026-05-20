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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    cut_opening_on_face,
    mesh_from_geometry,
    rounded_rect_profile,
)


BAR_LEN = 0.128
BAR_W = 0.040
BAR_T = 0.012

SLOT_LEN = 0.110
SLOT_W = 0.010
SLOT_DEPTH = 0.005
SLOT_Y = -0.011

DIAL_BOSS_R = 0.006
DIAL_BOSS_H = 0.0014
DIAL_R = 0.013
DIAL_H = 0.0052
DIAL_CAP_R = 0.009
DIAL_CAP_H = 0.0015

SLIDER_CAP_L = 0.022
SLIDER_CAP_W = 0.016
SLIDER_CAP_H = 0.006
SLIDER_STEM_L = 0.008
SLIDER_STEM_W = 0.006
SLIDER_STEM_H = 0.002
SLIDER_SHOE_L = 0.018
SLIDER_SHOE_W = 0.008
SLIDER_SHOE_H = 0.003
SLIDER_LEFT_X = -(SLOT_LEN * 0.5 - SLIDER_CAP_L * 0.5)
SLIDER_TRAVEL = SLOT_LEN - SLIDER_CAP_L
SLOT_FLOOR_Z = BAR_T - SLOT_DEPTH

TOGGLE_PIVOT_Y = 0.0
TOGGLE_PIVOT_Z = BAR_T * 0.5
TOGGLE_BARREL_R = 0.0032
TOGGLE_BARREL_L = 0.015
TOGGLE_SWEEP = math.radians(32.0)


def _build_body_mesh():
    shell = BoxGeometry((BAR_LEN, BAR_W, BAR_T))
    slot_profile = rounded_rect_profile(SLOT_LEN, SLOT_W, radius=SLOT_W * 0.45, corner_segments=10)
    return cut_opening_on_face(
        shell,
        face="+z",
        opening_profile=slot_profile,
        depth=SLOT_DEPTH,
        offset=(0.0, SLOT_Y),
    )


def _add_slider_knurl(part, material) -> None:
    rib_count = 5
    rib_pitch = 0.0037
    rib_length = 0.0015
    rib_height = 0.0011
    for idx in range(rib_count):
        x = (idx - (rib_count - 1) * 0.5) * rib_pitch
        part.visual(
            Box((rib_length, SLIDER_CAP_W * 0.82, rib_height)),
            origin=Origin(xyz=(x, 0.0, SLIDER_CAP_H * 0.5 + rib_height * 0.5)),
            material=material,
            name=f"knurl_rib_{idx}",
        )


def _build_slider_part(model: ArticulatedObject, material):
    slider = model.part("slider_knob")
    slider.visual(
        Box((SLIDER_CAP_L, SLIDER_CAP_W, SLIDER_CAP_H)),
        material=material,
        name="cap",
    )
    slider.visual(
        Box((SLIDER_STEM_L, SLIDER_STEM_W, SLIDER_STEM_H)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=material,
        name="stem",
    )
    slider.visual(
        Box((SLIDER_SHOE_L, SLIDER_SHOE_W, SLIDER_SHOE_H)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                SLOT_FLOOR_Z + SLIDER_SHOE_H * 0.5 - (BAR_T + SLIDER_CAP_H * 0.5),
            )
        ),
        material=material,
        name="shoe",
    )
    _add_slider_knurl(slider, material)
    slider.inertial = Inertial.from_geometry(
        Box((SLIDER_CAP_L, SLIDER_CAP_W, SLIDER_CAP_H + 0.008)),
        mass=0.03,
    )
    return slider


def _build_toggle_part(model: ArticulatedObject, name: str, direction: float, material, tip_material):
    toggle = model.part(name)
    toggle.visual(
        Cylinder(radius=TOGGLE_BARREL_R, length=TOGGLE_BARREL_L),
        origin=Origin(
            xyz=(direction * TOGGLE_BARREL_R, 0.0, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=material,
        name="pivot_barrel",
    )
    toggle.visual(
        Box((0.018, 0.010, 0.0036)),
        origin=Origin(xyz=(direction * 0.012, 0.0, 0.0046)),
        material=material,
        name="lever_body",
    )
    toggle.visual(
        Cylinder(radius=0.0031, length=0.010),
        origin=Origin(
            xyz=(direction * 0.019, 0.0, 0.0095),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=tip_material,
        name="lever_tip",
    )
    toggle.inertial = Inertial.from_geometry(
        Box((0.024, 0.014, 0.014)),
        mass=0.012,
        origin=Origin(xyz=(direction * 0.012, 0.0, 0.006)),
    )
    return toggle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dial_slide_fidget_bar")

    body_material = model.material("body_anodized", rgba=(0.20, 0.22, 0.25, 1.0))
    dial_material = model.material("dial_satin", rgba=(0.78, 0.80, 0.83, 1.0))
    control_material = model.material("control_black", rgba=(0.11, 0.11, 0.12, 1.0))
    switch_tip_material = model.material("switch_tip", rgba=(0.64, 0.66, 0.69, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_build_body_mesh(), "fidget_bar_body"),
        origin=Origin(xyz=(0.0, 0.0, BAR_T * 0.5)),
        material=body_material,
        name="shell",
    )
    body.visual(
        Cylinder(radius=DIAL_BOSS_R, length=DIAL_BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, BAR_T + DIAL_BOSS_H * 0.5)),
        material=body_material,
        name="dial_boss",
    )
    body.inertial = Inertial.from_geometry(
        Box((BAR_LEN, BAR_W, BAR_T + DIAL_BOSS_H)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, (BAR_T + DIAL_BOSS_H) * 0.5)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=DIAL_R, length=DIAL_H),
        origin=Origin(xyz=(0.0, 0.0, DIAL_H * 0.5)),
        material=dial_material,
        name="dial_disc",
    )
    dial.visual(
        Cylinder(radius=DIAL_CAP_R, length=DIAL_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, DIAL_H + DIAL_CAP_H * 0.5)),
        material=dial_material,
        name="dial_cap",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=DIAL_R, length=DIAL_H + DIAL_CAP_H),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, (DIAL_H + DIAL_CAP_H) * 0.5)),
    )

    slider = _build_slider_part(model, control_material)

    left_toggle = _build_toggle_part(
        model,
        "left_toggle",
        -1.0,
        control_material,
        switch_tip_material,
    )
    right_toggle = _build_toggle_part(
        model,
        "right_toggle",
        1.0,
        control_material,
        switch_tip_material,
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, BAR_T + DIAL_BOSS_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=18.0),
    )

    model.articulation(
        "body_to_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(SLIDER_LEFT_X, SLOT_Y, BAR_T + SLIDER_CAP_H * 0.5)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.14,
            lower=0.0,
            upper=SLIDER_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_left_toggle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_toggle,
        origin=Origin(xyz=(-BAR_LEN * 0.5, TOGGLE_PIVOT_Y, TOGGLE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-TOGGLE_SWEEP,
            upper=TOGGLE_SWEEP,
        ),
    )

    model.articulation(
        "body_to_right_toggle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_toggle,
        origin=Origin(xyz=(BAR_LEN * 0.5, TOGGLE_PIVOT_Y, TOGGLE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-TOGGLE_SWEEP,
            upper=TOGGLE_SWEEP,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    slider = object_model.get_part("slider_knob")
    left_toggle = object_model.get_part("left_toggle")
    right_toggle = object_model.get_part("right_toggle")

    dial_joint = object_model.get_articulation("body_to_dial")
    slider_joint = object_model.get_articulation("body_to_slider")
    left_joint = object_model.get_articulation("body_to_left_toggle")
    right_joint = object_model.get_articulation("body_to_right_toggle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "dial spins around the vertical axis",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS and dial_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}",
    )
    ctx.check(
        "slider travels along the bar length",
        slider_joint.articulation_type == ArticulationType.PRISMATIC
        and slider_joint.axis == (1.0, 0.0, 0.0)
        and slider_joint.motion_limits is not None
        and slider_joint.motion_limits.lower == 0.0
        and slider_joint.motion_limits.upper is not None
        and abs(slider_joint.motion_limits.upper - SLIDER_TRAVEL) < 1e-9,
        details=f"type={slider_joint.articulation_type}, axis={slider_joint.axis}, limits={slider_joint.motion_limits}",
    )
    ctx.check(
        "left toggle uses an end-face pivot",
        left_joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={left_joint.axis}",
    )
    ctx.check(
        "right toggle uses a mirrored end-face pivot",
        right_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={right_joint.axis}",
    )

    ctx.expect_origin_distance(
        dial,
        body,
        axes="xy",
        max_dist=0.001,
        name="dial stays centered on the top face",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="dial_disc",
        name="dial seats on the body boss without sinking",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="dial_disc",
        min_overlap=0.020,
        name="dial footprint stays within the bar deck",
    )

    with ctx.pose({slider_joint: 0.0}):
        ctx.expect_within(
            slider,
            body,
            axes="y",
            margin=0.0,
            name="slider stays inside the bar width at the left stop",
        )
        ctx.expect_gap(
            slider,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="cap",
            negative_elem="shell",
            name="slider cap rides on the body rails at the left stop",
        )

    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        extended_slider_pos = ctx.part_world_position(slider)
        ctx.expect_within(
            slider,
            body,
            axes="y",
            margin=0.0,
            name="slider stays inside the bar width at the right stop",
        )
        ctx.expect_gap(
            slider,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="cap",
            negative_elem="shell",
            name="slider cap rides on the body rails at the right stop",
        )

    ctx.check(
        "slider crosses nearly the full bar track",
        rest_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[0] > rest_slider_pos[0] + SLIDER_TRAVEL * 0.95,
        details=f"rest={rest_slider_pos}, extended={extended_slider_pos}, travel={SLIDER_TRAVEL}",
    )

    ctx.expect_gap(
        body,
        left_toggle,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem="pivot_barrel",
        name="left toggle barrel seats on the left end face",
    )
    ctx.expect_gap(
        right_toggle,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="pivot_barrel",
        name="right toggle barrel seats on the right end face",
    )

    with ctx.pose({left_joint: left_joint.motion_limits.lower}):
        left_low_tip = ctx.part_element_world_aabb(left_toggle, elem="lever_tip")
    with ctx.pose({left_joint: left_joint.motion_limits.upper}):
        left_high_tip = ctx.part_element_world_aabb(left_toggle, elem="lever_tip")
    ctx.check(
        "left toggle clicks upward across its sweep",
        left_low_tip is not None and left_high_tip is not None and left_high_tip[1][2] > left_low_tip[1][2] + 0.004,
        details=f"low={left_low_tip}, high={left_high_tip}",
    )

    with ctx.pose({right_joint: right_joint.motion_limits.lower}):
        right_low_tip = ctx.part_element_world_aabb(right_toggle, elem="lever_tip")
    with ctx.pose({right_joint: right_joint.motion_limits.upper}):
        right_high_tip = ctx.part_element_world_aabb(right_toggle, elem="lever_tip")
    ctx.check(
        "right toggle clicks upward across its sweep",
        right_low_tip is not None
        and right_high_tip is not None
        and right_high_tip[1][2] > right_low_tip[1][2] + 0.004,
        details=f"low={right_low_tip}, high={right_high_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
