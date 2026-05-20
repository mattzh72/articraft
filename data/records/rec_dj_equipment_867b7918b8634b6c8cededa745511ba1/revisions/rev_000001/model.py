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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _build_jog_wheel(model: ArticulatedObject, name: str, metal, dark, label):
    wheel = model.part(name)
    wheel.visual(
        Cylinder(radius=0.076, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark,
        name="outer_rim",
    )
    wheel.visual(
        Cylinder(radius=0.070, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=metal,
        name="platter",
    )
    wheel.visual(
        Cylinder(radius=0.052, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=dark,
        name="top_disc",
    )
    wheel.visual(
        Cylinder(radius=0.019, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0144)),
        material=label,
        name="center_label",
    )
    wheel.visual(
        Box((0.014, 0.006, 0.0026)),
        origin=Origin(xyz=(0.050, 0.0, 0.0153)),
        material=label,
        name="marker",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.076, length=0.018),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )
    return wheel


def _build_fader(model: ArticulatedObject, name: str, cap_material, stem_material):
    fader = model.part(name)
    fader.visual(
        Box((0.004, 0.020, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=stem_material,
        name="stem",
    )
    fader.visual(
        Box((0.017, 0.013, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
        material=cap_material,
        name="cap",
    )
    fader.inertial = Inertial.from_geometry(
        Box((0.017, 0.020, 0.013)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
    )
    return fader


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_controller")

    housing_body = model.material("housing_body", rgba=(0.13, 0.14, 0.15, 1.0))
    deck_surface = model.material("deck_surface", rgba=(0.07, 0.08, 0.09, 1.0))
    mixer_surface = model.material("mixer_surface", rgba=(0.10, 0.11, 0.12, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.63, 0.66, 0.69, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    wheel_label = model.material("wheel_label", rgba=(0.89, 0.90, 0.92, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.16, 0.38, 0.46, 0.45))
    slot_black = model.material("slot_black", rgba=(0.02, 0.02, 0.03, 1.0))
    fader_cap = model.material("fader_cap", rgba=(0.82, 0.83, 0.85, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.32, 0.34, 0.36, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.740, 0.370, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=housing_body,
        name="body_base",
    )
    housing.visual(
        Box((0.720, 0.350, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=housing_body,
        name="top_panel",
    )
    housing.visual(
        Box((0.720, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.125, 0.053)),
        material=housing_body,
        name="rear_riser",
    )
    housing.visual(
        Box((0.248, 0.308, 0.002)),
        origin=Origin(xyz=(-0.225, 0.0, 0.0465)),
        material=deck_surface,
        name="left_deck_panel",
    )
    housing.visual(
        Box((0.248, 0.308, 0.002)),
        origin=Origin(xyz=(0.225, 0.0, 0.0465)),
        material=deck_surface,
        name="right_deck_panel",
    )
    housing.visual(
        Box((0.156, 0.308, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0465)),
        material=mixer_surface,
        name="mixer_panel",
    )
    housing.visual(
        Cylinder(radius=0.081, length=0.002),
        origin=Origin(xyz=(-0.225, 0.0, 0.046)),
        material=slot_black,
        name="left_jog_bed",
    )
    housing.visual(
        Cylinder(radius=0.081, length=0.002),
        origin=Origin(xyz=(0.225, 0.0, 0.046)),
        material=slot_black,
        name="right_jog_bed",
    )
    housing.visual(
        Box((0.090, 0.056, 0.0016)),
        origin=Origin(xyz=(-0.225, 0.112, 0.0468)),
        material=screen_glass,
        name="left_display",
    )
    housing.visual(
        Box((0.090, 0.056, 0.0016)),
        origin=Origin(xyz=(0.225, 0.112, 0.0468)),
        material=screen_glass,
        name="right_display",
    )
    housing.visual(
        Box((0.038, 0.088, 0.0016)),
        origin=Origin(xyz=(0.0, 0.112, 0.0468)),
        material=screen_glass,
        name="level_meter",
    )
    housing.visual(
        Box((0.084, 0.050, 0.0016)),
        origin=Origin(xyz=(-0.225, -0.124, 0.0468)),
        material=accent_grey,
        name="left_transport_strip",
    )
    housing.visual(
        Box((0.084, 0.050, 0.0016)),
        origin=Origin(xyz=(0.225, -0.124, 0.0468)),
        material=accent_grey,
        name="right_transport_strip",
    )
    housing.visual(
        Box((0.008, 0.126, 0.0015)),
        origin=Origin(xyz=(-0.340, 0.010, 0.04625)),
        material=slot_black,
        name="left_pitch_slot",
    )
    housing.visual(
        Box((0.008, 0.126, 0.0015)),
        origin=Origin(xyz=(0.340, 0.010, 0.04625)),
        material=slot_black,
        name="right_pitch_slot",
    )

    fader_x_positions = (-0.045, -0.015, 0.015, 0.045)
    for index, x_pos in enumerate(fader_x_positions, start=1):
        housing.visual(
            Box((0.008, 0.102, 0.0015)),
            origin=Origin(xyz=(x_pos, -0.002, 0.04625)),
            material=slot_black,
            name=f"slot_{index}",
        )

    housing.inertial = Inertial.from_geometry(
        Box((0.740, 0.370, 0.062)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
    )

    left_jog = _build_jog_wheel(model, "left_jog_wheel", wheel_metal, wheel_dark, wheel_label)
    right_jog = _build_jog_wheel(model, "right_jog_wheel", wheel_metal, wheel_dark, wheel_label)

    model.articulation(
        "left_jog_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_jog,
        origin=Origin(xyz=(-0.225, 0.0, 0.047)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )
    model.articulation(
        "right_jog_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_jog,
        origin=Origin(xyz=(0.225, 0.0, 0.047)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    for index, x_pos in enumerate(fader_x_positions, start=1):
        fader = _build_fader(model, f"channel_fader_{index}", fader_cap, accent_grey)
        model.articulation(
            f"channel_fader_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(x_pos, -0.040, 0.047)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.18,
                lower=0.0,
                upper=0.074,
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

    housing = object_model.get_part("housing")
    left_jog = object_model.get_part("left_jog_wheel")
    right_jog = object_model.get_part("right_jog_wheel")
    left_jog_spin = object_model.get_articulation("left_jog_spin")
    right_jog_spin = object_model.get_articulation("right_jog_spin")

    ctx.check(
        "jog wheels use continuous z-axis rotation",
        left_jog_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_jog_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_jog_spin.axis == (0.0, 0.0, 1.0)
        and right_jog_spin.axis == (0.0, 0.0, 1.0)
        and left_jog_spin.motion_limits is not None
        and right_jog_spin.motion_limits is not None
        and left_jog_spin.motion_limits.lower is None
        and left_jog_spin.motion_limits.upper is None
        and right_jog_spin.motion_limits.lower is None
        and right_jog_spin.motion_limits.upper is None,
        details=(
            f"left={left_jog_spin.articulation_type}/{left_jog_spin.axis}/"
            f"{left_jog_spin.motion_limits}, right={right_jog_spin.articulation_type}/"
            f"{right_jog_spin.axis}/{right_jog_spin.motion_limits}"
        ),
    )

    ctx.expect_gap(
        left_jog,
        housing,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="platter",
        negative_elem="left_jog_bed",
        name="left jog wheel seats on its deck bed",
    )
    ctx.expect_gap(
        right_jog,
        housing,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="platter",
        negative_elem="right_jog_bed",
        name="right jog wheel seats on its deck bed",
    )

    with ctx.pose({left_jog_spin: 0.0}):
        left_rest_center = _aabb_center(ctx.part_element_world_aabb(left_jog, elem="marker"))
    with ctx.pose({left_jog_spin: math.pi / 2.0}):
        left_turn_center = _aabb_center(ctx.part_element_world_aabb(left_jog, elem="marker"))
    ctx.check(
        "left jog marker moves when wheel spins",
        left_rest_center is not None
        and left_turn_center is not None
        and abs(left_turn_center[0] - left_rest_center[0]) > 0.020
        and abs(left_turn_center[1] - left_rest_center[1]) > 0.020,
        details=f"rest={left_rest_center}, turned={left_turn_center}",
    )

    with ctx.pose({right_jog_spin: 0.0}):
        right_rest_center = _aabb_center(ctx.part_element_world_aabb(right_jog, elem="marker"))
    with ctx.pose({right_jog_spin: math.pi / 2.0}):
        right_turn_center = _aabb_center(ctx.part_element_world_aabb(right_jog, elem="marker"))
    ctx.check(
        "right jog marker moves when wheel spins",
        right_rest_center is not None
        and right_turn_center is not None
        and abs(right_turn_center[0] - right_rest_center[0]) > 0.020
        and abs(right_turn_center[1] - right_rest_center[1]) > 0.020,
        details=f"rest={right_rest_center}, turned={right_turn_center}",
    )

    for index in range(1, 5):
        fader = object_model.get_part(f"channel_fader_{index}")
        slide = object_model.get_articulation(f"channel_fader_{index}_slide")
        limits = slide.motion_limits

        ctx.check(
            f"channel fader {index} uses prismatic y-axis travel",
            slide.articulation_type == ArticulationType.PRISMATIC
            and slide.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper > 0.05,
            details=f"joint={slide.articulation_type}, axis={slide.axis}, limits={limits}",
        )

        ctx.expect_gap(
            fader,
            housing,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem="stem",
            negative_elem=f"slot_{index}",
            name=f"channel fader {index} stem stays seated on slot face",
        )
        ctx.expect_within(
            fader,
            housing,
            axes="xy",
            margin=0.0,
            inner_elem="stem",
            outer_elem=f"slot_{index}",
            name=f"channel fader {index} stem stays inside slot at rest",
        )

        with ctx.pose({slide: limits.upper}):
            ctx.expect_gap(
                fader,
                housing,
                axis="z",
                max_gap=0.0005,
                max_penetration=0.0,
                positive_elem="stem",
                negative_elem=f"slot_{index}",
                name=f"channel fader {index} stem stays seated at full throw",
            )
            ctx.expect_within(
                fader,
                housing,
                axes="xy",
                margin=0.0,
                inner_elem="stem",
                outer_elem=f"slot_{index}",
                name=f"channel fader {index} stem stays inside slot at full throw",
            )

        with ctx.pose({slide: 0.0}):
            rest_pos = ctx.part_world_position(fader)
        with ctx.pose({slide: limits.upper}):
            top_pos = ctx.part_world_position(fader)

        ctx.check(
            f"channel fader {index} moves upward in its slot",
            rest_pos is not None
            and top_pos is not None
            and top_pos[1] > rest_pos[1] + 0.05
            and abs(top_pos[0] - rest_pos[0]) < 1e-6
            and abs(top_pos[2] - rest_pos[2]) < 1e-6,
            details=f"rest={rest_pos}, top={top_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
