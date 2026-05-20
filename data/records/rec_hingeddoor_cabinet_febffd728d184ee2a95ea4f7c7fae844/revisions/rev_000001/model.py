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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _add_glazed_door(
    model: ArticulatedObject,
    *,
    name: str,
    wood,
    glass,
    span: float,
    height: float,
    thickness: float,
    frame_width: float,
    muntin_width: float,
) -> object:
    door = model.part(name)
    width_abs = abs(span)
    x_sign = 1.0 if span >= 0.0 else -1.0
    glass_embed = 0.012
    glass_width = width_abs - 2.0 * (frame_width - glass_embed)
    glass_height = height - 2.0 * (frame_width - glass_embed)

    door.visual(
        Box((frame_width, thickness, height)),
        origin=Origin(xyz=(x_sign * frame_width / 2.0, 0.0, height / 2.0)),
        material=wood,
        name="outer_stile",
    )
    door.visual(
        Box((frame_width, thickness, height)),
        origin=Origin(xyz=(span - x_sign * frame_width / 2.0, 0.0, height / 2.0)),
        material=wood,
        name="inner_stile",
    )
    door.visual(
        Box((width_abs - 2.0 * frame_width, thickness, frame_width)),
        origin=Origin(xyz=(span / 2.0, 0.0, frame_width / 2.0)),
        material=wood,
        name="bottom_rail",
    )
    door.visual(
        Box((width_abs - 2.0 * frame_width, thickness, frame_width)),
        origin=Origin(xyz=(span / 2.0, 0.0, height - frame_width / 2.0)),
        material=wood,
        name="top_rail",
    )
    door.visual(
        Box((muntin_width, thickness, height - 2.0 * frame_width + 2.0 * glass_embed)),
        origin=Origin(xyz=(span / 2.0, 0.0, height / 2.0)),
        material=wood,
        name="center_muntin",
    )
    door.visual(
        Box((glass_width, thickness * 0.45, glass_height)),
        origin=Origin(xyz=(span / 2.0, 0.0, height / 2.0)),
        material=glass,
        name="glass_lite",
    )
    return door


def _add_solid_door(
    model: ArticulatedObject,
    *,
    name: str,
    wood,
    span: float,
    height: float,
    thickness: float,
    frame_width: float,
) -> object:
    door = model.part(name)
    width_abs = abs(span)
    x_sign = 1.0 if span >= 0.0 else -1.0
    panel_embed = 0.012
    panel_width = width_abs - 2.0 * (frame_width - panel_embed)
    panel_height = height - 2.0 * (frame_width - panel_embed)

    door.visual(
        Box((frame_width, thickness, height)),
        origin=Origin(xyz=(x_sign * frame_width / 2.0, 0.0, height / 2.0)),
        material=wood,
        name="outer_stile",
    )
    door.visual(
        Box((frame_width, thickness, height)),
        origin=Origin(xyz=(span - x_sign * frame_width / 2.0, 0.0, height / 2.0)),
        material=wood,
        name="inner_stile",
    )
    door.visual(
        Box((width_abs - 2.0 * frame_width, thickness, frame_width)),
        origin=Origin(xyz=(span / 2.0, 0.0, frame_width / 2.0)),
        material=wood,
        name="bottom_rail",
    )
    door.visual(
        Box((width_abs - 2.0 * frame_width, thickness, frame_width)),
        origin=Origin(xyz=(span / 2.0, 0.0, height - frame_width / 2.0)),
        material=wood,
        name="top_rail",
    )
    door.visual(
        Box((panel_width, thickness * 0.72, panel_height)),
        origin=Origin(xyz=(span / 2.0, -thickness * 0.08, height / 2.0)),
        material=wood,
        name="field_panel",
    )
    return door


def _add_latch_knob(model: ArticulatedObject, *, name: str, metal) -> object:
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="backplate",
    )
    knob.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="stem",
    )
    knob.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.0, 0.034, 0.0)),
        material=metal,
        name="knob_head",
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hutch_cabinet")

    oak = model.material("oak", rgba=(0.66, 0.50, 0.33, 1.0))
    shadow_oak = model.material("shadow_oak", rgba=(0.53, 0.38, 0.24, 1.0))
    glass = model.material("glass", rgba=(0.78, 0.90, 0.96, 0.32))
    brass = model.material("brass", rgba=(0.78, 0.66, 0.34, 1.0))

    lower_width = 1.10
    lower_depth = 0.42
    lower_height = 0.92
    upper_width = 1.00
    upper_depth = 0.30
    upper_height = 0.90
    upper_center_y = 0.02

    lower_front_y = lower_depth / 2.0
    upper_front_y = upper_center_y + upper_depth / 2.0

    case = model.part("case")

    case.visual(
        Box((1.02, 0.32, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=shadow_oak,
        name="plinth",
    )
    case.visual(
        Box((0.026, lower_depth, 0.84)),
        origin=Origin(xyz=(-0.537, 0.0, 0.50)),
        material=oak,
        name="lower_left_side",
    )
    case.visual(
        Box((0.026, lower_depth, 0.84)),
        origin=Origin(xyz=(0.537, 0.0, 0.50)),
        material=oak,
        name="lower_right_side",
    )
    case.visual(
        Box((1.048, 0.39, 0.022)),
        origin=Origin(xyz=(0.0, -0.003, 0.091)),
        material=oak,
        name="lower_bottom",
    )
    case.visual(
        Box((1.048, 0.34, 0.020)),
        origin=Origin(xyz=(0.0, -0.018, 0.46)),
        material=oak,
        name="lower_shelf",
    )
    case.visual(
        Box((1.10, lower_depth, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.906)),
        material=oak,
        name="counter_deck",
    )
    case.visual(
        Box((1.048, 0.012, 0.84)),
        origin=Origin(xyz=(0.0, -0.204, 0.50)),
        material=shadow_oak,
        name="lower_back",
    )

    case.visual(
        Box((0.022, upper_depth, upper_height)),
        origin=Origin(xyz=(-0.489, upper_center_y, 1.37)),
        material=oak,
        name="upper_left_side",
    )
    case.visual(
        Box((0.022, upper_depth, upper_height)),
        origin=Origin(xyz=(0.489, upper_center_y, 1.37)),
        material=oak,
        name="upper_right_side",
    )
    case.visual(
        Box((0.956, 0.260, 0.018)),
        origin=Origin(xyz=(0.0, 0.005, 1.21)),
        material=oak,
        name="upper_shelf_lower",
    )
    case.visual(
        Box((0.956, 0.260, 0.018)),
        origin=Origin(xyz=(0.0, 0.005, 1.49)),
        material=oak,
        name="upper_shelf_upper",
    )
    case.visual(
        Box((0.956, 0.010, 0.88)),
        origin=Origin(xyz=(0.0, -0.125, 1.36)),
        material=shadow_oak,
        name="upper_back",
    )
    case.visual(
        Box((1.00, upper_depth, 0.024)),
        origin=Origin(xyz=(0.0, upper_center_y, 1.808)),
        material=oak,
        name="upper_top",
    )
    case.visual(
        Box((1.06, 0.34, 0.05)),
        origin=Origin(xyz=(0.0, upper_center_y, 1.845)),
        material=shadow_oak,
        name="crown",
    )

    upper_door_height = 0.82
    upper_door_thickness = 0.022
    upper_door_span = 0.498
    upper_door_bottom = 0.95
    upper_frame = 0.075

    lower_door_height = 0.78
    lower_door_thickness = 0.024
    lower_door_span = 0.548
    lower_door_bottom = 0.11
    lower_frame = 0.090

    upper_left_door = _add_glazed_door(
        model,
        name="upper_left_door",
        wood=oak,
        glass=glass,
        span=upper_door_span,
        height=upper_door_height,
        thickness=upper_door_thickness,
        frame_width=upper_frame,
        muntin_width=0.028,
    )
    upper_right_door = _add_glazed_door(
        model,
        name="upper_right_door",
        wood=oak,
        glass=glass,
        span=-upper_door_span,
        height=upper_door_height,
        thickness=upper_door_thickness,
        frame_width=upper_frame,
        muntin_width=0.028,
    )
    lower_left_door = _add_solid_door(
        model,
        name="lower_left_door",
        wood=oak,
        span=lower_door_span,
        height=lower_door_height,
        thickness=lower_door_thickness,
        frame_width=lower_frame,
    )
    lower_right_door = _add_solid_door(
        model,
        name="lower_right_door",
        wood=oak,
        span=-lower_door_span,
        height=lower_door_height,
        thickness=lower_door_thickness,
        frame_width=lower_frame,
    )
    lower_left_knob = _add_latch_knob(model, name="lower_left_knob", metal=brass)
    lower_right_knob = _add_latch_knob(model, name="lower_right_knob", metal=brass)

    model.articulation(
        "upper_left_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=upper_left_door,
        origin=Origin(xyz=(-upper_width / 2.0, upper_front_y + upper_door_thickness / 2.0, upper_door_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "upper_right_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=upper_right_door,
        origin=Origin(xyz=(upper_width / 2.0, upper_front_y + upper_door_thickness / 2.0, upper_door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "lower_left_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lower_left_door,
        origin=Origin(xyz=(-lower_width / 2.0, lower_front_y + lower_door_thickness / 2.0, lower_door_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.5, lower=0.0, upper=1.70),
    )
    model.articulation(
        "lower_right_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lower_right_door,
        origin=Origin(xyz=(lower_width / 2.0, lower_front_y + lower_door_thickness / 2.0, lower_door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.5, lower=0.0, upper=1.70),
    )
    model.articulation(
        "lower_left_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_left_door,
        child=lower_left_knob,
        origin=Origin(xyz=(lower_door_span - 0.060, lower_door_thickness / 2.0, 0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )
    model.articulation(
        "lower_right_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_right_door,
        child=lower_right_knob,
        origin=Origin(xyz=(-lower_door_span + 0.060, lower_door_thickness / 2.0, 0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
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
    case = object_model.get_part("case")
    upper_left_door = object_model.get_part("upper_left_door")
    upper_right_door = object_model.get_part("upper_right_door")
    lower_left_door = object_model.get_part("lower_left_door")
    lower_right_door = object_model.get_part("lower_right_door")
    lower_left_knob = object_model.get_part("lower_left_knob")
    lower_right_knob = object_model.get_part("lower_right_knob")

    upper_left_hinge = object_model.get_articulation("upper_left_hinge")
    upper_right_hinge = object_model.get_articulation("upper_right_hinge")
    lower_left_hinge = object_model.get_articulation("lower_left_hinge")
    lower_right_hinge = object_model.get_articulation("lower_right_hinge")
    lower_left_knob_spin = object_model.get_articulation("lower_left_knob_spin")
    lower_right_knob_spin = object_model.get_articulation("lower_right_knob_spin")

    ctx.check(
        "upper doors remain glazed",
        any(v.name == "glass_lite" for v in upper_left_door.visuals)
        and any(v.name == "glass_lite" for v in upper_right_door.visuals),
        details="Both upper doors should include named glass_lite visuals.",
    )

    with ctx.pose(
        {
            upper_left_hinge: 0.0,
            upper_right_hinge: 0.0,
            lower_left_hinge: 0.0,
            lower_right_hinge: 0.0,
        }
    ):
        ctx.expect_contact(
            upper_left_door,
            case,
            elem_b="upper_left_side",
            name="upper left door seats against the upper case side",
        )
        ctx.expect_contact(
            upper_right_door,
            case,
            elem_b="upper_right_side",
            name="upper right door seats against the upper case side",
        )
        ctx.expect_contact(
            lower_left_door,
            case,
            elem_b="lower_left_side",
            name="lower left door seats against the lower case side",
        )
        ctx.expect_contact(
            lower_right_door,
            case,
            elem_b="lower_right_side",
            name="lower right door seats against the lower case side",
        )
        ctx.expect_contact(
            lower_left_knob,
            lower_left_door,
            name="left lower knob is mounted on its door face",
        )
        ctx.expect_contact(
            lower_right_knob,
            lower_right_door,
            name="right lower knob is mounted on its door face",
        )

    left_closed = ctx.part_world_aabb(upper_left_door)
    with ctx.pose({upper_left_hinge: 1.15}):
        left_open = ctx.part_world_aabb(upper_left_door)
    ctx.check(
        "left-side doors open outward with positive rotation",
        left_closed is not None
        and left_open is not None
        and left_open[1][1] > left_closed[1][1] + 0.12,
        details=f"closed={left_closed}, open={left_open}",
    )

    right_closed = ctx.part_world_aabb(upper_right_door)
    with ctx.pose({upper_right_hinge: 1.15}):
        right_open = ctx.part_world_aabb(upper_right_door)
    ctx.check(
        "right-side doors open outward with positive rotation",
        right_closed is not None
        and right_open is not None
        and right_open[1][1] > right_closed[1][1] + 0.12,
        details=f"closed={right_closed}, open={right_open}",
    )

    hinge_limits_ok = True
    for hinge, expected_axis in (
        (upper_left_hinge, (0.0, 0.0, 1.0)),
        (lower_left_hinge, (0.0, 0.0, 1.0)),
        (upper_right_hinge, (0.0, 0.0, -1.0)),
        (lower_right_hinge, (0.0, 0.0, -1.0)),
    ):
        limits = hinge.motion_limits
        hinge_limits_ok = hinge_limits_ok and hinge.axis == expected_axis
        hinge_limits_ok = hinge_limits_ok and limits is not None
        hinge_limits_ok = hinge_limits_ok and limits.lower == 0.0
        hinge_limits_ok = hinge_limits_ok and limits.upper is not None and limits.upper > 1.5
    ctx.check(
        "all four cabinet doors use vertical side hinges",
        hinge_limits_ok,
        details="Expected left hinges on +Z, right hinges on -Z, each with realistic opening travel.",
    )

    knob_axes_ok = (
        lower_left_knob_spin.axis == (0.0, 1.0, 0.0)
        and lower_right_knob_spin.axis == (0.0, 1.0, 0.0)
        and lower_left_knob_spin.motion_limits is not None
        and lower_right_knob_spin.motion_limits is not None
        and lower_left_knob_spin.motion_limits.lower is None
        and lower_left_knob_spin.motion_limits.upper is None
        and lower_right_knob_spin.motion_limits.lower is None
        and lower_right_knob_spin.motion_limits.upper is None
    )
    ctx.check(
        "lower latch knobs spin on local fore-aft shafts",
        knob_axes_ok,
        details="Expected both lower knobs to use continuous rotation about their local +Y shaft axes.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
