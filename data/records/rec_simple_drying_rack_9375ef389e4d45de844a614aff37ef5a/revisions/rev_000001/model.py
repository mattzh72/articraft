from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_RAIL_RADIUS = 0.014
WING_RAIL_RADIUS = 0.012
RUNG_RADIUS = 0.0065


def _add_vertical_rail(model: ArticulatedObject, name: str, length: float, radius: float, material) -> object:
    part = model.part(name)
    rail_size = (radius * 2.0, radius * 2.0, length)
    part.visual(
        Box(rail_size),
        origin=Origin(xyz=(0.0, 0.0, length * 0.5)),
        material=material,
        name="rail",
    )
    part.inertial = Inertial.from_geometry(
        Box(rail_size),
        mass=max(0.18, length * radius * 38.0),
        origin=Origin(xyz=(0.0, 0.0, length * 0.5)),
    )
    return part


def _add_span_rail_x(model: ArticulatedObject, name: str, length: float, radius: float, material) -> object:
    part = model.part(name)
    rail_size = (length, radius * 2.0, radius * 2.0)
    part.visual(
        Box(rail_size),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=material,
        name="rail",
    )
    part.inertial = Inertial.from_geometry(
        Box(rail_size),
        mass=max(0.06, length * radius * 22.0),
    )
    return part


def _add_top_bracket(model: ArticulatedObject, name: str, size: tuple[float, float, float], material) -> object:
    part = model.part(name)
    part.visual(Box(size), material=material, name="shell")
    part.inertial = Inertial.from_geometry(Box(size), mass=size[0] * size[1] * size[2] * 450.0)
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    powder_gray = model.material("powder_gray", rgba=(0.70, 0.73, 0.76, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    plastic_dark = model.material("plastic_dark", rgba=(0.22, 0.23, 0.25, 1.0))

    center_width = 0.74
    center_height = 1.02
    center_rung_span = center_width - 2.0 * FRAME_RAIL_RADIUS
    wing_width = 0.29
    wing_height = 0.92
    wing_rung_span = wing_width - 2.0 * WING_RAIL_RADIUS
    support_width = 0.58
    support_height = 0.54
    support_rung_span = support_width - 2.0 * WING_RAIL_RADIUS

    central_left_rail = _add_vertical_rail(
        model,
        "central_left_rail",
        length=center_height,
        radius=FRAME_RAIL_RADIUS,
        material=powder_gray,
    )
    central_right_rail = _add_vertical_rail(
        model,
        "central_right_rail",
        length=center_height,
        radius=FRAME_RAIL_RADIUS,
        material=powder_gray,
    )
    central_top_bracket = _add_top_bracket(
        model,
        "central_top_bracket",
        size=(center_rung_span, 0.024, 0.050),
        material=plastic_dark,
    )
    central_support_bracket = model.part("central_support_bracket")
    central_support_bracket.visual(
        Box((0.18, 0.024, 0.040)),
        material=hinge_gray,
        name="hinge_block",
    )
    central_support_bracket.visual(
        Box((center_rung_span, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=hinge_gray,
        name="crossbar",
    )
    central_support_bracket.visual(
        Box((0.10, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=hinge_gray,
        name="web",
    )
    central_support_bracket.inertial = Inertial.from_geometry(
        Box((center_rung_span, 0.024, 0.090)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    for index, z in enumerate((0.16, 0.30, 0.44, 0.58, 0.72, 0.86)):
        _add_span_rail_x(
            model,
            f"central_rung_{index + 1}",
            length=center_rung_span,
            radius=RUNG_RADIUS,
            material=powder_gray,
        )

    model.articulation(
        "central_left_to_right",
        ArticulationType.FIXED,
        parent=central_left_rail,
        child=central_right_rail,
        origin=Origin(xyz=(center_width, 0.0, 0.0)),
    )
    model.articulation(
        "central_left_to_top_bracket",
        ArticulationType.FIXED,
        parent=central_left_rail,
        child=central_top_bracket,
        origin=Origin(xyz=(center_width * 0.5, FRAME_RAIL_RADIUS + 0.012, center_height + 0.025)),
    )
    model.articulation(
        "central_left_to_support_bracket",
        ArticulationType.FIXED,
        parent=central_left_rail,
        child=central_support_bracket,
        origin=Origin(xyz=(center_width * 0.5, -0.026, 0.34)),
    )

    for index, z in enumerate((0.16, 0.30, 0.44, 0.58, 0.72, 0.86)):
        model.articulation(
            f"central_left_to_rung_{index + 1}",
            ArticulationType.FIXED,
            parent=central_left_rail,
            child=f"central_rung_{index + 1}",
            origin=Origin(xyz=(center_width * 0.5, FRAME_RAIL_RADIUS + RUNG_RADIUS, z)),
        )

    left_wing_inner_rail = _add_vertical_rail(
        model,
        "left_wing_inner_rail",
        length=wing_height,
        radius=WING_RAIL_RADIUS,
        material=powder_gray,
    )
    left_wing_outer_rail = _add_vertical_rail(
        model,
        "left_wing_outer_rail",
        length=wing_height,
        radius=WING_RAIL_RADIUS,
        material=powder_gray,
    )
    left_wing_top_bracket = _add_top_bracket(
        model,
        "left_wing_top_bracket",
        size=(wing_rung_span, 0.024, 0.046),
        material=plastic_dark,
    )
    for index, z in enumerate((0.20, 0.40, 0.60, 0.80)):
        _add_span_rail_x(
            model,
            f"left_wing_rung_{index + 1}",
            length=wing_rung_span,
            radius=RUNG_RADIUS,
            material=powder_gray,
        )

    model.articulation(
        "left_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central_left_rail,
        child=left_wing_inner_rail,
        origin=Origin(xyz=(-(FRAME_RAIL_RADIUS + WING_RAIL_RADIUS), 0.0, 0.0), rpy=(0.0, 0.0, 0.95)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=0.95),
    )
    model.articulation(
        "left_wing_inner_to_outer",
        ArticulationType.FIXED,
        parent=left_wing_inner_rail,
        child=left_wing_outer_rail,
        origin=Origin(xyz=(-wing_width, 0.0, 0.0)),
    )
    model.articulation(
        "left_wing_inner_to_top_bracket",
        ArticulationType.FIXED,
        parent=left_wing_inner_rail,
        child=left_wing_top_bracket,
        origin=Origin(xyz=(-wing_width * 0.5, WING_RAIL_RADIUS + 0.012, wing_height + 0.023)),
    )
    for index, z in enumerate((0.20, 0.40, 0.60, 0.80)):
        model.articulation(
            f"left_wing_inner_to_rung_{index + 1}",
            ArticulationType.FIXED,
            parent=left_wing_inner_rail,
            child=f"left_wing_rung_{index + 1}",
            origin=Origin(xyz=(-wing_width * 0.5, WING_RAIL_RADIUS + RUNG_RADIUS, z)),
        )

    right_wing_inner_rail = _add_vertical_rail(
        model,
        "right_wing_inner_rail",
        length=wing_height,
        radius=WING_RAIL_RADIUS,
        material=powder_gray,
    )
    right_wing_outer_rail = _add_vertical_rail(
        model,
        "right_wing_outer_rail",
        length=wing_height,
        radius=WING_RAIL_RADIUS,
        material=powder_gray,
    )
    right_wing_top_bracket = _add_top_bracket(
        model,
        "right_wing_top_bracket",
        size=(wing_rung_span, 0.024, 0.046),
        material=plastic_dark,
    )
    for index, z in enumerate((0.20, 0.40, 0.60, 0.80)):
        _add_span_rail_x(
            model,
            f"right_wing_rung_{index + 1}",
            length=wing_rung_span,
            radius=RUNG_RADIUS,
            material=powder_gray,
        )

    model.articulation(
        "right_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central_right_rail,
        child=right_wing_inner_rail,
        origin=Origin(xyz=((FRAME_RAIL_RADIUS + WING_RAIL_RADIUS), 0.0, 0.0), rpy=(0.0, 0.0, -0.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=0.95),
    )
    model.articulation(
        "right_wing_inner_to_outer",
        ArticulationType.FIXED,
        parent=right_wing_inner_rail,
        child=right_wing_outer_rail,
        origin=Origin(xyz=(wing_width, 0.0, 0.0)),
    )
    model.articulation(
        "right_wing_inner_to_top_bracket",
        ArticulationType.FIXED,
        parent=right_wing_inner_rail,
        child=right_wing_top_bracket,
        origin=Origin(xyz=(wing_width * 0.5, WING_RAIL_RADIUS + 0.012, wing_height + 0.023)),
    )
    for index, z in enumerate((0.20, 0.40, 0.60, 0.80)):
        model.articulation(
            f"right_wing_inner_to_rung_{index + 1}",
            ArticulationType.FIXED,
            parent=right_wing_inner_rail,
            child=f"right_wing_rung_{index + 1}",
            origin=Origin(xyz=(wing_width * 0.5, WING_RAIL_RADIUS + RUNG_RADIUS, z)),
        )

    lower_support_top_bracket = _add_top_bracket(
        model,
        "lower_support_top_bracket",
        size=(support_rung_span, 0.020, 0.020),
        material=hinge_gray,
    )
    lower_support_left_rail = _add_vertical_rail(
        model,
        "lower_support_left_rail",
        length=support_height,
        radius=WING_RAIL_RADIUS,
        material=powder_gray,
    )
    lower_support_right_rail = _add_vertical_rail(
        model,
        "lower_support_right_rail",
        length=support_height,
        radius=WING_RAIL_RADIUS,
        material=powder_gray,
    )
    for index, z in enumerate((-0.16, -0.30, -0.44)):
        _add_span_rail_x(
            model,
            f"lower_support_rung_{index + 1}",
            length=support_rung_span,
            radius=RUNG_RADIUS,
            material=powder_gray,
        )

    model.articulation(
        "lower_support_hinge",
        ArticulationType.REVOLUTE,
        parent=central_support_bracket,
        child=lower_support_top_bracket,
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-0.72, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.0, lower=0.0, upper=2.25),
    )
    model.articulation(
        "lower_support_top_to_left_rail",
        ArticulationType.FIXED,
        parent=lower_support_top_bracket,
        child=lower_support_left_rail,
        origin=Origin(xyz=(-support_width * 0.5, 0.0, -0.010), rpy=(pi, 0.0, 0.0)),
    )
    model.articulation(
        "lower_support_top_to_right_rail",
        ArticulationType.FIXED,
        parent=lower_support_top_bracket,
        child=lower_support_right_rail,
        origin=Origin(xyz=(support_width * 0.5, 0.0, -0.010), rpy=(pi, 0.0, 0.0)),
    )
    for index, z in enumerate((-0.16, -0.30, -0.44)):
        model.articulation(
            f"lower_support_top_to_rung_{index + 1}",
            ArticulationType.FIXED,
            parent=lower_support_top_bracket,
            child=f"lower_support_rung_{index + 1}",
            origin=Origin(xyz=(0.0, -(WING_RAIL_RADIUS + RUNG_RADIUS), z)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    left_wing_hinge = object_model.get_articulation("left_wing_hinge")
    right_wing_hinge = object_model.get_articulation("right_wing_hinge")
    lower_support_hinge = object_model.get_articulation("lower_support_hinge")

    central_left_rail = object_model.get_part("central_left_rail")
    central_right_rail = object_model.get_part("central_right_rail")
    left_wing_inner_rail = object_model.get_part("left_wing_inner_rail")
    right_wing_inner_rail = object_model.get_part("right_wing_inner_rail")
    left_wing_outer_rail = object_model.get_part("left_wing_outer_rail")
    right_wing_outer_rail = object_model.get_part("right_wing_outer_rail")
    lower_support_left_rail = object_model.get_part("lower_support_left_rail")

    ctx.expect_contact(
        left_wing_inner_rail,
        central_left_rail,
        name="left wing hinge rail stays mounted to the center frame",
    )
    ctx.expect_contact(
        right_wing_inner_rail,
        central_right_rail,
        name="right wing hinge rail stays mounted to the center frame",
    )

    left_open = ctx.part_world_position(left_wing_outer_rail)
    right_open = ctx.part_world_position(right_wing_outer_rail)
    support_open = _aabb_center(ctx.part_world_aabb(lower_support_left_rail))

    with ctx.pose({left_wing_hinge: 0.95, right_wing_hinge: 0.95, lower_support_hinge: 2.25}):
        left_folded = ctx.part_world_position(left_wing_outer_rail)
        right_folded = ctx.part_world_position(right_wing_outer_rail)
        support_folded = _aabb_center(ctx.part_world_aabb(lower_support_left_rail))

    ctx.check(
        "left wing folds inward toward the main span",
        left_open is not None
        and left_folded is not None
        and left_folded[0] < left_open[0] - 0.08
        and abs(left_folded[1]) < abs(left_open[1]) - 0.08,
        details=f"open={left_open}, folded={left_folded}",
    )
    ctx.check(
        "right wing folds inward toward the main span",
        right_open is not None
        and right_folded is not None
        and right_folded[0] > right_open[0] + 0.08
        and abs(right_folded[1]) < abs(right_open[1]) - 0.08,
        details=f"open={right_open}, folded={right_folded}",
    )
    ctx.check(
        "lower support frame folds upward behind the rack",
        support_open is not None
        and support_folded is not None
        and support_folded[1] > support_open[1] + 0.20
        and support_folded[2] > support_open[2] + 0.16,
        details=f"open={support_open}, folded={support_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
