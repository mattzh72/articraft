from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LADDER_WIDTH = 0.52
RAIL_SPACING_X = 0.235
FRONT_RAIL_SIZE = (0.05, 0.03, 1.18)
REAR_RAIL_SIZE = (0.045, 0.025, 1.12)
FRONT_RAIL_LEAN = 0.18
REAR_FRAME_OPEN_ANGLE = -0.34


def _rail_part(
    model: ArticulatedObject,
    name: str,
    *,
    rail_size: tuple[float, float, float],
    body_material,
    foot_material,
) -> None:
    rail_w, rail_d, rail_len = rail_size
    part = model.part(name)
    part.visual(
        Box((rail_w, rail_d, rail_len)),
        origin=Origin(xyz=(0.0, 0.0, -rail_len * 0.5)),
        material=body_material,
        name="rail_body",
    )
    part.visual(
        Box((rail_w * 0.78, rail_d * 0.52, rail_len * 0.84)),
        origin=Origin(xyz=(0.0, rail_d * 0.15, -rail_len * 0.48)),
        material=body_material,
        name="rail_inner_web",
    )
    part.visual(
        Box((rail_w * 1.10, rail_d * 1.55, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -rail_len - 0.011)),
        material=foot_material,
        name="foot_pad",
    )


def _tread_part(
    model: ArticulatedObject,
    name: str,
    *,
    width: float,
    depth: float,
    step_material,
    tread_material,
) -> None:
    part = model.part(name)
    part.visual(
        Box((width, depth, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=step_material,
        name="step_body",
    )
    part.visual(
        Box((width * 0.92, depth * 0.78, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=tread_material,
        name="tread_pad",
    )
    part.visual(
        Box((width * 0.34, depth * 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=step_material,
        name="center_rib",
    )


def _brace_part(
    model: ArticulatedObject,
    name: str,
    *,
    width: float,
    brace_material,
) -> None:
    part = model.part(name)
    part.visual(
        Box((width, 0.025, 0.030)),
        material=brace_material,
        name="brace_body",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    tread_black = model.material("tread_black", rgba=(0.10, 0.10, 0.11, 1.0))
    top_cap_orange = model.material("top_cap_orange", rgba=(0.93, 0.45, 0.10, 1.0))
    foot_gray = model.material("foot_gray", rgba=(0.26, 0.27, 0.29, 1.0))

    front_top_bracket = model.part("front_top_bracket")
    front_top_bracket.visual(
        Box((0.32, 0.14, 0.048)),
        origin=Origin(xyz=(0.0, 0.070, -0.024)),
        material=top_cap_orange,
        name="top_cap",
    )
    front_top_bracket.visual(
        Box((0.30, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.018, -0.048)),
        material=top_cap_orange,
        name="hinge_spine",
    )
    front_top_bracket.visual(
        Box((0.120, 0.060, 0.040)),
        origin=Origin(xyz=(0.175, 0.080, -0.060)),
        material=top_cap_orange,
        name="left_shoulder",
    )
    front_top_bracket.visual(
        Box((0.120, 0.060, 0.040)),
        origin=Origin(xyz=(-0.175, 0.080, -0.060)),
        material=top_cap_orange,
        name="right_shoulder",
    )
    front_top_bracket.visual(
        Box((0.18, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.115, -0.008)),
        material=tread_black,
        name="handle_slot_rib",
    )

    rear_top_bracket = model.part("rear_top_bracket")
    rear_top_bracket.visual(
        Box((0.44, 0.070, 0.042)),
        origin=Origin(xyz=(0.0, -0.035, -0.021)),
        material=aluminum,
        name="rear_hinge_bar",
    )
    rear_top_bracket.visual(
        Box((0.040, 0.060, 0.050)),
        origin=Origin(xyz=(0.1925, -0.075, -0.050)),
        material=aluminum,
        name="left_hinge_leaf",
    )
    rear_top_bracket.visual(
        Box((0.040, 0.060, 0.050)),
        origin=Origin(xyz=(-0.1925, -0.075, -0.050)),
        material=aluminum,
        name="right_hinge_leaf",
    )
    rear_top_bracket.visual(
        Box((0.32, 0.045, 0.025)),
        origin=Origin(xyz=(0.0, -0.087, -0.050)),
        material=aluminum,
        name="rear_cap_core",
    )

    _rail_part(
        model,
        "front_left_rail",
        rail_size=FRONT_RAIL_SIZE,
        body_material=aluminum,
        foot_material=foot_gray,
    )
    _rail_part(
        model,
        "front_right_rail",
        rail_size=FRONT_RAIL_SIZE,
        body_material=aluminum,
        foot_material=foot_gray,
    )
    _rail_part(
        model,
        "rear_left_rail",
        rail_size=REAR_RAIL_SIZE,
        body_material=aluminum,
        foot_material=foot_gray,
    )
    _rail_part(
        model,
        "rear_right_rail",
        rail_size=REAR_RAIL_SIZE,
        body_material=aluminum,
        foot_material=foot_gray,
    )

    for tread_name, depth in (
        ("front_tread_1", 0.160),
        ("front_tread_2", 0.135),
        ("front_tread_3", 0.115),
        ("front_tread_4", 0.100),
    ):
        _tread_part(
            model,
            tread_name,
            width=0.420,
            depth=depth,
            step_material=aluminum,
            tread_material=tread_black,
        )

    _brace_part(
        model,
        "rear_upper_brace",
        width=0.425,
        brace_material=aluminum,
    )
    _brace_part(
        model,
        "rear_lower_brace",
        width=0.425,
        brace_material=aluminum,
    )

    model.articulation(
        "front_left_rail_mount",
        ArticulationType.FIXED,
        parent=front_top_bracket,
        child="front_left_rail",
        origin=Origin(xyz=(RAIL_SPACING_X, 0.050, -0.080), rpy=(FRONT_RAIL_LEAN, 0.0, 0.0)),
    )
    model.articulation(
        "front_right_rail_mount",
        ArticulationType.FIXED,
        parent=front_top_bracket,
        child="front_right_rail",
        origin=Origin(xyz=(-RAIL_SPACING_X, 0.050, -0.080), rpy=(FRONT_RAIL_LEAN, 0.0, 0.0)),
    )
    model.articulation(
        "rear_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=front_top_bracket,
        child=rear_top_bracket,
        origin=Origin(rpy=(REAR_FRAME_OPEN_ANGLE, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.8, lower=-0.08, upper=0.55),
    )
    model.articulation(
        "rear_left_rail_mount",
        ArticulationType.FIXED,
        parent=rear_top_bracket,
        child="rear_left_rail",
        origin=Origin(xyz=(RAIL_SPACING_X, -0.046, -0.050)),
    )
    model.articulation(
        "rear_right_rail_mount",
        ArticulationType.FIXED,
        parent=rear_top_bracket,
        child="rear_right_rail",
        origin=Origin(xyz=(-RAIL_SPACING_X, -0.046, -0.050)),
    )

    tread_mounts = (
        ("front_tread_1", 0.085, -0.250),
        ("front_tread_2", 0.122, -0.470),
        ("front_tread_3", 0.164, -0.690),
        ("front_tread_4", 0.205, -0.900),
    )
    for tread_name, y_pos, z_pos in tread_mounts:
        model.articulation(
            f"{tread_name}_mount",
            ArticulationType.FIXED,
            parent=front_top_bracket,
            child=tread_name,
            origin=Origin(xyz=(0.0, y_pos, z_pos)),
        )

    model.articulation(
        "rear_upper_brace_mount",
        ArticulationType.FIXED,
        parent=rear_top_bracket,
        child="rear_upper_brace",
        origin=Origin(xyz=(0.0, -0.034, -0.520)),
    )
    model.articulation(
        "rear_lower_brace_mount",
        ArticulationType.FIXED,
        parent=rear_top_bracket,
        child="rear_lower_brace",
        origin=Origin(xyz=(0.0, -0.034, -0.840)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_left_rail = object_model.get_part("front_left_rail")
    front_right_rail = object_model.get_part("front_right_rail")
    rear_left_rail = object_model.get_part("rear_left_rail")
    rear_right_rail = object_model.get_part("rear_right_rail")
    front_tread_2 = object_model.get_part("front_tread_2")
    rear_upper_brace = object_model.get_part("rear_upper_brace")
    rear_lower_brace = object_model.get_part("rear_lower_brace")
    rear_frame_hinge = object_model.get_articulation("rear_frame_hinge")

    ctx.expect_gap(
        front_left_rail,
        front_tread_2,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="rail_body",
        negative_elem="step_body",
        name="second tread reaches the left climbing rail",
    )
    ctx.expect_gap(
        front_tread_2,
        front_right_rail,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="step_body",
        negative_elem="rail_body",
        name="second tread reaches the right climbing rail",
    )
    ctx.expect_gap(
        rear_left_rail,
        rear_upper_brace,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="rail_body",
        negative_elem="brace_body",
        name="rear upper brace reaches the left support rail",
    )
    ctx.expect_gap(
        rear_upper_brace,
        rear_right_rail,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="brace_body",
        negative_elem="rail_body",
        name="rear upper brace reaches the right support rail",
    )

    open_pos = ctx.part_world_position(rear_lower_brace)
    with ctx.pose({rear_frame_hinge: 0.45}):
        folded_pos = ctx.part_world_position(rear_lower_brace)
    ctx.check(
        "rear frame folds forward toward the front frame",
        open_pos is not None and folded_pos is not None and folded_pos[1] > open_pos[1] + 0.12,
        details=f"open={open_pos}, folded={folded_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
