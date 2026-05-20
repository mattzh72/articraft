from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


ROOT_TO_OUTER = Origin(xyz=(0.32, 0.0, 0.695))

OUTER_LENGTH = 1.55
OUTER_WIDTH = 0.24
OUTER_HEIGHT = 0.20
OUTER_WALL = 0.012
OUTER_REAR_CAP = 0.10

MID_LENGTH = 1.36
MID_WIDTH = 0.188
MID_HEIGHT = 0.148
MID_WALL = 0.010

INNER_LENGTH = 1.18
INNER_WIDTH = 0.148
INNER_HEIGHT = 0.108
INNER_WALL = 0.010

TIP_LENGTH = 1.00
TIP_WIDTH = 0.108
TIP_HEIGHT = 0.078
TIP_WALL = 0.008
TIP_FRONT_CAP = 0.03

OUTER_TO_MID_OFFSET = 0.10
MID_TO_INNER_OFFSET = 0.09
INNER_TO_TIP_OFFSET = 0.08

OUTER_TO_MID_TRAVEL = 1.00
MID_TO_INNER_TRAVEL = 0.85
INNER_TO_TIP_TRAVEL = 0.68

FULLY_EXTENDED_POSE = {
    "outer_to_mid": OUTER_TO_MID_TRAVEL,
    "mid_to_inner": MID_TO_INNER_TRAVEL,
    "inner_to_tip": INNER_TO_TIP_TRAVEL,
}


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_tube_visuals(
    part,
    *,
    prefix: str,
    length: float,
    width: float,
    height: float,
    wall: float,
    material: str,
    rear_cap: float = 0.0,
    front_cap: float = 0.0,
) -> None:
    side_height = height - 2.0 * wall
    _add_box_visual(
        part,
        size=(length, width, wall),
        center=(length / 2.0, 0.0, -height / 2.0 + wall / 2.0),
        material=material,
        name=f"{prefix}_bottom_wall",
    )
    _add_box_visual(
        part,
        size=(length, width, wall),
        center=(length / 2.0, 0.0, height / 2.0 - wall / 2.0),
        material=material,
        name=f"{prefix}_top_wall",
    )
    _add_box_visual(
        part,
        size=(length, wall, side_height),
        center=(length / 2.0, -width / 2.0 + wall / 2.0, 0.0),
        material=material,
        name=f"{prefix}_left_wall",
    )
    _add_box_visual(
        part,
        size=(length, wall, side_height),
        center=(length / 2.0, width / 2.0 - wall / 2.0, 0.0),
        material=material,
        name=f"{prefix}_right_wall",
    )
    if rear_cap > 0.0:
        _add_box_visual(
            part,
            size=(rear_cap, width, height),
            center=(rear_cap / 2.0, 0.0, 0.0),
            material=material,
            name=f"{prefix}_rear_cap",
        )
    if front_cap > 0.0:
        _add_box_visual(
            part,
            size=(front_cap, width, height),
            center=(length - front_cap / 2.0, 0.0, 0.0),
            material=material,
            name=f"{prefix}_front_cap",
        )


def _add_guide_pads(
    part,
    *,
    prefix: str,
    x_center: float,
    pad_length: float,
    width: float,
    height: float,
    side_thickness: float,
    vertical_thickness: float,
) -> None:
    _add_box_visual(
        part,
        size=(pad_length, width * 0.44, vertical_thickness),
        center=(x_center, 0.0, height / 2.0 + vertical_thickness / 2.0),
        material="guide_pad",
        name=f"{prefix}_top_pad",
    )
    _add_box_visual(
        part,
        size=(pad_length, width * 0.44, vertical_thickness),
        center=(x_center, 0.0, -height / 2.0 - vertical_thickness / 2.0),
        material="guide_pad",
        name=f"{prefix}_bottom_pad",
    )
    _add_box_visual(
        part,
        size=(pad_length, side_thickness, height * 0.44),
        center=(x_center, width / 2.0 + side_thickness / 2.0, 0.0),
        material="guide_pad",
        name=f"{prefix}_right_pad",
    )
    _add_box_visual(
        part,
        size=(pad_length, side_thickness, height * 0.44),
        center=(x_center, -width / 2.0 - side_thickness / 2.0, 0.0),
        material="guide_pad",
        name=f"{prefix}_left_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_telescoping_boom")

    model.material("body_grey", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("boom_yellow", rgba=(0.88, 0.72, 0.14, 1.0))
    model.material("boom_yellow_dark", rgba=(0.80, 0.64, 0.12, 1.0))
    model.material("boom_yellow_light", rgba=(0.93, 0.80, 0.26, 1.0))
    model.material("guide_pad", rgba=(0.16, 0.16, 0.17, 1.0))

    root_body = model.part("root_body")
    _add_box_visual(
        root_body,
        size=(1.40, 0.96, 0.20),
        center=(-0.02, 0.0, 0.24),
        material="body_grey",
        name="lower_body",
    )
    _add_box_visual(
        root_body,
        size=(0.98, 0.78, 0.22),
        center=(-0.18, 0.0, 0.45),
        material="body_grey",
        name="upper_body",
    )
    _add_box_visual(
        root_body,
        size=(0.44, 0.62, 0.07),
        center=(0.22, 0.0, 0.375),
        material="body_grey",
        name="front_deck",
    )
    _add_box_visual(
        root_body,
        size=(0.26, 0.32, 0.21),
        center=(0.34, 0.0, 0.455),
        material="body_grey",
        name="boom_tower",
    )
    _add_box_visual(
        root_body,
        size=(0.28, 0.22, 0.07),
        center=(0.50, 0.0, 0.525),
        material="body_grey",
        name="boom_saddle",
    )
    _add_box_visual(
        root_body,
        size=(0.22, 0.05, 0.16),
        center=(0.33, 0.135, 0.435),
        material="body_grey",
        name="right_brace",
    )
    _add_box_visual(
        root_body,
        size=(0.22, 0.05, 0.16),
        center=(0.33, -0.135, 0.435),
        material="body_grey",
        name="left_brace",
    )
    for x_pos in (-0.54, 0.50):
        for y_pos in (-0.39, 0.39):
            pad_name = f"foot_pad_{'front' if x_pos > 0 else 'rear'}_{'right' if y_pos > 0 else 'left'}"
            leg_name = f"foot_leg_{'front' if x_pos > 0 else 'rear'}_{'right' if y_pos > 0 else 'left'}"
            _add_box_visual(
                root_body,
                size=(0.18, 0.12, 0.04),
                center=(x_pos, y_pos, 0.02),
                material="body_grey",
                name=pad_name,
            )
            _add_box_visual(
                root_body,
                size=(0.08, 0.08, 0.10),
                center=(x_pos, y_pos, 0.09),
                material="body_grey",
                name=leg_name,
            )
    root_body.inertial = Inertial.from_geometry(
        Box((1.40, 0.96, 0.56)),
        mass=820.0,
        origin=Origin(xyz=(-0.05, 0.0, 0.28)),
    )

    outer_boom = model.part("outer_boom")
    _add_tube_visuals(
        outer_boom,
        prefix="outer",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        material="boom_yellow",
        rear_cap=OUTER_REAR_CAP,
    )
    _add_box_visual(
        outer_boom,
        size=(0.24, 0.18, 0.035),
        center=(0.18, 0.0, -0.1175),
        material="body_grey",
        name="mount_foot",
    )
    _add_box_visual(
        outer_boom,
        size=(0.08, 0.18, 0.028),
        center=(0.04, 0.0, -0.086),
        material="guide_pad",
        name="rear_wear_block",
    )
    _add_guide_pads(
        outer_boom,
        prefix="outer",
        x_center=0.17,
        pad_length=0.10,
        width=MID_WIDTH,
        height=MID_HEIGHT,
        side_thickness=0.014,
        vertical_thickness=0.014,
    )
    outer_boom.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT + 0.035)),
        mass=135.0,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, -0.01)),
    )

    mid_boom = model.part("mid_boom")
    _add_tube_visuals(
        mid_boom,
        prefix="mid",
        length=MID_LENGTH,
        width=MID_WIDTH,
        height=MID_HEIGHT,
        wall=MID_WALL,
        material="boom_yellow_light",
    )
    _add_guide_pads(
        mid_boom,
        prefix="mid",
        x_center=0.14,
        pad_length=0.10,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        side_thickness=0.010,
        vertical_thickness=0.010,
    )
    mid_boom.inertial = Inertial.from_geometry(
        Box((MID_LENGTH, MID_WIDTH, MID_HEIGHT)),
        mass=92.0,
        origin=Origin(xyz=(MID_LENGTH / 2.0, 0.0, 0.0)),
    )

    inner_boom = model.part("inner_boom")
    _add_tube_visuals(
        inner_boom,
        prefix="inner",
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        material="boom_yellow_dark",
    )
    _add_guide_pads(
        inner_boom,
        prefix="inner",
        x_center=0.12,
        pad_length=0.10,
        width=TIP_WIDTH,
        height=TIP_HEIGHT,
        side_thickness=0.010,
        vertical_thickness=0.005,
    )
    inner_boom.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        mass=70.0,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, 0.0)),
    )

    tip_boom = model.part("tip_boom")
    _add_tube_visuals(
        tip_boom,
        prefix="tip",
        length=TIP_LENGTH,
        width=TIP_WIDTH,
        height=TIP_HEIGHT,
        wall=TIP_WALL,
        material="boom_yellow",
        front_cap=TIP_FRONT_CAP,
    )
    _add_box_visual(
        tip_boom,
        size=(0.08, TIP_WIDTH, TIP_HEIGHT * 0.72),
        center=(TIP_LENGTH - 0.04, 0.0, 0.0),
        material="boom_yellow",
        name="tip_nose",
    )
    tip_boom.inertial = Inertial.from_geometry(
        Box((TIP_LENGTH, TIP_WIDTH, TIP_HEIGHT)),
        mass=49.0,
        origin=Origin(xyz=(TIP_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "root_to_outer",
        ArticulationType.FIXED,
        parent=root_body,
        child=outer_boom,
        origin=ROOT_TO_OUTER,
    )
    model.articulation(
        "outer_to_mid",
        ArticulationType.PRISMATIC,
        parent=outer_boom,
        child=mid_boom,
        origin=Origin(xyz=(OUTER_TO_MID_OFFSET, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TO_MID_TRAVEL,
            effort=12000.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "mid_to_inner",
        ArticulationType.PRISMATIC,
        parent=mid_boom,
        child=inner_boom,
        origin=Origin(xyz=(MID_TO_INNER_OFFSET, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MID_TO_INNER_TRAVEL,
            effort=9000.0,
            velocity=0.38,
        ),
    )
    model.articulation(
        "inner_to_tip",
        ArticulationType.PRISMATIC,
        parent=inner_boom,
        child=tip_boom,
        origin=Origin(xyz=(INNER_TO_TIP_OFFSET, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_TO_TIP_TRAVEL,
            effort=7000.0,
            velocity=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_body = object_model.get_part("root_body")
    outer_boom = object_model.get_part("outer_boom")
    mid_boom = object_model.get_part("mid_boom")
    inner_boom = object_model.get_part("inner_boom")
    tip_boom = object_model.get_part("tip_boom")
    outer_to_mid = object_model.get_articulation("outer_to_mid")
    mid_to_inner = object_model.get_articulation("mid_to_inner")
    inner_to_tip = object_model.get_articulation("inner_to_tip")

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

    ctx.expect_contact(
        outer_boom,
        root_body,
        name="outer boom mount sits on the grounded root body",
    )

    ctx.expect_within(mid_boom, outer_boom, axes="yz", margin=0.0, name="mid boom stays centered in outer boom")
    ctx.expect_within(inner_boom, mid_boom, axes="yz", margin=0.0, name="inner boom stays centered in mid boom")
    ctx.expect_within(tip_boom, inner_boom, axes="yz", margin=0.0, name="tip boom stays centered in inner boom")

    with ctx.pose(FULLY_EXTENDED_POSE):
        ctx.expect_within(
            mid_boom,
            outer_boom,
            axes="yz",
            margin=0.0,
            name="extended mid boom remains guided by outer boom",
        )
        ctx.expect_within(
            inner_boom,
            mid_boom,
            axes="yz",
            margin=0.0,
            name="extended inner boom remains guided by mid boom",
        )
        ctx.expect_within(
            tip_boom,
            inner_boom,
            axes="yz",
            margin=0.0,
            name="extended tip boom remains guided by inner boom",
        )
        ctx.expect_overlap(
            mid_boom,
            outer_boom,
            axes="x",
            min_overlap=0.40,
            name="mid boom retains clear insertion inside outer boom",
        )
        ctx.expect_overlap(
            inner_boom,
            mid_boom,
            axes="x",
            min_overlap=0.40,
            name="inner boom retains clear insertion inside mid boom",
        )
        ctx.expect_overlap(
            tip_boom,
            inner_boom,
            axes="x",
            min_overlap=0.40,
            name="tip boom retains clear insertion inside inner boom",
        )
        mid_extended = ctx.part_world_position(mid_boom)
        inner_extended = ctx.part_world_position(inner_boom)
        tip_extended = ctx.part_world_position(tip_boom)

    mid_rest = ctx.part_world_position(mid_boom)
    inner_rest = ctx.part_world_position(inner_boom)
    tip_rest = ctx.part_world_position(tip_boom)
    ctx.check(
        "all telescoping stages extend outward along +X",
        (
            mid_rest is not None
            and inner_rest is not None
            and tip_rest is not None
            and mid_extended is not None
            and inner_extended is not None
            and tip_extended is not None
            and mid_extended[0] > mid_rest[0] + 0.80
            and inner_extended[0] > inner_rest[0] + 1.40
            and tip_extended[0] > tip_rest[0] + 2.00
        ),
        details=(
            f"mid_rest={mid_rest}, mid_extended={mid_extended}, "
            f"inner_rest={inner_rest}, inner_extended={inner_extended}, "
            f"tip_rest={tip_rest}, tip_extended={tip_extended}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
