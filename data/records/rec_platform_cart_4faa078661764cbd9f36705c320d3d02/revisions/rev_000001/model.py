from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


DECK_LENGTH = 1.05
DECK_WIDTH = 0.68
DECK_THICKNESS = 0.028
DECK_BOTTOM_Z = 0.142
DECK_TOP_Z = DECK_BOTTOM_Z + DECK_THICKNESS

PANEL_DEPTH = 0.22
PANEL_WIDTH = 0.18
PANEL_THICKNESS = 0.014

WHEEL_RADIUS = 0.062
WHEEL_WIDTH = 0.032
CASTER_TRAIL = 0.082
CASTER_AXLE_DROP = 0.080

CASTER_LAYOUT = {
    "front_left": (-0.410, 0.250),
    "front_right": (0.410, 0.250),
    "rear_left": (-0.410, -0.250),
    "rear_right": (0.410, -0.250),
}


def _add_caster(
    model: ArticulatedObject,
    deck,
    *,
    name: str,
    position: tuple[float, float],
    yaw: float = 0.0,
    steel_dark,
    steel_mid,
    tire_black,
) -> None:
    x_pos, y_pos = position

    caster = model.part(f"{name}_caster")
    caster.visual(
        Box((0.080, 0.055, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, -0.011)),
        material=steel_mid,
        name="caster_top_plate",
    )
    caster.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, -0.020)),
        material=steel_mid,
        name="swivel_race",
    )
    caster.visual(
        Cylinder(radius=0.013, length=0.038),
        origin=Origin(xyz=(0.000, 0.000, -0.037)),
        material=steel_dark,
        name="kingpin",
    )
    caster.visual(
        Box((0.020, 0.050, 0.016)),
        origin=Origin(xyz=(0.006, 0.000, -0.052)),
        material=steel_dark,
        name="fork_saddle",
    )
    for side_name, y_side in (("left", 0.022), ("right", -0.022)):
        caster.visual(
            Box((0.070, 0.008, 0.016)),
            origin=Origin(xyz=(0.049, y_side, -0.052)),
            material=steel_dark,
            name=f"{side_name}_upper_arm",
        )
        caster.visual(
            Box((0.018, 0.008, 0.052)),
            origin=Origin(xyz=(0.082, y_side, -0.078)),
            material=steel_dark,
            name=f"{side_name}_lower_arm",
        )
        caster.visual(
            Cylinder(radius=0.007, length=0.008),
            origin=Origin(xyz=(0.082, 0.024 if y_side > 0.0 else -0.024, -0.080), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel_mid,
            name=f"{side_name}_axle_boss",
        )
    caster.inertial = Inertial.from_geometry(
        Box((0.080, 0.055, 0.115)),
        mass=0.75,
        origin=Origin(xyz=(0.040, 0.000, -0.058)),
    )

    wheel = model.part(f"{name}_wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="wheel_tread",
    )
    wheel.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="wheel_hub",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.000, 0.014, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="outer_cap",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.000, -0.014, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="inner_cap",
    )
    wheel.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.000, 0.018, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="outer_spindle",
    )
    wheel.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.000, -0.018, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="inner_spindle",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.35,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        f"deck_to_{name}_caster",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=caster,
        origin=Origin(xyz=(x_pos, y_pos, DECK_BOTTOM_Z), rpy=(0.0, 0.0, yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=10.0),
    )
    model.articulation(
        f"{name}_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=caster,
        child=wheel,
        origin=Origin(xyz=(CASTER_TRAIL, 0.000, -CASTER_AXLE_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=24.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sheet_goods_platform_cart")

    frame_blue = model.material("frame_blue", rgba=(0.22, 0.38, 0.72, 1.0))
    deck_wood = model.material("deck_wood", rgba=(0.69, 0.59, 0.42, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.56, 0.58, 0.61, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.08, 1.0))
    panel_orange = model.material("panel_orange", rgba=(0.89, 0.45, 0.12, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.000, 0.000, DECK_BOTTOM_Z + DECK_THICKNESS / 2.0)),
        material=deck_wood,
        name="deck_board",
    )
    deck.visual(
        Box((DECK_LENGTH - 0.040, 0.050, 0.018)),
        origin=Origin(xyz=(0.000, DECK_WIDTH / 2.0 - 0.025, DECK_BOTTOM_Z - 0.009)),
        material=frame_blue,
        name="right_side_rail",
    )
    deck.visual(
        Box((DECK_LENGTH - 0.040, 0.050, 0.018)),
        origin=Origin(xyz=(0.000, -DECK_WIDTH / 2.0 + 0.025, DECK_BOTTOM_Z - 0.009)),
        material=frame_blue,
        name="left_side_rail",
    )
    deck.visual(
        Box((0.060, DECK_WIDTH - 0.100, 0.018)),
        origin=Origin(xyz=(DECK_LENGTH / 2.0 - 0.030, 0.000, DECK_BOTTOM_Z - 0.009)),
        material=frame_blue,
        name="front_end_rail",
    )
    deck.visual(
        Box((0.060, DECK_WIDTH - 0.100, 0.018)),
        origin=Origin(xyz=(-DECK_LENGTH / 2.0 + 0.030, 0.000, DECK_BOTTOM_Z - 0.009)),
        material=frame_blue,
        name="rear_end_rail",
    )
    deck.visual(
        Box((0.060, DECK_WIDTH - 0.200, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, DECK_BOTTOM_Z - 0.009)),
        material=frame_blue,
        name="center_crossmember",
    )

    for caster_name, (x_pos, y_pos) in CASTER_LAYOUT.items():
        deck.visual(
            Box((0.090, 0.065, 0.008)),
            origin=Origin(xyz=(x_pos, y_pos, DECK_BOTTOM_Z - 0.004)),
            material=frame_blue,
            name=f"{caster_name}_mount_pad",
        )

    hinge_y = CASTER_LAYOUT["front_left"][1] - 0.015
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.060)),
        mass=24.0,
        origin=Origin(xyz=(0.000, 0.000, DECK_BOTTOM_Z + 0.030)),
    )

    stop_panel = model.part("stop_panel")
    stop_panel.visual(
        Box((PANEL_DEPTH, PANEL_WIDTH, PANEL_THICKNESS)),
        origin=Origin(xyz=(PANEL_DEPTH / 2.0, 0.000, PANEL_THICKNESS / 2.0)),
        material=panel_orange,
        name="stop_panel_plate",
    )
    stop_panel.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.000, 0.000, 0.008), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="stop_panel_barrel",
    )
    stop_panel.inertial = Inertial.from_geometry(
        Box((PANEL_DEPTH, PANEL_WIDTH, PANEL_THICKNESS)),
        mass=2.8,
        origin=Origin(xyz=(PANEL_DEPTH / 2.0, 0.000, PANEL_THICKNESS / 2.0)),
    )

    model.articulation(
        "deck_to_stop_panel",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stop_panel,
        origin=Origin(xyz=(-DECK_LENGTH / 2.0, hinge_y, DECK_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.72,
        ),
    )

    for caster_name, caster_position in CASTER_LAYOUT.items():
        _add_caster(
            model,
            deck,
            name=caster_name,
            position=caster_position,
            yaw=0.0 if "left" in caster_name else pi,
            steel_dark=steel_dark,
            steel_mid=steel_mid,
            tire_black=tire_black,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    stop_panel = object_model.get_part("stop_panel")
    stop_hinge = object_model.get_articulation("deck_to_stop_panel")

    with ctx.pose({stop_hinge: 0.0}):
        ctx.expect_gap(
            stop_panel,
            deck,
            axis="z",
            positive_elem="stop_panel_plate",
            negative_elem="deck_board",
            max_gap=0.001,
            max_penetration=1e-6,
            name="folded stop panel rests on the deck",
        )
        ctx.expect_within(
            stop_panel,
            deck,
            axes="xy",
            inner_elem="stop_panel_plate",
            outer_elem="deck_board",
            margin=0.0,
            name="folded stop panel stays on the deck corner",
        )

    closed_panel_aabb = ctx.part_element_world_aabb(stop_panel, elem="stop_panel_plate")
    with ctx.pose({stop_hinge: 1.45}):
        open_panel_aabb = ctx.part_element_world_aabb(stop_panel, elem="stop_panel_plate")
    ctx.check(
        "stop panel opens upward",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.12,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    wheel_bottoms: list[float] = []
    deck_board_aabb = ctx.part_element_world_aabb(deck, elem="deck_board")
    wheels_clear_deck = True
    swivel_axes_ok = True
    wheel_axes_ok = True

    for caster_name in CASTER_LAYOUT:
        caster = object_model.get_part(f"{caster_name}_caster")
        wheel = object_model.get_part(f"{caster_name}_wheel")
        swivel = object_model.get_articulation(f"deck_to_{caster_name}_caster")
        spin = object_model.get_articulation(f"{caster_name}_caster_to_wheel")

        ctx.expect_contact(
            caster,
            deck,
            elem_a="caster_top_plate",
            elem_b=f"{caster_name}_mount_pad",
            contact_tol=0.001,
            name=f"{caster_name} caster mounts to the deck",
        )

        wheel_aabb = ctx.part_world_aabb(wheel)
        if wheel_aabb is not None:
            wheel_bottoms.append(wheel_aabb[0][2])
            if deck_board_aabb is not None and wheel_aabb[1][2] > deck_board_aabb[0][2] - 0.010:
                wheels_clear_deck = False

        swivel_axes_ok = swivel_axes_ok and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        wheel_axes_ok = wheel_axes_ok and tuple(spin.axis) == (0.0, 1.0, 0.0)

    ctx.check(
        "all wheels sit on the same floor plane",
        len(wheel_bottoms) == 4 and max(wheel_bottoms) - min(wheel_bottoms) < 0.003,
        details=f"wheel_bottoms={wheel_bottoms}",
    )
    ctx.check(
        "wheels remain below the deck",
        wheels_clear_deck,
        details=f"deck_board_aabb={deck_board_aabb}",
    )
    ctx.check(
        "caster swivel joints stay vertical",
        swivel_axes_ok,
        details="expected every caster swivel axis to be +Z",
    )
    ctx.check(
        "caster wheel spin joints stay transverse",
        wheel_axes_ok,
        details="expected every wheel spin axis to be +Y",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
