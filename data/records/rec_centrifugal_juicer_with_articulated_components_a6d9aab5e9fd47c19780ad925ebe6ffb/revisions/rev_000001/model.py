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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _circle_profile(
    radius: float,
    *,
    segments: int = 48,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        if clockwise:
            angle = -angle
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_steel_juicer")

    stainless = model.material("stainless", rgba=(0.75, 0.77, 0.79, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.15, 0.16, 0.17, 1.0))
    clear_lid = model.material("clear_lid", rgba=(0.84, 0.92, 0.97, 0.28))
    basket_steel = model.material("basket_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    plunger_grey = model.material("plunger_grey", rgba=(0.48, 0.50, 0.53, 1.0))
    pulse_accent = model.material("pulse_accent", rgba=(0.80, 0.32, 0.10, 1.0))

    base_shell = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.290, 0.220, 0.045, 0.000),
                _xy_section(0.308, 0.232, 0.050, 0.085),
                _xy_section(0.238, 0.182, 0.032, 0.182),
            ]
        ),
        "juicer_base_shell",
    )

    chamber_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.084, 0.000), (0.094, 0.010), (0.094, 0.082), (0.100, 0.088)],
            [(0.076, 0.004), (0.086, 0.012), (0.086, 0.080), (0.092, 0.084)],
            segments=64,
        ),
        "juicer_chamber_shell",
    )

    lid_plate = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.105),
            [_circle_profile(0.033, clockwise=True)],
            0.005,
            center=False,
        ),
        "juicer_lid_plate",
    )

    feed_tube = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.038, 0.000), (0.040, 0.010), (0.040, 0.136)],
            [(0.032, 0.002), (0.034, 0.012), (0.034, 0.134)],
            segments=48,
        ),
        "juicer_feed_tube",
    )

    basket_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.020, 0.000), (0.048, 0.010), (0.080, 0.064)],
            [(0.012, 0.004), (0.044, 0.014), (0.076, 0.060)],
            segments=56,
        ),
        "juicer_basket_shell",
    )

    base = model.part("base")
    base.visual(base_shell, material=stainless, name="housing")
    base.visual(
        Cylinder(radius=0.086, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.187)),
        material=dark_trim,
        name="top_collar",
    )
    base.visual(
        Box((0.008, 0.140, 0.092)),
        origin=Origin(xyz=(0.150, 0.000, 0.086)),
        material=dark_trim,
        name="control_band",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.310, 0.235, 0.200)),
        mass=5.8,
        origin=Origin(xyz=(0.000, 0.000, 0.100)),
    )

    chamber = model.part("chamber")
    chamber.visual(chamber_shell, material=clear_lid, name="wall")
    for index, y in enumerate((-0.032, 0.032)):
        chamber.visual(
            Box((0.016, 0.018, 0.018)),
            origin=Origin(xyz=(-0.104, y, 0.097)),
            material=dark_trim,
            name=f"hinge_post_{index}",
        )
        chamber.visual(
            Cylinder(radius=0.010, length=0.020),
            origin=Origin(xyz=(-0.108, y, 0.103), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name=f"hinge_lug_{index}",
        )
    chamber.inertial = Inertial.from_geometry(
        Box((0.230, 0.230, 0.115)),
        mass=0.8,
        origin=Origin(xyz=(0.000, 0.000, 0.057)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_plate,
        origin=Origin(xyz=(0.118, 0.000, -0.009)),
        material=clear_lid,
        name="lid_plate",
    )
    lid.visual(
        feed_tube,
        origin=Origin(xyz=(0.118, 0.000, -0.004)),
        material=clear_lid,
        name="feed_tube",
    )
    lid.visual(
        Box((0.024, 0.040, 0.020)),
        origin=Origin(xyz=(0.012, 0.000, 0.004)),
        material=dark_trim,
        name="hinge_web",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.046),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.215, 0.215, 0.150)),
        mass=0.45,
        origin=Origin(xyz=(0.118, 0.000, 0.050)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.030, length=0.122),
        origin=Origin(xyz=(0.000, 0.000, -0.023)),
        material=plunger_grey,
        name="shaft",
    )
    plunger.visual(
        Cylinder(radius=0.044, length=0.030),
        origin=Origin(xyz=(0.000, 0.000, 0.053)),
        material=plunger_grey,
        name="cap",
    )
    plunger.visual(
        Cylinder(radius=0.041, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=dark_trim,
        name="stop_flange",
    )
    plunger.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.000, 0.000, 0.076)),
        material=dark_trim,
        name="grip",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.088, 0.088, 0.170)),
        mass=0.28,
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
    )

    basket = model.part("basket")
    basket.visual(basket_shell, material=basket_steel, name="basket_shell")
    basket.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        material=dark_trim,
        name="hub",
    )
    basket.visual(
        Cylinder(radius=0.052, length=0.003),
        origin=Origin(xyz=(0.000, 0.000, 0.061)),
        material=basket_steel,
        name="grater_disc",
    )
    basket.visual(
        Cylinder(radius=0.081, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.064)),
        material=dark_trim,
        name="basket_rim",
    )
    for index in range(10):
        angle = 2.0 * math.pi * index / 10.0
        basket.visual(
            Box((0.018, 0.004, 0.046)),
            origin=Origin(
                xyz=(0.061 * math.cos(angle), 0.061 * math.sin(angle), 0.036),
                rpy=(0.0, 0.0, angle),
            ),
            material=basket_steel,
            name=f"tooth_{index}",
        )
    basket.inertial = Inertial.from_geometry(
        Box((0.170, 0.170, 0.072)),
        mass=0.35,
        origin=Origin(xyz=(0.000, 0.000, 0.036)),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.003, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="dial_stem",
    )
    selector_dial.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.014, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="dial_body",
    )
    selector_dial.visual(
        Box((0.006, 0.004, 0.012)),
        origin=Origin(xyz=(0.022, 0.000, 0.015)),
        material=stainless,
        name="dial_pointer",
    )
    selector_dial.inertial = Inertial.from_geometry(
        Box((0.028, 0.060, 0.060)),
        mass=0.08,
        origin=Origin(xyz=(0.014, 0.000, 0.000)),
    )

    pulse_button = model.part("pulse_button")
    pulse_button.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.004, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="button_stem",
    )
    pulse_button.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.012, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pulse_accent,
        name="button_cap",
    )
    pulse_button.inertial = Inertial.from_geometry(
        Box((0.020, 0.024, 0.024)),
        mass=0.02,
        origin=Origin(xyz=(0.010, 0.000, 0.000)),
    )

    model.articulation(
        "base_to_chamber",
        ArticulationType.FIXED,
        parent=base,
        child=chamber,
        origin=Origin(xyz=(0.000, 0.000, 0.192)),
    )
    model.articulation(
        "chamber_to_lid",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=lid,
        origin=Origin(xyz=(-0.108, 0.000, 0.103)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "lid_to_plunger",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=plunger,
        origin=Origin(xyz=(0.118, 0.000, 0.132)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.18,
            lower=0.0,
            upper=0.085,
        ),
    )
    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.000, 0.000, 0.191)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=32.0),
    )
    model.articulation(
        "base_to_selector_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=selector_dial,
        origin=Origin(xyz=(0.154, -0.034, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=4.0,
            lower=-2.1,
            upper=2.1,
        ),
    )
    model.articulation(
        "base_to_pulse_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pulse_button,
        origin=Origin(xyz=(0.154, 0.042, 0.086)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0015,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    plunger = object_model.get_part("plunger")
    basket = object_model.get_part("basket")
    selector_dial = object_model.get_part("selector_dial")
    pulse_button = object_model.get_part("pulse_button")

    lid_joint = object_model.get_articulation("chamber_to_lid")
    plunger_joint = object_model.get_articulation("lid_to_plunger")
    basket_joint = object_model.get_articulation("base_to_basket")
    dial_joint = object_model.get_articulation("base_to_selector_dial")
    pulse_joint = object_model.get_articulation("base_to_pulse_button")

    def _center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        lid,
        chamber,
        axis="z",
        positive_elem="lid_plate",
        negative_elem="wall",
        max_gap=0.008,
        max_penetration=0.0,
        name="lid plate sits just above the chamber wall",
    )
    ctx.expect_within(
        plunger,
        lid,
        axes="xy",
        inner_elem="shaft",
        outer_elem="feed_tube",
        margin=0.002,
        name="plunger shaft stays centered in the feed tube",
    )
    ctx.expect_within(
        basket,
        chamber,
        axes="xy",
        inner_elem="basket_rim",
        outer_elem="wall",
        margin=0.004,
        name="basket sits inside the clear chamber footprint",
    )

    rest_lid_center = _center(ctx.part_element_world_aabb(lid, elem="lid_plate"))
    open_lid_center = None
    with ctx.pose({lid_joint: math.radians(70.0)}):
        open_lid_center = _center(ctx.part_element_world_aabb(lid, elem="lid_plate"))
    ctx.check(
        "lid opens upward on the rear hinge",
        rest_lid_center is not None
        and open_lid_center is not None
        and open_lid_center[2] > rest_lid_center[2] + 0.07,
        details=f"rest={rest_lid_center}, open={open_lid_center}",
    )

    rest_plunger_pos = ctx.part_world_position(plunger)
    raised_plunger_pos = None
    with ctx.pose({plunger_joint: 0.085}):
        raised_plunger_pos = ctx.part_world_position(plunger)
    ctx.check(
        "plunger lifts vertically from the feed tube",
        rest_plunger_pos is not None
        and raised_plunger_pos is not None
        and raised_plunger_pos[2] > rest_plunger_pos[2] + 0.08
        and abs(raised_plunger_pos[0] - rest_plunger_pos[0]) < 1e-6
        and abs(raised_plunger_pos[1] - rest_plunger_pos[1]) < 1e-6,
        details=f"rest={rest_plunger_pos}, raised={raised_plunger_pos}",
    )

    rest_tooth_center = _center(ctx.part_element_world_aabb(basket, elem="tooth_0"))
    spun_tooth_center = None
    with ctx.pose({basket_joint: 0.9}):
        spun_tooth_center = _center(ctx.part_element_world_aabb(basket, elem="tooth_0"))
    ctx.check(
        "grater basket spins on the vertical drive axis",
        rest_tooth_center is not None
        and spun_tooth_center is not None
        and abs(spun_tooth_center[0] - rest_tooth_center[0]) > 0.015
        and abs(spun_tooth_center[2] - rest_tooth_center[2]) < 0.003,
        details=f"rest={rest_tooth_center}, spun={spun_tooth_center}",
    )

    rest_pointer_center = _center(ctx.part_element_world_aabb(selector_dial, elem="dial_pointer"))
    turned_pointer_center = None
    with ctx.pose({dial_joint: 1.1}):
        turned_pointer_center = _center(ctx.part_element_world_aabb(selector_dial, elem="dial_pointer"))
    ctx.check(
        "selector dial rotates about its front axis",
        rest_pointer_center is not None
        and turned_pointer_center is not None
        and abs(turned_pointer_center[1] - rest_pointer_center[1]) > 0.008,
        details=f"rest={rest_pointer_center}, turned={turned_pointer_center}",
    )

    rest_button_pos = ctx.part_world_position(pulse_button)
    pressed_button_pos = None
    with ctx.pose({pulse_joint: 0.0015}):
        pressed_button_pos = ctx.part_world_position(pulse_button)
    ctx.check(
        "pulse button depresses into the front housing",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[0] < rest_button_pos[0] - 0.0014,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
