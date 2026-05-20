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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _horizontal_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _base_housing_mesh():
    return section_loft(
        [
            _horizontal_section(0.340, 0.248, 0.050, 0.000),
            _horizontal_section(0.332, 0.240, 0.055, 0.040),
            _horizontal_section(0.312, 0.224, 0.060, 0.095),
            _horizontal_section(0.260, 0.186, 0.052, 0.138),
        ]
    )


def _chamber_shell_mesh():
    outer_profile = [
        (0.000, 0.000),
        (0.016, 0.004),
        (0.032, 0.010),
        (0.062, 0.026),
        (0.089, 0.060),
        (0.101, 0.094),
        (0.106, 0.112),
    ]
    inner_profile = [
        (0.000, 0.006),
        (0.014, 0.010),
        (0.026, 0.014),
        (0.054, 0.030),
        (0.082, 0.062),
        (0.092, 0.092),
        (0.096, 0.106),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
    )


def _lid_shell_mesh():
    outer_profile = [
        (0.104, 0.000),
        (0.104, 0.012),
        (0.090, 0.032),
        (0.070, 0.050),
        (0.050, 0.068),
        (0.036, 0.092),
        (0.035, 0.140),
    ]
    inner_profile = [
        (0.096, 0.003),
        (0.094, 0.012),
        (0.083, 0.028),
        (0.064, 0.044),
        (0.046, 0.060),
        (0.029, 0.084),
        (0.028, 0.136),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
    )


def _basket_shell_mesh():
    outer_profile = [
        (0.018, 0.000),
        (0.040, 0.006),
        (0.066, 0.016),
        (0.078, 0.030),
        (0.082, 0.058),
        (0.080, 0.078),
    ]
    inner_profile = [
        (0.000, 0.003),
        (0.022, 0.008),
        (0.046, 0.016),
        (0.070, 0.030),
        (0.074, 0.056),
        (0.071, 0.074),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_centrifugal_juicer")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.94, 1.0))
    body_shadow = model.material("body_shadow", rgba=(0.84, 0.84, 0.82, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.78, 0.86, 0.90, 0.28))
    clear_dark = model.material("clear_dark", rgba=(0.42, 0.47, 0.50, 0.48))
    stainless = model.material("stainless", rgba=(0.80, 0.81, 0.82, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.12, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.16, 0.16, 0.17, 1.0))

    base_mesh = mesh_from_geometry(_base_housing_mesh(), "juicer_base_housing")
    chamber_mesh = mesh_from_geometry(_chamber_shell_mesh(), "juicer_chamber_shell")
    lid_mesh = mesh_from_geometry(_lid_shell_mesh(), "juicer_lid_shell")
    basket_mesh = mesh_from_geometry(_basket_shell_mesh(), "juicer_filter_basket")

    base = model.part("base")
    base.visual(base_mesh, material=body_white, name="housing")
    base.visual(
        Box((0.022, 0.100, 0.038)),
        origin=Origin(xyz=(0.159, 0.000, 0.074)),
        material=body_shadow,
        name="front_panel",
    )
    base.visual(
        Cylinder(radius=0.057, length=0.006),
        origin=Origin(xyz=(-0.010, 0.000, 0.141)),
        material=body_shadow,
        name="chamber_seat",
    )
    for y in (-0.080, 0.080):
        for x in (-0.105, 0.105):
            base.visual(
                Cylinder(radius=0.013, length=0.010),
                origin=Origin(xyz=(x, y, 0.005)),
                material=rubber_dark,
                name=None,
            )
    base.inertial = Inertial.from_geometry(
        Box((0.340, 0.248, 0.148)),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.000, 0.074)),
    )

    chamber = model.part("chamber")
    chamber.visual(chamber_mesh, material=clear_dark, name="chamber_shell")
    chamber.visual(
        Box((0.090, 0.054, 0.044)),
        origin=Origin(xyz=(0.112, 0.000, 0.060)),
        material=clear_dark,
        name="spout_body",
    )
    chamber.visual(
        Box((0.050, 0.040, 0.016)),
        origin=Origin(xyz=(0.165, 0.000, 0.064)),
        material=clear_dark,
        name="spout_lip",
    )
    chamber.visual(
        Box((0.028, 0.112, 0.016)),
        origin=Origin(xyz=(-0.082, 0.000, 0.097)),
        material=clear_dark,
        name="rear_shroud",
    )
    for y in (-0.047, 0.047):
        chamber.visual(
            Box((0.020, 0.016, 0.030)),
            origin=Origin(xyz=(-0.101, y, 0.090)),
            material=clear_dark,
            name=None,
        )
    chamber.inertial = Inertial.from_geometry(
        Box((0.230, 0.160, 0.116)),
        mass=0.8,
        origin=Origin(xyz=(0.020, 0.000, 0.058)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_mesh,
        origin=Origin(xyz=(0.102, 0.000, -0.002)),
        material=clear_smoke,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.078),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.036, 0.088, 0.018)),
        origin=Origin(xyz=(0.020, 0.000, 0.010)),
        material=knob_black,
        name="rear_web",
    )
    lid.visual(
        Box((0.022, 0.050, 0.014)),
        origin=Origin(xyz=(0.204, 0.000, 0.010)),
        material=knob_black,
        name="front_tab",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.220, 0.140, 0.145)),
        mass=0.65,
        origin=Origin(xyz=(0.105, 0.000, 0.072)),
    )

    basket = model.part("basket")
    basket.visual(basket_mesh, material=stainless, name="basket_shell")
    basket.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=stainless,
        name="hub",
    )
    basket.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
        material=stainless,
        name="shaft",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.082, length=0.080),
        mass=0.35,
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(0.008, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="shaft",
    )
    selector.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.028, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="dial",
    )
    selector.visual(
        Box((0.006, 0.004, 0.016)),
        origin=Origin(xyz=(0.040, 0.000, 0.016)),
        material=stainless,
        name="pointer",
    )
    selector.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.034)),
        mass=0.08,
        origin=Origin(xyz=(0.025, 0.000, 0.000)),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.024, length=0.154),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=body_white,
        name="shaft",
    )
    pusher.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=body_shadow,
        name="stop_collar",
    )
    pusher.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.000, 0.000, 0.101)),
        material=body_white,
        name="cap",
    )
    pusher.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.120)),
        material=body_shadow,
        name="top_button",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.182),
        mass=0.18,
        origin=Origin(xyz=(0.000, 0.000, 0.030)),
    )

    model.articulation(
        "base_to_chamber",
        ArticulationType.FIXED,
        parent=base,
        child=chamber,
        origin=Origin(xyz=(-0.010, 0.000, 0.144)),
    )
    model.articulation(
        "chamber_to_lid",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=lid,
        origin=Origin(xyz=(-0.102, 0.000, 0.114)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "chamber_to_basket",
        ArticulationType.CONTINUOUS,
        parent=chamber,
        child=basket,
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=45.0,
        ),
    )
    model.articulation(
        "base_to_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=selector,
        origin=Origin(xyz=(0.170, 0.000, 0.074)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=3.0,
            lower=-1.1,
            upper=1.1,
        ),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.102, 0.000, 0.136)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.16,
            lower=0.0,
            upper=0.082,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    selector = object_model.get_part("selector")
    pusher = object_model.get_part("pusher")

    lid_hinge = object_model.get_articulation("chamber_to_lid")
    basket_spin = object_model.get_articulation("chamber_to_basket")
    selector_spin = object_model.get_articulation("base_to_selector")
    pusher_slide = object_model.get_articulation("lid_to_pusher")

    ctx.allow_overlap(
        chamber,
        basket,
        reason="The clear chamber is represented as a simplified closed shell mesh around the spinning filter basket cavity.",
    )
    ctx.allow_overlap(
        chamber,
        lid,
        elem_a="chamber_shell",
        elem_b="hinge_barrel",
        reason="The rear hinge pin is intentionally nested into the simplified chamber shell rim instead of a separately modeled knuckle cutout.",
    )

    with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.0}):
        ctx.expect_gap(
            lid,
            chamber,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="chamber_shell",
            max_gap=0.010,
            max_penetration=0.001,
            name="lid seats onto chamber rim",
        )
        ctx.expect_overlap(
            lid,
            chamber,
            axes="xy",
            elem_a="lid_shell",
            elem_b="chamber_shell",
            min_overlap=0.180,
            name="lid covers the spinning chamber",
        )
        ctx.expect_within(
            basket,
            chamber,
            axes="xy",
            inner_elem="basket_shell",
            outer_elem="chamber_shell",
            margin=0.010,
            name="filter basket stays inside chamber footprint",
        )

    ctx.expect_origin_gap(
        selector,
        base,
        axis="x",
        min_gap=0.150,
        max_gap=0.185,
        name="speed selector is mounted on the front of the base",
    )

    lid_front_rest = ctx.part_element_world_aabb(lid, elem="front_tab")
    with ctx.pose({lid_hinge: math.radians(70.0)}):
        lid_front_open = ctx.part_element_world_aabb(lid, elem="front_tab")
    ctx.check(
        "lid front rises when opened",
        lid_front_rest is not None
        and lid_front_open is not None
        and lid_front_open[0][2] > lid_front_rest[0][2] + 0.080,
        details=f"rest={lid_front_rest}, open={lid_front_open}",
    )

    with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.0}):
        pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.082}):
        pusher_low = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides downward in the chute",
        pusher_rest is not None
        and pusher_low is not None
        and pusher_low[2] < pusher_rest[2] - 0.060,
        details=f"rest={pusher_rest}, low={pusher_low}",
    )

    ctx.check(
        "filter basket spins about the vertical axis",
        tuple(round(v, 6) for v in basket_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={basket_spin.axis}",
    )
    ctx.check(
        "speed selector rotates on its front shaft",
        tuple(round(v, 6) for v in selector_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={selector_spin.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
