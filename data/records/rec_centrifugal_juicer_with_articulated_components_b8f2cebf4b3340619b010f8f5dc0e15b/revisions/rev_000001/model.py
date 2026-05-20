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
    place_on_surface,
)


def _collector_bowl_mesh():
    outer = [
        (0.100, 0.000),
        (0.100, 0.022),
        (0.096, 0.028),
    ]
    inner = [
        (0.086, 0.004),
        (0.086, 0.020),
        (0.082, 0.024),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer, inner, segments=56),
        "collector_bowl",
    )


def _lid_shell_mesh():
    outer = [
        (0.116, 0.000),
        (0.116, 0.052),
        (0.103, 0.090),
        (0.050, 0.113),
        (0.032, 0.125),
        (0.032, 0.205),
    ]
    inner = [
        (0.102, 0.006),
        (0.102, 0.050),
        (0.090, 0.084),
        (0.042, 0.106),
        (0.025, 0.125),
        (0.025, 0.201),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer, inner, segments=64),
        "lid_shell",
    )


def _basket_mesh():
    outer = [
        (0.032, 0.000),
        (0.050, 0.010),
        (0.079, 0.036),
        (0.084, 0.058),
    ]
    inner = [
        (0.022, 0.004),
        (0.042, 0.013),
        (0.071, 0.036),
        (0.077, 0.053),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer, inner, segments=48),
        "basket_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_centrifugal_juicer")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.93, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.16, 0.16, 0.17, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.72, 0.80, 0.84, 0.30))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))

    collector_bowl_mesh = _collector_bowl_mesh()
    lid_shell_mesh = _lid_shell_mesh()
    basket_shell_mesh = _basket_mesh()

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.125, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=trim_dark,
        name="foot_ring",
    )
    base.visual(
        Cylinder(radius=0.112, length=0.148),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=body_white,
        name="body_shell",
    )
    base.visual(
        Cylinder(radius=0.104, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.174)),
        material=body_white,
        name="upper_collar",
    )
    control_panel = base.visual(
        Box((0.012, 0.090, 0.060)),
        origin=Origin(xyz=(0.106, 0.0, 0.105)),
        material=trim_dark,
        name="control_panel",
    )
    base.visual(
        collector_bowl_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=body_white,
        name="collector_bowl",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        material=body_white,
        name="collector_floor",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
        material=steel,
        name="drive_post",
    )
    base.visual(
        Box((0.026, 0.060, 0.030)),
        origin=Origin(xyz=(-0.111, 0.0, 0.198)),
        material=body_white,
        name="hinge_pod",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.250, 0.250, 0.228)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
    )

    basket = model.part("basket")
    basket.visual(
        basket_shell_mesh,
        material=steel,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=steel,
        name="hub",
    )
    for index in range(6):
        angle = index * math.pi / 3.0
        basket.visual(
            Box((0.050, 0.004, 0.016)),
            origin=Origin(
                xyz=(0.037 * math.cos(angle), 0.037 * math.sin(angle), 0.020),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"blade_{index}",
        )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.084, length=0.060),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_shell_mesh,
        origin=Origin(xyz=(0.098, 0.0, -0.006)),
        material=smoke_clear,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.020, 0.050, 0.018)),
        origin=Origin(xyz=(0.008, 0.0, 0.038)),
        material=trim_dark,
        name="hinge_bridge",
    )
    lid.visual(
        Box((0.018, 0.052, 0.008)),
        origin=Origin(xyz=(0.206, 0.0, -0.002)),
        material=smoke_clear,
        name="front_lip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.230, 0.230, 0.215)),
        mass=0.9,
        origin=Origin(xyz=(0.102, 0.0, 0.102)),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.020, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=trim_dark,
        name="pusher_stem",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        pusher.visual(
            Box((0.010, 0.003, 0.024)),
            origin=Origin(
                xyz=(0.020 * math.cos(angle), 0.020 * math.sin(angle), -0.012),
                rpy=(0.0, 0.0, angle),
            ),
            material=trim_dark,
            name=f"guide_rib_{index}",
        )
    pusher.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim_dark,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.016, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=trim_dark,
        name="pusher_grip",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.023, length=0.138),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
    )

    for name, lateral in (("speed_dial", -0.026), ("selector_dial", 0.026)):
        dial = model.part(name)
        dial.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=trim_dark,
            name="shaft_collar",
        )
        dial.visual(
            Cylinder(radius=0.018, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=knob_black,
            name="knob",
        )
        dial.visual(
            Box((0.012, 0.002, 0.003)),
            origin=Origin(xyz=(0.0, 0.014, 0.021)),
            material=body_white,
            name="pointer",
        )
        dial.inertial = Inertial.from_geometry(
            Cylinder(radius=0.018, length=0.022),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
        )

        model.articulation(
            f"base_to_{name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=dial,
            origin=place_on_surface(
                dial,
                control_panel,
                point_hint=(0.112, lateral, 0.105),
                clearance=0.0,
                prefer_collisions=False,
                child_prefer_collisions=False,
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=4.0,
                lower=-2.4,
                upper=2.4,
            ),
        )

    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.189)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.098, 0.0, 0.216)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.098, 0.0, 0.199)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.12,
            lower=0.0,
            upper=0.040,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    speed_dial = object_model.get_part("speed_dial")
    selector_dial = object_model.get_part("selector_dial")

    basket_spin = object_model.get_articulation("base_to_basket")
    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")

    speed_pos = ctx.part_world_position(speed_dial)
    selector_pos = ctx.part_world_position(selector_dial)
    ctx.check(
        "control dials sit side by side on the front panel",
        speed_pos is not None
        and selector_pos is not None
        and speed_pos[0] > 0.10
        and selector_pos[0] > 0.10
        and abs(speed_pos[2] - selector_pos[2]) < 0.005
        and 0.045 < abs(speed_pos[1] - selector_pos[1]) < 0.065,
        details=f"speed={speed_pos}, selector={selector_pos}",
    )

    ctx.check(
        "basket spin articulation is continuous and vertical",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in basket_spin.axis) == (0.0, 0.0, 1.0),
        details=(
            f"type={basket_spin.articulation_type}, axis={basket_spin.axis}, "
            f"limits={basket_spin.motion_limits}"
        ),
    )

    ctx.expect_origin_distance(
        pusher,
        basket,
        axes="xy",
        max_dist=0.005,
        name="pusher stays centered over the basket axis",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.040}):
        pusher_pressed = ctx.part_world_position(pusher)
        ctx.expect_gap(
            pusher,
            basket,
            axis="z",
            min_gap=0.012,
            name="pressed pusher remains above the spinning basket",
        )
    ctx.check(
        "pusher travels downward inside the chute",
        pusher_rest is not None
        and pusher_pressed is not None
        and pusher_pressed[2] < pusher_rest[2] - 0.030,
        details=f"rest={pusher_rest}, pressed={pusher_pressed}",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({lid_hinge: 1.0}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    ctx.check(
        "lid front edge lifts when the rear hinge opens",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.10,
        details=f"closed={closed_front}, open={open_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
