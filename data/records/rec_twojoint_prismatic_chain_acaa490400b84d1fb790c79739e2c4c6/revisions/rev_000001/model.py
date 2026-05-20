from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_double_slide_module")

    black = Material("black_powdercoat", rgba=(0.025, 0.027, 0.030, 1.0))
    rail_steel = Material("dark_burnished_steel", rgba=(0.32, 0.34, 0.34, 1.0))
    carriage_blue = Material("blue_anodized_carriage", rgba=(0.05, 0.16, 0.34, 1.0))
    slider_silver = Material("brushed_inner_slider", rgba=(0.72, 0.74, 0.70, 1.0))
    fastener = Material("dark_socket_fasteners", rgba=(0.015, 0.014, 0.013, 1.0))

    base_support = model.part(
        "base_support",
        inertial=Inertial.from_geometry(
            Box((0.98, 0.090, 0.28)),
            mass=10.5,
            origin=Origin(xyz=(0.0, -0.020, 0.0)),
        ),
    )
    base_support.visual(
        Box((0.98, 0.035, 0.26)),
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        material=black,
        name="wall_plate",
    )
    base_support.visual(
        Box((0.94, 0.025, 0.025)),
        origin=Origin(xyz=(0.0, -0.015, 0.065)),
        material=rail_steel,
        name="upper_root_rail",
    )
    base_support.visual(
        Box((0.94, 0.025, 0.025)),
        origin=Origin(xyz=(0.0, -0.015, -0.065)),
        material=rail_steel,
        name="lower_root_rail",
    )
    base_support.visual(
        Box((0.030, 0.055, 0.200)),
        origin=Origin(xyz=(-0.485, -0.015, 0.0)),
        material=black,
        name="root_stop_0",
    )
    base_support.visual(
        Box((0.030, 0.055, 0.200)),
        origin=Origin(xyz=(0.485, -0.015, 0.0)),
        material=black,
        name="root_stop_1",
    )
    for x in (-0.38, 0.38):
        for z in (-0.085, 0.085):
            base_support.visual(
                Cylinder(radius=0.018, length=0.006),
                origin=Origin(xyz=(x, -0.025, z), rpy=(1.57079632679, 0.0, 0.0)),
                material=fastener,
                name=f"wall_bolt_{'n' if z < 0 else 'p'}_{'n' if x < 0 else 'p'}",
            )

    carriage = model.part(
        "carriage",
        inertial=Inertial.from_geometry(
            Box((0.44, 0.10, 0.16)),
            mass=3.2,
            origin=Origin(xyz=(0.0, 0.035, 0.0)),
        ),
    )
    carriage.visual(
        Box((0.42, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.0075, 0.065)),
        material=rail_steel,
        name="upper_bearing_shoe",
    )
    carriage.visual(
        Box((0.42, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.0075, -0.065)),
        material=rail_steel,
        name="lower_bearing_shoe",
    )
    carriage.visual(
        Box((0.40, 0.030, 0.100)),
        origin=Origin(xyz=(0.0, 0.0225, 0.0)),
        material=carriage_blue,
        name="carriage_web",
    )
    carriage.visual(
        Box((0.34, 0.013, 0.075)),
        origin=Origin(xyz=(0.0, 0.0435, 0.0)),
        material=carriage_blue,
        name="track_neck",
    )
    carriage.visual(
        Box((0.40, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, 0.065, 0.0)),
        material=rail_steel,
        name="carried_track",
    )

    inner_slider = model.part(
        "inner_slider",
        inertial=Inertial.from_geometry(
            Box((0.34, 0.065, 0.12)),
            mass=1.25,
            origin=Origin(xyz=(0.04, 0.035, 0.0)),
        ),
    )
    inner_slider.visual(
        Box((0.32, 0.034, 0.040)),
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material=slider_silver,
        name="inner_bar",
    )
    inner_slider.visual(
        Box((0.180, 0.020, 0.110)),
        origin=Origin(xyz=(0.110, 0.044, 0.0)),
        material=slider_silver,
        name="front_plate",
    )
    inner_slider.visual(
        Box((0.025, 0.046, 0.060)),
        origin=Origin(xyz=(0.1725, 0.023, 0.0)),
        material=black,
        name="inner_end_cap",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_support,
        child=carriage,
        origin=Origin(xyz=(-0.18, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.44),
        motion_properties=MotionProperties(damping=4.0, friction=1.2),
    )
    model.articulation(
        "carriage_to_inner",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=inner_slider,
        origin=Origin(xyz=(-0.06, 0.080, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.40, lower=0.0, upper=0.28),
        motion_properties=MotionProperties(damping=2.5, friction=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_support")
    carriage = object_model.get_part("carriage")
    inner = object_model.get_part("inner_slider")
    first = object_model.get_articulation("base_to_carriage")
    second = object_model.get_articulation("carriage_to_inner")

    ctx.check(
        "two sequential prismatic stages",
        first.articulation_type == ArticulationType.PRISMATIC
        and second.articulation_type == ArticulationType.PRISMATIC
        and first.parent == "base_support"
        and first.child == "carriage"
        and second.parent == "carriage"
        and second.child == "inner_slider",
        details=f"first={first.parent}->{first.child}, second={second.parent}->{second.child}",
    )
    ctx.check(
        "root stage is longer and heavier",
        base.inertial is not None
        and inner.inertial is not None
        and base.inertial.mass > inner.inertial.mass * 4.0,
        details=f"base_mass={getattr(base.inertial, 'mass', None)}, inner_mass={getattr(inner.inertial, 'mass', None)}",
    )

    with ctx.pose({first: 0.0, second: 0.0}):
        ctx.expect_gap(
            carriage,
            base,
            axis="y",
            positive_elem="upper_bearing_shoe",
            negative_elem="upper_root_rail",
            max_gap=0.001,
            max_penetration=0.00001,
            name="upper carriage shoe rides the root rail",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="y",
            positive_elem="lower_bearing_shoe",
            negative_elem="lower_root_rail",
            max_gap=0.001,
            max_penetration=0.00001,
            name="lower carriage shoe rides the root rail",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="xz",
            elem_a="upper_bearing_shoe",
            elem_b="upper_root_rail",
            min_overlap=0.020,
            name="upper root rail captures the first stage",
        )
        ctx.expect_gap(
            inner,
            carriage,
            axis="y",
            positive_elem="inner_bar",
            negative_elem="carried_track",
            max_gap=0.001,
            max_penetration=0.0,
            name="inner slider rides the carried track",
        )
        ctx.expect_overlap(
            inner,
            carriage,
            axes="xz",
            elem_a="inner_bar",
            elem_b="carried_track",
            min_overlap=0.020,
            name="carried track captures the second stage",
        )

    rest_carriage = ctx.part_world_position(carriage)
    rest_inner = ctx.part_world_position(inner)
    with ctx.pose({first: 0.44, second: 0.28}):
        extended_carriage = ctx.part_world_position(carriage)
        extended_inner = ctx.part_world_position(inner)
        ctx.expect_gap(
            base,
            carriage,
            axis="x",
            positive_elem="root_stop_1",
            negative_elem="upper_bearing_shoe",
            max_gap=0.001,
            max_penetration=0.00001,
            name="first stage reaches the positive travel stop",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="upper_bearing_shoe",
            elem_b="upper_root_rail",
            min_overlap=0.25,
            name="first stage retains root-rail insertion at full travel",
        )
        ctx.expect_overlap(
            inner,
            carriage,
            axes="x",
            elem_a="inner_bar",
            elem_b="carried_track",
            min_overlap=0.10,
            name="second stage retains carried-track insertion at full travel",
        )

    ctx.check(
        "both stages extend in positive x",
        rest_carriage is not None
        and extended_carriage is not None
        and rest_inner is not None
        and extended_inner is not None
        and extended_carriage[0] > rest_carriage[0] + 0.40
        and extended_inner[0] > rest_inner[0] + 0.65,
        details=(
            f"rest_carriage={rest_carriage}, extended_carriage={extended_carriage}, "
            f"rest_inner={rest_inner}, extended_inner={extended_inner}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
