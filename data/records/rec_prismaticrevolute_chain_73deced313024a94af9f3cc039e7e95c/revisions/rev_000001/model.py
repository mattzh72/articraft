from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HALF_PI = 1.5707963267948966


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_fixture_with_inspection_flap")

    painted_steel = Material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    ground_steel = Material("ground_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    carriage_blue = Material("carriage_blue", rgba=(0.05, 0.23, 0.58, 1.0))
    amber_polycarbonate = Material("amber_polycarbonate", rgba=(1.0, 0.58, 0.12, 0.62))
    black_rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    fixture = model.part("fixture")
    fixture.visual(
        Box((0.96, 0.34, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=painted_steel,
        name="base_plate",
    )
    fixture.visual(
        Box((0.09, 0.20, 0.12)),
        origin=Origin(xyz=(-0.42, 0.0, 0.09)),
        material=painted_steel,
        name="end_pedestal_0",
    )
    fixture.visual(
        Box((0.09, 0.20, 0.12)),
        origin=Origin(xyz=(0.42, 0.0, 0.09)),
        material=painted_steel,
        name="end_pedestal_1",
    )
    fixture.visual(
        Box((0.86, 0.08, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.1775)),
        material=ground_steel,
        name="linear_rail",
    )
    fixture.visual(
        Box((0.035, 0.12, 0.075)),
        origin=Origin(xyz=(-0.455, 0.0, 0.1875)),
        material=dark_steel,
        name="travel_stop_0",
    )
    fixture.visual(
        Box((0.035, 0.12, 0.075)),
        origin=Origin(xyz=(0.455, 0.0, 0.1875)),
        material=dark_steel,
        name="travel_stop_1",
    )
    for idx, (x, y) in enumerate(
        ((-0.39, -0.12), (-0.39, 0.12), (0.39, -0.12), (0.39, 0.12))
    ):
        fixture.visual(
            Cylinder(radius=0.018, length=0.009),
            origin=Origin(xyz=(x, y, 0.0335)),
            material=dark_steel,
            name=f"anchor_bolt_{idx}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.22, 0.19, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=carriage_blue,
        name="carriage_bridge",
    )
    carriage.visual(
        Box((0.22, 0.026, 0.075)),
        origin=Origin(xyz=(0.0, -0.068, 0.000)),
        material=carriage_blue,
        name="guide_shoe_0",
    )
    carriage.visual(
        Box((0.22, 0.026, 0.075)),
        origin=Origin(xyz=(0.0, 0.068, 0.000)),
        material=carriage_blue,
        name="guide_shoe_1",
    )
    carriage.visual(
        Box((0.052, 0.030, 0.050)),
        origin=Origin(xyz=(-0.073, -0.105, 0.040)),
        material=carriage_blue,
        name="hinge_stand_0",
    )
    carriage.visual(
        Box((0.052, 0.030, 0.050)),
        origin=Origin(xyz=(0.073, -0.105, 0.040)),
        material=carriage_blue,
        name="hinge_stand_1",
    )
    carriage.visual(
        Cylinder(radius=0.013, length=0.052),
        origin=Origin(xyz=(-0.073, -0.105, 0.066), rpy=(0.0, HALF_PI, 0.0)),
        material=ground_steel,
        name="fixed_barrel_0",
    )
    carriage.visual(
        Cylinder(radius=0.013, length=0.052),
        origin=Origin(xyz=(0.073, -0.105, 0.066), rpy=(0.0, HALF_PI, 0.0)),
        material=ground_steel,
        name="fixed_barrel_1",
    )
    carriage.visual(
        Cylinder(radius=0.005, length=0.218),
        origin=Origin(xyz=(0.0, -0.105, 0.066), rpy=(0.0, HALF_PI, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )
    carriage.visual(
        Box((0.060, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, -0.060, 0.047)),
        material=black_rubber,
        name="flap_stop_pad",
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=0.013, length=0.080),
        origin=Origin(rpy=(0.0, HALF_PI, 0.0)),
        material=ground_steel,
        name="flap_barrel",
    )
    flap.visual(
        Box((0.090, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, -0.010, 0.020)),
        material=dark_steel,
        name="barrel_strap",
    )
    flap.visual(
        Box((0.205, 0.016, 0.245)),
        origin=Origin(xyz=(0.0, -0.020, 0.1425)),
        material=amber_polycarbonate,
        name="inspection_panel",
    )
    flap.visual(
        Box((0.220, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.023, 0.028)),
        material=dark_steel,
        name="lower_frame",
    )
    flap.visual(
        Box((0.220, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.023, 0.257)),
        material=dark_steel,
        name="upper_frame",
    )
    flap.visual(
        Box((0.018, 0.018, 0.245)),
        origin=Origin(xyz=(-0.109, -0.023, 0.1425)),
        material=dark_steel,
        name="side_frame_0",
    )
    flap.visual(
        Box((0.018, 0.018, 0.245)),
        origin=Origin(xyz=(0.109, -0.023, 0.1425)),
        material=dark_steel,
        name="side_frame_1",
    )

    model.articulation(
        "fixture_to_carriage",
        ArticulationType.PRISMATIC,
        parent=fixture,
        child=carriage,
        origin=Origin(xyz=(-0.22, 0.0, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.44),
    )
    model.articulation(
        "carriage_to_flap",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=flap,
        origin=Origin(xyz=(0.0, -0.105, 0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixture = object_model.get_part("fixture")
    carriage = object_model.get_part("carriage")
    flap = object_model.get_part("flap")
    slide = object_model.get_articulation("fixture_to_carriage")
    flap_hinge = object_model.get_articulation("carriage_to_flap")

    ctx.allow_overlap(
        carriage,
        flap,
        elem_a="hinge_pin",
        elem_b="flap_barrel",
        reason="The small hinge pin is intentionally captured inside the rotating flap barrel.",
    )

    ctx.expect_gap(
        carriage,
        fixture,
        axis="z",
        positive_elem="carriage_bridge",
        negative_elem="linear_rail",
        max_gap=0.001,
        max_penetration=0.0005,
        name="carriage bridge rides on the linear rail",
    )
    ctx.expect_overlap(
        carriage,
        fixture,
        axes="x",
        elem_a="carriage_bridge",
        elem_b="linear_rail",
        min_overlap=0.20,
        name="carriage stays engaged on rail at rest",
    )
    ctx.expect_within(
        carriage,
        flap,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="flap_barrel",
        margin=0.0,
        name="hinge pin is centered in flap barrel",
    )
    ctx.expect_overlap(
        carriage,
        flap,
        axes="x",
        elem_a="hinge_pin",
        elem_b="flap_barrel",
        min_overlap=0.075,
        name="flap barrel is retained on hinge pin",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.44}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            fixture,
            axes="x",
            elem_a="carriage_bridge",
            elem_b="linear_rail",
            min_overlap=0.20,
            name="carriage stays engaged on rail at full travel",
        )
    ctx.check(
        "carriage travels horizontally along fixture",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.40
        and abs(extended_pos[1] - rest_pos[1]) < 0.001
        and abs(extended_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    def _aabb_center_z_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)

    closed_panel = _aabb_center_z_y(
        ctx.part_element_world_aabb(flap, elem="inspection_panel")
    )
    with ctx.pose({flap_hinge: 1.15}):
        open_panel = _aabb_center_z_y(
            ctx.part_element_world_aabb(flap, elem="inspection_panel")
        )
    ctx.check(
        "inspection flap rotates forward from carriage",
        closed_panel is not None
        and open_panel is not None
        and open_panel[0] < closed_panel[0] - 0.09
        and open_panel[1] < closed_panel[1] - 0.06,
        details=f"closed_yz={closed_panel}, open_yz={open_panel}",
    )

    return ctx.report()


object_model = build_object_model()
