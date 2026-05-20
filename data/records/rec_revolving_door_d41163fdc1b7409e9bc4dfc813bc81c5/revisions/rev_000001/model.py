from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_revolving_airlock")

    dark_steel = model.material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    floor_rubber = model.material("ribbed_black_floor", rgba=(0.025, 0.025, 0.025, 1.0))
    laminated_glass = model.material("laminated_glass", rgba=(0.45, 0.70, 0.86, 0.34))
    warning_yellow = model.material("warning_yellow", rgba=(1.0, 0.76, 0.08, 1.0))
    red_light = model.material("red_status_light", rgba=(0.9, 0.03, 0.02, 1.0))
    green_light = model.material("green_status_light", rgba=(0.02, 0.75, 0.18, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((5.20, 1.95, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=floor_rubber,
        name="floor_slab",
    )
    housing.visual(
        Box((5.20, 1.95, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.31)),
        material=dark_steel,
        name="ceiling_slab",
    )
    housing.visual(
        Box((5.20, 0.055, 2.18)),
        origin=Origin(xyz=(0.0, -0.975, 1.17)),
        material=dark_steel,
        name="front_threshold",
    )
    housing.visual(
        Box((5.20, 0.055, 2.18)),
        origin=Origin(xyz=(0.0, 0.975, 1.17)),
        material=dark_steel,
        name="rear_threshold",
    )
    housing.visual(
        Box((0.09, 1.95, 2.18)),
        origin=Origin(xyz=(-2.60, 0.0, 1.17)),
        material=dark_steel,
        name="entry_frame",
    )
    housing.visual(
        Box((0.09, 1.95, 2.18)),
        origin=Origin(xyz=(2.60, 0.0, 1.17)),
        material=dark_steel,
        name="exit_frame",
    )
    housing.visual(
        Box((0.12, 0.18, 2.06)),
        origin=Origin(xyz=(0.0, -0.78, 1.13)),
        material=dark_steel,
        name="interstage_jamb_0",
    )
    housing.visual(
        Box((0.12, 0.18, 2.06)),
        origin=Origin(xyz=(0.0, 0.78, 1.13)),
        material=dark_steel,
        name="interstage_jamb_1",
    )
    housing.visual(
        Box((0.12, 1.95, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 2.20)),
        material=dark_steel,
        name="interstage_lintel",
    )
    housing.visual(
        Box((0.035, 0.36, 1.72)),
        origin=Origin(xyz=(0.0, -0.62, 1.08)),
        material=laminated_glass,
        name="interstage_glass_0",
    )
    housing.visual(
        Box((0.035, 0.36, 1.72)),
        origin=Origin(xyz=(0.0, 0.62, 1.08)),
        material=laminated_glass,
        name="interstage_glass_1",
    )

    station_x = (-1.20, 1.20)
    cage_radius = 0.90
    for index, x in enumerate(station_x):
        for z, suffix in ((0.125, "lower"), (1.17, "waist"), (2.215, "upper")):
            housing.visual(
                mesh_from_geometry(
                    TorusGeometry(radius=cage_radius, tube=0.023, radial_segments=18, tubular_segments=72),
                    f"drum_{index}_{suffix}_cage_ring",
                ),
                origin=Origin(xyz=(x, 0.0, z)),
                material=dark_steel,
                name=f"{suffix}_cage_ring_{index}",
            )

        for bar_index in range(8):
            theta = math.tau * bar_index / 8.0
            bx = x + cage_radius * math.cos(theta)
            by = cage_radius * math.sin(theta)
            housing.visual(
                Cylinder(radius=0.024, length=2.18),
                origin=Origin(xyz=(bx, by, 1.17)),
                material=dark_steel,
                name=f"cage_bar_{index}_{bar_index}",
            )

        housing.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.095, tube=0.014, radial_segments=12, tubular_segments=48),
                f"drum_{index}_floor_bearing_ring",
            ),
            origin=Origin(xyz=(x, 0.0, 0.094)),
            material=brushed_steel,
            name=f"floor_bearing_{index}",
        )
        housing.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.095, tube=0.014, radial_segments=12, tubular_segments=48),
                f"drum_{index}_ceiling_bearing_ring",
            ),
            origin=Origin(xyz=(x, 0.0, 2.246)),
            material=brushed_steel,
            name=f"ceiling_bearing_{index}",
        )

        housing.visual(
            Box((0.36, 0.06, 0.08)),
            origin=Origin(xyz=(x - 0.24, -1.01, 2.14)),
            material=red_light if index == 0 else green_light,
            name=f"status_light_{index}_0",
        )
        housing.visual(
            Box((0.36, 0.06, 0.08)),
            origin=Origin(xyz=(x + 0.24, -1.01, 2.14)),
            material=green_light if index == 0 else red_light,
            name=f"status_light_{index}_1",
        )
        housing.visual(
            Box((0.38, 0.038, 1.90)),
            origin=Origin(xyz=(x, -0.915, 1.12)),
            material=laminated_glass,
            name=f"fixed_guard_glass_{index}",
        )

    for stripe_x in (-2.0, -0.4, 0.4, 2.0):
        housing.visual(
            Box((0.45, 0.035, 0.012)),
            origin=Origin(xyz=(stripe_x, -0.80, 0.086)),
            material=warning_yellow,
            name=f"floor_stripe_{stripe_x}",
        )

    def add_revolving_drum(index: int, x: float):
        drum = model.part(f"drum_{index}")
        drum.visual(
            Cylinder(radius=0.055, length=2.07),
            origin=Origin(xyz=(0.0, 0.0, 1.13)),
            material=brushed_steel,
            name="central_post",
        )
        drum.visual(
            Cylinder(radius=0.083, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, 0.125)),
            material=brushed_steel,
            name="lower_hub",
        )
        drum.visual(
            Cylinder(radius=0.070, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, 2.135)),
            material=brushed_steel,
            name="upper_hub",
        )

        wing_length = 0.76
        panel_height = 1.86
        panel_thickness = 0.036
        panel_center = (0.045 + wing_length) / 2.0
        panel_span = wing_length - 0.045
        for wing_index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
            cx = panel_center * math.cos(angle)
            cy = panel_center * math.sin(angle)
            if wing_index % 2 == 0:
                size = (panel_span, panel_thickness, panel_height)
            else:
                size = (panel_thickness, panel_span, panel_height)
            drum.visual(
                Box(size),
                origin=Origin(xyz=(cx, cy, 1.10)),
                material=laminated_glass,
                name=f"panel_wing_{wing_index}",
            )
            drum.visual(
                Box((panel_span + 0.035 if wing_index % 2 == 0 else 0.052,
                     0.052 if wing_index % 2 == 0 else panel_span + 0.035,
                     0.045)),
                origin=Origin(xyz=(cx, cy, 2.055)),
                material=dark_steel,
                name=f"top_rail_{wing_index}",
            )
            drum.visual(
                Box((panel_span + 0.035 if wing_index % 2 == 0 else 0.052,
                     0.052 if wing_index % 2 == 0 else panel_span + 0.035,
                     0.045)),
                origin=Origin(xyz=(cx, cy, 0.145)),
                material=dark_steel,
                name=f"bottom_rail_{wing_index}",
            )
            tip_x = wing_length * math.cos(angle)
            tip_y = wing_length * math.sin(angle)
            if wing_index % 2 == 0:
                stile_size = (0.042, 0.060, panel_height)
            else:
                stile_size = (0.060, 0.042, panel_height)
            drum.visual(
                Box(stile_size),
                origin=Origin(xyz=(tip_x, tip_y, 1.10)),
                material=dark_steel,
                name=f"outer_stile_{wing_index}",
            )

        model.articulation(
            f"drum_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=drum,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=120.0, velocity=1.4),
        )
        return drum

    add_revolving_drum(0, station_x[0])
    add_revolving_drum(1, station_x[1])
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    drum_0 = object_model.get_part("drum_0")
    drum_1 = object_model.get_part("drum_1")
    spin_0 = object_model.get_articulation("drum_0_spin")
    spin_1 = object_model.get_articulation("drum_1_spin")

    ctx.check(
        "two independent continuous drum joints",
        spin_0.articulation_type == ArticulationType.CONTINUOUS
        and spin_1.articulation_type == ArticulationType.CONTINUOUS
        and spin_0.child == "drum_0"
        and spin_1.child == "drum_1"
        and spin_0.parent == "housing"
        and spin_1.parent == "housing",
        details=f"spin_0={spin_0}, spin_1={spin_1}",
    )

    for index, drum in enumerate((drum_0, drum_1)):
        ctx.allow_overlap(
            drum,
            housing,
            elem_a="lower_hub",
            elem_b=f"floor_bearing_{index}",
            reason=(
                "The rotating hub is intentionally captured in the floor bearing race; "
                "the small hidden overlap represents bearing engagement rather than a collision."
            ),
        )
        ctx.expect_overlap(
            drum,
            housing,
            axes="z",
            elem_a="lower_hub",
            elem_b=f"floor_bearing_{index}",
            min_overlap=0.008,
            name=f"drum {index} lower hub remains seated in bearing",
        )
        ctx.expect_gap(
            drum,
            housing,
            axis="z",
            positive_elem="lower_hub",
            negative_elem=f"floor_bearing_{index}",
            max_penetration=0.012,
            name=f"drum {index} lower bearing embed stays shallow",
        )
        ctx.expect_gap(
            housing,
            drum,
            axis="z",
            positive_elem="ceiling_slab",
            negative_elem="central_post",
            min_gap=0.08,
            max_gap=0.13,
            name=f"drum {index} post clears overhead cap",
        )

    def aabb_center(aabb):
        if aabb is None:
            return None

        def coord(vec, axis_index):
            try:
                return vec[axis_index]
            except TypeError:
                return (vec.x, vec.y, vec.z)[axis_index]

        return tuple((coord(aabb[0], i) + coord(aabb[1], i)) * 0.5 for i in range(3))

    with ctx.pose({spin_0: 0.0, spin_1: 0.0}):
        rest_0 = aabb_center(ctx.part_element_world_aabb(drum_0, elem="panel_wing_0"))
        rest_1 = aabb_center(ctx.part_element_world_aabb(drum_1, elem="panel_wing_0"))

    with ctx.pose({spin_0: math.pi / 2.0, spin_1: 0.0}):
        turned_0 = aabb_center(ctx.part_element_world_aabb(drum_0, elem="panel_wing_0"))
        still_1 = aabb_center(ctx.part_element_world_aabb(drum_1, elem="panel_wing_0"))

    ctx.check(
        "first drum wing turns around its vertical post",
        rest_0 is not None
        and turned_0 is not None
        and rest_0[0] > -1.20 + 0.30
        and abs(rest_0[1]) < 0.04
        and abs(turned_0[0] - -1.20) < 0.05
        and turned_0[1] > 0.30,
        details=f"rest_0={rest_0}, turned_0={turned_0}",
    )
    ctx.check(
        "second drum remains independent while first turns",
        rest_1 is not None
        and still_1 is not None
        and abs(still_1[0] - rest_1[0]) < 0.01
        and abs(still_1[1] - rest_1[1]) < 0.01,
        details=f"rest_1={rest_1}, still_1={still_1}",
    )

    return ctx.report()


object_model = build_object_model()
