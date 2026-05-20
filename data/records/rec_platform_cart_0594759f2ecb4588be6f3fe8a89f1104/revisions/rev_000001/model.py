from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stock_cart")

    steel_blue = model.material("blue_powder_coat", rgba=(0.05, 0.16, 0.32, 1.0))
    deck_black = model.material("black_phenolic_deck", rgba=(0.015, 0.016, 0.014, 1.0))
    grip_black = model.material("ribbed_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    tire_black = model.material("solid_black_rubber", rgba=(0.012, 0.012, 0.011, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    hub_grey = model.material("pressed_grey_hub", rgba=(0.48, 0.50, 0.52, 1.0))
    caution = model.material("raised_caution_strips", rgba=(0.93, 0.70, 0.06, 1.0))

    # World/object frame: +X points toward the caster end, +Y across the deck,
    # +Z upward from the floor.  The deck top is a low load platform at 0.25 m.
    deck = model.part("deck")
    deck.visual(
        Box((1.25, 0.55, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=deck_black,
        name="deck_board",
    )
    deck.visual(
        Box((1.32, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, 0.300, 0.205)),
        material=steel_blue,
        name="side_rail_0",
    )
    deck.visual(
        Box((1.32, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, -0.300, 0.205)),
        material=steel_blue,
        name="side_rail_1",
    )
    deck.visual(
        Box((0.050, 0.63, 0.045)),
        origin=Origin(xyz=(0.645, 0.0, 0.205)),
        material=steel_blue,
        name="front_end_rail",
    )
    deck.visual(
        Box((0.050, 0.63, 0.045)),
        origin=Origin(xyz=(-0.645, 0.0, 0.205)),
        material=steel_blue,
        name="rear_end_rail",
    )
    for i, y in enumerate((-0.120, 0.120)):
        deck.visual(
            Box((1.05, 0.035, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.253)),
            material=caution,
            name=f"deck_strip_{i}",
        )

    # Rear fixed axle and its two drop brackets: the wheel bores ride on this
    # transverse tube, while the deck brackets visibly carry the axle load.
    deck.visual(
        Cylinder(radius=0.020, length=0.880),
        origin=Origin(xyz=(-0.500, 0.0, 0.140), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="rear_axle",
    )
    for i, y in enumerate((-0.285, 0.285)):
        deck.visual(
            Box((0.075, 0.045, 0.135)),
            origin=Origin(xyz=(-0.500, y, 0.165)),
            material=steel_blue,
            name=f"rear_axle_bracket_{i}",
        )

    # Front caster mounting plates are welded under the deck and are the parents
    # for the vertical-swivel caster assemblies.
    for i, y in enumerate((-0.205, 0.205)):
        deck.visual(
            Box((0.165, 0.145, 0.014)),
            origin=Origin(xyz=(0.475, y, 0.183)),
            material=galvanized,
            name=f"caster_plate_{i}",
        )

    # Hinge clips at both deck ends.  The handle pivot bars intentionally pass
    # through these solid proxy barrels; tests scope and justify that local
    # captured-pin overlap.
    for end_name, x in (("rear", -0.655), ("front", 0.655)):
        for i, y in enumerate((-0.230, 0.230)):
            deck.visual(
                Box((0.070, 0.074, 0.060)),
                origin=Origin(xyz=(x, y, 0.222)),
                material=steel_blue,
                name=f"{end_name}_hinge_block_{i}",
            )
            deck.visual(
                Cylinder(radius=0.026, length=0.078),
                origin=Origin(xyz=(x, y, 0.275), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=galvanized,
                name=f"{end_name}_hinge_socket_{i}",
            )

    # Two upright folding end handles.  Each handle is one connected tubular
    # hoop with a continuous bottom pivot bar clipped into the deck sockets.
    handle_specs = (
        ("rear_handle", -0.655, (0.0, 1.0, 0.0)),
        ("front_handle", 0.655, (0.0, -1.0, 0.0)),
    )
    for handle_name, hinge_x, fold_axis in handle_specs:
        handle = model.part(handle_name)
        handle.visual(
            Cylinder(radius=0.022, length=0.630),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name="pivot_bar",
        )
        for i, y in enumerate((-0.290, 0.290)):
            handle.visual(
                Cylinder(radius=0.018, length=0.720),
                origin=Origin(xyz=(0.0, y, 0.360)),
                material=steel_blue,
                name=f"upright_tube_{i}",
            )
        handle.visual(
            Cylinder(radius=0.018, length=0.640),
            origin=Origin(xyz=(0.0, 0.0, 0.720), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_blue,
            name="top_cross_tube",
        )
        handle.visual(
            Cylinder(radius=0.024, length=0.340),
            origin=Origin(xyz=(0.0, 0.0, 0.720), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=grip_black,
            name="rubber_grip",
        )
        model.articulation(
            f"deck_to_{handle_name}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=handle,
            origin=Origin(xyz=(hinge_x, 0.0, 0.275)),
            axis=fold_axis,
            motion_limits=MotionLimits(effort=25.0, velocity=1.3, lower=0.0, upper=1.42),
        )

    rear_tire_meshes = []
    rear_wheel_meshes = []
    for i in range(2):
        rear_tire_meshes.append(
            mesh_from_geometry(
                TireGeometry(
                    0.140,
                    0.058,
                    inner_radius=0.102,
                    carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.045),
                    tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.58),
                    grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.003),),
                    sidewall=TireSidewall(style="rounded", bulge=0.045),
                    shoulder=TireShoulder(width=0.007, radius=0.0035),
                ),
                f"rear_tire_{i}",
            )
        )
        rear_wheel_meshes.append(
            mesh_from_geometry(
                WheelGeometry(
                    0.102,
                    0.050,
                    rim=WheelRim(
                        inner_radius=0.068,
                        flange_height=0.006,
                        flange_thickness=0.004,
                        bead_seat_depth=0.003,
                    ),
                    hub=WheelHub(
                        radius=0.030,
                        width=0.044,
                        cap_style="domed",
                        bolt_pattern=BoltPattern(
                            count=5,
                            circle_diameter=0.044,
                            hole_diameter=0.005,
                        ),
                    ),
                    face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
                    spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.012),
                    bore=WheelBore(style="round", diameter=0.040),
                ),
                f"rear_wheel_rim_{i}",
            )
        )

    for i, y in enumerate((-0.390, 0.390)):
        wheel = model.part(f"rear_wheel_{i}")
        wheel.visual(rear_tire_meshes[i], material=tire_black, name="tire")
        wheel.visual(rear_wheel_meshes[i], material=hub_grey, name="rim")
        model.articulation(
            f"deck_to_rear_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=wheel,
            origin=Origin(xyz=(-0.500, y, 0.140), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
        )

    caster_tire_meshes = []
    caster_wheel_meshes = []
    for i in range(2):
        caster_tire_meshes.append(
            mesh_from_geometry(
                TireGeometry(
                    0.074,
                    0.036,
                    inner_radius=0.050,
                    tread=TireTread(style="ribbed", depth=0.003, count=14, land_ratio=0.60),
                    sidewall=TireSidewall(style="rounded", bulge=0.035),
                    shoulder=TireShoulder(width=0.004, radius=0.0025),
                ),
                f"caster_tire_{i}",
            )
        )
        caster_wheel_meshes.append(
            mesh_from_geometry(
                WheelGeometry(
                    0.050,
                    0.030,
                    rim=WheelRim(inner_radius=0.033, flange_height=0.004, flange_thickness=0.003),
                    hub=WheelHub(radius=0.018, width=0.026, cap_style="flat"),
                    face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
                    spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.006),
                    bore=WheelBore(style="round", diameter=0.018),
                ),
                f"caster_wheel_rim_{i}",
            )
        )

    for i, y in enumerate((-0.205, 0.205)):
        caster = model.part(f"caster_{i}")
        caster.visual(
            Cylinder(radius=0.014, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, -0.035)),
            material=galvanized,
            name="vertical_stem",
        )
        caster.visual(
            Cylinder(radius=0.040, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.007)),
            material=galvanized,
            name="swivel_bearing",
        )
        caster.visual(
            Box((0.072, 0.108, 0.020)),
            origin=Origin(xyz=(-0.048, 0.0, -0.014)),
            material=galvanized,
            name="fork_crown",
        )
        for j, fork_y in enumerate((-0.048, 0.048)):
            caster.visual(
                Box((0.064, 0.012, 0.140)),
                origin=Origin(xyz=(-0.095, fork_y, -0.090)),
                material=galvanized,
                name=f"fork_plate_{j}",
            )
        caster.visual(
            Cylinder(radius=0.0095, length=0.118),
            origin=Origin(xyz=(-0.095, 0.0, -0.102), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name="axle_pin",
        )
        model.articulation(
            f"deck_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=caster,
            origin=Origin(xyz=(0.475, y, 0.176)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=5.0),
        )

        caster_wheel = model.part(f"caster_wheel_{i}")
        caster_wheel.visual(caster_tire_meshes[i], material=tire_black, name="tire")
        caster_wheel.visual(caster_wheel_meshes[i], material=hub_grey, name="rim")
        model.articulation(
            f"caster_{i}_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=caster_wheel,
            origin=Origin(xyz=(-0.095, 0.0, -0.102), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=22.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    rear_handle = object_model.get_part("rear_handle")
    front_handle = object_model.get_part("front_handle")

    for end_name, handle in (("rear", rear_handle), ("front", front_handle)):
        for i in range(2):
            ctx.allow_overlap(
                deck,
                handle,
                elem_a=f"{end_name}_hinge_socket_{i}",
                elem_b="pivot_bar",
                reason=(
                    "The folding handle pivot bar is intentionally captured inside "
                    "the deck-end hinge socket proxy so the handle stays clipped to the cart."
                ),
            )
            ctx.expect_overlap(
                deck,
                handle,
                axes="xyz",
                elem_a=f"{end_name}_hinge_socket_{i}",
                elem_b="pivot_bar",
                min_overlap=0.018,
                name=f"{end_name} hinge socket {i} captures pivot bar",
            )

    for i in range(2):
        rear_wheel = object_model.get_part(f"rear_wheel_{i}")
        ctx.allow_overlap(
            deck,
            rear_wheel,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The fixed rear axle is intentionally represented as seated through the wheel hub bore.",
        )
        ctx.expect_overlap(
            deck,
            rear_wheel,
            axes="xyz",
            elem_a="rear_axle",
            elem_b="rim",
            min_overlap=0.025,
            name=f"rear wheel {i} is retained on the axle",
        )

        caster = object_model.get_part(f"caster_{i}")
        caster_wheel = object_model.get_part(f"caster_wheel_{i}")
        ctx.allow_overlap(
            caster,
            caster_wheel,
            elem_a="axle_pin",
            elem_b="rim",
            reason="The caster axle pin is intentionally captured through the caster wheel hub bore.",
        )
        ctx.expect_overlap(
            caster,
            caster_wheel,
            axes="xyz",
            elem_a="axle_pin",
            elem_b="rim",
            min_overlap=0.012,
            name=f"caster wheel {i} is retained on its fork pin",
        )

    # The primary mechanisms should be represented explicitly.
    expected_joints = {
        "deck_to_rear_wheel_0": ArticulationType.CONTINUOUS,
        "deck_to_rear_wheel_1": ArticulationType.CONTINUOUS,
        "deck_to_caster_0": ArticulationType.CONTINUOUS,
        "deck_to_caster_1": ArticulationType.CONTINUOUS,
        "caster_0_to_wheel": ArticulationType.CONTINUOUS,
        "caster_1_to_wheel": ArticulationType.CONTINUOUS,
        "deck_to_rear_handle": ArticulationType.REVOLUTE,
        "deck_to_front_handle": ArticulationType.REVOLUTE,
    }
    for joint_name, joint_type in expected_joints.items():
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} has correct motion type",
            joint.articulation_type == joint_type,
            details=f"expected {joint_type}, found {joint.articulation_type}",
        )

    rear_hinge = object_model.get_articulation("deck_to_rear_handle")
    front_hinge = object_model.get_articulation("deck_to_front_handle")
    rear_rest = ctx.part_world_aabb(rear_handle)
    front_rest = ctx.part_world_aabb(front_handle)
    with ctx.pose({rear_hinge: rear_hinge.motion_limits.upper, front_hinge: front_hinge.motion_limits.upper}):
        rear_folded = ctx.part_world_aabb(rear_handle)
        front_folded = ctx.part_world_aabb(front_handle)

    ctx.check(
        "rear handle folds inward and down",
        rear_rest is not None
        and rear_folded is not None
        and rear_folded[1][2] < rear_rest[1][2] - 0.17
        and rear_folded[1][0] > rear_rest[1][0] + 0.35,
        details=f"rest={rear_rest}, folded={rear_folded}",
    )
    ctx.check(
        "front handle folds inward and down",
        front_rest is not None
        and front_folded is not None
        and front_folded[1][2] < front_rest[1][2] - 0.17
        and front_folded[0][0] < front_rest[0][0] - 0.35,
        details=f"rest={front_rest}, folded={front_folded}",
    )

    return ctx.report()


object_model = build_object_model()
