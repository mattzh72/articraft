from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="low_stock_cart")

    deck_blue = model.material("powder_coated_blue", rgba=(0.08, 0.22, 0.38, 1.0))
    lip_yellow = model.material("safety_yellow", rgba=(0.95, 0.70, 0.10, 1.0))
    steel = model.material("zinc_plated_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    rim_gray = model.material("pressed_gray_rim", rgba=(0.72, 0.73, 0.70, 1.0))
    white_mark = model.material("white_tread_mark", rgba=(0.92, 0.90, 0.82, 1.0))

    deck_length = 1.10
    deck_width = 0.62
    deck_thickness = 0.050
    deck_bottom_z = 0.140
    deck_top_z = deck_bottom_z + deck_thickness

    deck = model.part("deck")
    deck.visual(
        Box((deck_length, deck_width, deck_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_bottom_z + deck_thickness / 2.0)),
        material=deck_blue,
        name="deck_slab",
    )
    deck.visual(
        Box((deck_length - 0.08, deck_width - 0.08, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, deck_top_z + 0.003)),
        material=dark_steel,
        name="rubber_tread_mat",
    )
    # Rolled steel edge frame, slightly proud of the wooden/steel deck slab.
    rail_z = deck_bottom_z + 0.018
    deck.visual(
        Box((deck_length + 0.035, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, deck_width / 2.0 + 0.0175, rail_z)),
        material=dark_steel,
        name="hinge_side_rail",
    )
    deck.visual(
        Box((deck_length + 0.035, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, -deck_width / 2.0 - 0.0175, rail_z)),
        material=dark_steel,
        name="side_rail",
    )
    deck.visual(
        Box((0.035, deck_width + 0.070, 0.040)),
        origin=Origin(xyz=(deck_length / 2.0 + 0.0175, 0.0, rail_z)),
        material=dark_steel,
        name="end_rail_0",
    )
    deck.visual(
        Box((0.035, deck_width + 0.070, 0.040)),
        origin=Origin(xyz=(-deck_length / 2.0 - 0.0175, 0.0, rail_z)),
        material=dark_steel,
        name="end_rail_1",
    )

    caster_xy = (
        (deck_length / 2.0 - 0.115, deck_width / 2.0 - 0.095),
        (-deck_length / 2.0 + 0.115, deck_width / 2.0 - 0.095),
        (-deck_length / 2.0 + 0.115, -deck_width / 2.0 + 0.095),
        (deck_length / 2.0 - 0.115, -deck_width / 2.0 + 0.095),
    )
    mount_thickness = 0.012
    mount_bottom_z = deck_bottom_z - mount_thickness
    for idx, (x, y) in enumerate(caster_xy):
        deck.visual(
            Box((0.135, 0.115, mount_thickness)),
            origin=Origin(xyz=(x, y, mount_bottom_z + mount_thickness / 2.0)),
            material=steel,
            name=f"caster_mount_{idx}",
        )
        deck.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(xyz=(x, y, mount_bottom_z + 0.009)),
            material=steel,
            name=f"swivel_socket_{idx}",
        )

    # Alternating hinge knuckles fixed to the deck side; the lip owns the
    # interleaved knuckles and rotates about the same long-side axis.
    hinge_y = deck_width / 2.0 + 0.020
    hinge_z = deck_top_z
    for idx, (x, length) in enumerate(((-0.380, 0.160), (0.0, 0.210), (0.380, 0.160))):
        deck.visual(
            Cylinder(radius=0.012, length=length),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"fixed_knuckle_{idx}",
        )
    deck.visual(
        Box((deck_length - 0.08, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, hinge_y - 0.010, hinge_z - 0.010)),
        material=steel,
        name="hinge_leaf",
    )

    lip = model.part("lip")
    lip.visual(
        Box((deck_length - 0.060, 0.032, 0.185)),
        origin=Origin(xyz=(0.0, 0.025, 0.0925)),
        material=lip_yellow,
        name="lip_panel",
    )
    lip.visual(
        Cylinder(radius=0.013, length=deck_length - 0.060),
        origin=Origin(xyz=(0.0, 0.025, 0.185), rpy=(0.0, pi / 2.0, 0.0)),
        material=lip_yellow,
        name="top_roll",
    )
    lip.visual(
        Box((0.020, 0.040, 0.185)),
        origin=Origin(xyz=(deck_length / 2.0 - 0.040, 0.025, 0.0925)),
        material=lip_yellow,
        name="end_stiffener_0",
    )
    lip.visual(
        Box((0.020, 0.040, 0.185)),
        origin=Origin(xyz=(-deck_length / 2.0 + 0.040, 0.025, 0.0925)),
        material=lip_yellow,
        name="end_stiffener_1",
    )
    for idx, (x, length) in enumerate(((-0.205, 0.170), (0.205, 0.170))):
        lip.visual(
            Cylinder(radius=0.012, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"lip_knuckle_{idx}",
        )

    model.articulation(
        "lip_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-1.57, upper=0.0),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.052,
            inner_radius=0.034,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.004, count=16, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.003),
        ),
        "caster_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.037,
            0.054,
            rim=WheelRim(inner_radius=0.026, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.014,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.020, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "caster_rim",
    )

    axle_y = -0.080
    axle_z = -0.073
    for idx, (x, y) in enumerate(caster_xy):
        fork = model.part(f"caster_fork_{idx}")
        fork.visual(
            Box((0.120, 0.105, 0.012)),
            origin=Origin(xyz=(0.0, -0.018, -0.006)),
            material=steel,
            name="swivel_plate",
        )
        fork.visual(
            Cylinder(radius=0.018, length=0.048),
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
            material=steel,
            name="kingpin_barrel",
        )
        fork.visual(
            Box((0.008, 0.120, 0.105)),
            origin=Origin(xyz=(0.037, -0.065, -0.065)),
            material=steel,
            name="fork_cheek_0",
        )
        fork.visual(
            Box((0.008, 0.120, 0.105)),
            origin=Origin(xyz=(-0.037, -0.065, -0.065)),
            material=steel,
            name="fork_cheek_1",
        )
        fork.visual(
            Box((0.074, 0.014, 0.018)),
            origin=Origin(xyz=(0.0, -0.003, -0.030)),
            material=steel,
            name="fork_bridge",
        )
        fork.visual(
            Cylinder(radius=0.0095, length=0.096),
            origin=Origin(xyz=(0.0, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="axle_pin",
        )
        fork.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(0.051, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="axle_cap_0",
        )
        fork.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(-0.051, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="axle_cap_1",
        )

        wheel = model.part(f"wheel_{idx}")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(wheel_mesh, material=rim_gray, name="rim")
        wheel.visual(
            Box((0.056, 0.012, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, 0.056)),
            material=white_mark,
            name="tread_mark",
        )

        model.articulation(
            f"caster_swivel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(x, y, mount_bottom_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=3.0),
        )
        model.articulation(
            f"wheel_axle_{idx}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, axle_y, axle_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    lip_hinge = object_model.get_articulation("lip_hinge")

    ctx.check(
        "lip hinge folds down from upright retaining position",
        lip_hinge.motion_limits is not None
        and lip_hinge.motion_limits.lower <= -1.5
        and lip_hinge.motion_limits.upper == 0.0
        and tuple(lip_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={lip_hinge.axis}, limits={lip_hinge.motion_limits}",
    )

    deck_box = ctx.part_world_aabb(deck)
    lip_box = ctx.part_world_aabb(lip)
    ctx.check(
        "retaining lip is upright above the low deck",
        deck_box is not None
        and lip_box is not None
        and lip_box[1][2] > deck_box[1][2] + 0.16,
        details=f"deck_aabb={deck_box}, lip_aabb={lip_box}",
    )

    upright_box = lip_box
    with ctx.pose({lip_hinge: -1.45}):
        folded_box = ctx.part_world_aabb(lip)
    ctx.check(
        "lip folds outward and downward",
        upright_box is not None
        and folded_box is not None
        and folded_box[1][1] > upright_box[1][1] + 0.09
        and folded_box[1][2] < upright_box[1][2] - 0.08,
        details=f"upright={upright_box}, folded={folded_box}",
    )

    ctx.check(
        "four caster swivel joints and four wheel axles",
        all(object_model.get_articulation(f"caster_swivel_{i}") is not None for i in range(4))
        and all(object_model.get_articulation(f"wheel_axle_{i}") is not None for i in range(4)),
    )

    for idx in range(4):
        fork = object_model.get_part(f"caster_fork_{idx}")
        wheel = object_model.get_part(f"wheel_{idx}")
        swivel = object_model.get_articulation(f"caster_swivel_{idx}")
        axle = object_model.get_articulation(f"wheel_axle_{idx}")

        ctx.allow_overlap(
            fork,
            wheel,
            elem_a="axle_pin",
            elem_b="rim",
            reason="The stationary caster axle is intentionally captured through the wheel hub/bearing.",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="x",
            elem_a="axle_pin",
            elem_b="rim",
            min_overlap=0.045,
            name=f"caster {idx} axle passes through hub",
        )
        ctx.expect_within(
            wheel,
            fork,
            axes="x",
            inner_elem="tire",
            outer_elem="axle_pin",
            margin=0.002,
            name=f"caster {idx} wheel is centered between fork cheeks",
        )
        ctx.expect_contact(
            deck,
            fork,
            elem_a=f"caster_mount_{idx}",
            elem_b="swivel_plate",
            contact_tol=0.0005,
            name=f"caster {idx} fork plate seats under deck mount",
        )
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            positive_elem=f"caster_mount_{idx}",
            negative_elem="tire",
            min_gap=0.010,
            max_gap=0.030,
            name=f"caster {idx} wheel clears underside of platform",
        )
        ctx.check(
            f"caster {idx} swivel is vertical and wheel axle is transverse",
            tuple(swivel.axis) == (0.0, 0.0, 1.0) and tuple(axle.axis) == (1.0, 0.0, 0.0),
            details=f"swivel_axis={swivel.axis}, axle_axis={axle.axis}",
        )

    wheel_0 = object_model.get_part("wheel_0")
    swivel_0 = object_model.get_articulation("caster_swivel_0")
    axle_0 = object_model.get_articulation("wheel_axle_0")
    rest_wheel_pos = ctx.part_world_position(wheel_0)
    with ctx.pose({swivel_0: pi / 2.0}):
        swiveled_wheel_pos = ctx.part_world_position(wheel_0)
    ctx.check(
        "caster swivel swings trailing wheel around vertical pivot",
        rest_wheel_pos is not None
        and swiveled_wheel_pos is not None
        and abs(swiveled_wheel_pos[0] - rest_wheel_pos[0]) > 0.06
        and abs(swiveled_wheel_pos[1] - rest_wheel_pos[1]) > 0.06,
        details=f"rest={rest_wheel_pos}, swiveled={swiveled_wheel_pos}",
    )

    mark_rest = aabb_center(ctx.part_element_world_aabb(wheel_0, elem="tread_mark"))
    with ctx.pose({axle_0: pi / 2.0}):
        mark_spun = aabb_center(ctx.part_element_world_aabb(wheel_0, elem="tread_mark"))
    ctx.check(
        "wheel axle spin moves the visible tread mark",
        mark_rest is not None
        and mark_spun is not None
        and mark_spun[2] < mark_rest[2] - 0.025
        and abs(mark_spun[1] - mark_rest[1]) > 0.025,
        details=f"rest_mark={mark_rest}, spun_mark={mark_spun}",
    )

    return ctx.report()


object_model = build_object_model()
