from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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


DECK_LENGTH = 1.15
DECK_WIDTH = 0.62
DECK_THICKNESS = 0.065
DECK_CENTER_Z = 0.2425
DECK_TOP_Z = DECK_CENTER_Z + DECK_THICKNESS / 2.0

MOUNT_Z = 0.205
MOUNT_THICKNESS = 0.014
CASTER_PIVOT_Z = MOUNT_Z - MOUNT_THICKNESS / 2.0

WHEEL_RADIUS = 0.075
WHEEL_WIDTH = 0.044
WHEEL_CENTER_Z_LOCAL = -0.123
WHEEL_TRAIL_Y = -0.035

CASTER_X = 0.46
CASTER_Y = 0.23
CASTER_POSITIONS = (
    (-CASTER_X, -CASTER_Y),
    (-CASTER_X, CASTER_Y),
    (CASTER_X, -CASTER_Y),
    (CASTER_X, CASTER_Y),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="warehouse_platform_cart")

    deck_mat = model.material("powder_coated_gray_deck", color=(0.48, 0.50, 0.51, 1.0))
    blue_mat = model.material("blue_powder_coated_handle", color=(0.02, 0.18, 0.58, 1.0))
    black_mat = model.material("black_rubber", color=(0.01, 0.01, 0.01, 1.0))
    tread_mat = model.material("black_non_slip_tread", color=(0.025, 0.025, 0.022, 1.0))
    steel_mat = model.material("zinc_plated_steel", color=(0.72, 0.72, 0.68, 1.0))
    bolt_mat = model.material("dark_bolt_heads", color=(0.12, 0.12, 0.12, 1.0))

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.049,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.035),
            tread=TireTread(style="block", depth=0.0035, count=18, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.004, radius=0.0025),
        ),
        "caster_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.050,
            0.038,
            rim=WheelRim(
                inner_radius=0.030,
                flange_height=0.004,
                flange_thickness=0.003,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.020,
                width=0.040,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.0035),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0035, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        "caster_wheel_core",
    )

    deck = model.part("deck")
    deck.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=deck_mat,
        name="flat_deck",
    )
    deck.visual(
        Box((DECK_LENGTH + 0.035, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, DECK_WIDTH / 2.0 + 0.013, DECK_CENTER_Z)),
        material=black_mat,
        name="side_bumper_0",
    )
    deck.visual(
        Box((DECK_LENGTH + 0.035, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, -DECK_WIDTH / 2.0 - 0.013, DECK_CENTER_Z)),
        material=black_mat,
        name="side_bumper_1",
    )
    deck.visual(
        Box((0.030, DECK_WIDTH, 0.060)),
        origin=Origin(xyz=(DECK_LENGTH / 2.0 + 0.013, 0.0, DECK_CENTER_Z)),
        material=black_mat,
        name="end_bumper_0",
    )
    deck.visual(
        Box((0.030, DECK_WIDTH, 0.060)),
        origin=Origin(xyz=(-DECK_LENGTH / 2.0 - 0.013, 0.0, DECK_CENTER_Z)),
        material=black_mat,
        name="end_bumper_1",
    )

    for strip_index, y in enumerate((-0.210, -0.126, -0.042, 0.042, 0.126, 0.210)):
        deck.visual(
            Box((0.90, 0.018, 0.004)),
            origin=Origin(xyz=(0.02, y, DECK_TOP_Z + 0.001)),
            material=tread_mat,
            name=f"tread_strip_{strip_index}",
        )

    for mount_index, (x, y) in enumerate(CASTER_POSITIONS):
        deck.visual(
            Box((0.165, 0.125, MOUNT_THICKNESS)),
            origin=Origin(xyz=(x, y, MOUNT_Z)),
            material=steel_mat,
            name=f"mount_plate_{mount_index}",
        )
        for bx in (-0.052, 0.052):
            for by in (-0.037, 0.037):
                deck.visual(
                    Cylinder(radius=0.009, length=0.006),
                    origin=Origin(xyz=(x + bx, y + by, MOUNT_Z - MOUNT_THICKNESS / 2.0 - 0.003)),
                    material=bolt_mat,
                    name=f"mount_bolt_{mount_index}_{bx:+.0e}_{by:+.0e}",
                )

    handle_x = -DECK_LENGTH / 2.0 + 0.085
    post_y = 0.245
    handle_top_z = 0.975
    post_height = handle_top_z - DECK_TOP_Z
    for side_index, y in enumerate((-post_y, post_y)):
        deck.visual(
            Box((0.105, 0.060, 0.014)),
            origin=Origin(xyz=(handle_x, y, DECK_TOP_Z + 0.006)),
            material=blue_mat,
            name=f"handle_foot_{side_index}",
        )
        deck.visual(
            Cylinder(radius=0.018, length=post_height),
            origin=Origin(xyz=(handle_x, y, DECK_TOP_Z + post_height / 2.0)),
            material=blue_mat,
            name=f"handle_post_{side_index}",
        )
    deck.visual(
        Cylinder(radius=0.019, length=post_y * 2.0),
        origin=Origin(xyz=(handle_x, 0.0, handle_top_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blue_mat,
        name="handle_crossbar",
    )
    deck.visual(
        Cylinder(radius=0.023, length=0.36),
        origin=Origin(xyz=(handle_x, 0.0, handle_top_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_mat,
        name="rubber_grip",
    )
    for side_index, y in enumerate((-post_y, post_y)):
        deck.visual(
            Cylinder(radius=0.009, length=0.25),
            origin=Origin(
                xyz=(handle_x + 0.040, y * 0.94, DECK_TOP_Z + 0.12),
                rpy=(0.0, pi / 5.0, 0.0),
            ),
            material=blue_mat,
            name=f"handle_gusset_{side_index}",
        )

    for caster_index, (x, y) in enumerate(CASTER_POSITIONS):
        caster = model.part(f"caster_{caster_index}")
        caster.visual(
            Cylinder(radius=0.044, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=steel_mat,
            name="swivel_race",
        )
        caster.visual(
            Cylinder(radius=0.016, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=steel_mat,
            name="swivel_stem",
        )
        caster.visual(
            Box((0.090, 0.100, 0.014)),
            origin=Origin(xyz=(0.0, WHEEL_TRAIL_Y / 2.0, -0.036)),
            material=steel_mat,
            name="fork_crown",
        )
        for side, sx in enumerate((-0.035, 0.035)):
            caster.visual(
                Box((0.012, 0.060, 0.142)),
                origin=Origin(xyz=(sx, WHEEL_TRAIL_Y, -0.110)),
                material=steel_mat,
                name=f"fork_cheek_{side}",
            )
            caster.visual(
                Cylinder(radius=0.018, length=0.010),
                origin=Origin(
                    xyz=(sx * 1.19, WHEEL_TRAIL_Y, WHEEL_CENTER_Z_LOCAL),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=bolt_mat,
                name=f"axle_cap_{side}",
            )
        caster.visual(
            Cylinder(radius=0.008, length=0.078),
            origin=Origin(xyz=(0.0, WHEEL_TRAIL_Y, WHEEL_CENTER_Z_LOCAL), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_mat,
            name="axle_pin",
        )

        wheel = model.part(f"wheel_{caster_index}")
        wheel.visual(tire_mesh, material=black_mat, name="tire")
        wheel.visual(wheel_mesh, material=steel_mat, name="rim")

        model.articulation(
            f"deck_to_caster_{caster_index}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=caster,
            origin=Origin(xyz=(x, y, CASTER_PIVOT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=5.0, lower=-pi, upper=pi),
        )
        model.articulation(
            f"caster_to_wheel_{caster_index}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, WHEEL_TRAIL_Y, WHEEL_CENTER_Z_LOCAL)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    caster_joints = [object_model.get_articulation(f"deck_to_caster_{i}") for i in range(4)]
    wheel_joints = [object_model.get_articulation(f"caster_to_wheel_{i}") for i in range(4)]

    ctx.check(
        "four swivel forks and four rolling wheels",
        len(caster_joints) == 4 and len(wheel_joints) == 4,
        details=f"casters={len(caster_joints)}, wheels={len(wheel_joints)}",
    )
    for caster_index in range(4):
        caster = object_model.get_part(f"caster_{caster_index}")
        wheel = object_model.get_part(f"wheel_{caster_index}")
        swivel = caster_joints[caster_index]
        axle = wheel_joints[caster_index]
        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="axle_pin",
            elem_b="rim",
            reason="The fixed caster axle is intentionally captured through the wheel hub bore so the wheel has a physical support path while rotating.",
        )
        ctx.check(
            f"caster {caster_index} has vertical swivel axis",
            tuple(round(v, 6) for v in swivel.axis) == (0.0, 0.0, 1.0),
            details=f"axis={swivel.axis}",
        )
        ctx.check(
            f"wheel {caster_index} spins on transverse axle",
            tuple(round(v, 6) for v in axle.axis) == (1.0, 0.0, 0.0),
            details=f"axis={axle.axis}",
        )
        ctx.expect_contact(
            deck,
            caster,
            elem_a=f"mount_plate_{caster_index}",
            elem_b="swivel_race",
            contact_tol=0.001,
            name=f"caster {caster_index} race seats under mount plate",
        )
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            min_gap=0.020,
            name=f"wheel {caster_index} clears underside of deck",
        )
        ctx.expect_within(
            wheel,
            caster,
            axes="x",
            inner_elem="tire",
            outer_elem="fork_crown",
            margin=0.006,
            name=f"wheel {caster_index} sits between fork cheeks",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="x",
            elem_a="axle_pin",
            elem_b="rim",
            min_overlap=0.035,
            name=f"axle {caster_index} remains captured through wheel hub",
        )
        ctx.expect_within(
            caster,
            wheel,
            axes="yz",
            inner_elem="axle_pin",
            outer_elem="rim",
            margin=0.002,
            name=f"axle {caster_index} is centered in the wheel hub",
        )

    wheel_0 = object_model.get_part("wheel_0")
    rest_pos = ctx.part_world_position(wheel_0)
    with ctx.pose({"deck_to_caster_0": pi / 2.0}):
        swivel_pos = ctx.part_world_position(wheel_0)
    with ctx.pose({"caster_to_wheel_0": 1.25}):
        spin_pos = ctx.part_world_position(wheel_0)
    ctx.check(
        "caster swivel carries the trailed wheel around the vertical pivot",
        rest_pos is not None
        and swivel_pos is not None
        and abs(swivel_pos[0] - rest_pos[0]) > 0.025
        and abs(swivel_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, swivel={swivel_pos}",
    )
    ctx.check(
        "wheel spin keeps axle center fixed",
        rest_pos is not None
        and spin_pos is not None
        and max(abs(spin_pos[i] - rest_pos[i]) for i in range(3)) < 0.001,
        details=f"rest={rest_pos}, spin={spin_pos}",
    )

    return ctx.report()


object_model = build_object_model()
