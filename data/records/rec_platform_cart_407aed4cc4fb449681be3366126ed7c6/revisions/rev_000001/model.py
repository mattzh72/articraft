from __future__ import annotations

import math

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
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_geometry,
)


FRONT_X = 1.0
DECK_LENGTH = 1.20
DECK_WIDTH = 0.72
DECK_THICKNESS = 0.12
DECK_CENTER_Z = 0.32
DECK_BOTTOM_Z = DECK_CENTER_Z - DECK_THICKNESS / 2.0
DECK_TOP_Z = DECK_CENTER_Z + DECK_THICKNESS / 2.0

CASTER_X = 0.43
CASTER_Y = 0.26
CASTER_JOINT_Z = DECK_BOTTOM_Z - 0.020
WHEEL_RADIUS = 0.095
WHEEL_WIDTH = 0.055
WHEEL_AXLE_LOCAL_Z = WHEEL_RADIUS - CASTER_JOINT_Z

TOW_HINGE_X = DECK_LENGTH / 2.0 + 0.060
TOW_HINGE_Z = 0.34
TOW_SWING_DOWN = math.pi / 2.0


def _caster_specs() -> tuple[tuple[str, float, float], ...]:
    return (
        ("front_caster_0", CASTER_X, -CASTER_Y),
        ("front_caster_1", CASTER_X, CASTER_Y),
        ("rear_caster_0", -CASTER_X, -CASTER_Y),
        ("rear_caster_1", -CASTER_X, CASTER_Y),
    )


def _add_caster_yoke_visuals(part, metal, dark_metal) -> None:
    """Add connected caster stem, bearing, fork, and axle visuals in caster local frame."""
    part.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=metal,
        name="swivel_stem",
    )
    part.visual(
        Cylinder(radius=0.055, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=dark_metal,
        name="swivel_bearing",
    )
    part.visual(
        Box((0.130, 0.130, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=metal,
        name="fork_crown",
    )
    for y, suffix in ((-0.046, "0"), (0.046, "1")):
        part.visual(
            Box((0.046, 0.012, 0.185)),
            origin=Origin(xyz=(0.0, y, -0.1175)),
            material=metal,
            name=f"fork_plate_{suffix}",
        )
    part.visual(
        Cylinder(radius=0.008, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_AXLE_LOCAL_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="axle_pin",
    )
    for y, suffix in ((-0.062, "0"), (0.062, "1")):
        part.visual(
            Cylinder(radius=0.015, length=0.014),
            origin=Origin(xyz=(0.0, y, WHEEL_AXLE_LOCAL_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"axle_cap_{suffix}",
        )


def _aabb_center(aabb):
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_industrial_platform_cart")

    deck_paint = model.material("safety_yellow_paint", rgba=(0.95, 0.66, 0.10, 1.0))
    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    red_tow = model.material("red_tow_bar_paint", rgba=(0.82, 0.08, 0.04, 1.0))
    handle_grip = model.material("black_handle_grip", rgba=(0.02, 0.025, 0.025, 1.0))

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.055,
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.58),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "caster_tire",
    )
    tow_eye_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.070, tube=0.012, radial_segments=16, tubular_segments=56),
        "tow_eye_ring",
    )

    deck = model.part("deck")
    deck.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=deck_paint,
        name="heavy_deck",
    )
    deck.visual(
        Box((DECK_LENGTH + 0.040, 0.040, 0.080)),
        origin=Origin(xyz=(0.0, DECK_WIDTH / 2.0, DECK_CENTER_Z + 0.005)),
        material=dark_steel,
        name="side_rub_rail_0",
    )
    deck.visual(
        Box((DECK_LENGTH + 0.040, 0.040, 0.080)),
        origin=Origin(xyz=(0.0, -DECK_WIDTH / 2.0, DECK_CENTER_Z + 0.005)),
        material=dark_steel,
        name="side_rub_rail_1",
    )
    deck.visual(
        Box((0.040, DECK_WIDTH + 0.040, 0.080)),
        origin=Origin(xyz=(-DECK_LENGTH / 2.0, 0.0, DECK_CENTER_Z + 0.005)),
        material=dark_steel,
        name="rear_rub_rail",
    )
    deck.visual(
        Box((0.040, DECK_WIDTH + 0.040, 0.080)),
        origin=Origin(xyz=(DECK_LENGTH / 2.0, 0.0, DECK_CENTER_Z + 0.005)),
        material=dark_steel,
        name="front_rub_rail",
    )
    for y, suffix in ((-0.235, "0"), (0.0, "1"), (0.235, "2")):
        deck.visual(
            Box((DECK_LENGTH - 0.120, 0.018, 0.006)),
            origin=Origin(xyz=(0.0, y, DECK_TOP_Z + 0.003)),
            material=dark_steel,
            name=f"anti_slip_strip_{suffix}",
        )
    for x, label in ((CASTER_X, "front"), (-CASTER_X, "rear")):
        deck.visual(
            Box((0.060, DECK_WIDTH - 0.100, 0.050)),
            origin=Origin(xyz=(x, 0.0, DECK_BOTTOM_Z + 0.015)),
            material=dark_steel,
            name=f"{label}_cross_member",
        )
    for label, x, y in _caster_specs():
        plate_name = label.replace("caster", "mount")
        deck.visual(
            Box((0.180, 0.160, 0.020)),
            origin=Origin(xyz=(x, y, DECK_BOTTOM_Z - 0.010)),
            material=galvanized,
            name=plate_name,
        )
        for bx in (-0.055, 0.055):
            for by in (-0.045, 0.045):
                deck.visual(
                    Cylinder(radius=0.008, length=0.006),
                    origin=Origin(xyz=(x + bx, y + by, DECK_BOTTOM_Z - 0.023)),
                    material=dark_steel,
                    name=f"{plate_name}_bolt_{0 if bx < 0 else 1}_{0 if by < 0 else 1}",
                )
    deck.visual(
        Box((0.050, 0.480, 0.065)),
        origin=Origin(xyz=(DECK_LENGTH / 2.0 + 0.005, 0.0, TOW_HINGE_Z)),
        material=galvanized,
        name="tow_hinge_backplate",
    )
    for y, suffix in ((-0.205, "0"), (0.205, "1")):
        deck.visual(
            Box((0.085, 0.025, 0.095)),
            origin=Origin(xyz=(TOW_HINGE_X - 0.010, y, TOW_HINGE_Z)),
            material=galvanized,
            name=f"tow_hinge_cheek_{suffix}",
        )
    deck.visual(
        Cylinder(radius=0.012, length=0.460),
        origin=Origin(xyz=(TOW_HINGE_X, 0.0, TOW_HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )

    push_handle = model.part("push_handle")
    for y, suffix in ((-0.280, "0"), (0.280, "1")):
        push_handle.visual(
            Box((0.120, 0.075, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.005)),
            material=galvanized,
            name=f"mount_foot_{suffix}",
        )
        push_handle.visual(
            Cylinder(radius=0.018, length=0.570),
            origin=Origin(xyz=(0.0, y, 0.285)),
            material=dark_steel,
            name=f"upright_tube_{suffix}",
        )
        push_handle.visual(
            Box((0.035, 0.035, 0.260)),
            origin=Origin(xyz=(-0.035, y, 0.180), rpy=(0.0, 0.300, 0.0)),
            material=dark_steel,
            name=f"diagonal_brace_{suffix}",
        )
    push_handle.visual(
        Cylinder(radius=0.020, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.570), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_grip,
        name="top_grip",
    )
    push_handle.visual(
        Box((0.050, 0.620, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=dark_steel,
        name="upper_crossbar",
    )
    model.articulation(
        "deck_to_push_handle",
        ArticulationType.FIXED,
        parent=deck,
        child=push_handle,
        origin=Origin(xyz=(-DECK_LENGTH / 2.0 + 0.065, 0.0, DECK_TOP_Z)),
    )

    tow_bar = model.part("tow_bar")
    tow_bar.visual(
        Cylinder(radius=0.023, length=0.340),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=red_tow,
        name="hinge_barrel",
    )
    tow_bar.visual(
        Box((0.064, 0.090, 0.045)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=red_tow,
        name="clevis_neck",
    )
    tow_bar.visual(
        Box((0.055, 0.045, 0.520)),
        origin=Origin(xyz=(0.075, 0.0, 0.2825)),
        material=red_tow,
        name="drawbar_tube",
    )
    tow_bar.visual(
        Box((0.050, 0.045, 0.080)),
        origin=Origin(xyz=(0.075, 0.0, 0.5525)),
        material=red_tow,
        name="eye_socket",
    )
    tow_bar.visual(
        tow_eye_mesh,
        origin=Origin(xyz=(0.075, 0.0, 0.635), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=red_tow,
        name="tow_eye",
    )
    model.articulation(
        "tow_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=tow_bar,
        origin=Origin(xyz=(TOW_HINGE_X, 0.0, TOW_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=TOW_SWING_DOWN),
    )

    for caster_name, x, y in _caster_specs():
        caster = model.part(caster_name)
        _add_caster_yoke_visuals(caster, galvanized, dark_steel)
        model.articulation(
            f"{caster_name}_swivel",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=caster,
            origin=Origin(xyz=(x, y, CASTER_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=6.0),
        )

        wheel_name = caster_name.replace("caster", "wheel")
        wheel = model.part(wheel_name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.057, length=WHEEL_WIDTH * 0.86),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name="wheel_hub",
        )
        wheel.visual(
            Cylinder(radius=0.026, length=WHEEL_WIDTH * 1.04),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="hub_boss",
        )
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, WHEEL_AXLE_LOCAL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    tow_bar = object_model.get_part("tow_bar")
    tow_hinge = object_model.get_articulation("tow_hinge")

    ctx.allow_overlap(
        deck,
        tow_bar,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The tow-bar hinge barrel is intentionally captured around the front hinge pin so the bar stays clipped on while swinging.",
    )
    ctx.expect_overlap(
        deck,
        tow_bar,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.30,
        name="tow hinge pin runs through the barrel",
    )
    ctx.expect_within(
        deck,
        tow_bar,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.002,
        name="hinge pin is radially inside tow-bar barrel",
    )

    rest_eye_aabb = ctx.part_element_world_aabb(tow_bar, elem="tow_eye")
    with ctx.pose({tow_hinge: TOW_SWING_DOWN}):
        towing_eye_aabb = ctx.part_element_world_aabb(tow_bar, elem="tow_eye")
    rest_eye = _aabb_center(rest_eye_aabb) if rest_eye_aabb is not None else None
    towing_eye = _aabb_center(towing_eye_aabb) if towing_eye_aabb is not None else None
    ctx.check(
        "tow bar swings down into towing position",
        rest_eye is not None
        and towing_eye is not None
        and towing_eye[0] > rest_eye[0] + 0.38
        and towing_eye[2] < rest_eye[2] - 0.45,
        details=f"rest_eye={rest_eye}, towing_eye={towing_eye}",
    )

    for caster_name, _x, _y in _caster_specs():
        mount_name = caster_name.replace("caster", "mount")
        caster = object_model.get_part(caster_name)
        wheel = object_model.get_part(caster_name.replace("caster", "wheel"))
        swivel = object_model.get_articulation(f"{caster_name}_swivel")
        spin = object_model.get_articulation(f"{caster_name.replace('caster', 'wheel')}_spin")

        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="axle_pin",
            elem_b="wheel_hub",
            reason="The caster axle pin intentionally passes through the wheel hub as the wheel's rolling bearing.",
        )
        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="axle_pin",
            elem_b="hub_boss",
            reason="The axle pin continues through the raised hub boss that represents the wheel bearing cap.",
        )

        ctx.check(
            f"{caster_name} has a vertical swivel",
            swivel.axis == (0.0, 0.0, 1.0),
            details=f"axis={swivel.axis}",
        )
        ctx.check(
            f"{wheel.name} spins on a lateral axle",
            spin.axis == (0.0, 1.0, 0.0),
            details=f"axis={spin.axis}",
        )
        ctx.expect_gap(
            deck,
            caster,
            axis="z",
            positive_elem=mount_name,
            negative_elem="swivel_stem",
            max_gap=0.001,
            max_penetration=0.000001,
            name=f"{caster_name} stem seats under its mount plate",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="y",
            elem_a="axle_pin",
            elem_b="wheel_hub",
            min_overlap=0.045,
            name=f"{wheel.name} hub is retained on caster axle",
        )
        ctx.expect_within(
            caster,
            wheel,
            axes="xz",
            inner_elem="axle_pin",
            outer_elem="wheel_hub",
            margin=0.004,
            name=f"{wheel.name} axle is centered in the wheel hub",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="y",
            elem_a="axle_pin",
            elem_b="hub_boss",
            min_overlap=0.050,
            name=f"{wheel.name} axle passes through hub boss",
        )
        ctx.expect_overlap(
            wheel,
            caster,
            axes="z",
            elem_a="tire",
            elem_b="fork_plate_0",
            min_overlap=0.050,
            name=f"{wheel.name} sits between caster fork plates",
        )

    return ctx.report()


object_model = build_object_model()
