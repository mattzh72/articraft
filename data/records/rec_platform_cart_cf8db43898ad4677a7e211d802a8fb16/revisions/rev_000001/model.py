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
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
)


DECK_LENGTH = 1.20
DECK_WIDTH = 0.66
DECK_THICKNESS = 0.070
HINGE_Y = DECK_WIDTH / 2.0 + 0.035
HINGE_Z = DECK_THICKNESS / 2.0 + 0.020
RAIL_HEIGHT = 0.420
CASTER_MOUNT_Z = -DECK_THICKNESS / 2.0 - 0.009
CASTER_JOINT_Z = CASTER_MOUNT_Z - 0.009
WHEEL_CENTER_Z = -0.175


def _cyl_x_origin(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder local +Z turned into local/world +X."""
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y_origin(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder local +Z turned into local/world +Y."""
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platform_cart_fold_down_rails")

    deck_blue = model.material("powder_coated_blue", rgba=(0.06, 0.16, 0.32, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.08, 1.0))
    zinc = model.material("zinc_plated_steel", rgba=(0.66, 0.67, 0.63, 1.0))
    dark_steel = model.material("dark_galvanized_steel", rgba=(0.18, 0.19, 0.18, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    grip = model.material("ribbed_black_grip", rgba=(0.02, 0.025, 0.025, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(),
        material=deck_blue,
        name="deck_plate",
    )

    # Raised rubber anti-slip ribs on the loading deck.
    for rib_i, y in enumerate((-0.24, -0.16, -0.08, 0.0, 0.08, 0.16, 0.24)):
        deck.visual(
            Box((DECK_LENGTH - 0.16, 0.018, 0.004)),
            origin=Origin(xyz=(0.025, y, DECK_THICKNESS / 2.0 + 0.002)),
            material=grip,
            name=f"deck_grip_rib_{rib_i}",
        )

    # Low boxed edge lips keep cargo on the deck while leaving hinge barrels exposed.
    deck.visual(
        Box((DECK_LENGTH, 0.020, 0.038)),
        origin=Origin(xyz=(0.0, DECK_WIDTH / 2.0 - 0.012, 0.017)),
        material=dark_steel,
        name="side_lip_0",
    )
    deck.visual(
        Box((DECK_LENGTH, 0.020, 0.038)),
        origin=Origin(xyz=(0.0, -DECK_WIDTH / 2.0 + 0.012, 0.017)),
        material=dark_steel,
        name="side_lip_1",
    )
    deck.visual(
        Box((0.030, DECK_WIDTH, 0.045)),
        origin=Origin(xyz=(DECK_LENGTH / 2.0 - 0.015, 0.0, 0.020)),
        material=dark_steel,
        name="front_lip",
    )
    deck.visual(
        Box((0.030, DECK_WIDTH, 0.045)),
        origin=Origin(xyz=(-DECK_LENGTH / 2.0 + 0.015, 0.0, 0.020)),
        material=dark_steel,
        name="handle_end_lip",
    )

    # Fixed push handle: welded tubular U frame at one end of the cart.
    handle_x = -DECK_LENGTH / 2.0 + 0.055
    for post_i, y in enumerate((-0.245, 0.245)):
        deck.visual(
            Cylinder(radius=0.023, length=0.790),
            origin=Origin(xyz=(handle_x, y, DECK_THICKNESS / 2.0 + 0.395)),
            material=zinc,
            name=f"handle_post_{post_i}",
        )
        deck.visual(
            Cylinder(radius=0.035, length=0.015),
            origin=Origin(xyz=(handle_x, y, DECK_THICKNESS / 2.0 + 0.007)),
            material=dark_steel,
            name=f"handle_base_collar_{post_i}",
        )
    deck.visual(
        Cylinder(radius=0.026, length=0.540),
        origin=_cyl_y_origin((handle_x, 0.0, DECK_THICKNESS / 2.0 + 0.790)),
        material=zinc,
        name="handle_crossbar",
    )
    deck.visual(
        Cylinder(radius=0.031, length=0.300),
        origin=_cyl_y_origin((handle_x, 0.0, DECK_THICKNESS / 2.0 + 0.790)),
        material=grip,
        name="handle_hand_grip",
    )

    # Alternating fixed hinge knuckles and support straps along both deck edges.
    deck_knuckle_centers = (-0.450, 0.0, 0.450)
    for side_i, side in enumerate((1.0, -1.0)):
        deck.visual(
            Box((DECK_LENGTH - 0.08, 0.026, 0.030)),
            origin=Origin(xyz=(0.0, side * (DECK_WIDTH / 2.0 + 0.006), HINGE_Z - 0.007)),
            material=dark_steel,
            name=f"hinge_leaf_{side_i}",
        )
        for k_i, x in enumerate(deck_knuckle_centers):
            deck.visual(
                Box((0.180, 0.030, 0.027)),
                origin=Origin(xyz=(x, side * (DECK_WIDTH / 2.0 + 0.021), HINGE_Z - 0.006)),
                material=dark_steel,
                name=f"hinge_bridge_{side_i}_{k_i}",
            )
            deck.visual(
                Cylinder(radius=0.017, length=0.180),
                origin=_cyl_x_origin((x, side * HINGE_Y, HINGE_Z)),
                material=zinc,
                name=f"fixed_hinge_knuckle_{side_i}_{k_i}",
            )

    # Caster mounting plates and exposed underside bolt heads.
    caster_positions = (
        (0.450, 0.245),
        (0.450, -0.245),
        (-0.450, 0.245),
        (-0.450, -0.245),
    )
    for c_i, (x, y) in enumerate(caster_positions):
        deck.visual(
            Box((0.170, 0.130, 0.018)),
            origin=Origin(xyz=(x, y, CASTER_MOUNT_Z)),
            material=dark_steel,
            name=f"caster_mount_{c_i}",
        )
        for b_i, (bx, by) in enumerate(((-0.055, -0.040), (-0.055, 0.040), (0.055, -0.040), (0.055, 0.040))):
            deck.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x + bx, y + by, CASTER_JOINT_Z - 0.003)),
                material=zinc,
                name=f"caster_bolt_{c_i}_{b_i}",
            )

    rail_knuckle_centers = (-0.220, 0.220)
    post_xs = (-0.500, -0.250, 0.0, 0.250, 0.500)
    side_specs = (
        ("side_rail_0", 1.0, (-1.0, 0.0, 0.0)),
        ("side_rail_1", -1.0, (1.0, 0.0, 0.0)),
    )
    for side_name, side, hinge_axis in side_specs:
        rail = model.part(side_name)
        for k_i, x in enumerate(rail_knuckle_centers):
            rail.visual(
                Cylinder(radius=0.016, length=0.170),
                origin=_cyl_x_origin((x, 0.0, 0.0)),
                material=zinc,
                name=f"rail_hinge_knuckle_{k_i}",
            )
            rail.visual(
                Box((0.052, 0.018, 0.095)),
                origin=Origin(xyz=(x, 0.0, 0.046)),
                material=yellow,
                name=f"hinge_clip_strap_{k_i}",
            )
        rail.visual(
            Cylinder(radius=0.015, length=1.060),
            origin=_cyl_x_origin((0.0, 0.0, 0.105)),
            material=yellow,
            name="lower_tube",
        )
        rail.visual(
            Cylinder(radius=0.013, length=1.040),
            origin=_cyl_x_origin((0.0, 0.0, 0.260)),
            material=yellow,
            name="middle_tube",
        )
        rail.visual(
            Cylinder(radius=0.016, length=1.060),
            origin=_cyl_x_origin((0.0, 0.0, RAIL_HEIGHT)),
            material=yellow,
            name="top_tube",
        )
        for p_i, x in enumerate(post_xs):
            rail.visual(
                Cylinder(radius=0.014, length=RAIL_HEIGHT - 0.105),
                origin=Origin(xyz=(x, 0.0, (RAIL_HEIGHT + 0.105) / 2.0)),
                material=yellow,
                name=f"upright_post_{p_i}",
            )
        rail.visual(
            Box((0.090, 0.022, 0.055)),
            origin=Origin(xyz=(-0.545, 0.0, 0.090)),
            material=zinc,
            name="spring_latch",
        )
        model.articulation(
            f"deck_to_{side_name}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=rail,
            origin=Origin(xyz=(0.0, side * HINGE_Y, HINGE_Z)),
            axis=hinge_axis,
            motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=0.0, upper=math.pi / 2.0),
        )

    # Four swivel caster forks with independently spinning wheels.
    tire_meshes = []
    for c_i in range(4):
        tire_meshes.append(
            mesh_from_geometry(
                TireGeometry(
                    0.075,
                    0.044,
                    inner_radius=0.052,
                    tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.55),
                    sidewall=TireSidewall(style="square", bulge=0.015),
                    shoulder=TireShoulder(width=0.005, radius=0.003),
                ),
                f"caster_tire_{c_i}",
            )
        )

    for c_i, (x, y) in enumerate(caster_positions):
        fork = model.part(f"caster_fork_{c_i}")
        fork.visual(
            Cylinder(radius=0.055, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=zinc,
            name="bearing_disc",
        )
        fork.visual(
            Cylinder(radius=0.015, length=0.075),
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
            material=zinc,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.090, 0.070, 0.032)),
            origin=Origin(xyz=(0.0, 0.0, -0.083)),
            material=zinc,
            name="fork_crown",
        )
        for cheek_i, cx in enumerate((-0.035, 0.035)):
            fork.visual(
                Box((0.012, 0.042, 0.135)),
                origin=Origin(xyz=(cx, 0.0, -0.165)),
                material=zinc,
                name=f"fork_cheek_{cheek_i}",
            )
        for cap_i, cx in enumerate((-0.047, 0.047)):
            fork.visual(
                Cylinder(radius=0.017, length=0.012),
                origin=_cyl_x_origin((cx, 0.0, WHEEL_CENTER_Z)),
                material=dark_steel,
                name=f"axle_cap_{cap_i}",
            )

        wheel = model.part(f"wheel_{c_i}")
        wheel.visual(
            tire_meshes[c_i],
            origin=Origin(),
            material=rubber,
            name="rubber_tire",
        )
        wheel.visual(
            Cylinder(radius=0.054, length=0.040),
            origin=_cyl_x_origin((0.0, 0.0, 0.0)),
            material=zinc,
            name="metal_rim",
        )
        wheel.visual(
            Cylinder(radius=0.024, length=0.052),
            origin=_cyl_x_origin((0.0, 0.0, 0.0)),
            material=dark_steel,
            name="axle_hub",
        )

        model.articulation(
            f"deck_to_caster_fork_{c_i}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(x, y, CASTER_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=8.0),
        )
        model.articulation(
            f"caster_fork_to_wheel_{c_i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    side_rail_0 = object_model.get_part("side_rail_0")
    side_rail_1 = object_model.get_part("side_rail_1")
    rail_joint_0 = object_model.get_articulation("deck_to_side_rail_0")
    rail_joint_1 = object_model.get_articulation("deck_to_side_rail_1")

    ctx.check(
        "side rail hinges have upright-to-flat travel",
        rail_joint_0.motion_limits is not None
        and rail_joint_1.motion_limits is not None
        and abs(rail_joint_0.motion_limits.lower - 0.0) < 1e-9
        and abs(rail_joint_1.motion_limits.lower - 0.0) < 1e-9
        and abs(rail_joint_0.motion_limits.upper - math.pi / 2.0) < 1e-6
        and abs(rail_joint_1.motion_limits.upper - math.pi / 2.0) < 1e-6,
        details=f"limits={rail_joint_0.motion_limits}, {rail_joint_1.motion_limits}",
    )

    def _center_from_aabb(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    rest_top_0 = _center_from_aabb(ctx.part_element_world_aabb(side_rail_0, elem="top_tube"))
    rest_top_1 = _center_from_aabb(ctx.part_element_world_aabb(side_rail_1, elem="top_tube"))
    rest_origin_0 = ctx.part_world_position(side_rail_0)
    rest_origin_1 = ctx.part_world_position(side_rail_1)

    with ctx.pose({rail_joint_0: math.pi / 2.0, rail_joint_1: math.pi / 2.0}):
        folded_top_0 = _center_from_aabb(ctx.part_element_world_aabb(side_rail_0, elem="top_tube"))
        folded_top_1 = _center_from_aabb(ctx.part_element_world_aabb(side_rail_1, elem="top_tube"))
        folded_origin_0 = ctx.part_world_position(side_rail_0)
        folded_origin_1 = ctx.part_world_position(side_rail_1)

    ctx.check(
        "side_rail_0 stays clipped to hinge while folding flat",
        rest_origin_0 is not None
        and folded_origin_0 is not None
        and abs(rest_origin_0[0] - folded_origin_0[0]) < 1e-6
        and abs(rest_origin_0[1] - folded_origin_0[1]) < 1e-6
        and abs(rest_origin_0[2] - folded_origin_0[2]) < 1e-6
        and rest_top_0[2] > HINGE_Z + 0.32
        and folded_top_0[1] > HINGE_Y + 0.32
        and abs(folded_top_0[2] - HINGE_Z) < 0.04,
        details=f"rest_origin={rest_origin_0}, folded_origin={folded_origin_0}, rest_top={rest_top_0}, folded_top={folded_top_0}",
    )
    ctx.check(
        "side_rail_1 stays clipped to hinge while folding flat",
        rest_origin_1 is not None
        and folded_origin_1 is not None
        and abs(rest_origin_1[0] - folded_origin_1[0]) < 1e-6
        and abs(rest_origin_1[1] - folded_origin_1[1]) < 1e-6
        and abs(rest_origin_1[2] - folded_origin_1[2]) < 1e-6
        and rest_top_1[2] > HINGE_Z + 0.32
        and folded_top_1[1] < -HINGE_Y - 0.32
        and abs(folded_top_1[2] - HINGE_Z) < 0.04,
        details=f"rest_origin={rest_origin_1}, folded_origin={folded_origin_1}, rest_top={rest_top_1}, folded_top={folded_top_1}",
    )

    for i in range(4):
        fork = object_model.get_part(f"caster_fork_{i}")
        wheel = object_model.get_part(f"wheel_{i}")
        swivel = object_model.get_articulation(f"deck_to_caster_fork_{i}")
        axle = object_model.get_articulation(f"caster_fork_to_wheel_{i}")
        ctx.check(
            f"caster_{i} has swivel and rolling axle",
            swivel.articulation_type == ArticulationType.CONTINUOUS
            and axle.articulation_type == ArticulationType.CONTINUOUS
            and tuple(swivel.axis) == (0.0, 0.0, 1.0)
            and tuple(axle.axis) == (1.0, 0.0, 0.0),
            details=f"swivel={swivel.articulation_type} axis={swivel.axis}, axle={axle.articulation_type} axis={axle.axis}",
        )
        ctx.expect_contact(
            fork,
            "deck",
            elem_a="bearing_disc",
            elem_b=f"caster_mount_{i}",
            contact_tol=1e-5,
            name=f"caster_{i} bearing is seated against deck plate",
        )
        ctx.expect_within(
            wheel,
            fork,
            axes="x",
            margin=0.002,
            name=f"wheel_{i} sits between fork cheeks along axle",
        )

    return ctx.report()


object_model = build_object_model()
