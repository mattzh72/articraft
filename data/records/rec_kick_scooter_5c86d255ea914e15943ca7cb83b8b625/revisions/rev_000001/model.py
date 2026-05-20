from __future__ import annotations

from math import atan2, pi

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
    TireSidewall,
    TireShoulder,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stunt_kick_scooter")

    aluminum = model.material("satin_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    dark_metal = model.material("black_anodized", rgba=(0.02, 0.023, 0.026, 1.0))
    deck_blue = model.material("deck_blue", rgba=(0.03, 0.22, 0.52, 1.0))
    grip = model.material("grip_tape", rgba=(0.01, 0.01, 0.009, 1.0))
    urethane = model.material("clear_blue_urethane", rgba=(0.10, 0.35, 0.95, 1.0))
    hub_white = model.material("white_hub", rgba=(0.92, 0.92, 0.88, 1.0))

    deck_length = 0.52
    deck_width = 0.13
    deck_thickness = 0.038
    deck_z = 0.125
    wheel_radius = 0.050
    wheel_width = 0.028
    axle_z = 0.052
    front_axle_x = 0.315
    rear_axle_x = -0.255

    frame = model.part("deck_frame")
    frame.visual(
        Box((deck_length, deck_width, deck_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_z)),
        material=deck_blue,
        name="short_deck",
    )
    frame.visual(
        Box((0.46, 0.094, 0.004)),
        origin=Origin(xyz=(-0.010, 0.0, deck_z + deck_thickness / 2.0 + 0.002)),
        material=grip,
        name="grip_tape",
    )
    frame.visual(
        Box((deck_length, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, deck_width / 2.0 + 0.003, deck_z - 0.004)),
        material=aluminum,
        name="deck_side_rail_0",
    )
    frame.visual(
        Box((deck_length, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, -deck_width / 2.0 - 0.003, deck_z - 0.004)),
        material=aluminum,
        name="deck_side_rail_1",
    )

    rear_drop_y = wheel_width / 2.0 + 0.014
    rear_drop_height = deck_z - deck_thickness / 2.0 - axle_z
    frame.visual(
        Box((0.052, 0.009, rear_drop_height)),
        origin=Origin(xyz=(rear_axle_x, rear_drop_y, axle_z + rear_drop_height / 2.0)),
        material=dark_metal,
        name="rear_dropout_0",
    )
    frame.visual(
        Box((0.052, 0.009, rear_drop_height)),
        origin=Origin(xyz=(rear_axle_x, -rear_drop_y, axle_z + rear_drop_height / 2.0)),
        material=dark_metal,
        name="rear_dropout_1",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=2.0 * (rear_drop_y + 0.010)),
        origin=Origin(xyz=(rear_axle_x, 0.0, axle_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle",
    )

    headset_y = 0.019
    headset_height = 0.100
    headset_center_z = 0.205
    frame.visual(
        Box((0.030, 0.010, headset_height)),
        origin=Origin(xyz=(front_axle_x, headset_y, headset_center_z)),
        material=aluminum,
        name="headset_cheek_0",
    )
    frame.visual(
        Box((0.030, 0.010, headset_height)),
        origin=Origin(xyz=(front_axle_x, -headset_y, headset_center_z)),
        material=aluminum,
        name="headset_cheek_1",
    )
    brace_dx = front_axle_x - (deck_length / 2.0 - 0.025)
    brace_dz = headset_center_z - deck_z
    brace_len = (brace_dx * brace_dx + brace_dz * brace_dz) ** 0.5
    brace_pitch = -atan2(brace_dz, brace_dx)
    for idx, y in enumerate((headset_y, -headset_y)):
        frame.visual(
            Box((brace_len, 0.014, 0.016)),
            origin=Origin(
                xyz=(front_axle_x - brace_dx / 2.0, y, deck_z + brace_dz / 2.0),
                rpy=(0.0, brace_pitch, 0.0),
            ),
            material=aluminum,
            name=f"neck_gusset_{idx}",
        )

    fork_bar = model.part("bar_fork")
    fork_leg_y = wheel_width / 2.0 + 0.011
    fork_leg_height = 0.100
    fork_bar.visual(
        Box((0.020, 0.008, fork_leg_height)),
        origin=Origin(xyz=(0.0, fork_leg_y, 0.040)),
        material=aluminum,
        name="fork_leg_0",
    )
    fork_bar.visual(
        Box((0.020, 0.008, fork_leg_height)),
        origin=Origin(xyz=(0.0, -fork_leg_y, 0.040)),
        material=aluminum,
        name="fork_leg_1",
    )
    fork_bar.visual(
        Box((0.030, 2.0 * fork_leg_y + 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=aluminum,
        name="fork_crown",
    )
    fork_bar.visual(
        Cylinder(radius=0.0045, length=2.0 * (fork_leg_y + 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_axle",
    )
    fork_bar.visual(
        Cylinder(radius=0.014, length=0.755),
        origin=Origin(xyz=(0.0, 0.0, 0.462)),
        material=aluminum,
        name="one_piece_stem",
    )
    fork_bar.visual(
        Cylinder(radius=0.013, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.835), rpy=(pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="one_piece_bar",
    )
    fork_bar.visual(
        Cylinder(radius=0.017, length=0.095),
        origin=Origin(xyz=(0.0, 0.163, 0.835), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="grip_0",
    )
    fork_bar.visual(
        Cylinder(radius=0.017, length=0.095),
        origin=Origin(xyz=(0.0, -0.163, 0.835), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="grip_1",
    )

    wheel_core = WheelGeometry(
        0.039,
        wheel_width,
        rim=WheelRim(inner_radius=0.025, flange_height=0.003, flange_thickness=0.002),
        hub=WheelHub(
            radius=0.014,
            width=0.022,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=3, circle_diameter=0.020, hole_diameter=0.003),
        ),
        face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="split_y", count=3, thickness=0.003, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.014),
    )
    tire = TireGeometry(
        wheel_radius,
        wheel_width,
        inner_radius=0.038,
        carcass=TireCarcass(belt_width_ratio=0.78, sidewall_bulge=0.04),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        mesh_from_geometry(tire, "front_urethane_tire"),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=urethane,
        name="urethane_tire",
    )
    front_wheel.visual(
        mesh_from_geometry(wheel_core, "front_spoked_hub"),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=hub_white,
        name="spoked_hub",
    )
    front_wheel.visual(
        Cylinder(radius=0.007, length=wheel_width * 0.90),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="bearing_sleeve",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        mesh_from_geometry(tire, "rear_urethane_tire"),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=urethane,
        name="urethane_tire",
    )
    rear_wheel.visual(
        mesh_from_geometry(wheel_core, "rear_spoked_hub"),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=hub_white,
        name="spoked_hub",
    )
    rear_wheel.visual(
        Cylinder(radius=0.007, length=wheel_width * 0.90),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="bearing_sleeve",
    )

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=fork_bar,
        origin=Origin(xyz=(front_axle_x, 0.0, axle_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=4.0, lower=-1.65, upper=1.65),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork_bar,
        child=front_wheel,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel,
        origin=Origin(xyz=(rear_axle_x, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    steering = object_model.get_articulation("steering_yaw")
    front_spin = object_model.get_articulation("front_wheel_spin")
    rear_spin = object_model.get_articulation("rear_wheel_spin")
    frame = object_model.get_part("deck_frame")
    fork_bar = object_model.get_part("bar_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    ctx.allow_overlap(
        fork_bar,
        front_wheel,
        elem_a="front_axle",
        elem_b="bearing_sleeve",
        reason="The wheel bearing sleeve is intentionally captured on the fork axle so the front wheel stays clipped between the fork legs while spinning.",
    )
    ctx.allow_overlap(
        frame,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="bearing_sleeve",
        reason="The rear bearing sleeve is intentionally captured on the deck-frame axle support while the rear wheel spins.",
    )

    ctx.check(
        "compact scooter uses only steering and wheel-spin joints",
        len(object_model.articulations) == 3
        and steering.articulation_type == ArticulationType.REVOLUTE
        and front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )

    ctx.expect_within(
        front_wheel,
        fork_bar,
        axes="y",
        inner_elem="urethane_tire",
        outer_elem="fork_crown",
        margin=0.0,
        name="front wheel is laterally captured by the narrow fork",
    )
    ctx.expect_overlap(
        front_wheel,
        fork_bar,
        axes="y",
        elem_a="bearing_sleeve",
        elem_b="front_axle",
        min_overlap=0.018,
        name="front wheel bearing remains on the fork axle",
    )
    ctx.expect_overlap(
        rear_wheel,
        frame,
        axes="y",
        elem_a="bearing_sleeve",
        elem_b="rear_axle",
        min_overlap=0.018,
        name="rear wheel bearing remains on the deck axle",
    )
    ctx.expect_gap(
        frame,
        rear_wheel,
        axis="z",
        positive_elem="short_deck",
        negative_elem="urethane_tire",
        min_gap=0.002,
        max_gap=0.020,
        name="rear wheel tucks just below deck tail",
    )
    ctx.expect_overlap(
        rear_wheel,
        frame,
        axes="x",
        elem_a="urethane_tire",
        elem_b="short_deck",
        min_overlap=0.020,
        name="rear wheel sits close under the deck tail",
    )

    rest_bar = ctx.part_element_world_aabb(fork_bar, elem="one_piece_bar")
    with ctx.pose({steering: 0.90, front_spin: 1.25, rear_spin: 1.25}):
        turned_bar = ctx.part_element_world_aabb(fork_bar, elem="one_piece_bar")
        ctx.expect_overlap(
            front_wheel,
            fork_bar,
            axes="y",
            elem_a="bearing_sleeve",
            elem_b="front_axle",
            min_overlap=0.010,
            name="front wheel remains clipped in the fork while steering",
        )

    ctx.check(
        "one-piece bar yaws with the fork assembly",
        rest_bar is not None
        and turned_bar is not None
        and (turned_bar[1][0] - turned_bar[0][0]) > (rest_bar[1][0] - rest_bar[0][0]) + 0.10,
        details=f"rest_bar={rest_bar}, turned_bar={turned_bar}",
    )

    return ctx.report()


object_model = build_object_model()
