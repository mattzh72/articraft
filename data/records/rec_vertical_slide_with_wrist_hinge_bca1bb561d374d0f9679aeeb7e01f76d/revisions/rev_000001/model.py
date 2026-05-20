from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_lift_module")

    painted_steel = model.material("painted_steel", rgba=(0.16, 0.18, 0.19, 1.0))
    rail_steel = model.material("ground_rail_steel", rgba=(0.42, 0.45, 0.46, 1.0))
    blue_carriage = model.material("blue_carriage", rgba=(0.05, 0.22, 0.55, 1.0))
    black_wear_pad = model.material("black_wear_pad", rgba=(0.02, 0.02, 0.018, 1.0))
    orange_face = model.material("safety_orange_face", rgba=(0.95, 0.38, 0.05, 1.0))
    dark_pin = model.material("dark_hinge_pin", rgba=(0.05, 0.055, 0.06, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.62, 0.50, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=painted_steel,
        name="floor_plate",
    )
    for idx, x in enumerate((-0.18, 0.18)):
        mast.visual(
            Box((0.055, 0.080, 1.45)),
            origin=Origin(xyz=(x, 0.0, 0.785)),
            material=rail_steel,
            name=("rail_0", "rail_1")[idx],
        )
    mast.visual(
        Box((0.10, 0.060, 1.45)),
        origin=Origin(xyz=(0.0, 0.095, 0.785)),
        material=painted_steel,
        name="rear_spine",
    )
    mast.visual(
        Box((0.46, 0.090, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=painted_steel,
        name="lower_tie",
    )
    mast.visual(
        Box((0.48, 0.105, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 1.540)),
        material=painted_steel,
        name="top_tie",
    )
    for idx, (x, y) in enumerate(((-0.24, -0.18), (0.24, -0.18), (-0.24, 0.18), (0.24, 0.18))):
        mast.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x, y, 0.063)),
            material=dark_pin,
            name=f"anchor_bolt_{idx}",
        )

    platen = model.part("platen")
    platen.visual(
        Box((0.34, 0.040, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue_carriage,
        name="lift_plate",
    )
    shoe_specs = (
        (-0.18, 0.14),
        (0.18, 0.14),
        (-0.18, -0.14),
        (0.18, -0.14),
    )
    for idx, (x, z) in enumerate(shoe_specs):
        platen.visual(
            Box((0.070, 0.053, 0.120)),
            origin=Origin(xyz=(x, 0.0465, z)),
            material=black_wear_pad,
            name=("guide_shoe_0", "guide_shoe_1", "guide_shoe_2", "guide_shoe_3")[idx],
        )
    platen.visual(
        Box((0.050, 0.008, 0.280)),
        origin=Origin(xyz=(-0.120, -0.024, 0.0)),
        material=blue_carriage,
        name="hinge_mount",
    )
    platen.visual(
        Box((0.16, 0.012, 0.026)),
        origin=Origin(xyz=(-0.060, -0.026, 0.145)),
        material=blue_carriage,
        name="upper_face_rib",
    )
    platen.visual(
        Box((0.16, 0.012, 0.026)),
        origin=Origin(xyz=(-0.060, -0.026, -0.145)),
        material=blue_carriage,
        name="lower_face_rib",
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        Box((0.18, 0.025, 0.22)),
        origin=Origin(xyz=(0.090, -0.018, 0.0)),
        material=orange_face,
        name="face_panel",
    )
    end_plate.visual(
        Cylinder(radius=0.012, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_pin,
        name="hinge_barrel",
    )
    end_plate.visual(
        Box((0.070, 0.012, 0.060)),
        origin=Origin(xyz=(0.036, -0.010, 0.0)),
        material=dark_pin,
        name="wrist_boss",
    )

    model.articulation(
        "lift_slide",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=platen,
        origin=Origin(xyz=(0.0, -0.115, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.22, lower=0.0, upper=0.85),
    )
    model.articulation(
        "wrist_hinge",
        ArticulationType.REVOLUTE,
        parent=platen,
        child=end_plate,
        origin=Origin(xyz=(-0.120, -0.040, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    platen = object_model.get_part("platen")
    end_plate = object_model.get_part("end_plate")
    lift_slide = object_model.get_articulation("lift_slide")
    wrist_hinge = object_model.get_articulation("wrist_hinge")

    ctx.expect_gap(
        mast,
        platen,
        axis="y",
        positive_elem="rail_0",
        negative_elem="guide_shoe_0",
        min_gap=0.0,
        max_gap=0.004,
        name="guide shoe runs with small clearance on mast rail",
    )
    ctx.expect_within(
        end_plate,
        platen,
        axes="z",
        inner_elem="face_panel",
        outer_elem="lift_plate",
        margin=0.005,
        name="wrist face stays shorter than lift plate",
    )
    ctx.expect_gap(
        platen,
        end_plate,
        axis="y",
        positive_elem="hinge_mount",
        negative_elem="hinge_barrel",
        max_gap=0.004,
        max_penetration=0.0002,
        name="hinge barrel sits just in front of carriage mount",
    )

    platen_aabb = ctx.part_world_aabb(platen)
    face_aabb = ctx.part_world_aabb(end_plate)
    if platen_aabb is not None and face_aabb is not None:
        platen_dims = tuple(platen_aabb[1][i] - platen_aabb[0][i] for i in range(3))
        face_dims = tuple(face_aabb[1][i] - face_aabb[0][i] for i in range(3))
    else:
        platen_dims = face_dims = None
    ctx.check(
        "hinged face is compact relative to carriage",
        platen_dims is not None
        and face_dims is not None
        and face_dims[0] < platen_dims[0] * 0.55
        and face_dims[2] < platen_dims[2] * 0.60,
        details=f"platen_dims={platen_dims}, face_dims={face_dims}",
    )

    rest_pos = ctx.part_world_position(platen)
    with ctx.pose({lift_slide: 0.85}):
        raised_pos = ctx.part_world_position(platen)
        ctx.expect_within(
            platen,
            mast,
            axes="z",
            inner_elem="lift_plate",
            outer_elem="rail_0",
            margin=0.01,
            name="raised platen remains within mast rail height",
        )
    ctx.check(
        "prismatic lift raises the platen",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.80,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    closed_aabb = ctx.part_world_aabb(end_plate)
    closed_center_y = None
    opened_center_y = None
    if closed_aabb is not None:
        closed_center_y = 0.5 * (closed_aabb[0][1] + closed_aabb[1][1])
    with ctx.pose({wrist_hinge: 1.0}):
        opened_aabb = ctx.part_world_aabb(end_plate)
        if opened_aabb is not None:
            opened_center_y = 0.5 * (opened_aabb[0][1] + opened_aabb[1][1])
    ctx.check(
        "wrist hinge swings the face outward from the carriage",
        closed_center_y is not None
        and opened_center_y is not None
        and opened_center_y < closed_center_y - 0.04,
        details=f"closed_y={closed_center_y}, opened_y={opened_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
