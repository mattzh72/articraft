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


# Real refrigerator proportions in meters.
WIDTH = 0.96
DEPTH = 0.70
HEIGHT = 1.86
FRONT_Y = -DEPTH / 2.0

HINGE_Y = FRONT_Y - 0.042
LEFT_HINGE_X = -0.475
RIGHT_HINGE_X = 0.475
CENTER_LEFT_HINGE_X = -0.026
CENTER_RIGHT_HINGE_X = 0.026

DOOR_WIDTH = 0.440
DOOR_THICKNESS = 0.055
HINGE_OFFSET = 0.018

LOWER_Z0 = 0.09
LOWER_HEIGHT = 0.77
UPPER_Z0 = 0.94
UPPER_HEIGHT = 0.88

BARREL_RADIUS = 0.018
PIN_RADIUS = 0.006


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_door_refrigerator")

    stainless = model.material("brushed_stainless", rgba=(0.73, 0.75, 0.76, 1.0))
    side_gray = model.material("warm_gray_cabinet", rgba=(0.58, 0.60, 0.61, 1.0))
    dark_gasket = model.material("dark_gasket", rgba=(0.025, 0.027, 0.030, 1.0))
    black_glass = model.material("black_glass", rgba=(0.02, 0.025, 0.032, 1.0))
    pin_metal = model.material("hinge_pin_metal", rgba=(0.80, 0.81, 0.80, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((WIDTH, DEPTH, HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT / 2.0)),
        material=side_gray,
        name="insulated_cabinet",
    )
    cabinet.visual(
        Box((0.030, 0.035, HEIGHT - 0.16)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.005, HEIGHT / 2.0 + 0.01)),
        material=dark_gasket,
        name="center_post",
    )
    cabinet.visual(
        Box((WIDTH - 0.08, 0.045, 0.052)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.014, 0.900)),
        material=dark_gasket,
        name="middle_rail",
    )
    cabinet.visual(
        Box((WIDTH - 0.12, 0.038, 0.074)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.018, 0.037)),
        material=dark_gasket,
        name="toe_grille",
    )

    def add_cabinet_hinge_hardware(prefix: str, hinge_x: float, z0: float, height: float) -> None:
        cabinet.visual(
            Cylinder(radius=PIN_RADIUS, length=height + 0.055),
            origin=Origin(xyz=(hinge_x, HINGE_Y, z0 + height / 2.0)),
            material=pin_metal,
            name=f"{prefix}_pin",
        )
        for i, zc in enumerate((z0 - 0.018, z0 + height + 0.018)):
            cabinet.visual(
                Box((0.046, 0.070, 0.030)),
                origin=Origin(xyz=(hinge_x, HINGE_Y + 0.014, zc)),
                material=pin_metal,
                name=f"{prefix}_pin_saddle_{i}",
            )

    add_cabinet_hinge_hardware("upper_left", LEFT_HINGE_X, UPPER_Z0, UPPER_HEIGHT)
    add_cabinet_hinge_hardware("lower_left", CENTER_LEFT_HINGE_X, LOWER_Z0, LOWER_HEIGHT)
    add_cabinet_hinge_hardware("upper_right", RIGHT_HINGE_X, UPPER_Z0, UPPER_HEIGHT)
    add_cabinet_hinge_hardware("lower_right", CENTER_RIGHT_HINGE_X, LOWER_Z0, LOWER_HEIGHT)

    def make_door(
        name: str,
        *,
        sign: float,
        height: float,
        handle_height: float,
        add_display: bool = False,
    ):
        """Create a door leaf whose local frame is on its vertical hinge axis.

        sign=+1 means the closed leaf extends toward +X from the hinge.
        sign=-1 means the closed leaf extends toward -X from the hinge.
        """
        door = model.part(name)
        slab_center_x = sign * (HINGE_OFFSET + DOOR_WIDTH / 2.0)
        free_edge_x = sign * (HINGE_OFFSET + DOOR_WIDTH - 0.026)
        inner_seam_x = sign * (HINGE_OFFSET + DOOR_WIDTH - 0.006)

        door.visual(
            Box((DOOR_WIDTH, DOOR_THICKNESS, height)),
            origin=Origin(xyz=(slab_center_x, 0.0, height / 2.0)),
            material=stainless,
            name="door_slab",
        )
        door.visual(
            Box((0.010, 0.010, height - 0.050)),
            origin=Origin(xyz=(inner_seam_x, -DOOR_THICKNESS / 2.0 - 0.002, height / 2.0)),
            material=dark_gasket,
            name="center_gasket",
        )
        door.visual(
            Box((0.020, 0.014, handle_height)),
            origin=Origin(xyz=(free_edge_x, -DOOR_THICKNESS / 2.0 - 0.006, height / 2.0)),
            material=dark_gasket,
            name="vertical_pull",
        )
        door.visual(
            Cylinder(radius=BARREL_RADIUS, length=height - 0.035),
            origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
            material=pin_metal,
            name="hinge_barrel",
        )
        for i, zc in enumerate((0.18 * height, 0.50 * height, 0.82 * height)):
            door.visual(
                Box((0.038, 0.030, 0.055)),
                origin=Origin(xyz=(sign * 0.026, 0.0, zc)),
                material=pin_metal,
                name=f"barrel_clip_{i}",
            )
        if add_display:
            door.visual(
                Box((0.150, 0.010, 0.245)),
                origin=Origin(
                    xyz=(
                        sign * (HINGE_OFFSET + DOOR_WIDTH * 0.52),
                        -DOOR_THICKNESS / 2.0 - 0.004,
                        height * 0.57,
                    )
                ),
                material=black_glass,
                name="display_glass",
            )
            door.visual(
                Box((0.110, 0.012, 0.012)),
                origin=Origin(
                    xyz=(
                        sign * (HINGE_OFFSET + DOOR_WIDTH * 0.52),
                        -DOOR_THICKNESS / 2.0 - 0.009,
                        height * 0.655,
                    )
                ),
                material=dark_gasket,
                name="display_trim",
            )
        return door

    upper_left_door = make_door(
        "upper_left_door",
        sign=1.0,
        height=UPPER_HEIGHT,
        handle_height=0.58,
        add_display=True,
    )
    lower_left_door = make_door(
        "lower_left_door",
        sign=-1.0,
        height=LOWER_HEIGHT,
        handle_height=0.43,
    )
    upper_right_door = make_door(
        "upper_right_door",
        sign=-1.0,
        height=UPPER_HEIGHT,
        handle_height=0.58,
    )
    lower_right_door = make_door(
        "lower_right_door",
        sign=1.0,
        height=LOWER_HEIGHT,
        handle_height=0.43,
    )

    hinge_limits = MotionLimits(effort=28.0, velocity=1.25, lower=0.0, upper=1.70)
    model.articulation(
        "upper_left_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=upper_left_door,
        origin=Origin(xyz=(LEFT_HINGE_X, HINGE_Y, UPPER_Z0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "lower_left_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lower_left_door,
        origin=Origin(xyz=(CENTER_LEFT_HINGE_X, HINGE_Y, LOWER_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "upper_right_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=upper_right_door,
        origin=Origin(xyz=(RIGHT_HINGE_X, HINGE_Y, UPPER_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "lower_right_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lower_right_door,
        origin=Origin(xyz=(CENTER_RIGHT_HINGE_X, HINGE_Y, LOWER_Z0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=hinge_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")

    door_specs = (
        ("upper_left_door", "upper_left_pin", UPPER_HEIGHT, "upper_left_hinge"),
        ("lower_left_door", "lower_left_pin", LOWER_HEIGHT, "lower_left_hinge"),
        ("upper_right_door", "upper_right_pin", UPPER_HEIGHT, "upper_right_hinge"),
        ("lower_right_door", "lower_right_pin", LOWER_HEIGHT, "lower_right_hinge"),
    )
    for door_name, pin_name, height, hinge_name in door_specs:
        door = object_model.get_part(door_name)
        hinge = object_model.get_articulation(hinge_name)
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=pin_name,
            elem_b="hinge_barrel",
            reason="The fixed refrigerator hinge pin is intentionally captured inside the door-leaf barrel.",
        )
        ctx.expect_within(
            cabinet,
            door,
            axes="xy",
            inner_elem=pin_name,
            outer_elem="hinge_barrel",
            margin=0.001,
            name=f"{door_name} pin centered in barrel",
        )
        ctx.expect_overlap(
            cabinet,
            door,
            axes="z",
            elem_a=pin_name,
            elem_b="hinge_barrel",
            min_overlap=height - 0.060,
            name=f"{door_name} pin retained through barrel length",
        )
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            min_gap=0.006,
            max_gap=0.025,
            positive_elem="insulated_cabinet",
            negative_elem="door_slab",
            name=f"{door_name} closes proud of cabinet face",
        )

        rest_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
        with ctx.pose({hinge: 0.95}):
            open_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
        ctx.check(
            f"{door_name} swings outward",
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[0][1] < rest_aabb[0][1] - 0.18,
            details=f"rest={rest_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
