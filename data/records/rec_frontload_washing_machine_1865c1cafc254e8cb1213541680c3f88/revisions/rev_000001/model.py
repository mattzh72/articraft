from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_compact_washer")

    white = model.material("warm_white_plastic", rgba=(0.96, 0.97, 0.95, 1.0))
    gloss_white = model.material("gloss_white_door", rgba=(0.98, 0.99, 0.98, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("smoked_glass", rgba=(0.22, 0.38, 0.52, 0.38))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark = model.material("shadow_black", rgba=(0.02, 0.022, 0.024, 1.0))
    bracket_metal = model.material("powder_coated_bracket", rgba=(0.62, 0.64, 0.66, 1.0))
    display = model.material("black_glass_panel", rgba=(0.03, 0.04, 0.05, 1.0))

    # Root link: a rigid wall bracket behind the appliance.  The two horizontal
    # rails project from the wall plate and touch the rear of the washer body.
    bracket = model.part("wall_bracket")
    bracket.visual(
        Box((0.66, 0.025, 0.60)),
        origin=Origin(xyz=(0.0, 0.145, 0.0)),
        material=bracket_metal,
        name="wall_plate",
    )
    bracket.visual(
        Box((0.50, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, 0.095, 0.235)),
        material=bracket_metal,
        name="upper_rail",
    )
    bracket.visual(
        Box((0.50, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, 0.095, -0.235)),
        material=bracket_metal,
        name="lower_rail",
    )
    for x, name in ((-0.245, "side_hook_0"), (0.245, "side_hook_1")):
        bracket.visual(
            Box((0.045, 0.075, 0.44)),
            origin=Origin(xyz=(x, 0.098, 0.0)),
            material=bracket_metal,
            name=name,
        )

    housing = model.part("housing")
    # Box-like side and rear walls keep the appliance slim while leaving a real
    # front porthole opening clear for the drum.
    housing.visual(
        Box((0.040, 0.200, 0.520)),
        origin=Origin(xyz=(-0.270, -0.045, 0.0)),
        material=white,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.040, 0.200, 0.520)),
        origin=Origin(xyz=(0.270, -0.045, 0.0)),
        material=white,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.580, 0.200, 0.040)),
        origin=Origin(xyz=(0.0, -0.045, 0.240)),
        material=white,
        name="top_wall",
    )
    housing.visual(
        Box((0.580, 0.200, 0.040)),
        origin=Origin(xyz=(0.0, -0.045, -0.240)),
        material=white,
        name="bottom_wall",
    )
    housing.visual(
        Box((0.580, 0.025, 0.520)),
        origin=Origin(xyz=(0.0, 0.0425, 0.0)),
        material=white,
        name="rear_wall",
    )
    housing.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_bearing",
    )
    front_panel_mesh = mesh_from_geometry(
        BezelGeometry(
            opening_size=(0.340, 0.340),
            outer_size=(0.580, 0.520),
            depth=0.030,
            opening_shape="circle",
            outer_shape="rounded_rect",
            outer_corner_radius=0.055,
            face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.003),
        ),
        "housing_front_panel",
    )
    housing.visual(
        front_panel_mesh,
        origin=Origin(xyz=(0.0, -0.145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="front_panel",
    )
    housing.visual(
        Box((0.120, 0.006, 0.038)),
        origin=Origin(xyz=(0.170, -0.162, 0.197)),
        material=display,
        name="status_panel",
    )
    # Stationary leaves and barrels for the two exposed left-edge door hinges.
    for z, leaf_name, barrel_name in (
        (0.125, "hinge_leaf_0", "hinge_barrel_0"),
        (-0.125, "hinge_leaf_1", "hinge_barrel_1"),
    ):
        housing.visual(
            Box((0.056, 0.020, 0.074)),
            origin=Origin(xyz=(-0.214, -0.149, z)),
            material=bracket_metal,
            name=leaf_name,
        )
        housing.visual(
            Cylinder(radius=0.010, length=0.074),
            origin=Origin(xyz=(-0.205, -0.151, z)),
            material=bracket_metal,
            name=barrel_name,
        )

    model.articulation(
        "bracket_to_housing",
        ArticulationType.FIXED,
        parent=bracket,
        child=housing,
        origin=Origin(),
    )

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=0.145, length=0.130),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.112, length=0.006),
        origin=Origin(xyz=(0.0, -0.068, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="dark_perforation_field",
    )
    for angle, name in ((0.0, "drum_baffle_0"), (2.094, "drum_baffle_1"), (4.188, "drum_baffle_2")):
        drum.visual(
            Box((0.116, 0.010, 0.016)),
            origin=Origin(xyz=(0.050 * math.cos(angle), -0.073, 0.050 * math.sin(angle)), rpy=(0.0, angle, 0.0)),
            material=steel,
            name=name,
        )

    model.articulation(
        "housing_to_drum",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=drum,
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )

    door = model.part("door")
    door_ring = mesh_from_geometry(
        BezelGeometry(
            opening_size=(0.270, 0.270),
            outer_size=(0.390, 0.390),
            depth=0.032,
            opening_shape="circle",
            outer_shape="circle",
            face=BezelFace(style="radiused_step", front_lip=0.003, fillet=0.003),
        ),
        "door_outer_ring",
    )
    gasket_ring = mesh_from_geometry(
        BezelGeometry(
            opening_size=(0.260, 0.260),
            outer_size=(0.320, 0.320),
            depth=0.014,
            opening_shape="circle",
            outer_shape="circle",
            face=BezelFace(style="plain", front_lip=0.001, fillet=0.001),
        ),
        "door_gasket_ring",
    )
    door.visual(
        door_ring,
        origin=Origin(xyz=(0.200, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_white,
        name="outer_ring",
    )
    door.visual(
        gasket_ring,
        origin=Origin(xyz=(0.200, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rubber_gasket",
    )
    door.visual(
        Cylinder(radius=0.142, length=0.010),
        origin=Origin(xyz=(0.200, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="glass_porthole",
    )
    for z, bridge_name, barrel_name in (
        (0.125, "door_leaf_0", "door_barrel_0"),
        (-0.125, "door_leaf_1", "door_barrel_1"),
    ):
        door.visual(
            Box((0.064, 0.018, 0.050)),
            origin=Origin(xyz=(0.032, 0.0, z)),
            material=gloss_white,
            name=bridge_name,
        )
        door.visual(
            Cylinder(radius=0.012, length=0.074),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=bracket_metal,
            name=barrel_name,
        )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(-0.200, -0.176, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("wall_bracket")
    housing = object_model.get_part("housing")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_joint = object_model.get_articulation("housing_to_drum")
    door_joint = object_model.get_articulation("housing_to_door")

    ctx.expect_contact(
        bracket,
        housing,
        elem_a="upper_rail",
        elem_b="rear_wall",
        contact_tol=0.001,
        name="upper bracket rail bears on rear housing",
    )
    ctx.expect_contact(
        bracket,
        housing,
        elem_a="lower_rail",
        elem_b="rear_wall",
        contact_tol=0.001,
        name="lower bracket rail bears on rear housing",
    )
    ctx.expect_within(
        drum,
        housing,
        axes="xz",
        margin=0.010,
        elem_a="drum_shell",
        elem_b="front_panel",
        name="drum is centered behind the circular porthole",
    )
    ctx.expect_gap(
        housing,
        door,
        axis="y",
        max_gap=0.004,
        max_penetration=0.003,
        positive_elem="front_panel",
        negative_elem="outer_ring",
        name="closed door sits just proud of front panel",
    )

    def element_center_y(part, elem: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return (lo[1] + hi[1]) * 0.5

    closed_y = element_center_y(door, "outer_ring")
    with ctx.pose({door_joint: 1.25}):
        opened_y = element_center_y(door, "outer_ring")
    ctx.check(
        "left hinged door opens outward",
        closed_y is not None and opened_y is not None and opened_y < closed_y - 0.08,
        details=f"closed_y={closed_y}, opened_y={opened_y}",
    )

    rest_drum = ctx.part_world_position(drum)
    with ctx.pose({drum_joint: math.pi / 2.0}):
        spun_drum = ctx.part_world_position(drum)
    ctx.check(
        "drum spins about fixed axle",
        rest_drum is not None
        and spun_drum is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_drum, spun_drum)),
        details=f"rest={rest_drum}, spun={spun_drum}",
    )

    return ctx.report()


object_model = build_object_model()
