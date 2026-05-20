from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)

SHAFT_WIDTH = 1.20
SHAFT_DEPTH = 1.28
SHAFT_HEIGHT = 3.05

CAR_WIDTH = 0.98
CAR_DEPTH = 0.96
CAR_HEIGHT = 1.96
CAR_TRAVEL = 0.92

OPENING_WIDTH = 0.80
OPENING_HEIGHT = 1.62
OPENING_BOTTOM = 0.09

GATE_PANEL_COUNT = 4
GATE_PANEL_WIDTH = OPENING_WIDTH / GATE_PANEL_COUNT
GATE_STILE_WIDTH = 0.022
GATE_DEPTH = 0.006
GATE_BAR_WIDTH = 0.008
GATE_PANEL_HEIGHT = 1.54
GATE_RAIL_WIDTH = GATE_PANEL_WIDTH - (2.0 * GATE_STILE_WIDTH)

GATE_PLANE_Y = 0.458
DOOR_PLANE_Y = 0.388


def _box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material=None,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_diagonal_bar(
    part,
    *,
    name: str,
    x1: float,
    z1: float,
    x2: float,
    z2: float,
    material,
) -> None:
    dx = x2 - x1
    dz = z2 - z1
    length = math.hypot(dx, dz)
    pitch = math.atan2(dx, dz)
    _box_visual(
        part,
        name=name,
        size=(GATE_BAR_WIDTH, GATE_DEPTH, length),
        xyz=((x1 + x2) * 0.5, 0.0, (z1 + z2) * 0.5),
        rpy=(0.0, pitch, 0.0),
        material=material,
    )


def _add_gate_cell(part, *, material) -> None:
    left_x = 0.5 * GATE_STILE_WIDTH
    right_x = GATE_PANEL_WIDTH - left_x
    mid_z = 0.5 * GATE_PANEL_HEIGHT
    low_z = 0.28
    high_z = GATE_PANEL_HEIGHT - 0.18

    _box_visual(
        part,
        name="left_stile",
        size=(GATE_STILE_WIDTH, GATE_DEPTH, GATE_PANEL_HEIGHT),
        xyz=(left_x, 0.0, 0.5 * GATE_PANEL_HEIGHT),
        material=material,
    )
    _box_visual(
        part,
        name="right_stile",
        size=(GATE_STILE_WIDTH, GATE_DEPTH, GATE_PANEL_HEIGHT),
        xyz=(right_x, 0.0, 0.5 * GATE_PANEL_HEIGHT),
        material=material,
    )
    _box_visual(
        part,
        name="top_rail",
        size=(GATE_RAIL_WIDTH, 0.024, 0.04),
        xyz=(0.5 * GATE_PANEL_WIDTH, -0.009, GATE_PANEL_HEIGHT - 0.02),
        material=material,
    )
    _box_visual(
        part,
        name="bottom_rail",
        size=(GATE_RAIL_WIDTH, 0.024, 0.04),
        xyz=(0.5 * GATE_PANEL_WIDTH, -0.009, 0.02),
        material=material,
    )
    _add_diagonal_bar(part, name="lower_up_bar", x1=left_x, z1=low_z, x2=right_x, z2=mid_z, material=material)
    _add_diagonal_bar(part, name="lower_down_bar", x1=left_x, z1=mid_z, x2=right_x, z2=low_z, material=material)
    _add_diagonal_bar(part, name="upper_up_bar", x1=left_x, z1=mid_z, x2=right_x, z2=high_z, material=material)
    _add_diagonal_bar(part, name="upper_down_bar", x1=left_x, z1=high_z, x2=right_x, z2=mid_z, material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_elevator")

    steel = model.material("steel", rgba=(0.57, 0.60, 0.63, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    shadow_steel = model.material("shadow_steel", rgba=(0.18, 0.19, 0.21, 1.0))

    shaft = model.part("shaft")
    _box_visual(shaft, name="shaft_floor", size=(SHAFT_WIDTH, SHAFT_DEPTH, 0.04), xyz=(0.0, 0.0, 0.02), material=dark_steel)
    _box_visual(
        shaft,
        name="left_post",
        size=(0.04, SHAFT_DEPTH, SHAFT_HEIGHT),
        xyz=(-0.58, 0.0, SHAFT_HEIGHT * 0.5),
        material=dark_steel,
    )
    _box_visual(
        shaft,
        name="right_post",
        size=(0.04, SHAFT_DEPTH, SHAFT_HEIGHT),
        xyz=(0.58, 0.0, SHAFT_HEIGHT * 0.5),
        material=dark_steel,
    )
    _box_visual(
        shaft,
        name="back_wall",
        size=(SHAFT_WIDTH, 0.04, SHAFT_HEIGHT),
        xyz=(0.0, -0.62, SHAFT_HEIGHT * 0.5),
        material=shadow_steel,
    )
    _box_visual(
        shaft,
        name="top_beam",
        size=(SHAFT_WIDTH, SHAFT_DEPTH, 0.06),
        xyz=(0.0, 0.0, SHAFT_HEIGHT + 0.03),
        material=dark_steel,
    )
    _box_visual(
        shaft,
        name="left_guide_rail",
        size=(0.04, 0.06, SHAFT_HEIGHT - 0.08),
        xyz=(-0.52, -0.50, 0.5 * SHAFT_HEIGHT),
        material=steel,
    )
    _box_visual(
        shaft,
        name="right_guide_rail",
        size=(0.04, 0.06, SHAFT_HEIGHT - 0.08),
        xyz=(0.52, -0.50, 0.5 * SHAFT_HEIGHT),
        material=steel,
    )

    car = model.part("car")
    _box_visual(car, name="floor", size=(CAR_WIDTH, 0.88, 0.05), xyz=(0.0, -0.03, 0.025), material=steel)
    _box_visual(
        car,
        name="threshold",
        size=(CAR_WIDTH, 0.10, 0.09),
        xyz=(0.0, 0.45, 0.045),
        material=steel,
    )
    _box_visual(
        car,
        name="left_wall",
        size=(0.04, 0.92, 1.86),
        xyz=(-0.47, 0.0, 0.98),
        material=steel,
    )
    _box_visual(
        car,
        name="right_wall",
        size=(0.04, 0.92, 1.86),
        xyz=(0.47, 0.0, 0.98),
        material=steel,
    )
    _box_visual(car, name="back_wall", size=(CAR_WIDTH, 0.04, 1.86), xyz=(0.0, -0.46, 0.98), material=steel)
    _box_visual(car, name="roof", size=(CAR_WIDTH, 0.92, 0.05), xyz=(0.0, 0.0, 1.935), material=steel)
    _box_visual(
        car,
        name="front_header",
        size=(CAR_WIDTH, 0.05, 0.18),
        xyz=(0.0, 0.445, OPENING_BOTTOM + OPENING_HEIGHT + 0.09),
        material=steel,
    )
    _box_visual(
        car,
        name="left_jamb",
        size=(0.09, 0.05, OPENING_HEIGHT),
        xyz=(-0.445, 0.445, OPENING_BOTTOM + 0.5 * OPENING_HEIGHT),
        material=steel,
    )
    _box_visual(
        car,
        name="right_jamb",
        size=(0.09, 0.05, OPENING_HEIGHT),
        xyz=(0.445, 0.445, OPENING_BOTTOM + 0.5 * OPENING_HEIGHT),
        material=steel,
    )
    _box_visual(
        car,
        name="top_track",
        size=(OPENING_WIDTH + 0.02, 0.018, 0.04),
        xyz=(0.0, 0.428, OPENING_BOTTOM + GATE_PANEL_HEIGHT + 0.02),
        material=shadow_steel,
    )
    _box_visual(
        car,
        name="bottom_track",
        size=(OPENING_WIDTH + 0.02, 0.018, 0.04),
        xyz=(0.0, 0.428, OPENING_BOTTOM + 0.02),
        material=shadow_steel,
    )
    _box_visual(
        car,
        name="door_hinge_mount",
        size=(0.010, 0.030, 1.60),
        xyz=(-0.405, 0.405, OPENING_BOTTOM + 0.80),
        material=shadow_steel,
    )
    car.visual(
        Cylinder(radius=0.004, length=1.64),
        origin=Origin(xyz=(-0.39, DOOR_PLANE_Y, OPENING_BOTTOM + 0.82)),
        material=shadow_steel,
        name="door_hinge_pin",
    )
    _box_visual(
        car,
        name="door_hinge_bracket_lower",
        size=(0.008, 0.010, 0.20),
        xyz=(-0.397, 0.393, OPENING_BOTTOM + 0.48),
        material=shadow_steel,
    )
    _box_visual(
        car,
        name="door_hinge_bracket_upper",
        size=(0.008, 0.010, 0.20),
        xyz=(-0.397, 0.393, OPENING_BOTTOM + 1.14),
        material=shadow_steel,
    )
    _box_visual(
        car,
        name="left_lower_guide_block",
        size=(0.06, 0.06, 0.20),
        xyz=(-0.52, -0.44, 0.36),
        material=shadow_steel,
    )
    _box_visual(
        car,
        name="right_lower_guide_block",
        size=(0.06, 0.06, 0.20),
        xyz=(0.52, -0.44, 0.36),
        material=shadow_steel,
    )
    _box_visual(
        car,
        name="left_upper_guide_block",
        size=(0.06, 0.06, 0.20),
        xyz=(-0.52, -0.44, 1.56),
        material=shadow_steel,
    )
    _box_visual(
        car,
        name="right_upper_guide_block",
        size=(0.06, 0.06, 0.20),
        xyz=(0.52, -0.44, 1.56),
        material=shadow_steel,
    )

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.8, lower=0.0, upper=CAR_TRAVEL),
    )

    door = model.part("inner_door")
    _box_visual(
        door,
        name="panel",
        size=(0.76, 0.035, OPENING_HEIGHT),
        xyz=(0.40, 0.0, 0.5 * OPENING_HEIGHT),
        material=steel,
    )
    _box_visual(
        door,
        name="handle",
        size=(0.03, 0.07, 0.12),
        xyz=(0.72, -0.035, 0.88),
        material=shadow_steel,
    )
    for name, z_pos in (
        ("hinge_knuckle_lower", 0.18),
        ("hinge_knuckle_mid", 0.81),
        ("hinge_knuckle_upper", 1.44),
    ):
        door.visual(
            Cylinder(radius=0.007, length=0.22),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=shadow_steel,
            name=name,
        )
        _box_visual(
            door,
            name=f"{name}_strap",
            size=(0.020, 0.012, 0.18),
            xyz=(0.017, 0.0, z_pos),
            material=shadow_steel,
        )
    model.articulation(
        "car_to_inner_door",
        ArticulationType.REVOLUTE,
        parent=car,
        child=door,
        origin=Origin(xyz=(-0.39, DOOR_PLANE_Y, OPENING_BOTTOM)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=0.0, upper=1.25),
    )

    gate_panels = [model.part(f"gate_panel_{index}") for index in range(1, GATE_PANEL_COUNT + 1)]
    for index, panel in enumerate(gate_panels, start=1):
        _add_gate_cell(panel, material=shadow_steel)
        if index == GATE_PANEL_COUNT:
            _box_visual(
                panel,
                name="pull_handle",
                size=(0.02, 0.04, 0.22),
                xyz=(GATE_PANEL_WIDTH - 0.01, -0.012, 0.90),
                material=shadow_steel,
            )

    model.articulation(
        "car_to_gate_panel_1",
        ArticulationType.FIXED,
        parent=car,
        child=gate_panels[0],
        origin=Origin(xyz=(-0.40, GATE_PLANE_Y, OPENING_BOTTOM)),
    )
    for index in range(1, GATE_PANEL_COUNT):
        model.articulation(
            f"gate_panel_{index}_to_gate_panel_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=gate_panels[index - 1],
            child=gate_panels[index],
            origin=Origin(xyz=(GATE_PANEL_WIDTH, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=40.0, velocity=1.8, lower=-1.10, upper=1.10),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    inner_door = object_model.get_part("inner_door")
    gate_panels = [object_model.get_part(f"gate_panel_{index}") for index in range(1, GATE_PANEL_COUNT + 1)]

    car_lift = object_model.get_articulation("shaft_to_car")
    door_hinge = object_model.get_articulation("car_to_inner_door")
    gate_hinges = [
        object_model.get_articulation(f"gate_panel_{index}_to_gate_panel_{index + 1}")
        for index in range(1, GATE_PANEL_COUNT)
    ]

    shaft_floor = shaft.get_visual("shaft_floor")
    left_guide_rail = shaft.get_visual("left_guide_rail")
    right_guide_rail = shaft.get_visual("right_guide_rail")
    car_floor = car.get_visual("floor")
    top_track = car.get_visual("top_track")
    bottom_track = car.get_visual("bottom_track")
    left_jamb = car.get_visual("left_jamb")
    right_jamb = car.get_visual("right_jamb")
    door_hinge_mount = car.get_visual("door_hinge_mount")
    door_hinge_pin = car.get_visual("door_hinge_pin")
    left_lower_guide_block = car.get_visual("left_lower_guide_block")
    right_lower_guide_block = car.get_visual("right_lower_guide_block")
    left_upper_guide_block = car.get_visual("left_upper_guide_block")
    right_upper_guide_block = car.get_visual("right_upper_guide_block")
    car_back_wall = car.get_visual("back_wall")

    door_panel = inner_door.get_visual("panel")
    door_handle = inner_door.get_visual("handle")
    door_knuckles = [
        inner_door.get_visual("hinge_knuckle_lower"),
        inner_door.get_visual("hinge_knuckle_mid"),
        inner_door.get_visual("hinge_knuckle_upper"),
    ]
    gate_top_rails = [panel.get_visual("top_rail") for panel in gate_panels]
    gate_bottom_rails = [panel.get_visual("bottom_rail") for panel in gate_panels]

    gate_left_stiles = [panel.get_visual("left_stile") for panel in gate_panels]
    gate_right_stiles = [panel.get_visual("right_stile") for panel in gate_panels]
    gate_pull_handle = gate_panels[-1].get_visual("pull_handle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    for knuckle in door_knuckles:
        ctx.allow_overlap(
            car,
            inner_door,
            elem_a=door_hinge_pin,
            elem_b=knuckle,
            reason="Door rotates on a steel hinge pin captured by interleaved hinge knuckles.",
        )
    ctx.fail_if_isolated_parts(max_pose_samples=10)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("car_lift_axis_vertical", tuple(car_lift.axis) == (0.0, 0.0, 1.0), details=f"axis={car_lift.axis}")
    ctx.check("door_hinge_axis_vertical", tuple(door_hinge.axis) == (0.0, 0.0, -1.0), details=f"axis={door_hinge.axis}")
    for index, hinge in enumerate(gate_hinges, start=1):
        ctx.check(
            f"gate_hinge_{index}_axis_vertical",
            tuple(hinge.axis) == (0.0, 0.0, 1.0),
            details=f"axis={hinge.axis}",
        )

    ctx.expect_origin_distance(car, shaft, axes="xy", max_dist=0.001)
    ctx.expect_gap(car, shaft, axis="z", positive_elem=car_floor, negative_elem=shaft_floor, max_gap=0.001, max_penetration=0.0)
    ctx.expect_contact(car, shaft, elem_a=left_lower_guide_block, elem_b=left_guide_rail)
    ctx.expect_contact(car, shaft, elem_a=right_lower_guide_block, elem_b=right_guide_rail)
    ctx.expect_contact(car, shaft, elem_a=left_upper_guide_block, elem_b=left_guide_rail)
    ctx.expect_contact(car, shaft, elem_a=right_upper_guide_block, elem_b=right_guide_rail)

    ctx.expect_gap(inner_door, car, axis="x", positive_elem=door_panel, negative_elem=door_hinge_mount, min_gap=0.02, max_gap=0.04)
    ctx.expect_overlap(inner_door, car, axes="z", elem_a=door_knuckles[1], elem_b=door_hinge_pin, min_overlap=0.20)
    ctx.expect_overlap(inner_door, car, axes="z", min_overlap=1.50)
    ctx.expect_gap(car, inner_door, axis="x", positive_elem=right_jamb, negative_elem=door_panel, max_gap=0.02, max_penetration=0.0)

    ctx.expect_gap(
        gate_panels[0],
        car,
        axis="x",
        positive_elem=gate_left_stiles[0],
        negative_elem=left_jamb,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        car,
        gate_panels[-1],
        axis="x",
        positive_elem=right_jamb,
        negative_elem=gate_right_stiles[-1],
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        gate_panels[-1],
        inner_door,
        axis="y",
        positive_elem=gate_pull_handle,
        negative_elem=door_panel,
        min_gap=0.02,
    )

    for index in range(GATE_PANEL_COUNT - 1):
        ctx.expect_contact(
            gate_panels[index],
            gate_panels[index + 1],
            elem_a=gate_right_stiles[index],
            elem_b=gate_left_stiles[index + 1],
        )
    for gate_panel, top_rail, bottom_rail in zip(gate_panels, gate_top_rails, gate_bottom_rails):
        ctx.expect_contact(gate_panel, car, elem_a=top_rail, elem_b=top_track)
        ctx.expect_contact(gate_panel, car, elem_a=bottom_rail, elem_b=bottom_track)
        ctx.expect_overlap(gate_panel, car, axes="z", min_overlap=1.40)

    with ctx.pose({car_lift: CAR_TRAVEL}):
        ctx.fail_if_isolated_parts(name="car_top_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="car_top_no_overlap")
        ctx.expect_origin_distance(car, shaft, axes="xy", max_dist=0.001)
        ctx.expect_gap(
            car,
            shaft,
            axis="z",
            positive_elem=car_floor,
            negative_elem=shaft_floor,
            min_gap=CAR_TRAVEL - 0.001,
            max_gap=CAR_TRAVEL + 0.001,
        )
        ctx.expect_contact(car, shaft, elem_a=left_lower_guide_block, elem_b=left_guide_rail)
        ctx.expect_contact(car, shaft, elem_a=right_lower_guide_block, elem_b=right_guide_rail)
        ctx.expect_contact(car, shaft, elem_a=left_upper_guide_block, elem_b=left_guide_rail)
        ctx.expect_contact(car, shaft, elem_a=right_upper_guide_block, elem_b=right_guide_rail)

    folded_gate_pose = {
        gate_hinges[0]: 0.60,
        gate_hinges[1]: -0.60,
        gate_hinges[2]: 0.60,
    }
    with ctx.pose(folded_gate_pose):
        ctx.fail_if_isolated_parts(name="gate_folded_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="gate_folded_no_overlap")
        ctx.expect_gap(
            car,
            gate_panels[-1],
            axis="x",
            positive_elem=right_jamb,
            negative_elem=gate_right_stiles[-1],
            min_gap=0.06,
        )
        ctx.expect_gap(
            gate_panels[-1],
            inner_door,
            axis="y",
            positive_elem=gate_pull_handle,
            negative_elem=door_panel,
            min_gap=0.02,
        )

    with ctx.pose({door_hinge: 1.15}):
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.expect_overlap(inner_door, car, axes="z", elem_a=door_knuckles[1], elem_b=door_hinge_pin, min_overlap=0.20)
        ctx.expect_gap(
            car,
            inner_door,
            axis="y",
            positive_elem=top_track,
            negative_elem=door_handle,
            min_gap=0.01,
        )
        ctx.expect_gap(
            inner_door,
            car,
            axis="y",
            positive_elem=door_panel,
            negative_elem=car_back_wall,
            min_gap=0.08,
        )

    with ctx.pose({car_lift: CAR_TRAVEL, door_hinge: 1.15, **folded_gate_pose}):
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.expect_contact(car, shaft, elem_a=left_lower_guide_block, elem_b=left_guide_rail)
        ctx.expect_contact(car, shaft, elem_a=right_lower_guide_block, elem_b=right_guide_rail)
        ctx.expect_overlap(inner_door, car, axes="z", elem_a=door_knuckles[1], elem_b=door_hinge_pin, min_overlap=0.20)
        ctx.expect_gap(
            gate_panels[-1],
            inner_door,
            axis="y",
            positive_elem=gate_pull_handle,
            negative_elem=door_panel,
            min_gap=0.02,
        )
        ctx.expect_gap(
            car,
            gate_panels[-1],
            axis="x",
            positive_elem=right_jamb,
            negative_elem=gate_right_stiles[-1],
            min_gap=0.06,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
