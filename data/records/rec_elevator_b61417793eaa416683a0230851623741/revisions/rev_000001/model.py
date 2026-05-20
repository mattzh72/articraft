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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _interior_positions(start: float, end: float, count: int) -> list[float]:
    if count <= 0:
        return []
    span = end - start
    return [start + span * (index + 1) / (count + 1) for index in range(count)]


def _add_xz_mesh_panel(
    part,
    *,
    y: float,
    x_min: float,
    x_max: float,
    z_min: float,
    z_max: float,
    wire_radius: float,
    vertical_count: int,
    horizontal_count: int,
    material,
) -> None:
    z_center = 0.5 * (z_min + z_max)
    x_center = 0.5 * (x_min + x_max)
    z_span = z_max - z_min
    x_span = x_max - x_min

    for x in _interior_positions(x_min, x_max, vertical_count):
        part.visual(
            Cylinder(radius=wire_radius, length=z_span),
            origin=Origin(xyz=(x, y, z_center)),
            material=material,
        )

    for z in _interior_positions(z_min, z_max, horizontal_count):
        part.visual(
            Cylinder(radius=wire_radius, length=x_span),
            origin=Origin(xyz=(x_center, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
        )


def _add_yz_mesh_panel(
    part,
    *,
    x: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
    wire_radius: float,
    vertical_count: int,
    horizontal_count: int,
    material,
) -> None:
    z_center = 0.5 * (z_min + z_max)
    y_center = 0.5 * (y_min + y_max)
    z_span = z_max - z_min
    y_span = y_max - y_min

    for y in _interior_positions(y_min, y_max, vertical_count):
        part.visual(
            Cylinder(radius=wire_radius, length=z_span),
            origin=Origin(xyz=(x, y, z_center)),
            material=material,
        )

    for z in _interior_positions(z_min, z_max, horizontal_count):
        part.visual(
            Cylinder(radius=wire_radius, length=y_span),
            origin=Origin(xyz=(x, y_center, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_goods_elevator")

    painted_steel = model.material("painted_steel", rgba=(0.31, 0.34, 0.38, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    galvanized = model.material("galvanized", rgba=(0.76, 0.78, 0.80, 1.0))
    mesh_steel = model.material("mesh_steel", rgba=(0.63, 0.66, 0.69, 1.0))
    deck_plate = model.material("deck_plate", rgba=(0.54, 0.56, 0.60, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(0.86, 0.70, 0.12, 1.0))

    tower = model.part("tower")
    tower.inertial = Inertial.from_geometry(
        Box((1.50, 1.35, 4.35)),
        mass=720.0,
        origin=Origin(xyz=(0.0, -0.10, 2.18)),
    )

    tower.visual(
        Box((1.18, 0.20, 0.10)),
        origin=Origin(xyz=(0.0, -0.60, 0.05)),
        material=dark_steel,
        name="rear_base_beam",
    )
    tower.visual(
        Box((1.38, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.60, 0.04)),
        material=dark_steel,
        name="front_base_beam",
    )
    for x_sign in (-1.0, 1.0):
        tower.visual(
            Box((0.10, 1.18, 0.08)),
            origin=Origin(xyz=(x_sign * 0.62, 0.07, 0.04)),
            material=dark_steel,
        )
        tower.visual(
            Box((0.18, 0.20, 0.16)),
            origin=Origin(xyz=(x_sign * 0.48, -0.60, 0.08)),
            material=dark_steel,
        )
        tower.visual(
            Box((0.10, 0.10, 4.00)),
            origin=Origin(xyz=(x_sign * 0.48, -0.60, 2.08)),
            material=painted_steel,
        )

    for z, depth in ((0.34, 0.10), (2.06, 0.08), (4.04, 0.10)):
        tower.visual(
            Box((1.06, depth, 0.08)),
            origin=Origin(xyz=(0.0, -0.665, z)),
            material=painted_steel,
        )

    for z in (0.28, 1.22, 2.18, 3.14):
        tower.visual(
            Box((0.98, 0.08, 0.06)),
            origin=Origin(xyz=(0.0, -0.665, z)),
            material=painted_steel,
        )

    tower.visual(
        Box((0.56, 0.30, 0.26)),
        origin=Origin(xyz=(0.0, -0.60, 4.17)),
        material=dark_steel,
        name="machine_head",
    )
    tower.visual(
        Cylinder(radius=0.09, length=0.34),
        origin=Origin(xyz=(0.0, -0.57, 4.08), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="top_sheave_drum",
    )

    tower.visual(
        Box((0.06, 0.09, 3.76)),
        origin=Origin(xyz=(-0.34, -0.63, 2.02)),
        material=galvanized,
        name="left_guide_rail",
    )
    tower.visual(
        Box((0.04, 0.04, 3.76)),
        origin=Origin(xyz=(-0.34, -0.695, 2.02)),
        material=mesh_steel,
    )
    tower.visual(
        Box((0.06, 0.09, 3.76)),
        origin=Origin(xyz=(0.34, -0.63, 2.02)),
        material=galvanized,
        name="right_guide_rail",
    )
    tower.visual(
        Box((0.04, 0.04, 3.76)),
        origin=Origin(xyz=(0.34, -0.695, 2.02)),
        material=mesh_steel,
    )
    for x_sign in (-1.0, 1.0):
        x_pos = x_sign * 0.41
        tower.visual(
            Box((0.16, 0.10, 0.08)),
            origin=Origin(xyz=(x_pos, -0.615, 0.34)),
            material=painted_steel,
        )
        tower.visual(
            Box((0.16, 0.10, 0.08)),
            origin=Origin(xyz=(x_pos, -0.615, 3.72)),
            material=painted_steel,
        )

    car = model.part("car")
    car.inertial = Inertial.from_geometry(
        Box((1.18, 1.05, 1.95)),
        mass=360.0,
        origin=Origin(xyz=(0.0, -0.02, 0.98)),
    )

    car.visual(
        Box((1.10, 0.94, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=deck_plate,
        name="car_floor",
    )
    car.visual(
        Box((0.94, 0.60, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=warning_yellow,
        name="floor_marking",
    )
    car.visual(
        Box((0.06, 0.94, 0.10)),
        origin=Origin(xyz=(-0.53, 0.0, 0.05)),
        material=painted_steel,
    )
    car.visual(
        Box((0.06, 0.94, 0.10)),
        origin=Origin(xyz=(0.53, 0.0, 0.05)),
        material=painted_steel,
    )
    car.visual(
        Box((1.00, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, -0.44, 0.05)),
        material=painted_steel,
    )
    car.visual(
        Box((1.00, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, 0.455, 0.04)),
        material=painted_steel,
        name="front_threshold",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            car.visual(
                Box((0.05, 0.05, 1.72)),
                origin=Origin(xyz=(x_sign * 0.535, y_sign * 0.445, 0.94)),
                material=painted_steel,
            )

    car.visual(
        Box((0.06, 0.94, 0.08)),
        origin=Origin(xyz=(-0.53, 0.0, 1.84)),
        material=painted_steel,
    )
    car.visual(
        Box((0.06, 0.94, 0.08)),
        origin=Origin(xyz=(0.53, 0.0, 1.84)),
        material=painted_steel,
    )
    car.visual(
        Box((1.00, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, -0.445, 1.84)),
        material=painted_steel,
    )
    car.visual(
        Box((1.00, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 0.445, 1.84)),
        material=painted_steel,
    )

    car.visual(
        Box((0.05, 0.90, 0.05)),
        origin=Origin(xyz=(-0.535, 0.0, 1.00)),
        material=painted_steel,
    )
    car.visual(
        Box((0.05, 0.90, 0.05)),
        origin=Origin(xyz=(0.535, 0.0, 1.00)),
        material=painted_steel,
    )
    car.visual(
        Box((0.12, 0.08, 1.60)),
        origin=Origin(xyz=(0.0, -0.54, 0.94)),
        material=dark_steel,
        name="carriage_spine",
    )
    car.visual(
        Box((0.04, 0.05, 1.70)),
        origin=Origin(xyz=(-0.12, -0.445, 0.94)),
        material=painted_steel,
    )
    car.visual(
        Box((0.04, 0.05, 1.70)),
        origin=Origin(xyz=(0.12, -0.445, 0.94)),
        material=painted_steel,
    )
    for z in (0.46, 1.42):
        car.visual(
            Box((0.30, 0.08, 0.08)),
            origin=Origin(xyz=(0.0, -0.49, z)),
            material=dark_steel,
        )

    for z in (0.46, 1.42):
        car.visual(
            Box((0.70, 0.06, 0.08)),
            origin=Origin(xyz=(0.0, -0.54, z)),
            material=dark_steel,
        )
        for x_pos in (-0.34, 0.34):
            car.visual(
                Box((0.10, 0.08, 0.08)),
                origin=Origin(xyz=(x_pos, -0.545, z)),
                material=dark_steel,
            )

    car.visual(
        Box((0.10, 0.06, 0.18)),
        origin=Origin(xyz=(-0.34, -0.555, 0.46)),
        material=galvanized,
        name="left_lower_guide_shoe",
    )
    car.visual(
        Box((0.10, 0.06, 0.18)),
        origin=Origin(xyz=(0.34, -0.555, 0.46)),
        material=galvanized,
        name="right_lower_guide_shoe",
    )
    car.visual(
        Box((0.10, 0.06, 0.18)),
        origin=Origin(xyz=(-0.34, -0.555, 1.42)),
        material=galvanized,
        name="left_upper_guide_shoe",
    )
    car.visual(
        Box((0.10, 0.06, 0.18)),
        origin=Origin(xyz=(0.34, -0.555, 1.42)),
        material=galvanized,
        name="right_upper_guide_shoe",
    )

    _add_yz_mesh_panel(
        car,
        x=-0.535,
        y_min=-0.40,
        y_max=0.40,
        z_min=0.12,
        z_max=1.76,
        wire_radius=0.006,
        vertical_count=4,
        horizontal_count=6,
        material=mesh_steel,
    )
    _add_yz_mesh_panel(
        car,
        x=0.535,
        y_min=-0.40,
        y_max=0.40,
        z_min=0.12,
        z_max=1.76,
        wire_radius=0.006,
        vertical_count=4,
        horizontal_count=6,
        material=mesh_steel,
    )
    _add_xz_mesh_panel(
        car,
        y=-0.445,
        x_min=-0.49,
        x_max=-0.14,
        z_min=0.12,
        z_max=1.76,
        wire_radius=0.006,
        vertical_count=3,
        horizontal_count=6,
        material=mesh_steel,
    )
    _add_xz_mesh_panel(
        car,
        y=-0.445,
        x_min=0.14,
        x_max=0.49,
        z_min=0.12,
        z_max=1.76,
        wire_radius=0.006,
        vertical_count=3,
        horizontal_count=6,
        material=mesh_steel,
    )

    car.visual(
        Box((0.02, 0.05, 1.72)),
        origin=Origin(xyz=(-0.515, 0.455, 0.94)),
        material=dark_steel,
        name="left_hinge_jamb",
    )

    gate = model.part("gate")
    gate.inertial = Inertial.from_geometry(
        Box((1.02, 0.08, 1.78)),
        mass=55.0,
        origin=Origin(xyz=(0.49, 0.0, 0.89)),
    )

    gate.visual(
        Box((0.04, 0.04, 1.66)),
        origin=Origin(xyz=(0.02, 0.0, 0.87)),
        material=painted_steel,
        name="gate_hinge_stile",
    )
    gate.visual(
        Box((0.04, 0.04, 1.66)),
        origin=Origin(xyz=(0.96, 0.0, 0.87)),
        material=painted_steel,
        name="gate_latch_stile",
    )
    gate.visual(
        Box((0.94, 0.04, 0.06)),
        origin=Origin(xyz=(0.49, 0.0, 0.03)),
        material=painted_steel,
        name="gate_bottom_rail",
    )
    gate.visual(
        Box((0.94, 0.04, 0.06)),
        origin=Origin(xyz=(0.49, 0.0, 1.71)),
        material=painted_steel,
        name="gate_top_rail",
    )
    gate.visual(
        Box((0.94, 0.03, 0.04)),
        origin=Origin(xyz=(0.49, 0.0, 0.90)),
        material=painted_steel,
        name="gate_mid_rail",
    )
    gate.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(xyz=(0.0, -0.020, 0.18)),
        material=galvanized,
    )
    gate.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(xyz=(0.0, -0.020, 0.89)),
        material=galvanized,
    )
    gate.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(xyz=(0.0, -0.020, 1.60)),
        material=galvanized,
    )
    gate.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(0.98, 0.022, 0.92), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warning_yellow,
    )

    _add_xz_mesh_panel(
        gate,
        y=0.0,
        x_min=0.07,
        x_max=0.91,
        z_min=0.09,
        z_max=1.65,
        wire_radius=0.005,
        vertical_count=4,
        horizontal_count=6,
        material=mesh_steel,
    )

    model.articulation(
        "tower_to_car",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.45, lower=0.0, upper=1.90),
    )
    model.articulation(
        "car_to_gate",
        ArticulationType.REVOLUTE,
        parent=car,
        child=gate,
        origin=Origin(xyz=(-0.485, 0.50, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    tower = object_model.get_part("tower")
    car = object_model.get_part("car")
    gate = object_model.get_part("gate")
    lift_joint = object_model.get_articulation("tower_to_car")
    gate_joint = object_model.get_articulation("car_to_gate")

    lift_upper = 1.90
    gate_upper = 1.45

    ctx.expect_contact(
        car,
        tower,
        elem_a="left_lower_guide_shoe",
        elem_b="left_guide_rail",
        contact_tol=0.001,
        name="left lower shoe bears on left guide rail at landing",
    )
    ctx.expect_contact(
        car,
        tower,
        elem_a="right_lower_guide_shoe",
        elem_b="right_guide_rail",
        contact_tol=0.001,
        name="right lower shoe bears on right guide rail at landing",
    )

    rest_car_pos = ctx.part_world_position(car)
    with ctx.pose({lift_joint: lift_upper}):
        ctx.expect_contact(
            car,
            tower,
            elem_a="left_upper_guide_shoe",
            elem_b="left_guide_rail",
            contact_tol=0.001,
            name="left upper shoe stays on the guide rail at top travel",
        )
        ctx.expect_contact(
            car,
            tower,
            elem_a="right_upper_guide_shoe",
            elem_b="right_guide_rail",
            contact_tol=0.001,
            name="right upper shoe stays on the guide rail at top travel",
        )
        top_car_pos = ctx.part_world_position(car)

    ctx.check(
        "car lifts upward along the mast",
        rest_car_pos is not None
        and top_car_pos is not None
        and top_car_pos[2] > rest_car_pos[2] + 1.80,
        details=f"rest={rest_car_pos}, top={top_car_pos}",
    )

    ctx.expect_overlap(
        gate,
        car,
        axes="xz",
        min_overlap=0.90,
        name="closed gate spans the front opening footprint",
    )

    closed_latch_aabb = ctx.part_element_world_aabb(gate, elem="gate_latch_stile")
    with ctx.pose({gate_joint: gate_upper}):
        open_latch_aabb = ctx.part_element_world_aabb(gate, elem="gate_latch_stile")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    closed_center = _aabb_center(closed_latch_aabb)
    open_center = _aabb_center(open_latch_aabb)
    ctx.check(
        "gate latch swings outward from the opening",
        closed_center is not None
        and open_center is not None
        and open_center[1] > closed_center[1] + 0.65
        and open_center[0] < closed_center[0] - 0.35,
        details=f"closed={closed_center}, open={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
