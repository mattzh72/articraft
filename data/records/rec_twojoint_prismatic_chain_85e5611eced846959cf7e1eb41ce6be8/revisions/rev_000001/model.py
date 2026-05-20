from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import ArticulatedObject, ArticulationType, Box, Inertial, MotionLimits, Origin, TestContext, TestReport


OUTER_LENGTH = 0.46
OUTER_SIZE = 0.10
OUTER_WALL = 0.008
REAR_CAP = 0.018
BASE_LENGTH = 0.18
BASE_WIDTH = 0.14
BASE_THICKNESS = 0.014

MID_LENGTH = 0.52
MID_SIZE = 0.072
MID_WALL = 0.006
MID_GUIDE_OUTER = OUTER_SIZE - 2.0 * OUTER_WALL
MID_GUIDE_START = 0.025
MID_GUIDE_LENGTH = 0.080
MID_JOINT_X = 0.020
MID_STROKE = 0.30

FRONT_LENGTH = 0.35
FRONT_SIZE = 0.050
FRONT_WALL = 0.005
FRONT_GUIDE_OUTER = MID_SIZE - 2.0 * MID_WALL
FRONT_GUIDE_START = 0.035
FRONT_GUIDE_LENGTH = 0.060
FRONT_JOINT_X = 0.170
FRONT_STROKE = 0.18

PLATE_THICKNESS = 0.012
PLATE_WIDTH = 0.11
PLATE_HEIGHT = 0.11


def add_square_tube_visuals(
    part,
    *,
    name_prefix: str,
    x_start: float,
    length: float,
    outer_size: float,
    wall: float,
    material,
) -> None:
    half = outer_size * 0.5
    wall_center = half - wall * 0.5
    x_center = x_start + length * 0.5

    part.visual(
        Box((length, outer_size, wall)),
        origin=Origin(xyz=(x_center, 0.0, wall_center)),
        material=material,
        name=f"{name_prefix}_top",
    )
    part.visual(
        Box((length, outer_size, wall)),
        origin=Origin(xyz=(x_center, 0.0, -wall_center)),
        material=material,
        name=f"{name_prefix}_bottom",
    )
    part.visual(
        Box((length, wall, outer_size)),
        origin=Origin(xyz=(x_center, wall_center, 0.0)),
        material=material,
        name=f"{name_prefix}_left",
    )
    part.visual(
        Box((length, wall, outer_size)),
        origin=Origin(xyz=(x_center, -wall_center, 0.0)),
        material=material,
        name=f"{name_prefix}_right",
    )


def add_square_guide_visuals(
    part,
    *,
    name_prefix: str,
    x_start: float,
    length: float,
    base_outer_size: float,
    guide_outer_size: float,
    material,
) -> None:
    pad = (guide_outer_size - base_outer_size) * 0.5
    if pad <= 0.0:
        return

    base_half = base_outer_size * 0.5
    pad_center = base_half + pad * 0.5
    x_center = x_start + length * 0.5

    part.visual(
        Box((length, guide_outer_size, pad)),
        origin=Origin(xyz=(x_center, 0.0, pad_center)),
        material=material,
        name=f"{name_prefix}_top",
    )
    part.visual(
        Box((length, guide_outer_size, pad)),
        origin=Origin(xyz=(x_center, 0.0, -pad_center)),
        material=material,
        name=f"{name_prefix}_bottom",
    )
    part.visual(
        Box((length, pad, guide_outer_size)),
        origin=Origin(xyz=(x_center, pad_center, 0.0)),
        material=material,
        name=f"{name_prefix}_left",
    )
    part.visual(
        Box((length, pad, guide_outer_size)),
        origin=Origin(xyz=(x_center, -pad_center, 0.0)),
        material=material,
        name=f"{name_prefix}_right",
    )


def add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inline_telescoping_ram")

    housing_mat = model.material("housing_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    slider_mat = model.material("slider_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    plate_mat = model.material("plate_steel", rgba=(0.58, 0.61, 0.66, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    add_square_tube_visuals(
        outer_sleeve,
        name_prefix="sleeve",
        x_start=REAR_CAP,
        length=OUTER_LENGTH - REAR_CAP,
        outer_size=OUTER_SIZE,
        wall=OUTER_WALL,
        material=housing_mat,
    )
    add_box_visual(
        outer_sleeve,
        name="rear_cap",
        size=(REAR_CAP, OUTER_SIZE, OUTER_SIZE),
        center=(REAR_CAP * 0.5, 0.0, 0.0),
        material=housing_mat,
    )
    add_box_visual(
        outer_sleeve,
        name="mount_base",
        size=(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS),
        center=(0.035 + BASE_LENGTH * 0.5, 0.0, -OUTER_SIZE * 0.5 - BASE_THICKNESS * 0.5),
        material=housing_mat,
    )
    add_box_visual(
        outer_sleeve,
        name="foot_left",
        size=(0.030, 0.030, 0.036),
        center=(0.080, 0.040, -0.082),
        material=housing_mat,
    )
    add_box_visual(
        outer_sleeve,
        name="foot_right",
        size=(0.030, 0.030, 0.036),
        center=(0.080, -0.040, -0.082),
        material=housing_mat,
    )
    add_square_guide_visuals(
        outer_sleeve,
        name_prefix="nose_collar",
        x_start=OUTER_LENGTH - 0.040,
        length=0.040,
        base_outer_size=OUTER_SIZE,
        guide_outer_size=OUTER_SIZE + 0.012,
        material=housing_mat,
    )
    outer_sleeve.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, BASE_WIDTH, OUTER_SIZE + BASE_THICKNESS)),
        mass=11.0,
        origin=Origin(xyz=(OUTER_LENGTH * 0.5, 0.0, -BASE_THICKNESS * 0.18)),
    )

    intermediate_tube = model.part("intermediate_tube")
    add_square_tube_visuals(
        intermediate_tube,
        name_prefix="intermediate",
        x_start=0.0,
        length=MID_LENGTH,
        outer_size=MID_SIZE,
        wall=MID_WALL,
        material=slider_mat,
    )
    add_square_guide_visuals(
        intermediate_tube,
        name_prefix="intermediate_rear_guide",
        x_start=MID_GUIDE_START,
        length=MID_GUIDE_LENGTH,
        base_outer_size=MID_SIZE,
        guide_outer_size=MID_GUIDE_OUTER,
        material=slider_mat,
    )
    intermediate_tube.inertial = Inertial.from_geometry(
        Box((MID_LENGTH, MID_GUIDE_OUTER, MID_GUIDE_OUTER)),
        mass=6.0,
        origin=Origin(xyz=(MID_LENGTH * 0.5, 0.0, 0.0)),
    )

    front_output_tube = model.part("front_output_tube")
    add_square_tube_visuals(
        front_output_tube,
        name_prefix="front",
        x_start=0.0,
        length=FRONT_LENGTH,
        outer_size=FRONT_SIZE,
        wall=FRONT_WALL,
        material=slider_mat,
    )
    add_square_guide_visuals(
        front_output_tube,
        name_prefix="front_rear_guide",
        x_start=FRONT_GUIDE_START,
        length=FRONT_GUIDE_LENGTH,
        base_outer_size=FRONT_SIZE,
        guide_outer_size=FRONT_GUIDE_OUTER,
        material=slider_mat,
    )
    add_box_visual(
        front_output_tube,
        name="tool_plate",
        size=(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT),
        center=(FRONT_LENGTH + PLATE_THICKNESS * 0.5, 0.0, 0.0),
        material=plate_mat,
    )
    add_box_visual(
        front_output_tube,
        name="tool_plate_rib_top",
        size=(0.028, 0.036, 0.008),
        center=(FRONT_LENGTH - 0.012, 0.0, 0.016),
        material=plate_mat,
    )
    add_box_visual(
        front_output_tube,
        name="tool_plate_rib_bottom",
        size=(0.028, 0.036, 0.008),
        center=(FRONT_LENGTH - 0.012, 0.0, -0.016),
        material=plate_mat,
    )
    front_output_tube.inertial = Inertial.from_geometry(
        Box((FRONT_LENGTH + PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)),
        mass=4.0,
        origin=Origin(xyz=((FRONT_LENGTH + PLATE_THICKNESS) * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "outer_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=intermediate_tube,
        origin=Origin(xyz=(MID_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.25, lower=0.0, upper=MID_STROKE),
    )
    model.articulation(
        "intermediate_to_front",
        ArticulationType.PRISMATIC,
        parent=intermediate_tube,
        child=front_output_tube,
        origin=Origin(xyz=(FRONT_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.25, lower=0.0, upper=FRONT_STROKE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_sleeve = object_model.get_part("outer_sleeve")
    intermediate_tube = object_model.get_part("intermediate_tube")
    front_output_tube = object_model.get_part("front_output_tube")
    outer_to_intermediate = object_model.get_articulation("outer_to_intermediate")
    intermediate_to_front = object_model.get_articulation("intermediate_to_front")
    tool_plate = front_output_tube.get_visual("tool_plate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "required_parts_present",
        {part.name for part in object_model.parts} == {"outer_sleeve", "intermediate_tube", "front_output_tube"},
        "Expected grounded sleeve, intermediate tube, and front output tube.",
    )
    ctx.check(
        "required_prismatic_joints_present",
        {joint.name for joint in object_model.articulations} == {"outer_to_intermediate", "intermediate_to_front"},
        "Expected two serial prismatic joints.",
    )
    ctx.check(
        "shared_axis_prismatic_setup",
        (
            outer_to_intermediate.articulation_type == ArticulationType.PRISMATIC
            and intermediate_to_front.articulation_type == ArticulationType.PRISMATIC
            and tuple(outer_to_intermediate.axis) == (1.0, 0.0, 0.0)
            and tuple(intermediate_to_front.axis) == (1.0, 0.0, 0.0)
            and outer_to_intermediate.motion_limits is not None
            and intermediate_to_front.motion_limits is not None
            and outer_to_intermediate.motion_limits.lower == 0.0
            and intermediate_to_front.motion_limits.lower == 0.0
            and outer_to_intermediate.motion_limits.upper == MID_STROKE
            and intermediate_to_front.motion_limits.upper == FRONT_STROKE
        ),
        "Both stages should slide on the shared +X axis with finite stroke.",
    )
    ctx.check(
        "tube_sizes_step_down",
        OUTER_SIZE > MID_SIZE > FRONT_SIZE,
        "Sleeve, intermediate tube, and front tube should step down in size.",
    )

    ctx.expect_contact(
        intermediate_tube,
        outer_sleeve,
        name="intermediate_supported_by_outer_sleeve",
    )
    ctx.expect_contact(
        front_output_tube,
        intermediate_tube,
        name="front_stage_supported_by_intermediate",
    )
    ctx.expect_gap(
        front_output_tube,
        outer_sleeve,
        axis="x",
        positive_elem=tool_plate,
        min_gap=0.015,
        name="tool_plate_projects_ahead_of_grounded_sleeve",
    )

    with ctx.pose({outer_to_intermediate: MID_STROKE, intermediate_to_front: FRONT_STROKE}):
        ctx.expect_contact(
            intermediate_tube,
            outer_sleeve,
            name="intermediate_remains_guided_at_full_stroke",
        )
        ctx.expect_contact(
            front_output_tube,
            intermediate_tube,
            name="front_stage_remains_guided_at_full_stroke",
        )
        ctx.expect_gap(
            front_output_tube,
            outer_sleeve,
            axis="x",
            positive_elem=tool_plate,
            min_gap=0.35,
            name="tool_plate_advances_far_forward_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
