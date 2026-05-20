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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


LEFT_LEG = (-0.62, 0.04)
RIGHT_LEG = (0.18, -0.18)
REAR_LEG = (0.06, 0.62)

COLUMN_OUTER = 0.088
COLUMN_WALL = 0.008
COLUMN_INNER = 0.064
SLEEVE_BOTTOM_Z = 0.11
SLEEVE_LENGTH = 0.36
SLEEVE_CENTER_Z = SLEEVE_BOTTOM_Z + SLEEVE_LENGTH / 2.0
HEIGHT_JOINT_Z = SLEEVE_BOTTOM_Z + SLEEVE_LENGTH
HEIGHT_TRAVEL = 0.27


def _beam(
    part,
    *,
    name: str,
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    height: float,
    z: float,
    material,
    clearance: float = 0.0,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    distance = math.hypot(dx, dy)
    if distance <= 2.0 * clearance + 1e-6:
        return
    yaw = math.atan2(dy, dx)
    part.visual(
        Box((distance - 2.0 * clearance, width, height)),
        origin=Origin(
            xyz=((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, z),
            rpy=(0.0, 0.0, yaw),
        ),
        material=material,
        name=name,
    )


def _add_leg_assembly(
    part,
    *,
    prefix: str,
    position: tuple[float, float],
    foot_axis: str,
    steel,
    glides,
) -> None:
    x, y = position
    if foot_axis == "y":
        foot_size = (0.090, 0.680, 0.030)
        pad_offsets = ((0.0, -0.290), (0.0, 0.290))
    else:
        foot_size = (0.680, 0.090, 0.030)
        pad_offsets = ((-0.290, 0.0), (0.290, 0.0))

    part.visual(
        Box(foot_size),
        origin=Origin(xyz=(x, y, 0.015)),
        material=steel,
        name=f"{prefix}_foot",
    )
    for index, (dx, dy) in enumerate(pad_offsets):
        part.visual(
            Cylinder(radius=0.025, length=0.012),
            origin=Origin(xyz=(x + dx, y + dy, 0.009)),
            material=glides,
            name=f"{prefix}_glide_{index}",
        )

    part.visual(
        Box((0.150, 0.130, 0.080)),
        origin=Origin(xyz=(x, y, 0.070)),
        material=steel,
        name=f"{prefix}_pedestal",
    )

    wall_half = COLUMN_OUTER / 2.0 - COLUMN_WALL / 2.0
    inner_span = COLUMN_OUTER - 2.0 * COLUMN_WALL
    part.visual(
        Box((COLUMN_OUTER, COLUMN_WALL, SLEEVE_LENGTH)),
        origin=Origin(xyz=(x, y - wall_half, SLEEVE_CENTER_Z)),
        material=steel,
        name=f"{prefix}_sleeve_front",
    )
    part.visual(
        Box((COLUMN_OUTER, COLUMN_WALL, SLEEVE_LENGTH)),
        origin=Origin(xyz=(x, y + wall_half, SLEEVE_CENTER_Z)),
        material=steel,
        name=f"{prefix}_sleeve_back",
    )
    part.visual(
        Box((COLUMN_WALL, inner_span, SLEEVE_LENGTH)),
        origin=Origin(xyz=(x - wall_half, y, SLEEVE_CENTER_Z)),
        material=steel,
        name=f"{prefix}_sleeve_left",
    )
    part.visual(
        Box((COLUMN_WALL, inner_span, SLEEVE_LENGTH)),
        origin=Origin(xyz=(x + wall_half, y, SLEEVE_CENTER_Z)),
        material=steel,
        name=f"{prefix}_sleeve_right",
    )


def _desktop_profile() -> list[tuple[float, float]]:
    return [
        (-0.860, -0.400),
        (0.360, -0.400),
        (0.360, 0.860),
        (-0.360, 0.860),
        (-0.360, 0.400),
        (-0.860, 0.400),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_standing_desk")

    wood_top = model.material("wood_top", rgba=(0.44, 0.31, 0.22, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.28, 0.29, 0.31, 1.0))
    drawer_front = model.material("drawer_front", rgba=(0.20, 0.21, 0.23, 1.0))
    drawer_body = model.material("drawer_body", rgba=(0.72, 0.73, 0.75, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    _add_leg_assembly(
        base,
        prefix="left",
        position=LEFT_LEG,
        foot_axis="y",
        steel=steel_dark,
        glides=plastic_black,
    )
    _add_leg_assembly(
        base,
        prefix="right",
        position=RIGHT_LEG,
        foot_axis="y",
        steel=steel_dark,
        glides=plastic_black,
    )
    _add_leg_assembly(
        base,
        prefix="rear",
        position=REAR_LEG,
        foot_axis="x",
        steel=steel_dark,
        glides=plastic_black,
    )

    base.visual(
        Box((0.190, 0.130, 0.070)),
        origin=Origin(xyz=(-0.16, 0.23, 0.405)),
        material=steel_mid,
        name="drive_hub",
    )
    _beam(
        base,
        name="left_drive_beam",
        start=LEFT_LEG,
        end=(-0.16, 0.23),
        width=0.080,
        height=0.045,
        z=0.405,
        material=steel_mid,
        clearance=0.050,
    )
    _beam(
        base,
        name="right_drive_beam",
        start=RIGHT_LEG,
        end=(-0.16, 0.23),
        width=0.080,
        height=0.045,
        z=0.405,
        material=steel_mid,
        clearance=0.050,
    )
    _beam(
        base,
        name="rear_drive_beam",
        start=REAR_LEG,
        end=(-0.16, 0.23),
        width=0.080,
        height=0.045,
        z=0.405,
        material=steel_mid,
        clearance=0.050,
    )
    base.inertial = Inertial.from_geometry(
        Box((1.360, 1.360, 0.510)),
        mass=29.0,
        origin=Origin(xyz=(-0.18, 0.18, 0.255)),
    )

    top_assembly = model.part("top_assembly")
    top_assembly.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(_desktop_profile(), 0.032),
            "corner_standing_desk_top",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.284)),
        material=wood_top,
        name="desktop",
    )
    top_assembly.visual(
        Box((1.080, 0.120, 0.016)),
        origin=Origin(xyz=(-0.250, 0.000, 0.276)),
        material=steel_mid,
        name="main_mount_plate",
    )
    top_assembly.visual(
        Box((0.120, 0.700, 0.016)),
        origin=Origin(xyz=(0.000, 0.530, 0.276)),
        material=steel_mid,
        name="return_mount_plate",
    )

    inner_column_length = 0.600
    inner_column_center_z = -0.040
    for prefix, (x, y) in (
        ("left", LEFT_LEG),
        ("right", RIGHT_LEG),
        ("rear", REAR_LEG),
    ):
        top_assembly.visual(
            Box((COLUMN_INNER, COLUMN_INNER, inner_column_length)),
            origin=Origin(xyz=(x, y, inner_column_center_z)),
            material=steel_dark,
            name=f"{prefix}_inner_column",
        )
        top_assembly.visual(
            Box((0.110, 0.110, 0.026)),
            origin=Origin(xyz=(x, y, 0.255)),
            material=steel_mid,
            name=f"{prefix}_column_head",
        )
        top_assembly.visual(
            Box((0.004, 0.016, 0.340)),
            origin=Origin(xyz=(x - 0.034, y, -0.190)),
            material=steel_mid,
            name=f"{prefix}_guide_pad_left",
        )
        top_assembly.visual(
            Box((0.004, 0.016, 0.340)),
            origin=Origin(xyz=(x + 0.034, y, -0.190)),
            material=steel_mid,
            name=f"{prefix}_guide_pad_right",
        )

    top_assembly.visual(
        Box((0.230, 0.140, 0.050)),
        origin=Origin(xyz=(-0.12, 0.14, 0.238)),
        material=steel_mid,
        name="control_box",
    )
    _beam(
        top_assembly,
        name="left_top_beam",
        start=LEFT_LEG,
        end=(-0.12, 0.14),
        width=0.085,
        height=0.050,
        z=0.238,
        material=steel_mid,
        clearance=0.055,
    )
    _beam(
        top_assembly,
        name="right_top_beam",
        start=RIGHT_LEG,
        end=(-0.12, 0.14),
        width=0.085,
        height=0.050,
        z=0.238,
        material=steel_mid,
        clearance=0.055,
    )
    _beam(
        top_assembly,
        name="rear_top_beam",
        start=REAR_LEG,
        end=(-0.12, 0.14),
        width=0.085,
        height=0.050,
        z=0.238,
        material=steel_mid,
        clearance=0.055,
    )
    top_assembly.inertial = Inertial.from_geometry(
        Box((1.240, 1.280, 0.360)),
        mass=26.0,
        origin=Origin(xyz=(-0.24, 0.22, 0.120)),
    )

    model.articulation(
        "base_to_top_assembly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=top_assembly,
        origin=Origin(xyz=(0.0, 0.0, HEIGHT_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.050,
            lower=0.0,
            upper=HEIGHT_TRAVEL,
        ),
    )

    drawer_housing = model.part("drawer_housing")
    drawer_housing.visual(
        Box((0.320, 0.270, 0.008)),
        origin=Origin(xyz=(0.0, 0.135, 0.074)),
        material=drawer_front,
        name="housing_top",
    )
    drawer_housing.visual(
        Box((0.010, 0.260, 0.070)),
        origin=Origin(xyz=(-0.155, 0.130, 0.035)),
        material=drawer_front,
        name="housing_left_side",
    )
    drawer_housing.visual(
        Box((0.010, 0.260, 0.070)),
        origin=Origin(xyz=(0.155, 0.130, 0.035)),
        material=drawer_front,
        name="housing_right_side",
    )
    drawer_housing.visual(
        Box((0.320, 0.010, 0.070)),
        origin=Origin(xyz=(0.0, 0.265, 0.035)),
        material=drawer_front,
        name="housing_back",
    )
    drawer_housing.visual(
        Box((0.010, 0.220, 0.014)),
        origin=Origin(xyz=(-0.145, 0.110, 0.038)),
        material=steel_mid,
        name="left_rail",
    )
    drawer_housing.visual(
        Box((0.010, 0.220, 0.014)),
        origin=Origin(xyz=(0.145, 0.110, 0.038)),
        material=steel_mid,
        name="right_rail",
    )
    drawer_housing.visual(
        Box((0.080, 0.035, 0.078)),
        origin=Origin(xyz=(-0.090, 0.055, 0.109)),
        material=steel_mid,
        name="mount_left",
    )
    drawer_housing.visual(
        Box((0.080, 0.035, 0.078)),
        origin=Origin(xyz=(0.090, 0.055, 0.109)),
        material=steel_mid,
        name="mount_right",
    )
    drawer_housing.inertial = Inertial.from_geometry(
        Box((0.320, 0.270, 0.148)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.135, 0.074)),
    )

    model.articulation(
        "top_assembly_to_drawer_housing",
        ArticulationType.FIXED,
        parent=top_assembly,
        child=drawer_housing,
        origin=Origin(xyz=(-0.300, -0.385, 0.136)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.280, 0.230, 0.006)),
        origin=Origin(xyz=(0.0, 0.115, 0.003)),
        material=drawer_body,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.010, 0.230, 0.058)),
        origin=Origin(xyz=(-0.135, 0.115, 0.032)),
        material=drawer_body,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((0.010, 0.230, 0.058)),
        origin=Origin(xyz=(0.135, 0.115, 0.032)),
        material=drawer_body,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((0.280, 0.010, 0.054)),
        origin=Origin(xyz=(0.0, 0.225, 0.029)),
        material=drawer_body,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.300, 0.020, 0.082)),
        origin=Origin(xyz=(0.0, -0.010, 0.041)),
        material=drawer_front,
        name="drawer_face",
    )
    drawer.visual(
        Cylinder(radius=0.005, length=0.120),
        origin=Origin(xyz=(0.0, -0.018, 0.042), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_black,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.300, 0.250, 0.082)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.110, 0.041)),
    )

    model.articulation(
        "drawer_housing_to_drawer",
        ArticulationType.PRISMATIC,
        parent=drawer_housing,
        child=drawer,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.300,
            lower=0.0,
            upper=0.160,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    top_assembly = object_model.get_part("top_assembly")
    drawer_housing = object_model.get_part("drawer_housing")
    drawer = object_model.get_part("drawer")
    height_joint = object_model.get_articulation("base_to_top_assembly")
    drawer_joint = object_model.get_articulation("drawer_housing_to_drawer")

    for part_name in ("base", "top_assembly", "drawer_housing", "drawer"):
        ctx.check(
            f"{part_name} exists",
            object_model.get_part(part_name) is not None,
            details=f"Missing required part {part_name}",
        )

    ctx.expect_contact(
        drawer_housing,
        top_assembly,
        name="drawer housing mounts to the underside of the desk",
    )

    rest_top_pos = ctx.part_world_position(top_assembly)
    with ctx.pose({height_joint: HEIGHT_TRAVEL}):
        raised_top_pos = ctx.part_world_position(top_assembly)
        ctx.expect_overlap(
            top_assembly,
            base,
            axes="z",
            min_overlap=0.060,
            name="telescoping columns retain insertion at full standing height",
        )

    ctx.check(
        "desk raises upward",
        rest_top_pos is not None
        and raised_top_pos is not None
        and raised_top_pos[2] > rest_top_pos[2] + 0.200,
        details=f"rest={rest_top_pos}, raised={raised_top_pos}",
    )

    with ctx.pose({drawer_joint: 0.0}):
        ctx.expect_within(
            drawer,
            drawer_housing,
            axes="xz",
            margin=0.003,
            name="closed drawer stays aligned in the housing",
        )
        ctx.expect_overlap(
            drawer,
            drawer_housing,
            axes="y",
            min_overlap=0.220,
            name="closed drawer remains deeply inserted in the housing",
        )
        closed_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({drawer_joint: 0.160}):
        ctx.expect_within(
            drawer,
            drawer_housing,
            axes="xz",
            margin=0.003,
            name="open drawer stays centered on its guide rails",
        )
        ctx.expect_overlap(
            drawer,
            drawer_housing,
            axes="y",
            min_overlap=0.060,
            name="open drawer keeps retained insertion on the rails",
        )
        open_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer opens outward",
        closed_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[1] < closed_drawer_pos[1] - 0.120,
        details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
