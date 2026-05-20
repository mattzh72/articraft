from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ROOT_LENGTH = 0.58
ROOT_WIDTH = 0.150
ROOT_HEIGHT = 0.126
ROOT_WALL = 0.008
ROOT_COLLAR_LENGTH = 0.050
ROOT_SIDE_SLOT_X0 = 0.22
ROOT_SIDE_SLOT_X1 = 0.56
ROOT_SIDE_SLOT_Z0 = 0.078
ROOT_SIDE_SLOT_Z1 = 0.102

MEDIUM_LENGTH = 0.62
MEDIUM_BODY_WIDTH = 0.084
MEDIUM_BODY_HEIGHT = 0.058
MEDIUM_BODY_Z = 0.006
MEDIUM_WALL = 0.005
MEDIUM_TRAVEL = 0.18
MEDIUM_HOME_X = 0.36
MEDIUM_HOME_Z = 0.018

TERMINAL_LENGTH = 0.50
TERMINAL_BODY_WIDTH = 0.058
TERMINAL_BODY_HEIGHT = 0.040
TERMINAL_BODY_Z = 0.004
TERMINAL_WALL = 0.004
TERMINAL_DOG_X0 = 0.126
TERMINAL_DOG_LENGTH = 0.028
TERMINAL_TRAVEL = 0.18
TERMINAL_HOME_X = 0.40
TERMINAL_HOME_Z = 0.010


def _box(length: float, width: float, height: float, *, x: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, False))
        .translate((x, 0.0, z))
    )


def _tube(
    length: float,
    width: float,
    height: float,
    wall: float,
    *,
    x: float = 0.0,
    z: float = 0.0,
    open_back: bool = False,
    open_front: bool = False,
) -> cq.Workplane:
    outer = _box(length, width, height, x=x, z=z)
    eps = 0.001
    inner_start = x - eps if open_back else x + wall
    inner_end = x + length + eps if open_front else x + length - wall
    inner = _box(
        inner_end - inner_start,
        width - 2.0 * wall,
        height - 2.0 * wall,
        x=inner_start,
        z=z + wall,
    )
    return outer.cut(inner)


def _xz_prism(points: list[tuple[float, float]], *, y_center: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(thickness, both=True)
        .translate((0.0, y_center, 0.0))
    )


def _x_cylinder(length: float, radius: float, *, x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x, y, z))


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    material: str,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _trussed_side_plate(*, x: float, z: float, length: float, height: float, y_center: float) -> cq.Workplane:
    plate = _box(length, 0.008, height, x=x, z=z).translate((0.0, y_center, 0.0))
    window_a = _xz_prism(
        [(x + 0.06, z + 0.018), (x + 0.18, z + 0.018), (x + 0.12, z + 0.060)],
        y_center=y_center,
        thickness=0.012,
    )
    window_b = _xz_prism(
        [(x + 0.22, z + 0.018), (x + 0.34, z + 0.018), (x + 0.28, z + 0.060)],
        y_center=y_center,
        thickness=0.012,
    )
    return plate.cut(window_a).cut(window_b)


def _root_housing_shape() -> cq.Workplane:
    housing = _box(0.16, 0.20, 0.05, x=0.0, z=-0.05)
    housing = housing.union(_box(0.12, 0.060, 0.060, x=0.02, z=0.0))
    housing = housing.union(_box(0.44, 0.116, 0.010, x=0.12, z=0.086))

    for y_center in (-0.052, 0.052):
        housing = housing.union(
            _trussed_side_plate(x=0.10, z=0.008, length=0.46, height=0.078, y_center=y_center)
        )
        housing = housing.union(
            _xz_prism(
                [(0.03, -0.05), (0.17, -0.05), (0.135, 0.008), (0.07, 0.008)],
                y_center=y_center + (0.008 if y_center > 0.0 else -0.008),
                thickness=0.014,
            )
        )

    for y_center in (-0.050, 0.050):
        housing = housing.union(_box(0.08, 0.012, 0.060, x=0.50, z=0.010).translate((0.0, y_center, 0.0)))
        housing = housing.union(_box(0.022, 0.010, 0.016, x=0.556, z=0.074).translate((0.0, y_center, 0.0)))

    housing = housing.union(_box(0.08, 0.116, 0.010, x=0.50, z=0.070))
    return housing


def _medium_stage_shape() -> cq.Workplane:
    beam = _box(0.20, 0.028, 0.018, x=0.0, z=0.020)

    for y_center in (-0.043, 0.043):
        beam = beam.union(_box(0.18, 0.010, 0.018, x=0.0, z=0.024).translate((0.0, y_center, 0.0)))

    for y_center in (-0.018, 0.018):
        beam = beam.union(_box(0.18, 0.010, 0.004, x=0.0, z=0.016).translate((0.0, y_center, 0.0)))

    beam = beam.union(_box(0.44, 0.024, 0.012, x=0.18, z=0.024))
    beam = beam.union(_box(0.40, 0.040, 0.004, x=0.22, z=0.040))

    for y_center in (-0.026, 0.026):
        beam = beam.union(_box(0.40, 0.004, 0.028, x=0.22, z=0.016).translate((0.0, y_center, 0.0)))

    beam = beam.union(_box(0.04, 0.018, 0.008, x=0.00, z=0.056))

    for y_center in (-0.031, 0.031):
        beam = beam.union(_box(0.12, 0.010, 0.044, x=0.50, z=0.010).translate((0.0, y_center, 0.0)))
        beam = beam.union(_box(0.016, 0.010, 0.012, x=0.604, z=0.052).translate((0.0, y_center, 0.0)))

    beam = beam.union(_box(0.12, 0.072, 0.008, x=0.50, z=0.054))
    return beam


def _terminal_beam_shape() -> cq.Workplane:
    beam = _box(0.14, 0.018, 0.014, x=0.0, z=0.018)

    for y_center in (-0.021, 0.021):
        beam = beam.union(_box(0.14, 0.010, 0.014, x=0.0, z=0.022).translate((0.0, y_center, 0.0)))

    for y_center in (-0.012, 0.012):
        beam = beam.union(_box(0.14, 0.008, 0.004, x=0.0, z=0.014).translate((0.0, y_center, 0.0)))

    beam = beam.union(_box(0.36, 0.018, 0.010, x=0.14, z=0.020))
    beam = beam.union(_box(0.34, 0.030, 0.004, x=0.16, z=0.034))

    for y_center in (-0.017, 0.017):
        beam = beam.union(_box(0.34, 0.004, 0.022, x=0.16, z=0.014).translate((0.0, y_center, 0.0)))

    beam = beam.union(_box(TERMINAL_DOG_LENGTH, 0.016, 0.008, x=0.00, z=0.040))
    beam = beam.union(_box(0.050, 0.028, 0.020, x=0.45, z=0.012))

    for y_center in (-0.018, 0.018):
        beam = beam.union(_box(0.060, 0.008, 0.036, x=0.44, z=0.004).translate((0.0, y_center, 0.0)))

    return beam


def _face_plate_shape() -> cq.Workplane:
    plate = _box(0.010, 0.110, 0.084, x=TERMINAL_LENGTH, z=0.008)
    boss = _x_cylinder(0.012, 0.011, x=TERMINAL_LENGTH + 0.010, y=0.0, z=0.050)
    plate = plate.union(boss)

    central_hole = _x_cylinder(0.028, 0.006, x=TERMINAL_LENGTH - 0.004, y=0.0, z=0.050)
    plate = plate.cut(central_hole)

    for y_pos in (-0.032, 0.032):
        for z_pos in (0.028, 0.072):
            plate = plate.cut(_x_cylinder(0.024, 0.0035, x=TERMINAL_LENGTH - 0.003, y=y_pos, z=z_pos))

    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_support_boom")

    model.material("housing_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("stage_gray", rgba=(0.57, 0.59, 0.62, 1.0))
    model.material("terminal_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("plate_black", rgba=(0.12, 0.13, 0.15, 1.0))

    root_housing = model.part("root_housing")
    _add_box_visual(root_housing, (0.18, 0.20, 0.05), (0.09, 0.0, -0.025), material="housing_gray", name="housing")
    _add_box_visual(root_housing, (0.12, 0.06, 0.06), (0.08, 0.0, 0.03), material="housing_gray")
    _add_box_visual(root_housing, (0.42, 0.116, 0.010), (0.33, 0.0, 0.089), material="housing_gray")
    _add_box_visual(root_housing, (0.46, 0.008, 0.010), (0.35, 0.052, 0.013), material="housing_gray")
    _add_box_visual(root_housing, (0.46, 0.008, 0.010), (0.35, -0.052, 0.013), material="housing_gray")
    _add_box_visual(root_housing, (0.46, 0.008, 0.010), (0.35, 0.052, 0.081), material="housing_gray")
    _add_box_visual(root_housing, (0.46, 0.008, 0.010), (0.35, -0.052, 0.081), material="housing_gray")
    _add_box_visual(root_housing, (0.44, 0.020, 0.014), (0.36, 0.0, 0.028), material="housing_gray")
    _add_box_visual(root_housing, (0.08, 0.012, 0.060), (0.54, 0.050, 0.040), material="housing_gray")
    _add_box_visual(root_housing, (0.08, 0.012, 0.060), (0.54, -0.050, 0.040), material="housing_gray")
    _add_box_visual(root_housing, (0.08, 0.116, 0.010), (0.54, 0.0, 0.075), material="housing_gray")
    _add_box_visual(root_housing, (0.10, 0.024, 0.018), (0.14, 0.040, 0.017), material="housing_gray")
    _add_box_visual(root_housing, (0.10, 0.024, 0.018), (0.14, -0.040, 0.017), material="housing_gray")
    _add_box_visual(root_housing, (0.18, 0.008, 0.008), (0.23, 0.052, 0.038), material="housing_gray", rpy=(0.0, 0.42, 0.0))
    _add_box_visual(root_housing, (0.18, 0.008, 0.008), (0.37, 0.052, 0.054), material="housing_gray", rpy=(0.0, -0.42, 0.0))
    _add_box_visual(root_housing, (0.18, 0.008, 0.008), (0.23, -0.052, 0.038), material="housing_gray", rpy=(0.0, 0.42, 0.0))
    _add_box_visual(root_housing, (0.18, 0.008, 0.008), (0.37, -0.052, 0.054), material="housing_gray", rpy=(0.0, -0.42, 0.0))
    root_housing.inertial = Inertial.from_geometry(
        Box((ROOT_LENGTH, 0.20, 0.176)),
        mass=4.2,
        origin=Origin(xyz=(ROOT_LENGTH / 2.0, 0.0, 0.038)),
    )

    medium_stage = model.part("medium_stage")
    _add_box_visual(medium_stage, (0.12, 0.028, 0.018), (0.06, 0.0, 0.029), material="stage_gray")
    _add_box_visual(medium_stage, (0.10, 0.016, 0.014), (0.05, 0.020, 0.025), material="stage_gray")
    _add_box_visual(medium_stage, (0.10, 0.016, 0.014), (0.05, -0.020, 0.025), material="stage_gray")
    _add_box_visual(medium_stage, (0.50, 0.024, 0.014), (0.37, 0.0, 0.026), material="stage_gray", name="beam")
    _add_box_visual(medium_stage, (0.46, 0.010, 0.010), (0.39, 0.016, 0.034), material="stage_gray")
    _add_box_visual(medium_stage, (0.46, 0.010, 0.010), (0.39, -0.016, 0.034), material="stage_gray")
    _add_box_visual(medium_stage, (0.44, 0.020, 0.022), (0.40, 0.018, 0.028), material="stage_gray")
    _add_box_visual(medium_stage, (0.44, 0.020, 0.022), (0.40, -0.018, 0.028), material="stage_gray")
    _add_box_visual(medium_stage, (0.12, 0.020, 0.044), (0.56, 0.022, 0.040), material="stage_gray")
    _add_box_visual(medium_stage, (0.12, 0.020, 0.044), (0.56, -0.022, 0.040), material="stage_gray")
    _add_box_visual(medium_stage, (0.12, 0.072, 0.008), (0.56, 0.0, 0.058), material="stage_gray")
    _add_box_visual(medium_stage, (0.04, 0.020, 0.014), (0.16, 0.0, 0.038), material="stage_gray")
    medium_stage.inertial = Inertial.from_geometry(
        Box((MEDIUM_LENGTH, 0.146, 0.110)),
        mass=1.8,
        origin=Origin(xyz=(MEDIUM_LENGTH / 2.0, 0.0, 0.055)),
    )

    terminal_stage = model.part("terminal_stage")
    _add_box_visual(terminal_stage, (0.08, 0.016, 0.012), (0.04, 0.0, 0.024), material="terminal_gray")
    _add_box_visual(terminal_stage, (0.08, 0.010, 0.014), (0.04, 0.003, 0.026), material="terminal_gray")
    _add_box_visual(terminal_stage, (0.08, 0.010, 0.014), (0.04, -0.003, 0.026), material="terminal_gray")
    _add_box_visual(terminal_stage, (0.42, 0.016, 0.010), (0.29, 0.0, 0.024), material="terminal_gray", name="beam")
    _add_box_visual(terminal_stage, (0.38, 0.022, 0.008), (0.31, 0.0, 0.031), material="terminal_gray")
    _add_box_visual(terminal_stage, (0.024, 0.014, 0.010), (0.050, 0.0, 0.032), material="terminal_gray")
    _add_box_visual(terminal_stage, (0.050, 0.028, 0.020), (0.475, 0.0, 0.022), material="terminal_gray")
    _add_box_visual(terminal_stage, (0.010, 0.110, 0.084), (0.505, 0.0, 0.050), material="plate_black", name="face_plate")
    terminal_stage.inertial = Inertial.from_geometry(
        Box((0.522, 0.110, 0.092)),
        mass=1.0,
        origin=Origin(xyz=(0.261, 0.0, 0.046)),
    )

    model.articulation(
        "root_to_medium",
        ArticulationType.PRISMATIC,
        parent=root_housing,
        child=medium_stage,
        origin=Origin(xyz=(MEDIUM_HOME_X, 0.0, MEDIUM_HOME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MEDIUM_TRAVEL,
            effort=160.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "medium_to_terminal",
        ArticulationType.PRISMATIC,
        parent=medium_stage,
        child=terminal_stage,
        origin=Origin(xyz=(TERMINAL_HOME_X, 0.0, TERMINAL_HOME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TERMINAL_TRAVEL,
            effort=110.0,
            velocity=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_housing = object_model.get_part("root_housing")
    medium_stage = object_model.get_part("medium_stage")
    terminal_stage = object_model.get_part("terminal_stage")
    root_to_medium = object_model.get_articulation("root_to_medium")
    medium_to_terminal = object_model.get_articulation("medium_to_terminal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "serial_prismatic_slide_metadata",
        (
            root_to_medium.articulation_type == ArticulationType.PRISMATIC
            and medium_to_terminal.articulation_type == ArticulationType.PRISMATIC
            and tuple(root_to_medium.axis) == (1.0, 0.0, 0.0)
            and tuple(medium_to_terminal.axis) == (1.0, 0.0, 0.0)
            and root_to_medium.motion_limits is not None
            and medium_to_terminal.motion_limits is not None
            and root_to_medium.motion_limits.upper == MEDIUM_TRAVEL
            and medium_to_terminal.motion_limits.upper == TERMINAL_TRAVEL
        ),
        "Both extensions must be serial +X prismatic slides with the authored travel limits.",
    )

    ctx.expect_contact(medium_stage, root_housing, name="medium_stage_supported_by_root_housing")
    ctx.expect_contact(terminal_stage, medium_stage, name="terminal_stage_supported_by_medium_stage")
    ctx.expect_overlap(
        medium_stage,
        root_housing,
        axes="yz",
        min_overlap=0.045,
        name="medium_stage_tracks_inside_root_profile",
    )
    ctx.expect_overlap(
        terminal_stage,
        medium_stage,
        axes="yz",
        min_overlap=0.040,
        name="terminal_stage_tracks_inside_medium_profile",
    )

    beam_aabb = ctx.part_element_world_aabb(terminal_stage, elem="beam")
    plate_aabb = ctx.part_element_world_aabb(terminal_stage, elem="face_plate")
    ctx.check(
        "face_plate_is_carried_ahead_of_terminal_beam",
        beam_aabb is not None and plate_aabb is not None and plate_aabb[0][0] >= beam_aabb[1][0] - 1e-6,
        "The terminal face plate should sit in front of the slender terminal beam, not buried inside it.",
    )

    with ctx.pose({root_to_medium: MEDIUM_TRAVEL}):
        ctx.expect_overlap(
            medium_stage,
            root_housing,
            axes="xyz",
            min_overlap=0.035,
            name="medium_stage_remains_engaged_at_max_root_travel",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="root_slide_clear_at_max_root_travel")

    with ctx.pose({root_to_medium: MEDIUM_TRAVEL, medium_to_terminal: TERMINAL_TRAVEL}):
        ctx.expect_contact(
            terminal_stage,
            medium_stage,
            name="terminal_stage_remains_grounded_at_full_extension",
        )
        ctx.expect_gap(
            terminal_stage,
            root_housing,
            axis="x",
            min_gap=0.10,
            name="terminal_stage_projects_clear_of_root_when_fully_extended",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="full_extension_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
