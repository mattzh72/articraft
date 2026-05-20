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


BACK_WIDTH = 0.240
BACK_HEIGHT = 0.320
BACK_PLATE_THICKNESS = 0.012
Y_RAIL_LENGTH = 0.160
Y_TRAVEL = 0.035
Z_TRAVEL = 0.115


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _back_support_shape() -> cq.Workplane:
    shape = _box((0.0, 0.0, 0.0), (BACK_PLATE_THICKNESS, BACK_WIDTH, BACK_HEIGHT))
    shape = shape.union(_box((0.010, 0.0, 0.138), (0.016, 0.180, 0.040)))
    for y_pos in (-0.068, 0.068):
        shape = shape.union(_box((0.010, y_pos, 0.050), (0.020, 0.024, 0.180)))
    shape = shape.union(_box((0.014, 0.0, 0.056), (0.016, Y_RAIL_LENGTH, 0.012)))
    shape = shape.union(_box((0.014, 0.0, 0.102), (0.016, Y_RAIL_LENGTH, 0.012)))
    return shape


def _y_slide_shape() -> cq.Workplane:
    shape = _box((0.021, 0.0, 0.0), (0.038, 0.078, 0.070))
    shape = shape.union(_box((0.038, 0.0, -0.006), (0.012, 0.056, 0.090)))
    shape = shape.union(_box((0.029, 0.0, -0.124), (0.018, 0.042, 0.192)))
    shape = shape.union(_box((0.035, 0.0, -0.210), (0.030, 0.050, 0.028)))
    for z_pos in (-0.023, 0.023):
        shape = shape.union(_box((0.0, 0.0, z_pos), (0.012, 0.050, 0.018)))
    for y_pos in (-0.014, 0.014):
        shape = shape.union(_box((0.043, y_pos, -0.122), (0.010, 0.010, 0.164)))
    shape = shape.cut(_box((0.037, 0.0, -0.122), (0.012, 0.012, 0.140)))
    return shape


def _z_carriage_shape() -> cq.Workplane:
    shape = _box((0.010, 0.0, -0.018), (0.018, 0.032, 0.022))
    for y_pos in (-0.012, 0.012):
        shape = shape.union(_box((0.0, y_pos, -0.018), (0.012, 0.008, 0.030)))
    shape = shape.union(_box((0.015, 0.0, -0.054), (0.024, 0.028, 0.082)))
    shape = shape.union(_box((0.028, 0.0, -0.056), (0.006, 0.024, 0.060)))
    shape = shape.union(_box((0.018, 0.0, -0.100), (0.034, 0.042, 0.010)))
    shape = shape.union(_box((0.026, 0.0, -0.112), (0.018, 0.018, 0.020)))
    return shape


def _add_mesh_proxy_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    material: str,
    box_size: tuple[float, float, float],
    box_center: tuple[float, float, float],
    mass: float,
):
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, name), material=material, name=f"{name}_body")
    part.inertial = Inertial.from_geometry(Box(box_size), mass=mass, origin=Origin(xyz=box_center))
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_yz_stage")
    model.material("back_support_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("y_stage_gray", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("z_stage_light", rgba=(0.80, 0.82, 0.85, 1.0))

    back_support = _add_mesh_proxy_part(
        model,
        name="back_support",
        shape=_back_support_shape(),
        material="back_support_dark",
        box_size=(0.060, BACK_WIDTH, BACK_HEIGHT),
        box_center=(0.018, 0.0, 0.0),
        mass=5.2,
    )
    y_slide = _add_mesh_proxy_part(
        model,
        name="y_slide",
        shape=_y_slide_shape(),
        material="y_stage_gray",
        box_size=(0.055, 0.080, 0.270),
        box_center=(0.022, 0.0, -0.090),
        mass=2.1,
    )
    z_carriage = _add_mesh_proxy_part(
        model,
        name="z_carriage",
        shape=_z_carriage_shape(),
        material="z_stage_light",
        box_size=(0.050, 0.044, 0.120),
        box_center=(0.018, 0.0, -0.060),
        mass=0.75,
    )

    model.articulation(
        "back_support_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=back_support,
        child=y_slide,
        origin=Origin(xyz=(0.028, 0.0, 0.079)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=180.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "y_slide_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=y_slide,
        child=z_carriage,
        origin=Origin(xyz=(0.052, 0.0, -0.046)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=140.0,
            velocity=0.28,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_support = object_model.get_part("back_support")
    y_slide = object_model.get_part("y_slide")
    z_carriage = object_model.get_part("z_carriage")
    y_joint = object_model.get_articulation("back_support_to_y_slide")
    z_joint = object_model.get_articulation("y_slide_to_z_carriage")

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

    ctx.expect_contact(
        y_slide,
        back_support,
        name="y slide stays mounted on the fixed back support",
    )
    ctx.expect_contact(
        z_carriage,
        y_slide,
        name="z carriage stays mounted on the hanging y slide guide",
    )
    ctx.expect_within(
        z_carriage,
        y_slide,
        axes="y",
        margin=0.006,
        name="z carriage stays laterally within the y slide guide width",
    )

    y_rest = ctx.part_world_position(y_slide)
    z_rest = ctx.part_world_position(z_carriage)
    with ctx.pose({y_joint: Y_TRAVEL}):
        ctx.expect_contact(
            y_slide,
            back_support,
            name="y slide remains supported at positive lateral travel",
        )
        y_shifted = ctx.part_world_position(y_slide)
    with ctx.pose({z_joint: Z_TRAVEL}):
        ctx.expect_contact(
            z_carriage,
            y_slide,
            name="z carriage remains supported at full downward travel",
        )
        ctx.expect_within(
            z_carriage,
            y_slide,
            axes="y",
            margin=0.006,
            name="z carriage stays aligned between the vertical rails when lowered",
        )
        z_lowered = ctx.part_world_position(z_carriage)

    ctx.check(
        "lateral stage moves along +Y",
        y_rest is not None and y_shifted is not None and y_shifted[1] > y_rest[1] + 0.02,
        details=f"rest={y_rest}, shifted={y_shifted}",
    )
    ctx.check(
        "vertical stage lowers along -Z",
        z_rest is not None and z_lowered is not None and z_lowered[2] < z_rest[2] - 0.06,
        details=f"rest={z_rest}, lowered={z_lowered}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
