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
    TorusGeometry,
    mesh_from_geometry,
)


BODY_WIDTH = 0.44
BODY_DEPTH = 0.34
BODY_HEIGHT = 0.84
WALL_THICKNESS = 0.016


def _add_caster(
    model: ArticulatedObject,
    housing,
    *,
    name: str,
    origin_xyz: tuple[float, float, float],
    dark_material,
    metal_material,
) -> None:
    caster = model.part(name)
    caster.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=metal_material,
        name="caster_stem",
    )
    caster.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=metal_material,
        name="caster_swivel",
    )
    caster.visual(
        Box((0.034, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=metal_material,
        name="caster_bridge",
    )
    caster.visual(
        Box((0.006, 0.018, 0.030)),
        origin=Origin(xyz=(-0.012, 0.0, -0.059)),
        material=metal_material,
        name="caster_left_fork",
    )
    caster.visual(
        Box((0.006, 0.018, 0.030)),
        origin=Origin(xyz=(0.012, 0.0, -0.059)),
        material=metal_material,
        name="caster_right_fork",
    )
    caster.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.069), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_material,
        name="caster_wheel",
    )
    caster.inertial = Inertial.from_geometry(
        Box((0.034, 0.022, 0.095)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.047)),
    )
    model.articulation(
        f"housing_to_{name}",
        ArticulationType.FIXED,
        parent=housing,
        child=caster,
        origin=Origin(xyz=origin_xyz),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_evaporative_cooler")

    body_white = model.material("body_white", rgba=(0.89, 0.91, 0.90, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.19, 0.21, 0.23, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    caster_metal = model.material("caster_metal", rgba=(0.54, 0.56, 0.58, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.75, 0.77, 0.78, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.31, 0.58, 0.72, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.28, 0.14, 0.050)),
        origin=Origin(xyz=(0.0, -0.030, BODY_HEIGHT + 0.025)),
        material=panel_grey,
        name="control_console",
    )
    housing.visual(
        Box((BODY_WIDTH - 0.010, BODY_DEPTH - 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=body_white,
        name="base_pan",
    )
    housing.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT - 0.010)),
        origin=Origin(xyz=(-(BODY_WIDTH / 2.0) + (WALL_THICKNESS / 2.0), 0.0, (BODY_HEIGHT - 0.010) / 2.0)),
        material=body_white,
        name="left_wall",
    )
    housing.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT - 0.010)),
        origin=Origin(xyz=((BODY_WIDTH / 2.0) - (WALL_THICKNESS / 2.0), 0.0, (BODY_HEIGHT - 0.010) / 2.0)),
        material=body_white,
        name="right_wall",
    )
    housing.visual(
        Box((BODY_WIDTH - 2.0 * WALL_THICKNESS + 0.004, BODY_DEPTH - 0.040, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.010, BODY_HEIGHT - 0.008)),
        material=body_white,
        name="top_cap",
    )
    housing.visual(
        Box((0.076, WALL_THICKNESS, 0.720)),
        origin=Origin(xyz=(-0.182, (BODY_DEPTH / 2.0) - (WALL_THICKNESS / 2.0), 0.366)),
        material=body_white,
        name="front_left_stile",
    )
    housing.visual(
        Box((0.076, WALL_THICKNESS, 0.720)),
        origin=Origin(xyz=(0.182, (BODY_DEPTH / 2.0) - (WALL_THICKNESS / 2.0), 0.366)),
        material=body_white,
        name="front_right_stile",
    )
    housing.visual(
        Box((0.292, WALL_THICKNESS, 0.402)),
        origin=Origin(xyz=(0.0, (BODY_DEPTH / 2.0) - (WALL_THICKNESS / 2.0), 0.201)),
        material=body_white,
        name="front_lower_panel",
    )
    housing.visual(
        Box((0.292, WALL_THICKNESS, 0.138)),
        origin=Origin(xyz=(0.0, (BODY_DEPTH / 2.0) - (WALL_THICKNESS / 2.0), 0.771)),
        material=body_white,
        name="front_upper_panel",
    )
    housing.visual(
        Box((BODY_WIDTH - 2.0 * WALL_THICKNESS + 0.004, WALL_THICKNESS, 0.140)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0) + (WALL_THICKNESS / 2.0), 0.070)),
        material=body_white,
        name="rear_lower_panel",
    )
    housing.visual(
        Box((BODY_WIDTH - 2.0 * WALL_THICKNESS + 0.004, WALL_THICKNESS, 0.360)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0) + (WALL_THICKNESS / 2.0), 0.660)),
        material=body_white,
        name="rear_upper_panel",
    )
    housing.visual(
        Box((0.094, WALL_THICKNESS, 0.344)),
        origin=Origin(xyz=(-0.157, -(BODY_DEPTH / 2.0) + (WALL_THICKNESS / 2.0), 0.312)),
        material=body_white,
        name="rear_left_jamb",
    )
    housing.visual(
        Box((0.094, WALL_THICKNESS, 0.344)),
        origin=Origin(xyz=(0.157, -(BODY_DEPTH / 2.0) + (WALL_THICKNESS / 2.0), 0.312)),
        material=body_white,
        name="rear_right_jamb",
    )
    housing.visual(
        Cylinder(radius=0.042, length=0.060),
        origin=Origin(xyz=(0.0, 0.065, 0.560), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="fan_motor_pod",
    )
    housing.visual(
        Box((0.050, 0.060, 0.090)),
        origin=Origin(xyz=(0.0, 0.125, 0.395)),
        material=trim_dark,
        name="fan_motor_column",
    )
    housing.visual(
        Box((0.028, 0.020, 0.160)),
        origin=Origin(xyz=(0.0, 0.085, 0.510)),
        material=trim_dark,
        name="fan_motor_neck",
    )
    housing.visual(
        Box((0.110, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.035, 0.830)),
        material=accent_blue,
        name="panel_badge",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT + 0.060)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
    )

    front_grille = model.part("front_grille")
    grille_ring = mesh_from_geometry(
        TorusGeometry(radius=0.115, tube=0.006).rotate_x(math.pi / 2.0),
        "cooler_front_grille_ring",
    )
    front_grille.visual(
        Box((0.284, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.004, 0.141)),
        material=grille_dark,
        name="grille_top_frame",
    )
    front_grille.visual(
        Box((0.284, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.004, -0.141)),
        material=grille_dark,
        name="grille_bottom_frame",
    )
    front_grille.visual(
        Box((0.018, 0.008, 0.300)),
        origin=Origin(xyz=(-0.133, 0.004, 0.0)),
        material=grille_dark,
        name="grille_left_frame",
    )
    front_grille.visual(
        Box((0.018, 0.008, 0.300)),
        origin=Origin(xyz=(0.133, 0.004, 0.0)),
        material=grille_dark,
        name="grille_right_frame",
    )
    front_grille.visual(
        grille_ring,
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=grille_dark,
        name="grille_ring",
    )
    front_grille.visual(
        Box((0.034, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.004, 0.123)),
        material=grille_dark,
        name="grille_top_brace",
    )
    front_grille.visual(
        Box((0.034, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.004, -0.123)),
        material=grille_dark,
        name="grille_bottom_brace",
    )
    front_grille.visual(
        Box((0.018, 0.008, 0.034)),
        origin=Origin(xyz=(-0.123, 0.004, 0.0)),
        material=grille_dark,
        name="grille_left_brace",
    )
    front_grille.visual(
        Box((0.018, 0.008, 0.034)),
        origin=Origin(xyz=(0.123, 0.004, 0.0)),
        material=grille_dark,
        name="grille_right_brace",
    )
    front_grille.visual(
        Cylinder(radius=0.005, length=0.226),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_dark,
        name="grille_horizontal_bar",
    )
    front_grille.visual(
        Cylinder(radius=0.005, length=0.226),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=grille_dark,
        name="grille_vertical_bar",
    )
    front_grille.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_dark,
        name="grille_center_hub",
    )
    front_grille.inertial = Inertial.from_geometry(
        Box((0.284, 0.012, 0.300)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
    )
    model.articulation(
        "housing_to_front_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=front_grille,
        origin=Origin(xyz=(0.0, 0.170, 0.560)),
    )

    rear_fill_door = model.part("rear_fill_door")
    rear_fill_door.visual(
        Box((0.214, 0.008, 0.332)),
        origin=Origin(xyz=(0.107, 0.0, 0.0)),
        material=panel_grey,
        name="door_panel",
    )
    rear_fill_door.visual(
        Box((0.014, 0.012, 0.120)),
        origin=Origin(xyz=(0.190, -0.008, 0.0)),
        material=trim_dark,
        name="door_pull",
    )
    rear_fill_door.inertial = Inertial.from_geometry(
        Box((0.214, 0.014, 0.332)),
        mass=0.65,
        origin=Origin(xyz=(0.107, -0.002, 0.0)),
    )
    model.articulation(
        "housing_to_rear_fill_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=rear_fill_door,
        origin=Origin(xyz=(-0.110, -0.166, 0.312)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(xyz=(0.0, 0.0275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="fan_shaft",
    )
    fan_rotor.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="fan_hub",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        fan_rotor.visual(
            Box((0.098, 0.008, 0.032)),
            origin=Origin(
                xyz=(0.050 * math.cos(angle), 0.060, 0.050 * math.sin(angle)),
                rpy=(0.22, -angle, 0.0),
            ),
            material=trim_dark,
            name=f"fan_blade_{index}",
        )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.070),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "housing_to_fan_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan_rotor,
        origin=Origin(xyz=(0.0, 0.095, 0.560)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=18.0,
        ),
    )

    flow_control_knob = model.part("flow_control_knob")
    flow_control_knob.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=trim_dark,
        name="knob_body",
    )
    flow_control_knob.visual(
        Box((0.006, 0.016, 0.004)),
        origin=Origin(xyz=(0.013, 0.0, 0.022)),
        material=accent_blue,
        name="knob_indicator",
    )
    flow_control_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.024),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )
    model.articulation(
        "housing_to_flow_control_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=flow_control_knob,
        origin=Origin(xyz=(0.086, -0.040, BODY_HEIGHT + 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=6.0,
        ),
    )

    _add_caster(
        model,
        housing,
        name="front_left_caster",
        origin_xyz=(-0.155, 0.110, 0.0),
        dark_material=wheel_rubber,
        metal_material=caster_metal,
    )
    _add_caster(
        model,
        housing,
        name="front_right_caster",
        origin_xyz=(0.155, 0.110, 0.0),
        dark_material=wheel_rubber,
        metal_material=caster_metal,
    )
    _add_caster(
        model,
        housing,
        name="rear_left_caster",
        origin_xyz=(-0.155, -0.110, 0.0),
        dark_material=wheel_rubber,
        metal_material=caster_metal,
    )
    _add_caster(
        model,
        housing,
        name="rear_right_caster",
        origin_xyz=(0.155, -0.110, 0.0),
        dark_material=wheel_rubber,
        metal_material=caster_metal,
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

    housing = object_model.get_part("housing")
    front_grille = object_model.get_part("front_grille")
    rear_fill_door = object_model.get_part("rear_fill_door")
    fan_rotor = object_model.get_part("fan_rotor")
    flow_control_knob = object_model.get_part("flow_control_knob")

    rear_fill_door_joint = object_model.get_articulation("housing_to_rear_fill_door")
    fan_joint = object_model.get_articulation("housing_to_fan_rotor")
    knob_joint = object_model.get_articulation("housing_to_flow_control_knob")

    ctx.expect_contact(
        front_grille,
        housing,
        contact_tol=0.0015,
        name="front grille is mounted onto the housing face",
    )
    ctx.expect_contact(
        flow_control_knob,
        housing,
        contact_tol=0.0015,
        name="flow-rate knob seats on the control console",
    )

    closed_aabb = ctx.part_world_aabb(rear_fill_door)
    with ctx.pose({rear_fill_door_joint: math.radians(95.0)}):
        open_aabb = ctx.part_world_aabb(rear_fill_door)

    ctx.check(
        "rear fill door opens outward from the back panel",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.050,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    ctx.check(
        "rear fill door hinge is vertical on the side edge",
        tuple(round(value, 3) for value in rear_fill_door_joint.axis) == (0.0, 0.0, -1.0)
        and rear_fill_door_joint.origin.xyz[0] < -0.10,
        details=f"axis={rear_fill_door_joint.axis}, origin={rear_fill_door_joint.origin}",
    )
    ctx.check(
        "fan blade articulation is continuous around the front-back axis",
        fan_joint.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 3) for value in fan_joint.axis) == (0.0, 1.0, 0.0)
        and fan_joint.motion_limits is not None
        and fan_joint.motion_limits.lower is None
        and fan_joint.motion_limits.upper is None,
        details=f"type={fan_joint.joint_type}, axis={fan_joint.axis}, limits={fan_joint.motion_limits}",
    )
    ctx.check(
        "flow-rate knob articulation is continuous around its vertical axis",
        knob_joint.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 3) for value in knob_joint.axis) == (0.0, 0.0, 1.0)
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"type={knob_joint.joint_type}, axis={knob_joint.axis}, limits={knob_joint.motion_limits}",
    )

    ctx.expect_overlap(
        fan_rotor,
        front_grille,
        axes="xz",
        min_overlap=0.11,
        name="fan rotor is centered behind the grille opening",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
