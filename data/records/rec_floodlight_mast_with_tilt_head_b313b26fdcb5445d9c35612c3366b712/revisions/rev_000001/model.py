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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _section_loop(width: float, height: float, y_pos: float, radius: float) -> tuple[tuple[float, float, float], ...]:
    profile = rounded_rect_profile(width, height, radius=radius, corner_segments=6)
    return tuple((x, y_pos, z) for x, z in profile)


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def _build_wheel_meshes():
    tire_profile = [
        (0.046, -0.019),
        (0.064, -0.019),
        (0.072, -0.015),
        (0.075, -0.008),
        (0.076, 0.000),
        (0.075, 0.008),
        (0.072, 0.015),
        (0.064, 0.019),
        (0.046, 0.019),
        (0.040, 0.010),
        (0.038, 0.000),
        (0.040, -0.010),
        (0.046, -0.019),
    ]
    tire_mesh = _save_mesh(
        "studio_floodlight_tire",
        LatheGeometry(tire_profile, segments=56).rotate_y(math.pi / 2.0),
    )
    return tire_mesh


def _add_wheel_part(model: ArticulatedObject, name: str, tire_mesh, materials: dict[str, object]):
    wheel = model.part(name)
    wheel.visual(tire_mesh, material=materials["rubber"], name="tire")
    wheel.visual(
        Cylinder(radius=0.048, length=0.030),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["rim_paint"],
        name="rim_shell",
    )
    wheel.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["dark_metal"],
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.012, length=0.038),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["steel"],
        name="axle_cap",
    )
    wheel.visual(
        Box((0.006, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=materials["steel"],
        name="valve_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.076, length=0.038),
        mass=1.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_floodlight_on_dolly")

    materials = {
        "stand_paint": model.material("stand_paint", rgba=(0.18, 0.20, 0.22, 1.0)),
        "dark_metal": model.material("dark_metal", rgba=(0.11, 0.12, 0.13, 1.0)),
        "rim_paint": model.material("rim_paint", rgba=(0.44, 0.46, 0.48, 1.0)),
        "steel": model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0)),
        "housing_paint": model.material("housing_paint", rgba=(0.28, 0.29, 0.31, 1.0)),
        "lens_glass": model.material("lens_glass", rgba=(0.82, 0.88, 0.92, 0.60)),
        "rubber": model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0)),
        "knob": model.material("knob", rgba=(0.15, 0.15, 0.16, 1.0)),
    }

    tire_mesh = _build_wheel_meshes()

    lamp_shell_mesh = _save_mesh(
        "studio_floodlight_head_shell",
        section_loft(
            [
                _section_loop(0.26, 0.22, -0.050, 0.018),
                _section_loop(0.31, 0.27, 0.030, 0.022),
                _section_loop(0.38, 0.32, 0.148, 0.026),
            ]
        ),
    )
    handle_mesh = _save_mesh(
        "studio_floodlight_head_handle",
        tube_from_spline_points(
            [
                (-0.090, 0.018, 0.132),
                (-0.080, 0.016, 0.198),
                (-0.030, 0.014, 0.235),
                (0.030, 0.014, 0.235),
                (0.080, 0.016, 0.198),
                (0.090, 0.018, 0.132),
            ],
            radius=0.008,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    dolly_column = model.part("dolly_column")
    dolly_column.visual(
        Box((0.74, 0.05, 0.045)),
        origin=Origin(xyz=(0.0, 0.34, 0.180)),
        material=materials["stand_paint"],
        name="front_rail",
    )
    dolly_column.visual(
        Box((0.74, 0.05, 0.045)),
        origin=Origin(xyz=(0.0, -0.34, 0.180)),
        material=materials["stand_paint"],
        name="rear_rail",
    )
    dolly_column.visual(
        Box((0.05, 0.74, 0.045)),
        origin=Origin(xyz=(-0.34, 0.0, 0.180)),
        material=materials["stand_paint"],
        name="left_rail",
    )
    dolly_column.visual(
        Box((0.05, 0.74, 0.045)),
        origin=Origin(xyz=(0.34, 0.0, 0.180)),
        material=materials["stand_paint"],
        name="right_rail",
    )
    dolly_column.visual(
        Box((0.68, 0.06, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=materials["stand_paint"],
        name="cross_rail_x",
    )
    dolly_column.visual(
        Box((0.06, 0.68, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=materials["stand_paint"],
        name="cross_rail_y",
    )
    dolly_column.visual(
        Cylinder(radius=0.120, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        material=materials["dark_metal"],
        name="base_hub",
    )
    dolly_column.visual(
        Cylinder(radius=0.060, length=1.360),
        origin=Origin(xyz=(0.0, 0.0, 0.880)),
        material=materials["stand_paint"],
        name="column_tube",
    )
    dolly_column.visual(
        Cylinder(radius=0.078, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 1.630)),
        material=materials["dark_metal"],
        name="top_bearing_housing",
    )
    dolly_column.visual(
        Cylinder(radius=0.060, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 1.760)),
        material=materials["stand_paint"],
        name="spigot_post",
    )

    wheel_mounts = {
        "front_left": (-0.34, 0.34),
        "front_right": (0.34, 0.34),
        "rear_left": (-0.34, -0.34),
        "rear_right": (0.34, -0.34),
    }
    for mount_name, (wheel_x, wheel_y) in wheel_mounts.items():
        dolly_column.visual(
            Box((0.070, 0.045, 0.012)),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.153)),
            material=materials["dark_metal"],
            name=f"{mount_name}_fork_crown",
        )
        dolly_column.visual(
            Box((0.012, 0.045, 0.092)),
            origin=Origin(xyz=(wheel_x - 0.031, wheel_y, 0.102)),
            material=materials["dark_metal"],
            name=f"{mount_name}_fork_left",
        )
        dolly_column.visual(
            Box((0.012, 0.045, 0.092)),
            origin=Origin(xyz=(wheel_x + 0.031, wheel_y, 0.102)),
            material=materials["dark_metal"],
            name=f"{mount_name}_fork_right",
        )
        dolly_column.visual(
            Box((0.034, 0.050, 0.026)),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.168)),
            material=materials["stand_paint"],
            name=f"{mount_name}_stem",
        )
    dolly_column.inertial = Inertial.from_geometry(
        Box((0.78, 0.78, 1.84)),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, 0.920)),
    )

    pan_bracket = model.part("pan_bracket")
    pan_bracket.visual(
        Cylinder(radius=0.090, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=materials["dark_metal"],
        name="pan_drum",
    )
    pan_bracket.visual(
        Box((0.100, 0.060, 0.110)),
        origin=Origin(xyz=(0.0, -0.090, 0.115)),
        material=materials["stand_paint"],
        name="pan_spine",
    )
    pan_bracket.visual(
        Box((0.420, 0.050, 0.070)),
        origin=Origin(xyz=(0.0, -0.110, 0.240)),
        material=materials["stand_paint"],
        name="ear_bridge",
    )
    pan_bracket.visual(
        Box((0.030, 0.210, 0.210)),
        origin=Origin(xyz=(-0.207, 0.020, 0.215)),
        material=materials["stand_paint"],
        name="left_ear",
    )
    pan_bracket.visual(
        Box((0.030, 0.210, 0.210)),
        origin=Origin(xyz=(0.207, 0.020, 0.215)),
        material=materials["stand_paint"],
        name="right_ear",
    )
    pan_bracket.visual(
        Box((0.120, 0.060, 0.080)),
        origin=Origin(xyz=(0.0, -0.095, 0.175)),
        material=materials["dark_metal"],
        name="yoke_core",
    )
    pan_bracket.visual(
        Cylinder(radius=0.046, length=0.038),
        origin=Origin(xyz=(-0.207, 0.020, 0.215), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["dark_metal"],
        name="left_boss",
    )
    pan_bracket.visual(
        Cylinder(radius=0.046, length=0.038),
        origin=Origin(xyz=(0.207, 0.020, 0.215), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["dark_metal"],
        name="right_boss",
    )
    pan_bracket.inertial = Inertial.from_geometry(
        Box((0.45, 0.24, 0.32)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        lamp_shell_mesh,
        material=materials["housing_paint"],
        name="housing_shell",
    )
    lamp_head.visual(
        Box((0.392, 0.030, 0.332)),
        origin=Origin(xyz=(0.0, 0.160, 0.0)),
        material=materials["stand_paint"],
        name="front_bezel",
    )
    lamp_head.visual(
        Box((0.344, 0.012, 0.282)),
        origin=Origin(xyz=(0.0, 0.175, 0.0)),
        material=materials["lens_glass"],
        name="lens_face",
    )
    lamp_head.visual(
        Box((0.200, 0.040, 0.160)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=materials["dark_metal"],
        name="rear_cap",
    )
    lamp_head.visual(
        Box((0.160, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.062, 0.060)),
        material=materials["dark_metal"],
        name="cooling_fin_upper",
    )
    lamp_head.visual(
        Box((0.160, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.062, 0.020)),
        material=materials["dark_metal"],
        name="cooling_fin_mid",
    )
    lamp_head.visual(
        Box((0.160, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.062, -0.020)),
        material=materials["dark_metal"],
        name="cooling_fin_lower",
    )
    lamp_head.visual(handle_mesh, material=materials["steel"], name="carry_handle")
    lamp_head.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(-0.172, 0.030, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["knob"],
        name="left_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(0.172, 0.030, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["knob"],
        name="right_trunnion",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.40, 0.26, 0.34)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
    )

    front_left_wheel = _add_wheel_part(model, "front_left_wheel", tire_mesh, materials)
    front_right_wheel = _add_wheel_part(model, "front_right_wheel", tire_mesh, materials)
    rear_left_wheel = _add_wheel_part(model, "rear_left_wheel", tire_mesh, materials)
    rear_right_wheel = _add_wheel_part(model, "rear_right_wheel", tire_mesh, materials)

    model.articulation(
        "column_to_pan",
        ArticulationType.CONTINUOUS,
        parent=dolly_column,
        child=pan_bracket,
        origin=Origin(xyz=(0.0, 0.0, 1.820)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=3.0),
    )
    model.articulation(
        "pan_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_bracket,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.020, 0.215)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.10, upper=0.85),
    )

    wheel_parts = {
        "front_left_wheel_spin": (front_left_wheel, (-0.34, 0.34, 0.075)),
        "front_right_wheel_spin": (front_right_wheel, (0.34, 0.34, 0.075)),
        "rear_left_wheel_spin": (rear_left_wheel, (-0.34, -0.34, 0.075)),
        "rear_right_wheel_spin": (rear_right_wheel, (0.34, -0.34, 0.075)),
    }
    for joint_name, (wheel_part, xyz) in wheel_parts.items():
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=dolly_column,
            child=wheel_part,
            origin=Origin(xyz=xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=25.0),
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
    dolly_column = object_model.get_part("dolly_column")
    pan_bracket = object_model.get_part("pan_bracket")
    lamp_head = object_model.get_part("lamp_head")

    pan_joint = object_model.get_articulation("column_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_head_tilt")

    ctx.expect_gap(
        pan_bracket,
        dolly_column,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="pan bracket seats on column top",
    )
    ctx.expect_within(
        lamp_head,
        pan_bracket,
        axes="x",
        margin=0.004,
        name="lamp head stays between yoke ears",
    )

    lens_rest = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="lens_face"))
    with ctx.pose({tilt_joint: 0.75}):
        lens_tilted = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="lens_face"))
    ctx.check(
        "tilt articulation raises the lamp face",
        lens_rest is not None and lens_tilted is not None and lens_tilted[2] > lens_rest[2] + 0.10,
        details=f"rest={lens_rest}, tilted={lens_tilted}",
    )

    with ctx.pose({pan_joint: math.pi / 2.0}):
        lens_panned = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="lens_face"))
    ctx.check(
        "pan articulation swings the lamp around the column",
        lens_rest is not None
        and lens_panned is not None
        and abs(lens_panned[0] - lens_rest[0]) > 0.16
        and abs(lens_panned[1] - lens_rest[1]) > 0.16,
        details=f"rest={lens_rest}, panned={lens_panned}",
    )

    wheel_joint_names = (
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    )
    for joint_name in wheel_joint_names:
        wheel_joint = object_model.get_articulation(joint_name)
        limits = wheel_joint.motion_limits
        ctx.check(
            f"{joint_name} is continuous wheel rotation",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and wheel_joint.axis == (1.0, 0.0, 0.0),
            details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}, limits={limits}",
        )

    front_left_wheel = object_model.get_part("front_left_wheel")
    front_left_joint = object_model.get_articulation("front_left_wheel_spin")
    valve_rest = _aabb_center(ctx.part_element_world_aabb(front_left_wheel, elem="valve_cap"))
    with ctx.pose({front_left_joint: math.pi / 2.0}):
        valve_spun = _aabb_center(ctx.part_element_world_aabb(front_left_wheel, elem="valve_cap"))
    ctx.check(
        "front left wheel spin moves an asymmetric wheel detail",
        valve_rest is not None
        and valve_spun is not None
        and abs(valve_spun[1] - valve_rest[1]) > 0.04
        and abs(valve_spun[2] - valve_rest[2]) > 0.04,
        details=f"rest={valve_rest}, spun={valve_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
