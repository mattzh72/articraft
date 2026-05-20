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
)


def _add_parallel_arm(
    part,
    *,
    rod_radius: float,
    rod_length: float,
    rod_spacing: float,
    rod_z: float,
    rod_start_x: float,
    bridge_size: tuple[float, float, float],
    bridge_center_x: float,
    bridge_name: str,
    tip_size: tuple[float, float, float],
    tip_center_x: float,
    tip_name: str,
    arm_material,
    hardware_material,
) -> None:
    part.visual(
        Box(bridge_size),
        origin=Origin(xyz=(bridge_center_x, 0.0, rod_z)),
        material=hardware_material,
        name=bridge_name,
    )
    for side in (-1.0, 1.0):
        part.visual(
            Cylinder(radius=rod_radius, length=rod_length),
            origin=Origin(
                xyz=(rod_start_x + rod_length * 0.5, side * rod_spacing * 0.5, rod_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=arm_material,
        )
    part.visual(
        Box(tip_size),
        origin=Origin(xyz=(tip_center_x, 0.0, rod_z)),
        material=hardware_material,
        name=tip_name,
    )


def _build_shade_shell():
    outer_profile = [
        (0.020, 0.000),
        (0.024, 0.012),
        (0.039, 0.055),
        (0.055, 0.105),
        (0.068, 0.155),
    ]
    inner_profile = [
        (0.014, 0.006),
        (0.018, 0.018),
        (0.032, 0.055),
        (0.049, 0.105),
        (0.061, 0.151),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
        "drafting_lamp_shade_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clip_on_drafting_lamp")

    painted_steel = model.material("painted_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    arm_aluminum = model.material("arm_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.30, 0.31, 0.34, 1.0))
    shade_enamel = model.material("shade_enamel", rgba=(0.90, 0.90, 0.87, 1.0))
    rubber_pad = model.material("rubber_pad", rgba=(0.08, 0.08, 0.09, 1.0))

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        Box((0.105, 0.052, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, 0.047)),
        material=painted_steel,
        name="upper_jaw",
    )
    clamp_base.visual(
        Box((0.060, 0.040, 0.004)),
        origin=Origin(xyz=(0.035, 0.0, 0.036)),
        material=rubber_pad,
        name="upper_pad",
    )
    clamp_base.visual(
        Box((0.082, 0.044, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, -0.049)),
        material=painted_steel,
        name="lower_jaw",
    )
    clamp_base.visual(
        Box((0.045, 0.036, 0.004)),
        origin=Origin(xyz=(0.026, 0.0, -0.038)),
        material=rubber_pad,
        name="lower_pad",
    )
    clamp_base.visual(
        Box((0.020, 0.052, 0.112)),
        origin=Origin(xyz=(-0.030, 0.0, -0.003)),
        material=painted_steel,
        name="rear_spine",
    )
    clamp_base.visual(
        Cylinder(radius=0.015, length=0.196),
        origin=Origin(xyz=(-0.032, 0.0, 0.140)),
        material=painted_steel,
        name="post",
    )
    clamp_base.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=Origin(xyz=(-0.032, 0.0, 0.244)),
        material=dark_hardware,
        name="post_cap",
    )
    clamp_base.visual(
        Cylinder(radius=0.005, length=0.072),
        origin=Origin(xyz=(0.042, 0.0, -0.010)),
        material=dark_hardware,
        name="clamp_screw",
    )
    clamp_base.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.042, 0.0, 0.029)),
        material=dark_hardware,
        name="pressure_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.004, length=0.044),
        origin=Origin(xyz=(0.042, 0.0, -0.048), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="screw_handle",
    )
    clamp_base.inertial = Inertial.from_geometry(
        Box((0.150, 0.060, 0.310)),
        mass=1.8,
        origin=Origin(xyz=(0.015, 0.0, 0.105)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_hardware,
        name="shoulder_hub",
    )
    _add_parallel_arm(
        lower_arm,
        rod_radius=0.0055,
        rod_length=0.255,
        rod_spacing=0.044,
        rod_z=0.014,
        rod_start_x=0.028,
        bridge_size=(0.050, 0.055, 0.012),
        bridge_center_x=0.025,
        bridge_name="shoulder_bridge",
        tip_size=(0.034, 0.055, 0.016),
        tip_center_x=0.300,
        tip_name="elbow_mount",
        arm_material=arm_aluminum,
        hardware_material=dark_hardware,
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.334, 0.060, 0.040)),
        mass=0.55,
        origin=Origin(xyz=(0.167, 0.0, 0.014)),
    )

    upper_arm = model.part("upper_arm")
    _add_parallel_arm(
        upper_arm,
        rod_radius=0.0050,
        rod_length=0.240,
        rod_spacing=0.040,
        rod_z=0.014,
        rod_start_x=0.026,
        bridge_size=(0.026, 0.048, 0.032),
        bridge_center_x=0.013,
        bridge_name="elbow_hub",
        tip_size=(0.030, 0.052, 0.018),
        tip_center_x=0.280,
        tip_name="shade_mount",
        arm_material=arm_aluminum,
        hardware_material=dark_hardware,
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.295, 0.055, 0.045)),
        mass=0.42,
        origin=Origin(xyz=(0.148, 0.0, 0.014)),
    )

    shade = model.part("shade")
    shade_down_angle = 0.72
    shade_shell = _build_shade_shell()
    shade.visual(
        Box((0.012, 0.050, 0.016)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=dark_hardware,
        name="shade_yoke",
    )
    for side in (-1.0, 1.0):
        shade.visual(
            Box((0.020, 0.008, 0.030)),
            origin=Origin(xyz=(0.010, side * 0.021, -0.001)),
            material=dark_hardware,
            name=f"yoke_cheek_{'left' if side > 0 else 'right'}",
        )
    shade.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(
            xyz=(0.026, 0.0, -0.004),
            rpy=(0.0, math.pi / 2.0 + shade_down_angle, 0.0),
        ),
        material=dark_hardware,
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(
            xyz=(0.040, 0.0, -0.012),
            rpy=(0.0, math.pi / 2.0 + shade_down_angle, 0.0),
        ),
        material=dark_hardware,
        name="lamp_socket",
    )
    shade.visual(
        shade_shell,
        origin=Origin(xyz=(0.030, 0.0, -0.006), rpy=(0.0, shade_down_angle, 0.0)),
        material=shade_enamel,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.185, 0.140, 0.110)),
        mass=0.36,
        origin=Origin(xyz=(0.090, 0.0, -0.050)),
    )

    model.articulation(
        "clamp_to_lower_arm_swivel",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(xyz=(-0.032, 0.0, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-2.5,
            upper=2.5,
        ),
    )
    model.articulation(
        "lower_to_upper_arm_elbow",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.317, 0.0, 0.014)),
        # The upper arm extends along local +X from the elbow.
        # -Y makes positive q raise the tip toward +Z.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=-1.05,
            upper=1.25,
        ),
    )
    model.articulation(
        "upper_arm_to_shade_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(0.295, 0.0, 0.014)),
        # The shade shell is authored already angled downward at q=0.
        # -Y makes positive q tilt the beam back upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.2,
            lower=-0.75,
            upper=0.95,
        ),
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

    clamp_base = object_model.get_part("clamp_base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    swivel = object_model.get_articulation("clamp_to_lower_arm_swivel")
    elbow = object_model.get_articulation("lower_to_upper_arm_elbow")
    tilt = object_model.get_articulation("upper_arm_to_shade_tilt")

    ctx.check(
        "shoulder swivel uses a vertical axis",
        swivel.axis == (0.0, 0.0, 1.0),
        details=f"axis={swivel.axis}",
    )
    ctx.check(
        "elbow hinge raises on positive motion",
        elbow.axis == (0.0, -1.0, 0.0),
        details=f"axis={elbow.axis}",
    )
    ctx.check(
        "shade tilt uses a lateral axis",
        tilt.axis == (0.0, -1.0, 0.0),
        details=f"axis={tilt.axis}",
    )

    with ctx.pose({swivel: 0.0, elbow: 0.0, tilt: 0.0}):
        ctx.expect_gap(
            lower_arm,
            clamp_base,
            axis="z",
            positive_elem="shoulder_hub",
            negative_elem="post_cap",
            max_gap=0.001,
            max_penetration=0.0,
            name="lower arm shoulder seats on the clamp post",
        )
        ctx.expect_overlap(
            lower_arm,
            clamp_base,
            axes="xy",
            elem_a="shoulder_hub",
            elem_b="post_cap",
            min_overlap=0.030,
            name="shoulder hub stays centered over the clamp post",
        )
        ctx.expect_gap(
            upper_arm,
            lower_arm,
            axis="x",
            positive_elem="elbow_hub",
            negative_elem="elbow_mount",
            max_gap=0.001,
            max_penetration=0.0,
            name="upper arm elbow hub meets the lower arm clevis",
        )
        ctx.expect_overlap(
            upper_arm,
            lower_arm,
            axes="yz",
            elem_a="elbow_hub",
            elem_b="elbow_mount",
            min_overlap=0.009,
            name="elbow plates overlap around the hinge axis",
        )
        ctx.expect_gap(
            shade,
            upper_arm,
            axis="x",
            positive_elem="shade_yoke",
            negative_elem="shade_mount",
            max_gap=0.001,
            max_penetration=0.0,
            name="shade yoke sits at the tip block",
        )
        ctx.expect_overlap(
            shade,
            upper_arm,
            axes="yz",
            elem_a="shade_yoke",
            elem_b="shade_mount",
            min_overlap=0.014,
            name="shade pivot stays aligned with the arm tip",
        )

        rest_elbow_pos = ctx.part_world_position(upper_arm)
        rest_shade_tip = ctx.part_world_position(shade)
        rest_shade_aabb = ctx.part_world_aabb(shade)

    with ctx.pose({swivel: 0.80, elbow: 0.0, tilt: 0.0}):
        swung_elbow_pos = ctx.part_world_position(upper_arm)

    ctx.check(
        "base swivel swings the arm laterally",
        rest_elbow_pos is not None
        and swung_elbow_pos is not None
        and swung_elbow_pos[1] > rest_elbow_pos[1] + 0.18,
        details=f"rest={rest_elbow_pos}, swung={swung_elbow_pos}",
    )

    with ctx.pose({swivel: 0.0, elbow: 0.85, tilt: 0.0}):
        raised_shade_tip = ctx.part_world_position(shade)

    ctx.check(
        "elbow lift raises the shade tip",
        rest_shade_tip is not None
        and raised_shade_tip is not None
        and raised_shade_tip[2] > rest_shade_tip[2] + 0.10,
        details=f"rest={rest_shade_tip}, raised={raised_shade_tip}",
    )

    with ctx.pose({swivel: 0.0, elbow: 0.0, tilt: 0.60}):
        tilted_shade_aabb = ctx.part_world_aabb(shade)

    ctx.check(
        "shade tilt can lift the cone upward",
        rest_shade_aabb is not None
        and tilted_shade_aabb is not None
        and tilted_shade_aabb[1][2] > rest_shade_aabb[1][2] + 0.020,
        details=f"rest={rest_shade_aabb}, tilted={tilted_shade_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
