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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _seat_section(
    x: float,
    *,
    width: float,
    thickness: float,
    z_shift: float,
) -> list[tuple[float, float, float]]:
    radius = min(0.035, thickness * 0.45, width * 0.14)
    profile = rounded_rect_profile(width, thickness, radius=radius, corner_segments=8)
    return [(x, y, z + z_shift) for y, z in profile]


def _build_seat_shell():
    sections = [
        _seat_section(-0.210, width=0.400, thickness=0.034, z_shift=0.000),
        _seat_section(-0.080, width=0.450, thickness=0.046, z_shift=0.002),
        _seat_section(0.090, width=0.465, thickness=0.050, z_shift=-0.003),
        _seat_section(0.230, width=0.430, thickness=0.030, z_shift=-0.010),
    ]
    return _save_mesh("task_chair_seat_shell", section_loft(sections))


def _build_back_frame():
    loop = wire_from_points(
        [
            (-0.022, -0.195, 0.050),
            (-0.022, -0.195, 0.355),
            (-0.022, -0.150, 0.445),
            (-0.022, -0.070, 0.505),
            (-0.022, 0.070, 0.505),
            (-0.022, 0.150, 0.445),
            (-0.022, 0.195, 0.355),
            (-0.022, 0.195, 0.050),
        ],
        radius=0.012,
        radial_segments=16,
        closed_path=True,
        cap_ends=False,
        corner_mode="fillet",
        corner_radius=0.050,
        corner_segments=10,
        up_hint=(1.0, 0.0, 0.0),
    )
    return _save_mesh("task_chair_back_frame", loop)


def _build_hook_shape():
    return _save_mesh(
        "task_chair_hook_shape",
        tube_from_spline_points(
            [
                (-0.002, 0.0, -0.004),
                (-0.008, 0.0, -0.026),
                (-0.020, 0.0, -0.060),
                (-0.023, 0.0, -0.094),
                (-0.014, 0.0, -0.116),
                (0.006, 0.0, -0.106),
            ],
            radius=0.0045,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
    )


def _build_star_arm():
    return _save_mesh(
        "task_chair_star_arm",
        sweep_profile_along_spline(
            [
                (0.044, 0.0, 0.078),
                (0.165, 0.0, 0.074),
                (0.315, 0.0, 0.067),
            ],
            profile=rounded_rect_profile(0.055, 0.018, radius=0.007, corner_segments=6),
            samples_per_segment=16,
            cap_profile=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_task_chair")

    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    soft_black = model.material("soft_black", rgba=(0.15, 0.15, 0.16, 1.0))
    silver = model.material("silver", rgba=(0.72, 0.74, 0.76, 1.0))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.28, 0.29, 0.31, 1.0))
    nylon = model.material("nylon", rgba=(0.14, 0.14, 0.15, 1.0))

    seat_shell_mesh = _build_seat_shell()
    back_frame_mesh = _build_back_frame()
    hook_shape_mesh = _build_hook_shape()
    star_arm_mesh = _build_star_arm()

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.060, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=silver,
        name="hub_core",
    )
    base.visual(
        Cylinder(radius=0.039, length=0.292),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=graphite,
        name="column_shroud",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.387)),
        material=dark_grey,
        name="top_plate",
    )
    for index in range(5):
        angle = index * 2.0 * math.pi / 5.0
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            star_arm_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=silver,
            name=f"star_arm_{index}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(0.315 * c, 0.315 * s, 0.064)),
            material=dark_grey,
            name=f"caster_pad_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 0.40)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
    )

    upper_assembly = model.part("upper_assembly")
    upper_assembly.visual(
        Cylinder(radius=0.024, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=chrome,
        name="inner_column",
    )
    upper_assembly.visual(
        Cylinder(radius=0.051, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=black,
        name="swivel_plate",
    )
    upper_assembly.visual(
        Box((0.165, 0.175, 0.036)),
        origin=Origin(xyz=(0.000, 0.000, 0.044)),
        material=graphite,
        name="tilt_housing",
    )
    upper_assembly.visual(
        Box((0.230, 0.250, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.060)),
        material=black,
        name="seat_support_plate",
    )
    upper_assembly.visual(
        seat_shell_mesh,
        origin=Origin(xyz=(0.032, 0.0, 0.084)),
        material=soft_black,
        name="seat_shell",
    )
    upper_assembly.visual(
        Box((0.070, 0.020, 0.012)),
        origin=Origin(xyz=(0.035, -0.094, 0.042)),
        material=black,
        name="height_lever",
    )
    upper_assembly.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(
            xyz=(0.070, -0.102, 0.042),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black,
        name="height_lever_grip",
    )
    upper_assembly.visual(
        Box((0.090, 0.360, 0.026)),
        origin=Origin(xyz=(-0.110, 0.0, 0.064)),
        material=graphite,
        name="rear_bridge",
    )
    for side, y in (("left", 0.172), ("right", -0.172)):
        upper_assembly.visual(
            Box((0.030, 0.016, 0.078)),
            origin=Origin(xyz=(-0.168, y, 0.095)),
            material=graphite,
            name=f"back_hinge_bracket_{side}",
        )
    upper_assembly.inertial = Inertial.from_geometry(
        Box((0.50, 0.50, 0.18)),
        mass=6.5,
        origin=Origin(xyz=(0.020, 0.0, 0.090)),
    )

    backrest = model.part("backrest")
    for side, y in (("left", 0.152), ("right", -0.152)):
        backrest.visual(
            Cylinder(radius=0.013, length=0.016),
            origin=Origin(
                xyz=(0.0, y, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=graphite,
            name=f"pivot_lug_{side}",
        )
    backrest.visual(
        Box((0.040, 0.330, 0.034)),
        origin=Origin(xyz=(-0.012, 0.0, 0.026)),
        material=graphite,
        name="lower_crossbar",
    )
    backrest.visual(
        back_frame_mesh,
        material=graphite,
        name="back_frame",
    )
    backrest.visual(
        Box((0.012, 0.338, 0.392)),
        origin=Origin(xyz=(-0.014, 0.0, 0.245)),
        material=soft_black,
        name="back_panel",
    )
    backrest.visual(
        Box((0.022, 0.070, 0.120)),
        origin=Origin(xyz=(-0.010, 0.0, 0.150)),
        material=soft_black,
        name="lumbar_spine",
    )
    for side, y in (("left", 0.022), ("right", -0.022)):
        backrest.visual(
            Box((0.020, 0.008, 0.040)),
            origin=Origin(xyz=(-0.026, y, 0.388)),
            material=graphite,
            name=f"hook_mount_{side}",
        )
    backrest.inertial = Inertial.from_geometry(
        Box((0.08, 0.44, 0.54)),
        mass=3.5,
        origin=Origin(xyz=(-0.010, 0.0, 0.260)),
    )

    hook = model.part("hook")
    hook.visual(
        Cylinder(radius=0.007, length=0.032),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=nylon,
        name="hook_barrel",
    )
    hook.visual(
        hook_shape_mesh,
        material=nylon,
        name="hook_arm",
    )
    hook.visual(
        Sphere(radius=0.0065),
        origin=Origin(xyz=(0.006, 0.0, -0.106)),
        material=nylon,
        name="hook_tip",
    )
    hook.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, 0.130)),
        mass=0.12,
        origin=Origin(xyz=(-0.010, 0.0, -0.060)),
    )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=upper_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.393)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5),
    )
    model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=upper_assembly,
        child=backrest,
        origin=Origin(xyz=(-0.188, 0.0, 0.082)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=0.38,
        ),
    )
    model.articulation(
        "back_to_hook",
        ArticulationType.REVOLUTE,
        parent=backrest,
        child=hook,
        origin=Origin(xyz=(-0.030, 0.0, 0.388)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    caster_angles = []
    for index in range(5):
        angle = index * 2.0 * math.pi / 5.0
        caster_angles.append(angle)
        c = math.cos(angle)
        s = math.sin(angle)

        fork = model.part(f"caster_fork_{index}")
        fork.visual(
            Cylinder(radius=0.005, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=graphite,
            name="stem",
        )
        fork.visual(
            Box((0.022, 0.022, 0.010)),
            origin=Origin(xyz=(0.010, 0.0, -0.014)),
            material=graphite,
            name="crown",
        )
        fork.visual(
            Box((0.008, 0.003, 0.028)),
            origin=Origin(xyz=(0.014, 0.0105, -0.033)),
            material=graphite,
            name="fork_arm_left",
        )
        fork.visual(
            Box((0.008, 0.003, 0.028)),
            origin=Origin(xyz=(0.014, -0.0105, -0.033)),
            material=graphite,
            name="fork_arm_right",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.040, 0.036, 0.070)),
            mass=0.18,
            origin=Origin(xyz=(0.0, 0.0, -0.024)),
        )

        wheel = model.part(f"caster_wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(
                xyz=(0.0, 0.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black,
            name="tread",
        )
        wheel.visual(
            Cylinder(radius=0.0105, length=0.012),
            origin=Origin(
                xyz=(0.0, 0.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_grey,
            name="hub",
        )
        wheel.visual(
            Box((0.004, 0.006, 0.012)),
            origin=Origin(xyz=(0.015, 0.0, 0.0)),
            material=dark_grey,
            name="valve_stem",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.024, length=0.014),
            mass=0.12,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"base_to_caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(
                xyz=(0.315 * c, 0.315 * s, 0.062),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=8.0),
        )
        model.articulation(
            f"caster_to_wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.014, 0.0, -0.039)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=20.0),
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
    base = object_model.get_part("base")
    upper_assembly = object_model.get_part("upper_assembly")
    backrest = object_model.get_part("backrest")
    hook = object_model.get_part("hook")

    swivel = object_model.get_articulation("base_swivel")
    back_recline = object_model.get_articulation("back_recline")
    hook_joint = object_model.get_articulation("back_to_hook")
    caster_swivel = object_model.get_articulation("base_to_caster_swivel_0")
    caster_spin = object_model.get_articulation("caster_to_wheel_spin_0")

    ctx.check(
        "primary joints use expected types",
        swivel.joint_type == ArticulationType.CONTINUOUS
        and back_recline.joint_type == ArticulationType.REVOLUTE
        and hook_joint.joint_type == ArticulationType.REVOLUTE
        and caster_swivel.joint_type == ArticulationType.CONTINUOUS
        and caster_spin.joint_type == ArticulationType.CONTINUOUS,
        details=(
            f"swivel={swivel.joint_type}, back={back_recline.joint_type}, "
            f"hook={hook_joint.joint_type}, caster_swivel={caster_swivel.joint_type}, "
            f"caster_spin={caster_spin.joint_type}"
        ),
    )
    ctx.check(
        "joint axes match intended mechanisms",
        swivel.axis == (0.0, 0.0, 1.0)
        and back_recline.axis == (0.0, -1.0, 0.0)
        and hook_joint.axis == (0.0, 1.0, 0.0)
        and caster_swivel.axis == (0.0, 0.0, 1.0)
        and caster_spin.axis == (0.0, 1.0, 0.0),
        details=(
            f"swivel={swivel.axis}, back={back_recline.axis}, "
            f"hook={hook_joint.axis}, caster_swivel={caster_swivel.axis}, "
            f"caster_spin={caster_spin.axis}"
        ),
    )

    ctx.expect_contact(
        upper_assembly,
        base,
        elem_a="swivel_plate",
        elem_b="top_plate",
        contact_tol=0.0005,
        name="swivel plate seats on the base top plate",
    )
    ctx.expect_overlap(
        upper_assembly,
        base,
        axes="xy",
        elem_a="swivel_plate",
        elem_b="top_plate",
        min_overlap=0.090,
        name="swivel bearing stays centered over the column",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    lever_rest = _center_from_aabb(
        ctx.part_element_world_aabb(upper_assembly, elem="height_lever_grip")
    )
    with ctx.pose({swivel: math.pi / 2.0}):
        lever_turned = _center_from_aabb(
            ctx.part_element_world_aabb(upper_assembly, elem="height_lever_grip")
        )
    ctx.check(
        "seat assembly swivels about the column",
        lever_rest is not None
        and lever_turned is not None
        and abs(lever_turned[1] - lever_rest[1]) > 0.15
        and abs(
            math.hypot(lever_rest[0], lever_rest[1])
            - math.hypot(lever_turned[0], lever_turned[1])
        )
        < 0.005,
        details=f"rest={lever_rest}, turned={lever_turned}",
    )

    back_rest = _center_from_aabb(ctx.part_element_world_aabb(backrest, elem="back_panel"))
    with ctx.pose({back_recline: back_recline.motion_limits.upper}):
        back_reclined = _center_from_aabb(
            ctx.part_element_world_aabb(backrest, elem="back_panel")
        )
    ctx.check(
        "backrest reclines rearward",
        back_rest is not None
        and back_reclined is not None
        and back_reclined[0] < back_rest[0] - 0.06,
        details=f"upright={back_rest}, reclined={back_reclined}",
    )

    hook_rest = _center_from_aabb(ctx.part_element_world_aabb(hook, elem="hook_tip"))
    with ctx.pose({hook_joint: hook_joint.motion_limits.upper}):
        hook_open = _center_from_aabb(ctx.part_element_world_aabb(hook, elem="hook_tip"))
    ctx.check(
        "coat hook folds outward from behind the back",
        hook_rest is not None
        and hook_open is not None
        and hook_open[0] < hook_rest[0] - 0.030,
        details=f"rest={hook_rest}, open={hook_open}",
    )

    wheel = object_model.get_part("caster_wheel_0")
    wheel_stem_rest = _center_from_aabb(ctx.part_element_world_aabb(wheel, elem="valve_stem"))
    with ctx.pose({caster_swivel: math.pi / 2.0}):
        wheel_after_swivel = ctx.part_element_world_aabb(wheel, elem="tread")
    with ctx.pose({caster_spin: math.pi / 2.0}):
        wheel_stem_turned = _center_from_aabb(
            ctx.part_element_world_aabb(wheel, elem="valve_stem")
        )

    wheel_dims_after_swivel = None
    if wheel_after_swivel is not None:
        wheel_dims_after_swivel = tuple(
            wheel_after_swivel[1][i] - wheel_after_swivel[0][i] for i in range(3)
        )

    ctx.check(
        "caster swivel rotates wheel orientation in plan",
        wheel_dims_after_swivel is not None and wheel_dims_after_swivel[1] > 0.045,
        details=f"dims_after_swivel={wheel_dims_after_swivel}",
    )
    ctx.check(
        "caster wheel spins around its axle",
        wheel_stem_rest is not None
        and wheel_stem_turned is not None
        and abs(wheel_stem_rest[2] - wheel_stem_turned[2]) > 0.012,
        details=f"rest={wheel_stem_rest}, turned={wheel_stem_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
