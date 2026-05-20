from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tilting_fan", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    shell_graphite = model.material("shell_graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.64, 0.66, 0.70, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    smoked_blade = model.material("smoked_blade", rgba=(0.38, 0.42, 0.45, 0.76))
    warm_indicator = model.material("warm_indicator", rgba=(0.80, 0.68, 0.44, 1.0))

    def add_ring_segments(
        part,
        *,
        prefix: str,
        radius: float,
        x_pos: float,
        rod_radius: float,
        segments: int,
        material,
        ref_name: str | None = None,
    ) -> None:
        half_step = math.pi / segments
        chord_radius = radius * math.cos(half_step)
        seg_len = (2.0 * radius * math.sin(half_step)) + (2.6 * rod_radius)
        for index in range(segments):
            angle = (index + 0.5) * math.tau / segments
            name = ref_name if index == 0 and ref_name is not None else f"{prefix}_{index}"
            part.visual(
                Cylinder(radius=rod_radius, length=seg_len),
                origin=Origin(
                    xyz=(x_pos, chord_radius * math.cos(angle), chord_radius * math.sin(angle)),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=material,
                name=name,
            )

    def add_radial_spokes(
        part,
        *,
        prefix: str,
        x_pos: float,
        inner_radius: float,
        outer_radius: float,
        rod_radius: float,
        count: int,
        material,
        phase: float = 0.0,
    ) -> None:
        spoke_len = (outer_radius - inner_radius) + (2.2 * rod_radius)
        mid_radius = 0.5 * (inner_radius + outer_radius)
        for index in range(count):
            angle = phase + (index * math.tau / count)
            part.visual(
                Cylinder(radius=rod_radius, length=spoke_len),
                origin=Origin(
                    xyz=(x_pos, mid_radius * math.cos(angle), mid_radius * math.sin(angle)),
                    rpy=(angle - (math.pi / 2.0), 0.0, 0.0),
                ),
                material=material,
                name=f"{prefix}_{index}",
            )

    def add_axial_links(
        part,
        *,
        prefix: str,
        radius: float,
        x_start: float,
        x_end: float,
        rod_radius: float,
        count: int,
        material,
        phase: float = 0.0,
    ) -> None:
        link_len = (x_end - x_start) + (2.2 * rod_radius)
        x_mid = 0.5 * (x_start + x_end)
        for index in range(count):
            angle = phase + (index * math.tau / count)
            part.visual(
                Cylinder(radius=rod_radius, length=link_len),
                origin=Origin(
                    xyz=(x_mid, radius * math.cos(angle), radius * math.sin(angle)),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=material,
                name=f"{prefix}_{index}",
            )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.094, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_rubber,
        name="foot_ring",
    )
    base.visual(
        Cylinder(radius=0.118, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=matte_graphite,
        name="weighted_body",
    )
    base.visual(
        Cylinder(radius=0.092, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0245)),
        material=satin_aluminum,
        name="top_cap",
    )
    base.visual(
        Cylinder(radius=0.104, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=satin_aluminum,
        name="base_trim_ring",
    )
    base.visual(
        Cylinder(radius=0.021, length=0.188),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=shell_graphite,
        name="stem",
    )
    base.visual(
        Cylinder(radius=0.031, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=satin_aluminum,
        name="stem_collar",
    )
    base.visual(
        Box((0.064, 0.038, 0.012)),
        origin=Origin(xyz=(0.050, 0.0, 0.032)),
        material=shell_graphite,
        name="control_pod",
    )
    base.visual(
        Box((0.048, 0.026, 0.004)),
        origin=Origin(xyz=(0.058, 0.0, 0.040)),
        material=satin_aluminum,
        name="control_pod_bezel",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.118, length=0.242),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=satin_aluminum,
        name="clamp_collar",
    )
    yoke.visual(
        Box((0.040, 0.154, 0.020)),
        origin=Origin(xyz=(-0.010, 0.0, 0.010)),
        material=satin_aluminum,
        name="rear_bridge",
    )
    yoke.visual(
        Box((0.012, 0.110, 0.032)),
        origin=Origin(xyz=(-0.020, 0.0, 0.026)),
        material=satin_aluminum,
        name="front_gusset",
    )
    for side, cap_radius, cap_length, cap_name in (
        (-1.0, 0.014, 0.010, "left_pivot_cap"),
        (1.0, 0.018, 0.012, "right_tilt_knob"),
    ):
        side_name = "left" if side < 0.0 else "right"
        yoke.visual(
            Box((0.020, 0.020, 0.108)),
            origin=Origin(xyz=(0.000, side * 0.078, 0.064)),
            material=satin_aluminum,
            name=f"{side_name}_arm",
        )
        yoke.visual(
            Cylinder(radius=0.014, length=0.016),
            origin=Origin(
                xyz=(0.000, side * 0.074, 0.084),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=satin_aluminum,
            name=f"{side_name}_pivot_boss",
        )
        yoke.visual(
            Cylinder(radius=cap_radius, length=cap_length),
            origin=Origin(
                xyz=(0.000, side * 0.090, 0.084),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=matte_graphite if side < 0.0 else shell_graphite,
            name=cap_name,
        )
    yoke.inertial = Inertial.from_geometry(
        Box((0.048, 0.186, 0.128)),
        mass=1.2,
        origin=Origin(xyz=(-0.002, 0.0, 0.064)),
    )

    head_carriage = model.part("head_carriage")
    head_carriage.visual(
        Box((0.024, 0.100, 0.028)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=shell_graphite,
        name="mount_block",
    )
    head_carriage.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
        material=shell_graphite,
        name="rear_dome",
    )
    head_carriage.visual(
        Cylinder(radius=0.042, length=0.070),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_graphite,
        name="motor_shell",
    )
    head_carriage.visual(
        Cylinder(radius=0.045, length=0.004),
        origin=Origin(xyz=(0.079, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="head_trim_ring",
    )
    head_carriage.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="front_mount",
    )
    head_carriage.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="motor_nose",
    )
    for side, name in ((-1.0, "left_trunnion"), (1.0, "right_trunnion")):
        head_carriage.visual(
            Cylinder(radius=0.013, length=0.020),
            origin=Origin(
                xyz=(0.000, side * 0.058, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=shell_graphite,
            name=name,
        )
    head_carriage.inertial = Inertial.from_geometry(
        Box((0.132, 0.120, 0.090)),
        mass=1.6,
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
    )

    guard_assembly = model.part("guard_assembly")
    guard_assembly.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(-0.017, 0.049, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="left_mount_bar",
    )
    guard_assembly.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(-0.017, -0.049, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="right_mount_bar",
    )
    add_ring_segments(
        guard_assembly,
        prefix="rear_ring",
        radius=0.112,
        x_pos=-0.006,
        rod_radius=0.0032,
        segments=12,
        material=satin_aluminum,
        ref_name="rear_ring_ref",
    )
    add_ring_segments(
        guard_assembly,
        prefix="front_ring",
        radius=0.118,
        x_pos=0.086,
        rod_radius=0.0034,
        segments=12,
        material=satin_aluminum,
        ref_name="front_guard",
    )
    add_radial_spokes(
        guard_assembly,
        prefix="rear_spoke",
        x_pos=-0.006,
        inner_radius=0.032,
        outer_radius=0.112,
        rod_radius=0.0020,
        count=6,
        material=satin_aluminum,
    )
    add_radial_spokes(
        guard_assembly,
        prefix="front_spoke",
        x_pos=0.086,
        inner_radius=0.026,
        outer_radius=0.118,
        rod_radius=0.0018,
        count=12,
        material=satin_aluminum,
    )
    add_axial_links(
        guard_assembly,
        prefix="axial_link",
        radius=0.112,
        x_start=-0.006,
        x_end=0.086,
        rod_radius=0.0018,
        count=6,
        material=satin_aluminum,
        phase=0.0,
    )
    guard_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.120, length=0.100),
        mass=0.8,
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.000, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="hub_spindle",
    )
    rotor.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_graphite,
        name="hub_shell",
    )
    rotor.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material=satin_aluminum,
        name="spinner",
    )
    for index in range(5):
        angle = index * math.tau / 5.0
        rotor.visual(
            Box((0.034, 0.054, 0.004)),
            origin=Origin(
                xyz=(0.032, 0.046 * math.cos(angle), 0.046 * math.sin(angle)),
                rpy=(angle, math.radians(8.0), 0.0),
            ),
            material=smoked_blade,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.076, length=0.060),
        mass=0.45,
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=matte_graphite,
        name="dial_base",
    )
    speed_dial.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=shell_graphite,
        name="dial_crown",
    )
    speed_dial.visual(
        Box((0.004, 0.016, 0.003)),
        origin=Origin(xyz=(0.013, 0.0, 0.0135)),
        material=warm_indicator,
        name="dial_pointer",
    )
    speed_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.016),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.FIXED,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head_carriage,
        origin=Origin(xyz=(0.000, 0.0, 0.084)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.5,
            lower=math.radians(-18.0),
            upper=math.radians(24.0),
        ),
    )
    model.articulation(
        "head_to_guard",
        ArticulationType.FIXED,
        parent=head_carriage,
        child=guard_assembly,
        origin=Origin(xyz=(0.107, 0.0, 0.0)),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head_carriage,
        child=rotor,
        origin=Origin(xyz=(0.107, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=24.0),
    )
    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(0.050, 0.0, 0.037)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=2.0,
            lower=math.radians(-110.0),
            upper=math.radians(110.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    head_carriage = object_model.get_part("head_carriage")
    guard_assembly = object_model.get_part("guard_assembly")
    rotor = object_model.get_part("rotor")
    speed_dial = object_model.get_part("speed_dial")

    yoke_to_head = object_model.get_articulation("yoke_to_head")
    head_to_rotor = object_model.get_articulation("head_to_rotor")
    base_to_speed_dial = object_model.get_articulation("base_to_speed_dial")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(yoke, base, name="yoke_mount_contact")
    ctx.expect_contact(head_carriage, yoke, name="tilt_pivot_contact")
    ctx.expect_contact(guard_assembly, head_carriage, name="guard_mount_contact")
    ctx.expect_contact(rotor, head_carriage, name="rotor_mount_contact")
    ctx.expect_contact(speed_dial, base, name="speed_dial_mount_contact")

    ctx.expect_within(rotor, guard_assembly, axes="yz", margin=0.0, name="rotor_within_guard")
    ctx.expect_gap(
        guard_assembly,
        rotor,
        axis="x",
        min_gap=0.006,
        positive_elem="front_guard",
        negative_elem="spinner",
        name="front_guard_spinner_clearance",
    )
    ctx.expect_gap(
        rotor,
        guard_assembly,
        axis="x",
        min_gap=0.001,
        max_gap=0.010,
        positive_elem="hub_shell",
        negative_elem="rear_ring_ref",
        name="rear_guard_hub_clearance",
    )

    tilt_limits = yoke_to_head.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        for pose_name, pose_value in (
            ("tilt_lower", tilt_limits.lower),
            ("tilt_upper", tilt_limits.upper),
        ):
            with ctx.pose({yoke_to_head: pose_value}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{pose_name}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{pose_name}_no_floating")
                ctx.expect_contact(head_carriage, yoke, name=f"{pose_name}_pivot_stays_seated")
                ctx.expect_within(
                    rotor,
                    guard_assembly,
                    axes="yz",
                    margin=0.0,
                    name=f"{pose_name}_rotor_within_guard",
                )

    blade_rest = ctx.part_element_world_aabb(rotor, elem="blade_0")
    blade_rest_center = None
    if blade_rest is not None:
        blade_rest_center = (
            (blade_rest[0][0] + blade_rest[1][0]) * 0.5,
            (blade_rest[0][1] + blade_rest[1][1]) * 0.5,
            (blade_rest[0][2] + blade_rest[1][2]) * 0.5,
        )
    with ctx.pose({head_to_rotor: math.radians(40.0)}):
        blade_spun = ctx.part_element_world_aabb(rotor, elem="blade_0")
        ctx.expect_within(rotor, guard_assembly, axes="yz", margin=0.0, name="spun_rotor_within_guard")
        if blade_rest_center is not None and blade_spun is not None:
            blade_spun_center = (
                (blade_spun[0][0] + blade_spun[1][0]) * 0.5,
                (blade_spun[0][1] + blade_spun[1][1]) * 0.5,
                (blade_spun[0][2] + blade_spun[1][2]) * 0.5,
            )
            moved = (
                abs(blade_spun_center[1] - blade_rest_center[1]) > 0.010
                or abs(blade_spun_center[2] - blade_rest_center[2]) > 0.010
            )
            ctx.check(
                "rotor_spin_moves_named_blade",
                moved,
                details=f"blade center did not move enough: rest={blade_rest_center}, spun={blade_spun_center}",
            )

    pointer_rest = ctx.part_element_world_aabb(speed_dial, elem="dial_pointer")
    pointer_rest_center = None
    if pointer_rest is not None:
        pointer_rest_center = (
            (pointer_rest[0][0] + pointer_rest[1][0]) * 0.5,
            (pointer_rest[0][1] + pointer_rest[1][1]) * 0.5,
            (pointer_rest[0][2] + pointer_rest[1][2]) * 0.5,
        )
    with ctx.pose({base_to_speed_dial: math.radians(95.0)}):
        pointer_turned = ctx.part_element_world_aabb(speed_dial, elem="dial_pointer")
        ctx.expect_contact(speed_dial, base, name="dial_remains_seated_when_turned")
        if pointer_rest_center is not None and pointer_turned is not None:
            pointer_turned_center = (
                (pointer_turned[0][0] + pointer_turned[1][0]) * 0.5,
                (pointer_turned[0][1] + pointer_turned[1][1]) * 0.5,
                (pointer_turned[0][2] + pointer_turned[1][2]) * 0.5,
            )
            moved = (
                abs(pointer_turned_center[0] - pointer_rest_center[0]) > 0.006
                and abs(pointer_turned_center[1] - pointer_rest_center[1]) > 0.006
            )
            ctx.check(
                "speed_dial_pointer_rotates",
                moved,
                details=(
                    f"dial pointer center did not rotate enough: "
                    f"rest={pointer_rest_center}, turned={pointer_turned_center}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
